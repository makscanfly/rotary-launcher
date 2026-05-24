#pragma region INCLUDES
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <cmath>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#pragma endregion


#pragma region ZMIENNE_GLOBALNE_W_TYM_FLAGI

// zmienne do wystrzału
uint32_t fi_us = 0;

// Prototypy ISR
void IRAM_ATTR isrButtonPressed();
void IRAM_ATTR isrErrorPressed();
void IRAM_ATTR onDetectorISR();
void IRAM_ATTR onTimerISR();

// kody zdarzeń
static constexpr uint8_t EVT_EMERGENCY_STOP = 1;
static constexpr uint8_t EVT_PARAMS_READ    = 2;
static constexpr uint8_t EVT_SHOT_DONE      = 3;

// Timer
hw_timer_t* g_timer = nullptr;
portMUX_TYPE g_timerMux = portMUX_INITIALIZER_UNLOCKED;

// filtr fałszywych pomiarow
uint32_t filter_us = 500;

// POMIARY!!!
volatile uint32_t g_measured_period_us = 0;
volatile uint32_t g_last_detection_time_us = 0;
uint32_t correct_speed_revolutions = 0;
volatile bool g_new_revolution = false;

// --- Flagi przyciski ---
static volatile bool g_btnPressed       = false;  // przycisk główny (GPIO4)
static volatile bool g_emergencyPressed     = false;  // Emergency stop (GPIO5)

// parametry do PWM
float a = 0.005232035229580135f;
float b = -0.06060162774904451f;
static uint16_t pwm_target = 0;
uint16_t g_motor_pwm = 0;

// stany FSM
enum class LauncherState : uint8_t {
  Idle = 0,
  Arm = 1,
  SpinUp = 2,
  Launch = 3,
  Reset = 4,
  EmergencyStop = 5
};

LauncherState state = LauncherState::Idle;

// flagi
static volatile bool g_connected = false;
static volatile bool g_newParamsReceived = false;
static LauncherState g_prevState = LauncherState::Idle;
volatile bool g_ready_to_launch = false;
volatile bool g_counting_down = false;

// parametry wystrzału (odebrane z laptopa)
static uint32_t g_period_us = 0;
static float   g_fi_deg    = 0.0f;

// MUX do ochrony spójności danych współdzielonych między callbackami BLE i loop()
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

// BLE: wskaźniki na charakterystyki notify
static NimBLECharacteristic* g_notifyEvent = nullptr;

#pragma endregion


#pragma region KOD_BLUETOOTH_BLE

static const char* DEV_NAME = "LauncherESP";

// Service UUID
static NimBLEUUID SERVICE_UUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");

// RX: laptop -> ESP (Write)
static NimBLEUUID CHAR_RX_UUID("beb5483e-36e1-4688-b7f5-ea07361b26a8");

// Notify ESP -> laptop
static NimBLEUUID CHAR_EVENT_UUID("44444444-4444-4444-4444-444444444444");

// Pomocnicze wysyłki notify
static inline void notifyEventIfPossible(uint8_t eventCode) {
  if (!g_connected || g_notifyEvent == nullptr) return;

  g_notifyEvent->setValue(&eventCode, 1);
  g_notifyEvent->notify();
}

static void notifyParamsRead() {
  notifyEventIfPossible(EVT_PARAMS_READ);
}

// wystrał zakończony
void bleNotifyShotDone() {
  notifyEventIfPossible(EVT_SHOT_DONE);
}

// Emergency Stop
void bleNotifyEmergencyStop() {
  notifyEventIfPossible(EVT_EMERGENCY_STOP);
}


// connect / disconnect
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
    g_connected = true;
  }

  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
    g_connected = false;
    NimBLEDevice::startAdvertising();
  }
};

// RX (Write)
class RxCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {

    LauncherState s;
    portENTER_CRITICAL(&g_mux);
    s = state;
    portEXIT_CRITICAL(&g_mux);

    if (s != LauncherState::Idle) {
        return;
    }

    std::string value = pCharacteristic->getValue();
    if (value.size() != 8) {
        return;
    }

    uint32_t i = 0;
    float f = 0.0f;
    memcpy(&i, value.data(), 4);
    memcpy(&f, value.data() + 4, 4);

    portENTER_CRITICAL(&g_mux);
    g_period_us = i;
    g_fi_deg    = f;
    g_newParamsReceived = true;
    portEXIT_CRITICAL(&g_mux);
    }
};

static void bleSetup() {
  NimBLEDevice::init(DEV_NAME);

  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  NimBLEService* service = server->createService(SERVICE_UUID);

  // write
  NimBLECharacteristic* rxChar = service->createCharacteristic(
    CHAR_RX_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
  );
  rxChar->setCallbacks(new RxCallbacks());

  // notify
  g_notifyEvent = service->createCharacteristic(
    CHAR_EVENT_UUID,
    NIMBLE_PROPERTY::NOTIFY
  );

  service->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setName(DEV_NAME);
  adv->start();
}

#pragma endregion


#pragma region GPIO_PWM_KONFIGURACJA

// --- Wejścia ---
static constexpr gpio_num_t PIN_BTN        = GPIO_NUM_4;
static constexpr gpio_num_t PIN_BTN_ERROR  = GPIO_NUM_5;
static constexpr gpio_num_t PIN_SLOT       = GPIO_NUM_6;

// --- Wyjścia ---
static constexpr gpio_num_t PIN_DIR        = GPIO_NUM_7;
static constexpr gpio_num_t PIN_MOSFET     = GPIO_NUM_15;
static constexpr gpio_num_t PIN_LED        = GPIO_NUM_16;

// PWM (LEDC)
static constexpr gpio_num_t PIN_PWM        = GPIO_NUM_18;
static constexpr int PWM_CH               = 0;      // kanał LEDC 0..15
static constexpr int PWM_FREQ_HZ           = 18000;  // np. 20 kHz
static constexpr int PWM_RES_BITS          = 10;     // 0..1023

static void setupPins() {
  // Wejścia
  pinMode((int)PIN_BTN, INPUT_PULLUP);
  pinMode((int)PIN_BTN_ERROR, INPUT_PULLUP);
  pinMode((int)PIN_SLOT, INPUT);

  // Wyjścia
  digitalWrite((int)PIN_DIR, LOW);
  digitalWrite((int)PIN_MOSFET, LOW);
  digitalWrite((int)PIN_LED, LOW);

  pinMode((int)PIN_DIR, OUTPUT);
  pinMode((int)PIN_MOSFET, OUTPUT);
  pinMode((int)PIN_LED, OUTPUT);

  // PWM
  ledcSetup(PWM_CH, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin((int)PIN_PWM, PWM_CH);
  ledcWrite(PWM_CH, 0);
}

#pragma endregion


#pragma region PRZERWANIA_I_ISR

// Konfiguracja przerwań od przyciskow i czujnika szczelinowego
static void setupInterruptsAndTimer() {
  // Przyciski
  attachInterrupt(digitalPinToInterrupt((int)PIN_BTN),       isrButtonPressed, FALLING);
  attachInterrupt(digitalPinToInterrupt((int)PIN_BTN_ERROR), isrErrorPressed,  FALLING);

  // Czujnik szczelinowy
  attachInterrupt(digitalPinToInterrupt((int)PIN_SLOT), onDetectorISR, FALLING);

  // Timer
  g_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(g_timer, &onTimerISR, true);

  timerAlarmDisable(g_timer);
  timerWrite(g_timer, 0);

}

// ISR: przycisk główny
void IRAM_ATTR isrButtonPressed() {
  portENTER_CRITICAL_ISR(&g_mux);
  g_btnPressed = true;
  portEXIT_CRITICAL_ISR(&g_mux);
}

// ISR: Emergency stop
void IRAM_ATTR isrErrorPressed() {
  portENTER_CRITICAL_ISR(&g_mux);
  g_emergencyPressed = true;
  portEXIT_CRITICAL_ISR(&g_mux);
}

// usuwanie flagi przyciski główny
static inline bool consumeBtnPressed() {
  bool v;
  portENTER_CRITICAL(&g_mux);
  v = g_btnPressed;
  g_btnPressed = false;
  portEXIT_CRITICAL(&g_mux);
  return v;
}

// emergency stop usuwanie flagi
static inline bool consumeEmergencyPressed() {
  bool v;
  portENTER_CRITICAL(&g_mux);
  v = g_emergencyPressed;
  g_emergencyPressed = false;
  portEXIT_CRITICAL(&g_mux);
  return v;
}

// ISR: czujnik szczelinowy
void IRAM_ATTR onDetectorISR() {
  uint32_t now = (uint32_t)micros();
  portENTER_CRITICAL_ISR(&g_timerMux);
  uint32_t local_measured_period_us = now - g_last_detection_time_us;
  portEXIT_CRITICAL_ISR(&g_timerMux);

  // filtr sygnałów za częstych
  if (local_measured_period_us > filter_us) {

    portENTER_CRITICAL_ISR(&g_timerMux);

    g_measured_period_us = local_measured_period_us;
    g_last_detection_time_us = now;
    g_new_revolution = true;
    portEXIT_CRITICAL_ISR(&g_timerMux);
  }
}

#pragma endregion


#pragma region LOGIKA_POMOCNICZA

//  ELEKTROMAGNES
inline void solenoid_on() {
  digitalWrite((int)PIN_MOSFET, HIGH);
}

inline void solenoid_off() {
  digitalWrite((int)PIN_MOSFET, LOW);
}

// SILNIK DC
void prepare_DCmotor(uint32_t period_us) {
    // Zabezpieczenie przed dzieleniem przez zero (period_us == 0 nie ma sensu fizycznie)
    if (period_us == 0) {
        pwm_target = 0;
        return;
    }

    const float hz = 1.0e6f / static_cast<float>(period_us);

    if (std::fabs(a) < 1.0e-9f) {
        pwm_target = 0;
        return;
    }

    const float temp = (hz - b) / a;

    int pwm_i = (int)std::lround(temp);
    const int pwm_max = (1 << PWM_RES_BITS) - 1;

    if (pwm_i < 0) pwm_i = 0;
    if (pwm_i > pwm_max) pwm_i = pwm_max;

    pwm_target = static_cast<uint16_t>(pwm_i);
}



// OBLICZENIA KąTA
uint32_t calculate_fi_us(uint32_t period_us, float fi_deg){
    return static_cast<uint32_t> (period_us*fi_deg/360.0f);
}

// OBSŁUGA LED
void LED_on(){
    digitalWrite((int)PIN_LED, HIGH);
}

void LED_off(){
    digitalWrite((int)PIN_LED, LOW);
}

// Ustawienie PWM silnika
static inline void DCmotor_set_PWM(uint16_t pwm) {
    const uint16_t pwm_max = static_cast<uint16_t>((1u << PWM_RES_BITS) - 1u);
    if (pwm > pwm_max) pwm = pwm_max;
    g_motor_pwm = pwm;
    ledcWrite(PWM_CH, g_motor_pwm);
}

// stopniowe hamowanie
static inline void deaccelerate(float factor = 0.75f, uint16_t stopThreshold = 100) {
    // Walidacja parametru
    if (!(factor > 0.0f && factor < 1.0f)) {
        factor = 0.75f;
    }

    if (g_motor_pwm <= stopThreshold) {
        DCmotor_set_PWM(0);
        return;
    }

    const float scaled = static_cast<float>(g_motor_pwm) * factor;
    int pwm_i = static_cast<int>(std::lround(scaled));

    // clamp
    const int pwm_max = (1 << PWM_RES_BITS) - 1;
    if (pwm_i < 0) pwm_i = 0;
    if (pwm_i > pwm_max) pwm_i = pwm_max;

    if (pwm_i <= static_cast<int>(stopThreshold)) {
        DCmotor_set_PWM(0);
        return;
    }

    DCmotor_set_PWM(static_cast<uint16_t>(pwm_i));
}

// stopniowe rozpędzanie
static inline void accelerate_to(uint16_t target) {
    const uint16_t pwm_max = static_cast<uint16_t>((1u << PWM_RES_BITS) - 1u);

    // clamp
    if (target > pwm_max) target = pwm_max;
    if (target > 850)     target = 850;

    if (g_motor_pwm >= target) {
        return;
    }

    if (target <= 500) {
        DCmotor_set_PWM(target);
        return;
    }

    if (target <= 800) {
        if (g_motor_pwm < 500) {
            DCmotor_set_PWM(500);
            return;
        }

        uint16_t next = static_cast<uint16_t>(g_motor_pwm + 100);

        if (next <= target) {
            DCmotor_set_PWM(next);
        } else {
            DCmotor_set_PWM(target);
        }
        return;
    }

    if (g_motor_pwm < 800) {
        DCmotor_set_PWM(800);
        return;
    }

    uint16_t next = static_cast<uint16_t>(g_motor_pwm + 10);

    if (next <= target) {
        DCmotor_set_PWM(next);
    } else {
        DCmotor_set_PWM(target);
    }
}



#pragma endregion


void setup() {
  Serial.begin(115200);
  Serial.println("eooe");

  setupPins();
  Serial.println("Pins set");

  setupInterruptsAndTimer();
  Serial.println("Interrupts and timer set");

  bleSetup();
  Serial.println("BLE set");
}


void loop() {

    // sprawdznie czy emergency pressed
    if (consumeEmergencyPressed()) {
    state = LauncherState::EmergencyStop;
    }

    // sprawdznie czy stan się zmienił
    bool stateChanged = (state != g_prevState);
    if (stateChanged) {
        g_prevState = state;
    }

    switch (state) {
    case LauncherState::Idle:
    {
        uint32_t period_us = 0;
        bool haveNew = false;

        portENTER_CRITICAL(&g_mux);
        haveNew = g_newParamsReceived;
        if (haveNew) {
            period_us = g_period_us;
            g_newParamsReceived = false;
        }
        portEXIT_CRITICAL(&g_mux);

        if (haveNew) {
            notifyParamsRead();

            prepare_DCmotor(period_us);

            state = LauncherState::Arm;
        }
    }
    break;

    case LauncherState::Arm:
    {
        if (stateChanged) {
            LED_on();
            solenoid_on();
        }

        if (consumeBtnPressed()) {
            state = LauncherState::SpinUp;
        }
    }
    break;

    case LauncherState::SpinUp:
    {
        if (stateChanged) {       
            // trzeba ruszyć silnik
            uint16_t first = (pwm_target <= 500) ? pwm_target : 500;
            g_motor_pwm = first;
            DCmotor_set_PWM(g_motor_pwm);
            
            correct_speed_revolutions = 0;
        }

        uint32_t measured = 0;
        bool newRev = false;

        portENTER_CRITICAL(&g_timerMux);
        measured = g_measured_period_us;
        newRev   = g_new_revolution;
        if (newRev) {
            g_new_revolution = false;   // consume
        }
        portEXIT_CRITICAL(&g_timerMux);

        // cała logika tylko wtedy, gdy faktycznie przyszedł nowy obrót
        if (newRev) {
            accelerate_to(pwm_target);

            uint32_t diff = (measured > g_period_us)
                ? (measured - g_period_us)
                : (g_period_us - measured);

            if (diff >= 50000) {
                correct_speed_revolutions = 0;
            } else {
                correct_speed_revolutions++;
                if (correct_speed_revolutions >= 14) {
                    state = LauncherState::Launch;
                }
            }
        }
    }
    break;

    case LauncherState::Launch:
    {
      // odliczanie do wystrzału
      if (g_counting_down){
        // pobranie czasu
        uint32_t now = (uint32_t)micros();

        // obliczanie czasu który upłyał od ostatniego przecięcia czujnika szczelinowego
        portENTER_CRITICAL(&g_timerMux);
        uint32_t time_passed_us = now - g_last_detection_time_us;
        portEXIT_CRITICAL(&g_timerMux);

        // wstrzał i reset
        if (time_passed_us >= fi_us){
          solenoid_off();
          state = LauncherState::Reset;
        }
      }


      if (stateChanged) {
          uint32_t measured = 0;
          portENTER_CRITICAL(&g_timerMux);
          measured = g_measured_period_us;
          portEXIT_CRITICAL(&g_timerMux);

          fi_us = calculate_fi_us(measured, g_fi_deg);

          g_ready_to_launch = true;
          }


      bool newRev = false;
      portENTER_CRITICAL(&g_timerMux);
      newRev = g_new_revolution;
      if (newRev) g_new_revolution = false;
      portEXIT_CRITICAL(&g_timerMux);

      if (newRev && g_ready_to_launch){
        g_counting_down = true;
      }
    }
    break;

    case LauncherState::Reset:
    {
      if(stateChanged){
        bleNotifyShotDone();
        g_counting_down = false;
        g_ready_to_launch = false;
      }

      bool newRev = false;

      portENTER_CRITICAL(&g_timerMux);
      newRev = g_new_revolution;
      if (newRev) g_new_revolution = false;
      portEXIT_CRITICAL(&g_timerMux);

      if (newRev) {
          deaccelerate(0.85f, 150);
}
    }
    break;

    case LauncherState::EmergencyStop:
    {
        solenoid_on();

      bool newRev = false;

      portENTER_CRITICAL(&g_timerMux);
      newRev = g_new_revolution;
      if (newRev) g_new_revolution = false;
      portEXIT_CRITICAL(&g_timerMux);

      if (newRev) {
          deaccelerate(0.85f, 150);
      }

        if (stateChanged) {
            timerAlarmDisable(g_timer);
            timerWrite(g_timer, 0);

            portENTER_CRITICAL(&g_timerMux);
            g_ready_to_launch = false;
            g_new_revolution = false;
            g_counting_down = false;
            portEXIT_CRITICAL(&g_timerMux);

            correct_speed_revolutions = 0;

            // notify że stop
            bleNotifyEmergencyStop();
        }
    }
    break;

    }
}

