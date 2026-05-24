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

// --- Flagi zdarzeń z ISR (przyciski) ---
static volatile bool g_btnPressed       = false;  // przycisk główny (GPIO4)
static volatile bool g_emergencyPressed     = false;  // przycisk "Error"/E-Stop (GPIO5)

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

// Service UUID (dowolny, byle stały)
static NimBLEUUID SERVICE_UUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");

// RX: laptop -> ESP (Write)
static NimBLEUUID CHAR_RX_UUID("beb5483e-36e1-4688-b7f5-ea07361b26a8");

// Notify: EVENT (jedna char dla wszystkich zdarzeń)
static NimBLEUUID CHAR_EVENT_UUID("44444444-4444-4444-4444-444444444444");

// Pomocnicze wysyłki notify
static inline void notifyEventIfPossible(uint8_t eventCode) {
  if (!g_connected || g_notifyEvent == nullptr) return;

  g_notifyEvent->setValue(&eventCode, 1);
  g_notifyEvent->notify();
}

// Wywołaj po „przyjęciu” parametrów w logice (FSM)
static void notifyParamsRead() {
  notifyEventIfPossible(EVT_PARAMS_READ);
}

// Wywołaj, gdy logika wyrzutni uzna, że „wystrzał zakończony”
void bleNotifyShotDone() {
  notifyEventIfPossible(EVT_SHOT_DONE);
}

// Wywołaj, gdy wciśnięto Emergency Stop
void bleNotifyEmergencyStop() {
  notifyEventIfPossible(EVT_EMERGENCY_STOP);
}


// Callbacki serwera (connect/disconnect)
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
    g_connected = true;
  }

  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
    g_connected = false;
    NimBLEDevice::startAdvertising();
  }
};

// RX (Write) callback
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

  // RX: Write/WriteNR
  NimBLECharacteristic* rxChar = service->createCharacteristic(
    CHAR_RX_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
  );
  rxChar->setCallbacks(new RxCallbacks());

  // Notify: EVENT (jedna charakterystyka)
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

  // Wyjścia: ustaw stan początkowy zanim przełączysz na OUTPUT
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

// ISR: przycisk Error / Emergency
void IRAM_ATTR isrErrorPressed() {
  portENTER_CRITICAL_ISR(&g_mux);
  g_emergencyPressed = true;
  portEXIT_CRITICAL_ISR(&g_mux);
}

// Przycisk usuwanie flagi
static inline bool consumeBtnPressed() {
  bool v;
  portENTER_CRITICAL(&g_mux);
  v = g_btnPressed;
  g_btnPressed = false;
  portEXIT_CRITICAL(&g_mux);
  return v;
}

// Przycisk emergency stop usuwanie flagi
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
  uint32_t now = (uint32_t)micros();  // pobierz czas
  //sekcja krytyczna bo g_last_detection jest teraz też używane do odliczania do wystrzału
  portENTER_CRITICAL_ISR(&g_timerMux);
  uint32_t local_measured_period_us = now - g_last_detection_time_us;
  portEXIT_CRITICAL_ISR(&g_timerMux);

  // filtr sygnałów za częstych
  if (local_measured_period_us > filter_us) {

    portENTER_CRITICAL_ISR(&g_timerMux);

    g_measured_period_us = local_measured_period_us;
    g_last_detection_time_us = now;
    g_new_revolution = true;

    // nie uzbrajamy timera i nie ustawiamy g_ready_to_launch

    portEXIT_CRITICAL_ISR(&g_timerMux);
  }
}

// Przerwanie timer   <- nie używam timera

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

    // Hz = 10^6 / period_us
    const float hz = 1.0e6f / static_cast<float>(period_us);

    // pwm = (Hz - b) / a
    // (opcjonalnie) zabezpieczenie, gdyby a było 0 lub bardzo bliskie 0
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

// Ustawienie PWM silnika (jedno miejsce prawdy: ledcWrite + g_motor_pwm)
static inline void DCmotor_set_PWM(uint16_t pwm) {
    const uint16_t pwm_max = static_cast<uint16_t>((1u << PWM_RES_BITS) - 1u);
    if (pwm > pwm_max) pwm = pwm_max;
    g_motor_pwm = pwm;
    ledcWrite(PWM_CH, g_motor_pwm);
}

// Hamowanie ramienia: wywołuj raz na obrót (np. gdy g_new_revolution == true)
static inline void deaccelerate(float factor = 0.75f, uint16_t stopThreshold = 100) {
    // Walidacja parametru
    if (!(factor > 0.0f && factor < 1.0f)) {
        factor = 0.75f;
    }

    // Jeśli już małe — zatrzymaj
    if (g_motor_pwm <= stopThreshold) {
        DCmotor_set_PWM(0);
        return;
    }

    // Skala w float + kontrolowane zaokrąglenie
    const float scaled = static_cast<float>(g_motor_pwm) * factor;
    int pwm_i = static_cast<int>(std::lround(scaled));

    // Clamp do zakresu
    const int pwm_max = (1 << PWM_RES_BITS) - 1;
    if (pwm_i < 0) pwm_i = 0;
    if (pwm_i > pwm_max) pwm_i = pwm_max;

    // Jeśli po redukcji spadło poniżej progu — zatrzymaj całkiem
    if (pwm_i <= static_cast<int>(stopThreshold)) {
        DCmotor_set_PWM(0);
        return;
    }

    // Normalny krok hamowania
    DCmotor_set_PWM(static_cast<uint16_t>(pwm_i));
}

// Rozpędzanie: JEDEN KROK na wywołanie (np. raz na obrót)
static inline void accelerate_to(uint16_t target) {
    const uint16_t pwm_max = static_cast<uint16_t>((1u << PWM_RES_BITS) - 1u);

    // clamp do zakresu PWM i limitu "bezpiecznego"
    if (target > pwm_max) target = pwm_max;
    if (target > 850)     target = 850;

    // Jeżeli już jesteśmy na/ponad target — nie rozpędzamy "w dół"
    if (g_motor_pwm >= target) {
        return;
    }

    // Target <= 500: ustaw bezpośrednio
    if (target <= 500) {
        DCmotor_set_PWM(target);
        return;
    }

    // 501..800: logika 500 +100 +100 ... +reszta (1 krok na wywołanie)
    if (target <= 800) {
        if (g_motor_pwm < 500) {
            DCmotor_set_PWM(500);
            return;
        }

        uint16_t next = static_cast<uint16_t>(g_motor_pwm + 100);

        if (next <= target) {
            DCmotor_set_PWM(next);     // krok 100
        } else {
            DCmotor_set_PWM(target);   // końcowa "reszta"
        }
        return;
    }

    // 801..850: logika 800 +10 +10 ... +reszta (1 krok na wywołanie)
    if (g_motor_pwm < 800) {
        DCmotor_set_PWM(800);
        return;
    }

    uint16_t next = static_cast<uint16_t>(g_motor_pwm + 10);

    if (next <= target) {
        DCmotor_set_PWM(next);         // krok 10
    } else {
        DCmotor_set_PWM(target);       // końcowa "reszta"
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

    // sprawdznie czy stan się zmienił (przejścia między stanami)
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
            g_newParamsReceived = false;   // konsumujesz atomowo
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
            

            // Dla pewności wyzeruj licznik stabilnych obrotów na wejściu w SpinUp
            correct_speed_revolutions = 0;
        }

        // Skopiuj dane z ISR atomowo i skonsumuj flagę "nowy obrót"
        uint32_t measured = 0;
        bool newRev = false;

        portENTER_CRITICAL(&g_timerMux);
        measured = g_measured_period_us;
        newRev   = g_new_revolution;
        if (newRev) {
            g_new_revolution = false;   // consume
        }
        portEXIT_CRITICAL(&g_timerMux);

        // Cała logika tylko wtedy, gdy faktycznie przyszedł nowy obrót
        if (newRev) {
            // 1 krok rozpędzania na obrót
            accelerate_to(pwm_target);

            // Ocena stabilności prędkości na podstawie świeżego pomiaru
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
      // bezpośrednio po wejściu do stanu jeśli doliczamy do wystrzału
      if (g_counting_down){
        // pobierz czas
        uint32_t now = (uint32_t)micros();

        // obliczanie czasu, ktry upłynął od ostatniego przecięcia czujnika szczelinowego
        portENTER_CRITICAL(&g_timerMux);
        uint32_t time_passed_us = now - g_last_detection_time_us;
        portEXIT_CRITICAL(&g_timerMux);

        // jeśli czas upłyną wypuść pocisk i przejdź do reset
        if (time_passed_us >= fi_us){
          solenoid_off();
          state = LauncherState::Reset;
        }
      }


      if (stateChanged) {
          // Snapshot okresu z ISR (spójny odczyt)
          uint32_t measured = 0;
          portENTER_CRITICAL(&g_timerMux);
          measured = g_measured_period_us;
          portEXIT_CRITICAL(&g_timerMux);

          // Wyliczenie czasu jaki zajmie rameiniu wykonanie fi_deg stopni z 360
          fi_us = calculate_fi_us(measured, g_fi_deg);

          // ustawienie flagi ale bez sekcji krytycznej bo nie ma potrzeby
          g_ready_to_launch = true;
          }


      // jeśli nowe okrążenie i ready to launch zacznij odliczanie w loop()
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
        //DCmotor_off();
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
        // wymuszaj stan bezpieczny
        //DCmotor_off();
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
            // 1) Zatrzymaj timer (żeby nie generował kolejnego ISR)
            timerAlarmDisable(g_timer);
            timerWrite(g_timer, 0);

            // 2) Wyczyść flagi powiązane z timerem/czujnikiem atomowo
            portENTER_CRITICAL(&g_timerMux);
            g_ready_to_launch = false;
            g_new_revolution = false;
            g_counting_down = false;
            portEXIT_CRITICAL(&g_timerMux);

            // (opcjonalnie) wyzeruj licznik stabilnych obrotów
            correct_speed_revolutions = 0;

            // 3) Notify tylko raz
            bleNotifyEmergencyStop();
        }
    }
    break;

    }
}

