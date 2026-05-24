#pragma region INCLUDES
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <cmath>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#pragma endregion


#pragma region ZMIENNE_GLOBALNE_W_TYM_FLAGI

// Kody poleceń
enum class Command : uint8_t {
  LedOn = 1,
  LedOff = 2,   
  ResetSampleCollecting = 3,    // przygotowana wcześniej funkcja
  FinishSampleCollecting  = 4,  // g_samples_ready = true, g_capture_enabled = false
  SetMotorPwm = 5,              // ustawia wypełnienie PWM'a zgodne z wartością podaną przez BLE
  StopMotor = 6,                // ustawienie wypełnienia na 0 niezależnie od wartości przekazanej prez BLE
  ElectromagnetOn = 7,
  ElectromagnetOff = 8,
  DirHigh = 9,
  DirLow = 10,
  Deaccelerate = 11,
  Accelerate_to = 12
};

// PWM docelowe
volatile uint16_t pwm_target = 0;

// -- wartość ostatniego ustawionego pwm --
volatile uint16_t g_motor_pwm = 0;

// -- Kontrolowane wyhamowywanie i rozpędzanie --
bool g_slowing_down = false;
bool g_accelerating = false;

// --- Snapshot control ---
volatile bool g_capture_enabled = true;  // zbieramy dopóki nie zrobimy snapshotu / nie wyłączymy

// --- Status BLE ---
static volatile bool g_connected = false;

// --- MUXy ---
static portMUX_TYPE g_timerMux   = portMUX_INITIALIZER_UNLOCKED; // ISR czujnika
static portMUX_TYPE g_cmdMux     = portMUX_INITIALIZER_UNLOCKED; // BLE cmd + pwm
static portMUX_TYPE g_samplesMux = portMUX_INITIALIZER_UNLOCKED; // próbki

// --- Prototypy ISR ---
void IRAM_ATTR onDetectorISR();

// --- Pojedynczy pomiar z ISR ---
volatile uint32_t g_measured_period_us = 0;
volatile uint32_t g_last_detection_time_us = 0;
volatile bool     g_new_revolution = false;
static constexpr uint32_t filter_us = 500;

// --- Polecenia BLE (Write) ---
volatile uint8_t  g_cmd = 0;
volatile uint16_t g_pwm = 0;
volatile bool     g_newCmd = false;

// --- Snapshot pomiarów (Read) ---
static constexpr uint16_t SAMPLES_COUNT = 100;
volatile uint32_t g_samples[SAMPLES_COUNT];
volatile uint16_t g_samples_index = 0;
volatile bool     g_samples_ready = false;

#pragma endregion


#pragma region KOD_BLUETOOTH

static const char* DEV_NAME = "LauncherESP";

// Service UUID (stały)
static NimBLEUUID SERVICE_UUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");

// Charakterystyki
static NimBLEUUID CHAR_RX_UUID("beb5483e-36e1-4688-b7f5-ea07361b26a8"); // Write
static NimBLEUUID CHAR_TX_UUID("aaaaaaaa-aaaa-aaaa-aaaa-aaaaaaaaaaaa"); // Read (NOWY)

// --- Callbacki serwera ---
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*, NimBLEConnInfo&) override {
    g_connected = true;
  }

  void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override {
    g_connected = false;
    NimBLEDevice::startAdvertising();
  }
};

// --- RX: Write callback (cmd + pwm) ---
class RxCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* ch, NimBLEConnInfo&) override {

    std::string value = ch->getValue();
    if (value.size() != 3) {
      return; // oczekujemy: uint8 + uint16
    }

    uint8_t  cmd;
    uint16_t pwm;

    memcpy(&cmd, value.data(), 1);
    memcpy(&pwm, value.data() + 1, 2);

    portENTER_CRITICAL(&g_cmdMux);
    g_cmd = cmd;
    g_pwm = pwm;
    g_newCmd = true;
    portEXIT_CRITICAL(&g_cmdMux);
  }
};

// --- TX: Read callback (snapshot 500x uint32) ---
class TxCallbacks : public NimBLECharacteristicCallbacks {
  void onRead(NimBLECharacteristic* ch, NimBLEConnInfo&) override {

    portENTER_CRITICAL(&g_samplesMux);
    ch->setValue((uint8_t*)g_samples, sizeof(g_samples));
    portEXIT_CRITICAL(&g_samplesMux);
  }
};

static void bleSetup() {
  NimBLEDevice::init(DEV_NAME);

  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  NimBLEService* service = server->createService(SERVICE_UUID);

  // RX – Write
  NimBLECharacteristic* rxChar = service->createCharacteristic(
    CHAR_RX_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
  );
  rxChar->setCallbacks(new RxCallbacks());

  // TX – Read
  NimBLECharacteristic* txChar = service->createCharacteristic(
    CHAR_TX_UUID,
    NIMBLE_PROPERTY::READ
  );
  txChar->setCallbacks(new TxCallbacks());

  service->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setName(DEV_NAME);
  adv->start();
}

#pragma endregion


#pragma region GPIO_PWM

// --- Wejścia ---
static constexpr gpio_num_t PIN_SLOT   = GPIO_NUM_6;

// --- Wyjścia ---
static constexpr gpio_num_t PIN_DIR    = GPIO_NUM_7;
static constexpr gpio_num_t PIN_MOSFET = GPIO_NUM_15;
static constexpr gpio_num_t PIN_LED    = GPIO_NUM_16;

// PWM (LEDC)
static constexpr gpio_num_t PIN_PWM  = GPIO_NUM_18;
static constexpr int PWM_CH          = 0;
static constexpr int PWM_FREQ_HZ     = 18000;
static constexpr int PWM_RES_BITS    = 10;

static void setupPins() {
  pinMode((int)PIN_SLOT, INPUT);

  digitalWrite((int)PIN_DIR, LOW);
  digitalWrite((int)PIN_MOSFET, LOW);
  digitalWrite((int)PIN_LED, LOW);

  pinMode((int)PIN_DIR, OUTPUT);
  pinMode((int)PIN_MOSFET, OUTPUT);
  pinMode((int)PIN_LED, OUTPUT);

  ledcSetup(PWM_CH, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin((int)PIN_PWM, PWM_CH);
  ledcWrite(PWM_CH, 0);
}

#pragma endregion


#pragma region PRZERWANIA_ISR

static void setupInterrupts() {
  attachInterrupt(digitalPinToInterrupt((int)PIN_SLOT), onDetectorISR, FALLING);
}

void IRAM_ATTR onDetectorISR() {
  uint32_t now = (uint32_t)micros();
  uint32_t period = now - g_last_detection_time_us;

  if (period > filter_us) {
    portENTER_CRITICAL_ISR(&g_timerMux);
    g_measured_period_us = period;
    g_last_detection_time_us = now;
    g_new_revolution = true;
    portEXIT_CRITICAL_ISR(&g_timerMux);
  }
}

#pragma endregion


#pragma region LOGIKA_POMOCNICZA

// Elektromagnes
inline void solenoid_on()  { digitalWrite((int)PIN_MOSFET, HIGH); }
inline void solenoid_off() { digitalWrite((int)PIN_MOSFET, LOW);  }

// Silnk DC
void DCmotor_set_PWM(uint16_t value) {
  ledcWrite(PWM_CH, value);
}

inline bool pwm_in_range(uint16_t pwm) {
  // LEDC: rozdzielczość 10 bit -> 0..1023
  return (pwm <= 1023);
}


// Blue LED
inline void LED_on()  { digitalWrite((int)PIN_LED, HIGH); }
inline void LED_off() { digitalWrite((int)PIN_LED, LOW);  }

static void resetSamples() {
  portENTER_CRITICAL(&g_samplesMux);

  // wyzeruj całą tablicę
  for (uint16_t i = 0; i < SAMPLES_COUNT; i++) {
    g_samples[i] = 0;
  }

  // metadane snapshotu
  g_samples_index = 0;
  g_samples_ready = false;

  // po resecie znów możemy zbierać
  g_capture_enabled = true;

  portEXIT_CRITICAL(&g_samplesMux);
}

// Dir obsługa
inline void setDirHigh() {
  digitalWrite((int)PIN_DIR, HIGH);
}

inline void setDirLow() {
  digitalWrite((int)PIN_DIR, LOW);
}

// Hamowanie ramienia: wywołuj raz na obrót (np. gdy g_new_revolution == true)
static inline void deaccelerate(float factor = 0.75f, uint16_t stopThreshold = 100) {
    if (!(factor > 0.0f && factor < 1.0f)) {
        factor = 0.75f;
    }

    if (g_motor_pwm <= stopThreshold) {
        g_motor_pwm = 0;
        DCmotor_set_PWM(0);
        return;
    }

    const float scaled = static_cast<float>(g_motor_pwm) * factor;
    int pwm_i = static_cast<int>(std::lround(scaled));

    const int pwm_max = (1 << PWM_RES_BITS) - 1;
    if (pwm_i < 0) pwm_i = 0;
    if (pwm_i > pwm_max) pwm_i = pwm_max;

    if (pwm_i <= static_cast<int>(stopThreshold)) {
        g_motor_pwm = 0;
        DCmotor_set_PWM(0);
        return;
    }

    g_motor_pwm = static_cast<uint16_t>(pwm_i);
    DCmotor_set_PWM(g_motor_pwm);
}

// Rozpędzanie
// Rozpędzanie: JEDEN KROK na wywołanie (np. raz na obrót)
static inline void accelerate_to(uint16_t target) {
  const uint16_t pwm_max = static_cast<uint16_t>((1u << PWM_RES_BITS) - 1u);
  if (target > pwm_max) target = pwm_max;
  if (target > 850)     target = 850;

  // Jeżeli już jesteśmy na/ponad target — nie rozpędzamy "w dół"
  if (g_motor_pwm >= target) {
    // jeśli chcesz twardo ustawić do targetu (zamiast zostawić wyżej), odkomentuj:
    // g_motor_pwm = target;
    // DCmotor_set_PWM(g_motor_pwm);
    return;
  }

  // Target <= 500: ustaw bezpośrednio
  if (target <= 500) {
    g_motor_pwm = target;
    DCmotor_set_PWM(g_motor_pwm);
    return;
  }

  // 501..800: logika 500 +100 +100 ... +reszta (ale 1 krok na wywołanie)
  if (target <= 800) {
    if (g_motor_pwm < 500) {
      g_motor_pwm = 500;
      DCmotor_set_PWM(g_motor_pwm);
      return;
    }

    // następny "setkowy" próg
    uint16_t next = static_cast<uint16_t>(g_motor_pwm + 100);

    if (next <= target) {
      g_motor_pwm = next;                // krok 100
    } else {
      g_motor_pwm = target;              // końcowa "reszta"
    }
    DCmotor_set_PWM(g_motor_pwm);
    return;
  }

  // 801..850: logika 800 +10 +10 ... +reszta (1 krok na wywołanie)
  // (target jest tu w [801..850])
  if (g_motor_pwm < 800) {
    g_motor_pwm = 800;
    DCmotor_set_PWM(g_motor_pwm);
    return;
  }

  uint16_t next = static_cast<uint16_t>(g_motor_pwm + 10);

  if (next <= target) {
    g_motor_pwm = next;                  // krok 10
  } else {
    g_motor_pwm = target;                // końcowa "reszta"
  }
  DCmotor_set_PWM(g_motor_pwm);
}







#pragma endregion


void setup() {
  Serial.begin(115200);
  Serial.println("Ciao ciao");

  setupPins();
  Serial.println("-- Pins set --");

  setupInterrupts();
  Serial.println("-- Interrupts set --");

  bleSetup();
  Serial.println("-- BLE set --");

  // ważne: inicjalizacja znacznika czasu, żeby pierwszy okres nie był "od zera"
  portENTER_CRITICAL(&g_timerMux);
  g_last_detection_time_us = (uint32_t)micros();
  g_new_revolution = false;
  g_measured_period_us = 0;
  portEXIT_CRITICAL(&g_timerMux);

  resetSamples();
  Serial.println("Samples reset");
}


void loop() {
  // ============================================================
  // (A) Odbiór i obsługa nowej komendy BLE
  // ============================================================
  bool haveCmd = false;
  uint8_t  cmdRaw = 0;
  uint16_t pwmLocal = 0;

  portENTER_CRITICAL(&g_cmdMux);
  if (g_newCmd) {
    cmdRaw = g_cmd;
    pwmLocal = g_pwm;
    g_newCmd = false;
    haveCmd = true;
  }
  portEXIT_CRITICAL(&g_cmdMux);


  if (haveCmd) {
    const Command cmd = static_cast<Command>(cmdRaw);

    switch (cmd) {
      case Command::LedOn:
        LED_on();
        break;

      case Command::LedOff:
        LED_off();
        break;

      case Command::ResetSampleCollecting:
        resetSamples(); // wyzeruje tablicę + metadane + włączy capture
        break;

      case Command::FinishSampleCollecting:
        // Finalize snapshot: blokujemy dalsze próbki i oznaczamy gotowość odczytu
        portENTER_CRITICAL(&g_samplesMux);
        g_capture_enabled = false;
        g_samples_ready = true;
        portEXIT_CRITICAL(&g_samplesMux);
        break;

      case Command::SetMotorPwm:
        if (pwm_in_range(pwmLocal)) {
          DCmotor_set_PWM(pwmLocal);
          g_motor_pwm = pwmLocal;
        }
        // else: wartość poza zakresem -> ignorujemy polecenie
        break;


      case Command::StopMotor:
        DCmotor_set_PWM(0);
        g_motor_pwm = 0;
        break;

      case Command::ElectromagnetOn:
        solenoid_on();
        break;

      case Command::ElectromagnetOff:
        solenoid_off();
        break;

      case Command::DirHigh:
        setDirHigh();
        break;

      case Command::DirLow:
        setDirLow();
        break;
      
      case Command::Deaccelerate:
        g_slowing_down = true;
        g_accelerating = false;
        break;

      case Command::Accelerate_to:
        g_accelerating = true;
        pwm_target = pwmLocal;

        g_slowing_down = false;
        break;

      default:
        // Nieznana komenda -> ignorujemy
        break;

    }
  }

  if (g_slowing_down){

      if (g_accelerating){
        g_accelerating = false;
      }
      bool newRev = false;

      portENTER_CRITICAL(&g_timerMux);
      newRev = g_new_revolution;
      // flaga g_new_revolution i tak jest usywana w następnej części loop() podczas logowania pomiaru.
      portEXIT_CRITICAL(&g_timerMux);

      if (newRev) {
          deaccelerate(0.85f, 150);
      }

      if (g_motor_pwm == 0) {
          g_slowing_down = false;
      }
  }

  if (g_accelerating) {

    // Jeśli stoi (brak obrotów = brak newRev), musimy wykonać pierwszy krok od razu
    if (g_motor_pwm == 0 && pwm_target > 0) {
      // Pierwszy krok:
      // - jeśli target <= 500 -> ustaw target
      // - jeśli target > 500  -> ustaw 500 (start rampy)
      uint16_t first = (pwm_target <= 500) ? pwm_target : 500;
      g_motor_pwm = first;
      DCmotor_set_PWM(g_motor_pwm);

      // Nie kończymy tu od razu całego przyspieszania, bo rampowanie ma iść dalej na obrotach
    }

    bool newRev = false;
    portENTER_CRITICAL(&g_timerMux);
    newRev = g_new_revolution;  // nie kasujemy tutaj flagi
    portEXIT_CRITICAL(&g_timerMux);

    // Dalsze kroki rampy wykonuj raz na obrót
    if (newRev) {
      accelerate_to(pwm_target);
    }

    if (g_motor_pwm >= pwm_target) {
      // w naszej funkcji nie schodzimy w dół, więc >= oznacza "osiągnięte"
      g_accelerating = false;
    }
  }


  // ============================================================
  // (B) Odbiór nowego pomiaru z ISR i dopisanie do tablicy próbek
  // ============================================================
  bool havePeriod = false;
  uint32_t periodLocal = 0;

  portENTER_CRITICAL(&g_timerMux);
  if (g_new_revolution) {
    periodLocal = g_measured_period_us;
    g_new_revolution = false;
    havePeriod = true;
  }
  portEXIT_CRITICAL(&g_timerMux);

  if (havePeriod) {
    portENTER_CRITICAL(&g_samplesMux);

    if (g_capture_enabled && !g_samples_ready) {
      if (g_samples_index < SAMPLES_COUNT) {
        g_samples[g_samples_index] = periodLocal;
        g_samples_index++;

        // Jeżeli uzbieraliśmy pełny snapshot 500 próbek -> automatycznie finalizuj
        if (g_samples_index >= SAMPLES_COUNT) {
          g_samples_ready = true;
          g_capture_enabled = false;
        }
      } else {
        // redundancja bezpieczeństwa
        g_samples_ready = true;
        g_capture_enabled = false;
      }
    }

    portEXIT_CRITICAL(&g_samplesMux);
  }

  // Brak delay — świadomie. Jeśli okaże się, że CPU jest zbyt obciążone,
  // dodamy minimalne odciążenie (np. delay(1)).
}



