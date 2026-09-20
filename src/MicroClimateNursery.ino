#define BLYNK_TEMPLATE_ID "TEMPLATE_ID"
#define BLYNK_TEMPLATE_NAME "Micro Climate Nursery"
#define BLYNK_AUTH_TOKEN "AUTH_TOKEN"
#define BLYNK_PRINT Serial


#include <Arduino.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <ESP32Servo.h>
#include <ThingSpeak.h>

// ======================================================
// WIFI CREDENTIALS
// ======================================================

char ssid[] = "WIFI_NAME";
char pass[] = "WIFI_PASSWORD";

// ======================================================
// THINGSPEAK CONFIGURATION
// ======================================================

unsigned long thingSpeakChannelID = ID; //THINGSPEAK CHANNEL ID
const char *thingSpeakWriteAPIKey = "THINGSPEAK_WRITE_API_KEY";
WiFiClient thingSpeakClient;

// ======================================================
// PIN DEFINITIONS
// ======================================================

#define DHT_PIN 4
#define DHT_TYPE DHT22

#define LDR_PIN 34

#define OLED_SDA 21
#define OLED_SCL 22
#define OLED_RESET 19
#define OLED_ADDRESS 0x3C

#define SERVO_PIN 18

#define GROW_LED_1 25
#define GROW_LED_2 26
#define GROW_LED_3 27

#define WARNING_LED 13
#define FAULT_LED 14
#define MANUAL_LED 16

#define BUZZER_PIN 17

#define MANUAL_BUTTON 32
#define RESET_BUTTON 33

// ======================================================
// OBJECTS
// ======================================================

DHT dht(DHT_PIN, DHT_TYPE);

Adafruit_SSD1306 display(
  128,
  64,
  &Wire,
  OLED_RESET
);

Servo ventServo;

// ======================================================
// OPERATING MODES
// ======================================================

enum SystemMode
{
  AUTO_MODE,
  MANUAL_MODE,
  SAFETY_MODE
};

SystemMode currentMode = AUTO_MODE;

// ======================================================
// SENSOR VALUES
// ======================================================

float temperature = 0.0;
float humidity = 0.0;

int lightRaw = 0;

bool dhtFault = false;
bool ldrFault = false;

// ======================================================
// OUTPUT VALUES
// ======================================================

int growLevel = 0;
int ventAngle = 0;

// ======================================================
// LDR SETTINGS
// TEMPORARY - recalibrate later
// ======================================================

const int LIGHT_BRIGHT = 1600;
const int LIGHT_MEDIUM = 2100;
const int LIGHT_DARK   = 2900;

const int LIGHT_HYSTERESIS = 100;

const int LDR_FAULT_LOW = 30;
const int LDR_FAULT_HIGH = 4065;

int ldrExtremeCounter = 0;

const int LDR_FAULT_CONFIRM_COUNT = 3;

// ======================================================
// TEMPERATURE SETTINGS
// ======================================================

const float WARNING_TEMP = 30.0;

const float VENT_START_TEMP = 25.5;
const float VENT_FULL_TEMP = 31.0;

const float MIN_VALID_TEMP = -10.0;
const float MAX_VALID_TEMP = 60.0;

const float MIN_VALID_HUMIDITY = 0.0;
const float MAX_VALID_HUMIDITY = 100.0;

// ======================================================
// SAFETY RECOVERY
// ======================================================

int validRecoveryReads = 0;

const int REQUIRED_RECOVERY_READS = 3;

// ======================================================
// BUTTON DEBOUNCE
// ======================================================

bool previousManualButton = LOW;
bool previousResetButton = LOW;

unsigned long lastManualPress = 0;
unsigned long lastResetPress = 0;

const unsigned long DEBOUNCE_MS = 200;

// ======================================================
// HARDWARE TIMER
// ======================================================

hw_timer_t *systemTimer = nullptr;

volatile uint32_t pendingTicks = 0;

portMUX_TYPE timerMux =
  portMUX_INITIALIZER_UNLOCKED;

// Timer tick = 100 ms

uint16_t sensorTicks = 0;
uint16_t displayTicks = 0;
uint16_t blynkTicks = 0;
uint16_t buzzerTicks = 0;
uint16_t connectionTicks = 0;
uint16_t thingSpeakTicks = 0;

const uint16_t SENSOR_PERIOD = 20;      // 2 sec
const uint16_t DISPLAY_PERIOD = 5;      // 0.5 sec
const uint16_t BLYNK_PERIOD = 20;       // 2 sec
const uint16_t BUZZER_PERIOD = 5;       // 0.5 sec
const uint16_t CONNECTION_PERIOD = 50;  // 5 sec
const uint16_t THINGSPEAK_PERIOD = 200;   // 20 sec

bool sensorTaskDue = false;
bool displayTaskDue = false;
bool blynkTaskDue = false;
bool buzzerTaskDue = false;
bool connectionTaskDue = false;
bool thingSpeakTaskDue = false;

// ======================================================
// FUNCTION PROTOTYPES
// ======================================================

void initialisePins();
void initialiseOLED();
void initialiseTimer();
void initialiseNetwork();

void processTimerTicks();

void readSensors();
void checkLDRFault();
bool sensorsValid();

void updateSystem();

void runAutoMode();
void runManualMode();
void runSafetyMode();

void updateGrowLights();
void setGrowLights(int level);

int calculateVentAngle(float temp);
void setVentAngle(int angle);

void readButtons();
void updateOLED();
void updateBuzzer();

void maintainNetwork();

void sendToBlynk();
void syncBlynkManualSwitch();
void sendToThingSpeak();
int getModeNumber();

void processUART();
void printStatus();

const char *getModeName();

// ======================================================
// HARDWARE TIMER ISR
// ======================================================

/**
 * @brief Hardware timer ISR.
 *
 * Only increments a counter. No Serial, OLED, sensor,
 * servo or network functions are executed here.
 *
 * @return Nothing.
 */
void ARDUINO_ISR_ATTR onSystemTimer()
{
  portENTER_CRITICAL_ISR(&timerMux);

  pendingTicks++;

  portEXIT_CRITICAL_ISR(&timerMux);
}

// ======================================================
// SETUP
// ======================================================

void setup()
{
  Serial.begin(115200);

  initialisePins();
  initialiseOLED();

  dht.begin();

  // Servo
  ventServo.setPeriodHertz(50);
  ventServo.attach(SERVO_PIN, 500, 2400);

  setVentAngle(0);
  setGrowLights(0);

  digitalWrite(WARNING_LED, LOW);
  digitalWrite(FAULT_LED, LOW);
  digitalWrite(MANUAL_LED, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  // Startup screen
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 5);
  display.println("Micro-Climate");

  display.setCursor(0, 22);
  display.println("Nursery System");

  display.setCursor(0, 40);
  display.println("Starting...");

  display.display();

  initialiseNetwork();
  ThingSpeak.begin(thingSpeakClient);
  initialiseTimer();

  Serial.println();
  Serial.println("==========================");
  Serial.println("MICRO-CLIMATE NURSERY");
  Serial.println("==========================");
  Serial.println("UART commands:");
  Serial.println("STATUS");
  Serial.println("AUTO");
  Serial.println("MANUAL");
  Serial.println("RESET");
  Serial.println("==========================");

  sensorTaskDue = true;
  displayTaskDue = true;
  connectionTaskDue = true;
}

// ======================================================
// LOOP
// ======================================================

void loop()
{
  readButtons();

  processUART();

  processTimerTicks();

  if (Blynk.connected())
  {
    Blynk.run();
  }

  if (connectionTaskDue)
  {
    connectionTaskDue = false;

    maintainNetwork();
  }

  if (sensorTaskDue)
  {
    sensorTaskDue = false;

    readSensors();
    checkLDRFault();
    updateSystem();
  }

  if (displayTaskDue)
  {
    displayTaskDue = false;

    updateOLED();
  }

  if (buzzerTaskDue)
  {
    buzzerTaskDue = false;

    updateBuzzer();
  }

  if (blynkTaskDue)
  {
    blynkTaskDue = false;

    sendToBlynk();
  }

  if (thingSpeakTaskDue)
  {
    thingSpeakTaskDue = false;

    sendToThingSpeak();
  }
}

// ======================================================
// PIN INITIALISATION
// ======================================================

void initialisePins()
{
  pinMode(LDR_PIN, INPUT);

  pinMode(GROW_LED_1, OUTPUT);
  pinMode(GROW_LED_2, OUTPUT);
  pinMode(GROW_LED_3, OUTPUT);

  pinMode(WARNING_LED, OUTPUT);
  pinMode(FAULT_LED, OUTPUT);
  pinMode(MANUAL_LED, OUTPUT);

  pinMode(BUZZER_PIN, OUTPUT);

  // External 10k pull-down resistors
  pinMode(MANUAL_BUTTON, INPUT);
  pinMode(RESET_BUTTON, INPUT);
}

// ======================================================
// OLED
// ======================================================

void initialiseOLED()
{
  Wire.begin(
    OLED_SDA,
    OLED_SCL
  );

  if (!display.begin(
        SSD1306_SWITCHCAPVCC,
        OLED_ADDRESS))
  {
    Serial.println(
      "OLED INITIALISATION FAILED"
    );

    while (true)
    {
    }
  }

  Serial.println(
    "OLED INITIALISED"
  );
}

// ======================================================
// HARDWARE TIMER
// ======================================================

void initialiseTimer()
{
  // 1 MHz = 1 count per microsecond
  systemTimer =
    timerBegin(1000000);

  if (systemTimer == nullptr)
  {
    Serial.println(
      "TIMER INITIALISATION FAILED"
    );

    return;
  }

  timerAttachInterrupt(
    systemTimer,
    &onSystemTimer
  );

  // 100000 microseconds = 100 ms
  timerAlarm(
    systemTimer,
    100000,
    true,
    0
  );

  Serial.println(
    "HARDWARE TIMER ACTIVE"
  );
}

// ======================================================
// NETWORK INITIALISATION
// ======================================================

void initialiseNetwork()
{
  WiFi.mode(WIFI_STA);

  WiFi.begin(
    ssid,
    pass
  );

  Blynk.config(
    BLYNK_AUTH_TOKEN
  );

  Serial.print(
    "Connecting to WiFi: "
  );

  Serial.println(ssid);
}

// ======================================================
// TIMER SCHEDULER
// ======================================================

void processTimerTicks()
{
  uint32_t ticks = 0;

  portENTER_CRITICAL(&timerMux);

  ticks = pendingTicks;
  pendingTicks = 0;

  portEXIT_CRITICAL(&timerMux);

  if (ticks == 0)
  {
    return;
  }

  sensorTicks += ticks;
  displayTicks += ticks;
  blynkTicks += ticks;
  buzzerTicks += ticks;
  connectionTicks += ticks;
  thingSpeakTicks += ticks;

  if (sensorTicks >= SENSOR_PERIOD)
  {
    sensorTicks %= SENSOR_PERIOD;
    sensorTaskDue = true;
  }

  if (displayTicks >= DISPLAY_PERIOD)
  {
    displayTicks %= DISPLAY_PERIOD;
    displayTaskDue = true;
  }

  if (blynkTicks >= BLYNK_PERIOD)
  {
    blynkTicks %= BLYNK_PERIOD;
    blynkTaskDue = true;
  }

  if (buzzerTicks >= BUZZER_PERIOD)
  {
    buzzerTicks %= BUZZER_PERIOD;
    buzzerTaskDue = true;
  }

  if (connectionTicks >= CONNECTION_PERIOD)
  {
    connectionTicks %= CONNECTION_PERIOD;
    connectionTaskDue = true;
  }

  if (thingSpeakTicks >= THINGSPEAK_PERIOD)
  {
    thingSpeakTicks %= THINGSPEAK_PERIOD;
    thingSpeakTaskDue = true;
  }
}

// ======================================================
// SENSOR ACQUISITION
// ======================================================

void readSensors()
{
  float newTemperature =
    dht.readTemperature();

  float newHumidity =
    dht.readHumidity();

  // DHT validation
  if (
    isnan(newTemperature) ||
    isnan(newHumidity) ||
    newTemperature < MIN_VALID_TEMP ||
    newTemperature > MAX_VALID_TEMP ||
    newHumidity < MIN_VALID_HUMIDITY ||
    newHumidity > MAX_VALID_HUMIDITY
  )
  {
    dhtFault = true;
  }
  else
  {
    dhtFault = false;

    temperature =
      newTemperature;

    humidity =
      newHumidity;
  }

  // LDR averaging
  const int SAMPLE_COUNT = 20;

  long total = 0;

  for (int i = 0;
       i < SAMPLE_COUNT;
       i++)
  {
    total +=
      analogRead(LDR_PIN);
  }

  lightRaw =
    total / SAMPLE_COUNT;

  Serial.println();
  Serial.println(
    "----- SENSOR DATA -----"
  );

  if (dhtFault)
  {
    Serial.println(
      "DHT22: FAULT"
    );
  }
  else
  {
    Serial.print(
      "Temperature: "
    );

    Serial.print(
      temperature,
      1
    );

    Serial.println(" C");

    Serial.print(
      "Humidity: "
    );

    Serial.print(
      humidity,
      1
    );

    Serial.println(" %");
  }

  Serial.print(
    "Light ADC: "
  );

  Serial.println(
    lightRaw
  );
}

// ======================================================
// LDR FAULT DETECTION
// ======================================================

void checkLDRFault()
{
  bool extreme =
    lightRaw <= LDR_FAULT_LOW ||
    lightRaw >= LDR_FAULT_HIGH;

  if (extreme)
  {
    ldrExtremeCounter++;

    if (
      ldrExtremeCounter >=
      LDR_FAULT_CONFIRM_COUNT
    )
    {
      ldrFault = true;
    }
  }
  else
  {
    ldrExtremeCounter = 0;
    ldrFault = false;
  }
}

// ======================================================
// SENSOR STATUS
// ======================================================

bool sensorsValid()
{
  return
    !dhtFault &&
    !ldrFault;
}

// ======================================================
// PRIORITY LOGIC
// ======================================================

void updateSystem()
{
  // SAFETY has highest priority
  if (!sensorsValid())
  {
    if (
      currentMode !=
      SAFETY_MODE
    )
    {
      Serial.println(
        "*** ENTERING SAFETY MODE ***"
      );
    }

    currentMode =
      SAFETY_MODE;

    validRecoveryReads = 0;

    runSafetyMode();

    syncBlynkManualSwitch();

    return;
  }

  // Safety remains latched
  if (
    currentMode ==
    SAFETY_MODE
  )
  {
    validRecoveryReads++;

    Serial.print(
      "Recovery reading: "
    );

    Serial.print(
      validRecoveryReads
    );

    Serial.print("/");

    Serial.println(
      REQUIRED_RECOVERY_READS
    );

    runSafetyMode();

    return;
  }

  if (
    currentMode ==
    MANUAL_MODE
  )
  {
    runManualMode();
  }
  else
  {
    runAutoMode();
  }
}

// ======================================================
// AUTO
// ======================================================

void runAutoMode()
{
  digitalWrite(
    FAULT_LED,
    LOW
  );

  digitalWrite(
    MANUAL_LED,
    LOW
  );

  updateGrowLights();

  setVentAngle(
    calculateVentAngle(
      temperature
    )
  );

  digitalWrite(
    WARNING_LED,
    temperature >= WARNING_TEMP
      ? HIGH
      : LOW
  );
}

// ======================================================
// MANUAL
// ======================================================

void runManualMode()
{
  digitalWrite(
    MANUAL_LED,
    HIGH
  );

  digitalWrite(
    WARNING_LED,
    LOW
  );

  digitalWrite(
    FAULT_LED,
    LOW
  );

  setGrowLights(0);

  setVentAngle(90);
}

// ======================================================
// SAFETY
// ======================================================

void runSafetyMode()
{
  digitalWrite(
    FAULT_LED,
    HIGH
  );

  digitalWrite(
    WARNING_LED,
    LOW
  );

  digitalWrite(
    MANUAL_LED,
    LOW
  );

  setGrowLights(0);

  setVentAngle(45);
}

// ======================================================
// GROW LIGHT CONTROL
// ======================================================

void updateGrowLights()
{
  switch (growLevel)
  {
    case 0:
      // Move from level 0 -> 1 only when clearly darker
      if (lightRaw > LIGHT_BRIGHT + LIGHT_HYSTERESIS)
      {
        setGrowLights(1);
      }
      break;

    case 1:
      // Move brighter -> level 0
      if (lightRaw < LIGHT_BRIGHT - LIGHT_HYSTERESIS)
      {
        setGrowLights(0);
      }

      // Move darker -> level 2
      else if (lightRaw > LIGHT_MEDIUM + LIGHT_HYSTERESIS)
      {
        setGrowLights(2);
      }
      break;

    case 2:
      // Move brighter -> level 1
      if (lightRaw < LIGHT_MEDIUM - LIGHT_HYSTERESIS)
      {
        setGrowLights(1);
      }

      // Move darker -> level 3
      else if (lightRaw > LIGHT_DARK + LIGHT_HYSTERESIS)
      {
        setGrowLights(3);
      }
      break;

    case 3:
      // Move from level 3 -> 2 only when clearly brighter
      if (lightRaw < LIGHT_DARK - LIGHT_HYSTERESIS)
      {
        setGrowLights(2);
      }
      break;

    default:
      setGrowLights(0);
      break;
  }
}

// ======================================================
// SET GROW LIGHTS
// ======================================================

void setGrowLights(int level)
{
  growLevel =
    constrain(
      level,
      0,
      3
    );

  digitalWrite(
    GROW_LED_1,
    growLevel >= 1
      ? HIGH
      : LOW
  );

  digitalWrite(
    GROW_LED_2,
    growLevel >= 2
      ? HIGH
      : LOW
  );

  digitalWrite(
    GROW_LED_3,
    growLevel >= 3
      ? HIGH
      : LOW
  );
}

// ======================================================
// SERVO CALCULATION
// ======================================================

int calculateVentAngle(float temp)
{
  if (
    temp <=
    VENT_START_TEMP
  )
  {
    return 0;
  }

  if (
    temp >=
    VENT_FULL_TEMP
  )
  {
    return 90;
  }

  float ratio =
    (
      temp -
      VENT_START_TEMP
    ) /
    (
      VENT_FULL_TEMP -
      VENT_START_TEMP
    );

  int calculatedAngle =
    round(
      ratio * 90.0
    );

  return constrain(
    calculatedAngle,
    0,
    90
  );
}

// ======================================================
// SERVO OUTPUT
// ======================================================

void setVentAngle(int angle)
{
  ventAngle =
    constrain(
      angle,
      0,
      90
    );

  ventServo.write(
    ventAngle
  );
}

// ======================================================
// PHYSICAL BUTTONS
// ======================================================

void readButtons()
{
  unsigned long now =
    millis();

  bool manualState =
    digitalRead(
      MANUAL_BUTTON
    );

  bool resetState =
    digitalRead(
      RESET_BUTTON
    );

  // Manual button
  if (
    manualState == HIGH &&
    previousManualButton == LOW &&
    now - lastManualPress >=
      DEBOUNCE_MS
  )
  {
    lastManualPress = now;

    if (
      currentMode ==
      SAFETY_MODE
    )
    {
      Serial.println(
        "Manual rejected: SAFETY active"
      );
    }
    else if (
      currentMode ==
      AUTO_MODE
    )
    {
      currentMode =
        MANUAL_MODE;

      runManualMode();

      Serial.println(
        "MANUAL enabled"
      );
    }
    else
    {
      currentMode =
        AUTO_MODE;

      runAutoMode();

      Serial.println(
        "AUTO enabled"
      );
    }

    syncBlynkManualSwitch();
  }

  // Reset button
  if (
    resetState == HIGH &&
    previousResetButton == LOW &&
    now - lastResetPress >=
      DEBOUNCE_MS
  )
  {
    lastResetPress = now;

    if (
      currentMode ==
        SAFETY_MODE &&
      sensorsValid() &&
      validRecoveryReads >=
        REQUIRED_RECOVERY_READS
    )
    {
      currentMode =
        AUTO_MODE;

      validRecoveryReads = 0;

      digitalWrite(
        FAULT_LED,
        LOW
      );

      digitalWrite(
        BUZZER_PIN,
        LOW
      );

      runAutoMode();

      Serial.println(
        "SAFETY cleared"
      );

      syncBlynkManualSwitch();
    }
    else if (
      currentMode ==
      SAFETY_MODE
    )
    {
      Serial.println(
        "Reset denied - sensors not stable"
      );
    }
  }

  previousManualButton =
    manualState;

  previousResetButton =
    resetState;
}

// ======================================================
// BUZZER
// ======================================================

void updateBuzzer()
{
  if (
    currentMode ==
    SAFETY_MODE
  )
  {
    digitalWrite(
      BUZZER_PIN,
      !digitalRead(
        BUZZER_PIN
      )
    );
  }
  else
  {
    digitalWrite(
      BUZZER_PIN,
      LOW
    );
  }
}

// ======================================================
// OLED
// ======================================================

void updateOLED()
{
  display.clearDisplay();

  display.setTextSize(1);

  display.setTextColor(
    SSD1306_WHITE
  );

  // SAFETY SCREEN
  if (
    currentMode ==
    SAFETY_MODE
  )
  {
    display.setCursor(0, 0);
    display.println(
      "SAFETY MODE"
    );

    display.setCursor(0, 14);

    if (
      dhtFault &&
      ldrFault
    )
    {
      display.println(
        "DHT + LDR FAILED"
      );
    }
    else if (dhtFault)
    {
      display.println(
        "DHT22 FAILED"
      );
    }
    else if (ldrFault)
    {
      display.println(
        "LDR FAILED"
      );
    }
    else
    {
      display.println(
        "Sensors recovered"
      );
    }

    display.setCursor(0, 28);
    display.println(
      "Vent: SAFE 45deg"
    );

    display.setCursor(0, 42);
    display.println(
      "Grow LEDs: OFF"
    );

    display.setCursor(0, 54);
    display.println(
      "Check + RESET"
    );

    display.display();

    return;
  }

  // MANUAL SCREEN
  if (
    currentMode ==
    MANUAL_MODE
  )
  {
    display.setCursor(0, 0);

    display.println(
      "MANUAL OVERRIDE"
    );

    display.setCursor(0, 14);

    display.print("T:");
    display.print(
      temperature,
      1
    );

    display.print("C H:");
    display.print(
      humidity,
      0
    );

    display.println("%");

    display.setCursor(0, 29);

    display.print(
      "Light:"
    );

    display.println(
      lightRaw
    );

    display.setCursor(0, 43);

    display.println(
      "Vent: OPEN 90deg"
    );

    display.setCursor(0, 54);

    display.println(
      "Automation OFF"
    );

    display.display();

    return;
  }

  // AUTO SCREEN
  display.setCursor(0, 0);

  if (
    temperature >=
    WARNING_TEMP
  )
  {
    display.println(
      "AUTO - WARNING"
    );
  }
  else
  {
    display.println(
      "AUTO - NORMAL"
    );
  }

  display.setCursor(0, 14);

  display.print("T:");
  display.print(
    temperature,
    1
  );

  display.print("C H:");
  display.print(
    humidity,
    0
  );

  display.println("%");

  display.setCursor(0, 28);

  display.print(
    "Light:"
  );

  display.println(
    lightRaw
  );

  display.setCursor(0, 41);

  display.print(
    "Grow:"
  );

  display.print(
    growLevel
  );

  display.println("/3");

  display.setCursor(0, 54);

  display.print(
    "Vent:"
  );

  display.print(
    ventAngle
  );

  display.println("deg");

  display.display();
}

// ======================================================
// NETWORK MAINTENANCE
// ======================================================

void maintainNetwork()
{
  static unsigned long
    lastWiFiAttempt = 0;

  const unsigned long
    WIFI_RETRY_INTERVAL =
      15000;

  if (
    WiFi.status() !=
    WL_CONNECTED
  )
  {
    if (
      millis() -
      lastWiFiAttempt >=
      WIFI_RETRY_INTERVAL
    )
    {
      lastWiFiAttempt =
        millis();

      Serial.println(
        "WiFi offline - retrying"
      );

      WiFi.disconnect();

      WiFi.begin(
        ssid,
        pass
      );
    }

    return;
  }

  if (!Blynk.connected())
  {
    Serial.println(
      "Connecting to Blynk..."
    );

    Blynk.connect(500);
  }
}

// ======================================================
// SEND REAL VALUES TO BLYNK
// ======================================================

void sendToBlynk()
{
  if (!Blynk.connected())
  {
    return;
  }

  Blynk.virtualWrite(
    V0,
    temperature
  );

  Blynk.virtualWrite(
    V1,
    humidity
  );

  Blynk.virtualWrite(
    V2,
    lightRaw
  );

  Blynk.virtualWrite(
    V3,
    ventAngle
  );

  Blynk.virtualWrite(
    V4,
    growLevel
  );

  Blynk.virtualWrite(
    V5,
    getModeName()
  );
}

// ======================================================
// SYNC BLYNK SWITCH
// ======================================================

void syncBlynkManualSwitch()
{
  if (!Blynk.connected())
  {
    return;
  }

  Blynk.virtualWrite(
    V6,
    currentMode ==
      MANUAL_MODE
      ? 1
      : 0
  );

  Blynk.virtualWrite(
    V5,
    getModeName()
  );
}

// ======================================================
// BLYNK MANUAL OVERRIDE
// ======================================================

BLYNK_WRITE(V6)
{
  int manualRequest =
    param.asInt();

  if (
    currentMode ==
    SAFETY_MODE
  )
  {
    Serial.println(
      "Blynk Manual rejected: SAFETY active"
    );

    Blynk.virtualWrite(
      V6,
      0
    );

    return;
  }

  if (
    manualRequest == 1
  )
  {
    currentMode =
      MANUAL_MODE;

    runManualMode();

    Serial.println(
      "MANUAL selected from Blynk"
    );
  }
  else
  {
    currentMode =
      AUTO_MODE;

    runAutoMode();

    Serial.println(
      "AUTO selected from Blynk"
    );
  }

  Blynk.virtualWrite(
    V5,
    getModeName()
  );
}

// ======================================================
// BLYNK CONNECTED CALLBACK
// ======================================================

BLYNK_CONNECTED()
{
  Serial.println(
    "BLYNK CONNECTED"
  );

  sendToBlynk();

  syncBlynkManualSwitch();
}

// ======================================================
// THINGSPEAK HISTORICAL LOGGING
// ======================================================

/**
 * @brief Sends current nursery measurements and operating state to ThingSpeak.
 *
 * Field 1 = Temperature (C)
 * Field 2 = Humidity (%)
 * Field 3 = Light ADC
 * Field 4 = Vent angle (degrees)
 * Field 5 = Grow-light level (0-3)
 * Field 6 = System mode (0=AUTO, 1=MANUAL, 2=SAFETY)
 *
 * @return Nothing.
 */
void sendToThingSpeak()
{
  // Cloud logging is optional. Local control continues if Wi-Fi is unavailable.
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("ThingSpeak skipped: WiFi offline");
    return;
  }

  ThingSpeak.setField(1, temperature);
  ThingSpeak.setField(2, humidity);
  ThingSpeak.setField(3, lightRaw);
  ThingSpeak.setField(4, ventAngle);
  ThingSpeak.setField(5, growLevel);
  ThingSpeak.setField(6, getModeNumber());

  int responseCode = ThingSpeak.writeFields(
    thingSpeakChannelID,
    thingSpeakWriteAPIKey
  );

  if (responseCode == 200)
  {
    Serial.println("ThingSpeak update successful");
  }
  else
  {
    Serial.print("ThingSpeak update failed. HTTP code: ");
    Serial.println(responseCode);
  }
}

/**
 * @brief Converts the current operating mode to a numeric value for ThingSpeak.
 *
 * @return 0 for AUTO, 1 for MANUAL, 2 for SAFETY, -1 if unknown.
 */
int getModeNumber()
{
  switch (currentMode)
  {
    case AUTO_MODE:
      return 0;

    case MANUAL_MODE:
      return 1;

    case SAFETY_MODE:
      return 2;

    default:
      return -1;
  }
}

// ======================================================
// UART
// ======================================================

void processUART()
{
  if (!Serial.available())
  {
    return;
  }

  String command =
    Serial.readStringUntil(
      '\n'
    );

  command.trim();
  command.toUpperCase();

  if (
    command ==
    "STATUS"
  )
  {
    printStatus();
  }

  else if (
    command ==
    "AUTO"
  )
  {
    if (
      currentMode !=
      SAFETY_MODE
    )
    {
      currentMode =
        AUTO_MODE;

      runAutoMode();

      syncBlynkManualSwitch();

      Serial.println(
        "AUTO selected via UART"
      );
    }
  }

  else if (
    command ==
    "MANUAL"
  )
  {
    if (
      currentMode !=
      SAFETY_MODE
    )
    {
      currentMode =
        MANUAL_MODE;

      runManualMode();

      syncBlynkManualSwitch();

      Serial.println(
        "MANUAL selected via UART"
      );
    }
  }

  else if (
    command ==
    "RESET"
  )
  {
    if (
      currentMode ==
        SAFETY_MODE &&
      sensorsValid() &&
      validRecoveryReads >=
        REQUIRED_RECOVERY_READS
    )
    {
      currentMode =
        AUTO_MODE;

      validRecoveryReads = 0;

      digitalWrite(
        FAULT_LED,
        LOW
      );

      digitalWrite(
        BUZZER_PIN,
        LOW
      );

      runAutoMode();

      syncBlynkManualSwitch();

      Serial.println(
        "SAFETY reset via UART"
      );
    }
  }
}

// ======================================================
// STATUS
// ======================================================

void printStatus()
{
  Serial.println();
  Serial.println(
    "======================"
  );

  Serial.print(
    "Mode: "
  );

  Serial.println(
    getModeName()
  );

  Serial.print(
    "Temperature: "
  );

  Serial.println(
    temperature
  );

  Serial.print(
    "Humidity: "
  );

  Serial.println(
    humidity
  );

  Serial.print(
    "Light ADC: "
  );

  Serial.println(
    lightRaw
  );

  Serial.print(
    "Grow Level: "
  );

  Serial.println(
    growLevel
  );

  Serial.print(
    "Vent Angle: "
  );

  Serial.println(
    ventAngle
  );

  Serial.print(
    "WiFi: "
  );

  Serial.println(
    WiFi.status() ==
      WL_CONNECTED
      ? "CONNECTED"
      : "OFFLINE"
  );

  Serial.print(
    "Blynk: "
  );

  Serial.println(
    Blynk.connected()
      ? "CONNECTED"
      : "OFFLINE"
  );

  Serial.println(
    "======================"
  );
}

// ======================================================
// MODE NAME
// ======================================================

const char *getModeName()
{
  switch (currentMode)
  {
    case AUTO_MODE:
      return "AUTO";

    case MANUAL_MODE:
      return "MANUAL";

    case SAFETY_MODE:
      return "SAFETY";

    default:
      return "UNKNOWN";
  }
}
