#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <ESP32Servo.h>

// ======================================================
// PIN DEFINITIONS
// ======================================================

#define DHT_PIN 4
#define DHT_TYPE DHT22

#define LDR_PIN 34

#define OLED_SDA 21
#define OLED_SCL 22
#define OLED_RESET -1
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
// ======================================================

const int LIGHT_BRIGHT = 1600;
const int LIGHT_MEDIUM = 2100;
const int LIGHT_DARK = 2900;

const int LIGHT_HYSTERESIS = 100;

const int LDR_FAULT_LOW = 20;
const int LDR_FAULT_HIGH = 4075;

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

uint16_t sensorTicks = 0;
uint16_t displayTicks = 0;
uint16_t buzzerTicks = 0;

const uint16_t SENSOR_PERIOD = 20;
const uint16_t DISPLAY_PERIOD = 5;
const uint16_t BUZZER_PERIOD = 5;

bool sensorTaskDue = false;
bool displayTaskDue = false;
bool buzzerTaskDue = false;

// ======================================================
// FUNCTION PROTOTYPES
// ======================================================

void initialisePins();
void initialiseOLED();
void initialiseTimer();

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

void processUART();
void printStatus();

const char *getModeName();

// ======================================================
// HARDWARE TIMER ISR
// ======================================================

/**
 * @brief Hardware timer ISR.
 *
 * Only increments a counter.
 *
 * @return Nothing.
 */
void IRAM_ATTR onSystemTimer()
{
  portENTER_CRITICAL_ISR(&timerMux);

  pendingTicks++;

  portEXIT_CRITICAL_ISR(&timerMux);
}

// ======================================================
// SETUP
// ======================================================

/**
 * @brief Initialises the complete nursery simulation.
 *
 * @return Nothing.
 */
void setup()
{
  Serial.begin(115200);

  initialisePins();
  initialiseOLED();

  dht.begin();

  ventServo.setPeriodHertz(50);

  ventServo.attach(
    SERVO_PIN,
    500,
    2400
  );

  setVentAngle(0);
  setGrowLights(0);

  digitalWrite(
    WARNING_LED,
    LOW
  );

  digitalWrite(
    FAULT_LED,
    LOW
  );

  digitalWrite(
    MANUAL_LED,
    LOW
  );

  digitalWrite(
    BUZZER_PIN,
    LOW
  );

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(
    SSD1306_WHITE
  );

  display.setCursor(0, 5);
  display.println(
    "Micro-Climate"
  );

  display.setCursor(0, 22);
  display.println(
    "Nursery System"
  );

  display.setCursor(0, 40);
  display.println(
    "Starting..."
  );

  display.display();

  initialiseTimer();

  Serial.println();
  Serial.println(
    "=========================="
  );

  Serial.println(
    "MICRO-CLIMATE NURSERY"
  );

  Serial.println(
    "=========================="
  );

  Serial.println(
    "UART commands:"
  );

  Serial.println("STATUS");
  Serial.println("AUTO");
  Serial.println("MANUAL");
  Serial.println("RESET");

  Serial.println(
    "=========================="
  );

  sensorTaskDue = true;
  displayTaskDue = true;
}

// ======================================================
// LOOP
// ======================================================

/**
 * @brief Executes all scheduled system tasks.
 *
 * @return Nothing.
 */
void loop()
{
  readButtons();

  processUART();

  processTimerTicks();

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
}

// ======================================================
// PIN INITIALISATION
// ======================================================

/**
 * @brief Configures all GPIO pins.
 *
 * @return Nothing.
 */
void initialisePins()
{
  pinMode(
    LDR_PIN,
    INPUT
  );

  pinMode(
    GROW_LED_1,
    OUTPUT
  );

  pinMode(
    GROW_LED_2,
    OUTPUT
  );

  pinMode(
    GROW_LED_3,
    OUTPUT
  );

  pinMode(
    WARNING_LED,
    OUTPUT
  );

  pinMode(
    FAULT_LED,
    OUTPUT
  );

  pinMode(
    MANUAL_LED,
    OUTPUT
  );

  pinMode(
    BUZZER_PIN,
    OUTPUT
  );

  pinMode(
    MANUAL_BUTTON,
    INPUT
  );

  pinMode(
    RESET_BUTTON,
    INPUT
  );
}

// ======================================================
// OLED
// ======================================================

/**
 * @brief Initialises the OLED display.
 *
 * @return Nothing.
 */
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
      "OLED FAILED"
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

/**
 * @brief Configures hardware timer 0 for a 100 ms interrupt.
 *
 * Prescaler 80 converts the 80 MHz ESP32 timer clock
 * to 1 MHz.
 *
 * @return Nothing.
 */
void initialiseTimer()
{
  systemTimer = timerBegin(
    0,
    80,
    true
  );

  if (systemTimer == nullptr)
  {
    Serial.println(
      "Timer failed"
    );

    return;
  }

  timerAttachInterrupt(
    systemTimer,
    &onSystemTimer,
    true
  );

  timerAlarmWrite(
    systemTimer,
    100000,
    true
  );

  timerAlarmEnable(
    systemTimer
  );

  Serial.println(
    "Hardware timer active"
  );
}

// ======================================================
// TIMER SCHEDULER
// ======================================================

/**
 * @brief Converts pending timer ticks into scheduled tasks.
 *
 * @return Nothing.
 */
void processTimerTicks()
{
  uint32_t ticks = 0;

  portENTER_CRITICAL(
    &timerMux
  );

  ticks = pendingTicks;

  pendingTicks = 0;

  portEXIT_CRITICAL(
    &timerMux
  );

  if (ticks == 0)
  {
    return;
  }

  sensorTicks += ticks;
  displayTicks += ticks;
  buzzerTicks += ticks;

  if (
    sensorTicks >=
    SENSOR_PERIOD
  )
  {
    sensorTicks %=
      SENSOR_PERIOD;

    sensorTaskDue = true;
  }

  if (
    displayTicks >=
    DISPLAY_PERIOD
  )
  {
    displayTicks %=
      DISPLAY_PERIOD;

    displayTaskDue = true;
  }

  if (
    buzzerTicks >=
    BUZZER_PERIOD
  )
  {
    buzzerTicks %=
      BUZZER_PERIOD;

    buzzerTaskDue = true;
  }
}

// ======================================================
// SENSOR ACQUISITION
// ======================================================

/**
 * @brief Reads and validates DHT22 and LDR sensor data.
 *
 * @return Nothing.
 */
void readSensors()
{
  float newTemperature =
    dht.readTemperature();

  float newHumidity =
    dht.readHumidity();

  if (
    isnan(newTemperature) ||
    isnan(newHumidity) ||
    newTemperature <
      MIN_VALID_TEMP ||
    newTemperature >
      MAX_VALID_TEMP ||
    newHumidity <
      MIN_VALID_HUMIDITY ||
    newHumidity >
      MAX_VALID_HUMIDITY
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

  const int SAMPLE_COUNT = 20;

  long total = 0;

  for (
    int i = 0;
    i < SAMPLE_COUNT;
    i++
  )
  {
    total +=
      analogRead(
        LDR_PIN
      );
  }

  lightRaw =
    total /
    SAMPLE_COUNT;

  Serial.println();

  Serial.println(
    "------ SENSOR DATA ------"
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

  Serial.print(
    "Mode: "
  );

  Serial.println(
    getModeName()
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
}

// ======================================================
// LDR FAULT DETECTION
// ======================================================

/**
 * @brief Detects persistent extreme LDR readings.
 *
 * @return Nothing.
 */
void checkLDRFault()
{
  bool extreme =
    lightRaw <=
      LDR_FAULT_LOW ||
    lightRaw >=
      LDR_FAULT_HIGH;

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
// SENSOR VALIDITY
// ======================================================

/**
 * @brief Checks whether all monitored sensors are valid.
 *
 * @return true if DHT22 and LDR are valid.
 */
bool sensorsValid()
{
  return
    !dhtFault &&
    !ldrFault;
}

// ======================================================
// SYSTEM CONTROL
// ======================================================

/**
 * @brief Applies Safety > Manual > Auto priority.
 *
 * @return Nothing.
 */
void updateSystem()
{
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

    return;
  }

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
// AUTO MODE
// ======================================================

/**
 * @brief Executes automatic nursery control.
 *
 * @return Nothing.
 */
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
    temperature >=
      WARNING_TEMP
      ? HIGH
      : LOW
  );
}

// ======================================================
// MANUAL MODE
// ======================================================

/**
 * @brief Executes Manual Override behaviour.
 *
 * @return Nothing.
 */
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
// SAFETY MODE
// ======================================================

/**
 * @brief Executes fail-safe behaviour.
 *
 * @return Nothing.
 */
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

/**
 * @brief Controls staged grow-light output using hysteresis.
 *
 * @return Nothing.
 */
void updateGrowLights()
{
  switch (growLevel)
  {
    case 0:

      if (
        lightRaw >
        LIGHT_BRIGHT +
          LIGHT_HYSTERESIS
      )
      {
        setGrowLights(1);
      }

      break;

    case 1:

      if (
        lightRaw <
        LIGHT_BRIGHT -
          LIGHT_HYSTERESIS
      )
      {
        setGrowLights(0);
      }
      else if (
        lightRaw >
        LIGHT_MEDIUM +
          LIGHT_HYSTERESIS
      )
      {
        setGrowLights(2);
      }

      break;

    case 2:

      if (
        lightRaw <
        LIGHT_MEDIUM -
          LIGHT_HYSTERESIS
      )
      {
        setGrowLights(1);
      }
      else if (
        lightRaw >
        LIGHT_DARK +
          LIGHT_HYSTERESIS
      )
      {
        setGrowLights(3);
      }

      break;

    case 3:

      if (
        lightRaw <
        LIGHT_DARK -
          LIGHT_HYSTERESIS
      )
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

/**
 * @brief Sets the number of active grow LEDs.
 *
 * @param level Grow-light level from 0 to 3.
 * @return Nothing.
 */
void setGrowLights(
  int level
)
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
// VENT CALCULATION
// ======================================================

/**
 * @brief Calculates proportional servo angle from temperature.
 *
 * @param temp Current temperature in degrees Celsius.
 * @return Required servo angle between 0 and 90 degrees.
 */
int calculateVentAngle(
  float temp
)
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

/**
 * @brief Moves the servo to the requested vent angle.
 *
 * @param angle Requested angle in degrees.
 * @return Nothing.
 */
void setVentAngle(
  int angle
)
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
// BUTTON HANDLING
// ======================================================

/**
 * @brief Processes Manual and Reset button input.
 *
 * @return Nothing.
 */
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

  if (
    manualState == HIGH &&
    previousManualButton == LOW &&
    now -
      lastManualPress >=
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
  }

  if (
    resetState == HIGH &&
    previousResetButton == LOW &&
    now -
      lastResetPress >=
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
    }

    else if (
      currentMode ==
      SAFETY_MODE
    )
    {
      Serial.println(
        "RESET denied - sensors not stable"
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

/**
 * @brief Generates intermittent buzzer output in Safety.
 *
 * @return Nothing.
 */
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
// OLED DISPLAY
// ======================================================

/**
 * @brief Updates OLED information for the active mode.
 *
 * @return Nothing.
 */
void updateOLED()
{
  display.clearDisplay();

  display.setTextSize(1);

  display.setTextColor(
    SSD1306_WHITE
  );

  // SAFETY
  if (
    currentMode ==
    SAFETY_MODE
  )
  {
    display.setCursor(
      0,
      0
    );

    display.println(
      "SAFETY MODE"
    );

    display.setCursor(
      0,
      14
    );

    if (
      dhtFault &&
      ldrFault
    )
    {
      display.println(
        "DHT + LDR FAILED"
      );
    }
    else if (
      dhtFault
    )
    {
      display.println(
        "DHT22 FAILED"
      );
    }
    else if (
      ldrFault
    )
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

    display.setCursor(
      0,
      28
    );

    display.println(
      "Vent: SAFE 45deg"
    );

    display.setCursor(
      0,
      42
    );

    display.println(
      "Grow LEDs: OFF"
    );

    display.setCursor(
      0,
      54
    );

    display.println(
      "Check + RESET"
    );

    display.display();

    return;
  }

  // MANUAL
  if (
    currentMode ==
    MANUAL_MODE
  )
  {
    display.setCursor(
      0,
      0
    );

    display.println(
      "MANUAL OVERRIDE"
    );

    display.setCursor(
      0,
      14
    );

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

    display.setCursor(
      0,
      29
    );

    display.print(
      "Light:"
    );

    display.println(
      lightRaw
    );

    display.setCursor(
      0,
      43
    );

    display.println(
      "Vent: OPEN 90deg"
    );

    display.setCursor(
      0,
      54
    );

    display.println(
      "Automation OFF"
    );

    display.display();

    return;
  }

  // AUTO
  display.setCursor(
    0,
    0
  );

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

  display.setCursor(
    0,
    14
  );

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

  display.setCursor(
    0,
    28
  );

  display.print(
    "Light:"
  );

  display.println(
    lightRaw
  );

  display.setCursor(
    0,
    41
  );

  display.print(
    "Grow:"
  );

  display.print(
    growLevel
  );

  display.println(
    "/3"
  );

  display.setCursor(
    0,
    54
  );

  display.print(
    "Vent:"
  );

  display.print(
    ventAngle
  );

  display.println(
    "deg"
  );

  display.display();
}

// ======================================================
// UART COMMANDS
// ======================================================

/**
 * @brief Processes UART commands from Serial Monitor.
 *
 * @return Nothing.
 */
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

      Serial.println(
        "MANUAL selected via UART"
      );
    }
    else
    {
      Serial.println(
        "MANUAL rejected: SAFETY active"
      );
    }
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

      Serial.println(
        "AUTO selected via UART"
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

      Serial.println(
        "SAFETY reset via UART"
      );
    }
    else
    {
      Serial.println(
        "RESET unavailable"
      );
    }
  }
}

// ======================================================
// STATUS
// ======================================================

/**
 * @brief Prints complete system status to Serial Monitor.
 *
 * @return Nothing.
 */
void printStatus()
{
  Serial.println();

  Serial.println(
    "========================"
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

  Serial.println(
    "========================"
  );
}

// ======================================================
// MODE NAME
// ======================================================

/**
 * @brief Converts the current mode into readable text.
 *
 * @return AUTO, MANUAL, SAFETY or UNKNOWN.
 */
const char *getModeName()
{
  switch (
    currentMode
  )
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
