// Including Header Files
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include "ZMPT101B.h"
#include <mcp_can.h>
// Defining Constants
#define CAN_RECEIVE_ID 0x123
#define CAN_IMP_ID 0x100
#define CAN_INT_PIN 2
#define CAN_CS_PIN 4
#define SWITCH_INT_PIN 3
#define DISPLAY_ON_OFF_PIN A7
#define MODE_PIN A6
#define ACTION_PIN 6
#define BUZZER_PIN 9
#define RELAY_PIN 5
#define HALT_PIN A3
#define ERROR_PIN 10
#define VOLTAGE_SENSOR_PIN A0
#define MAINS_LED_PIN 8
#define MOTOR_LED_PIN 7
#define MODE_RED_LED_PIN A1
#define MODE_GREEN_LED_PIN A2
#define EEPROM_BACKLIGHT 0
#define EEPROM_MODE 1
#define DRY_RUN_WAIT_TIME 60000
// Declaring Variables
unsigned long last_interrupt_time = 0;
unsigned long interrupt_time = 0;
boolean displayOnOffButton = false;                    // Is backlight button pressed
boolean modeButton = false;                            // Is mode button pressed
boolean actionButton = false;                          // Is action button pressed
boolean displayBacklightState = true;                  // Is backlight on/off
boolean systemState = true;                            // Is all sensors working
boolean tankState = false, normalMessage = false;      // CAN is message available
boolean motorState = false, motorStateRunOnce = false; // Motor on/off
unsigned long dryRunStartTime = 0;                     // Dry run timer start time
boolean setTimer = false;                              // Dry run timer start
boolean dryRunTimerStarted = false;                    // Is dry run timer started
boolean dryRunTimerExpired = false;                    // Is dry run timer ended
boolean needAction = false;
boolean motorStopOnce = false;
byte errorCount = 0;
byte previousFirstRowLength = 0;
byte previousSecondRowLength = 0;
short mode = 0;
short mainsVoltage = 0;
struct can_frame
{
  unsigned long id;
  byte length;
  byte data[8];
} canFrame;
struct LCD_DATA
{
  short waterLevel;
  short voltage;
  short temperature;
} lcdData;
// Creating Instances
LiquidCrystal_I2C lcd(0x27, 16, 2);
ZMPT101B voltageSensor(VOLTAGE_SENSOR_PIN);
MCP_CAN CAN_BUS(4);
// Declaring Funtions
void checkState(boolean);
void errorBeep(void);
void normalBeep(void);
void beep(void);
void handleButtonPress(void);
void lcdFirstRow(String, boolean);
void lcdSecondRow(String, boolean);
void modeLED(short, boolean);
void motorLED(boolean);
void mainsLED(boolean);
void eepromWrite(int, byte);
void handleTankState(void);
void handleNormalMessage(void);
void handleMode(void);
void handleTimer(void);
void handleMotorState(void);
void canISR(void);
void buttonPress(void);

void setup()
{
  // Configuring Pins
  pinMode(CAN_INT_PIN, INPUT);
  pinMode(SWITCH_INT_PIN, INPUT);
  pinMode(DISPLAY_ON_OFF_PIN, INPUT);
  pinMode(MODE_PIN, INPUT);
  pinMode(ACTION_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(MODE_RED_LED_PIN, OUTPUT);
  pinMode(MODE_GREEN_LED_PIN, OUTPUT);
  pinMode(MAINS_LED_PIN, OUTPUT);
  pinMode(MOTOR_LED_PIN, OUTPUT);
  pinMode(HALT_PIN, OUTPUT);
  pinMode(ERROR_PIN, INPUT_PULLUP);
  // Initialize EEPROM if new
  if (EEPROM.read(EEPROM_BACKLIGHT) == 255)
  {
    EEPROM.write(EEPROM_BACKLIGHT, true);
  }
  if (EEPROM.read(EEPROM_MODE) == 255)
  {
    EEPROM.write(EEPROM_MODE, 0);
  }
  // Setting values from EEPROM
  mode = EEPROM.read(EEPROM_MODE);
  displayBacklightState = EEPROM.read(EEPROM_BACKLIGHT);
  // Initialize CAN
  if (CAN_BUS.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK)
  {
    systemState = false;
  }
  CAN_BUS.setMode(MCP_NORMAL);
  delay(100);
  // Initialize display
  lcd.init();
  lcd.setBacklight(displayBacklightState);
  lcd.clear();
  // Attaching interrupts
  attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), canISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(SWITCH_INT_PIN), buttonPress, FALLING);
  // Initialize LEDs
  modeLED(0, false);
  mainsLED(false);
  motorLED(false);
  // Check error state
  checkState(systemState);
  delay(1000);
}

void loop()
{
  // Handle button press
  handleButtonPress();
  // Measure voltage
  mainsVoltage = round(voltageSensor.acVoltage());
  // Handle tank state change message
  handleTankState();
  // Handle normal message
  handleNormalMessage();
  // Handle timer
  handleTimer();
  // Handle mode
  handleMode();
  // Set motor state
  handleMotorState();
  // Handle error
  if (digitalRead(ERROR_PIN) == LOW)
  {
    errorCount++;
  }
  else
  {
    errorCount = 0;
  }
  if (errorCount >= 10)
  {
    checkState(false);
  }
  // Set line power led
  if (mainsVoltage > 100)
  {
    mainsLED(true);
  }
  else
  {
    mainsLED(false);
  }
  delay(50);
}

// Handler functions
void handleMotorState()
{
  if (!dryRunTimerStarted && !dryRunTimerExpired)
  {
    setTimer = motorState;
  }
  if (motorState && mode == 0)
  {
    if (mainsVoltage < 100)
    {
      motorState = false;
      mode = 2;
    }
    else
    {
      if (setTimer)
      {
        setTimer = false;
        dryRunStartTime = millis();
        dryRunTimerStarted = true;
      }
      motorLED(true);
      digitalWrite(RELAY_PIN, HIGH);
    }
  }
  else
  {
    motorLED(false);
    digitalWrite(RELAY_PIN, LOW);
    if (mode != 2)
    {
      mode = EEPROM.read(EEPROM_MODE);
    }
    dryRunTimerStarted = false;
    dryRunTimerExpired = false;
  }
}

void handleTimer()
{
  if (millis() < DRY_RUN_WAIT_TIME || dryRunStartTime < millis())
  {
    dryRunStartTime = 0;
  }
  if (dryRunTimerStarted && (millis() - dryRunStartTime) >= DRY_RUN_WAIT_TIME)
  {
    dryRunTimerExpired = true;
    dryRunTimerStarted = false;
  }
}

void handleMode()
{
  modeLED(mode, true);
  if (mode == 2 && !needAction)
  {
    if (mainsVoltage > 100)
    {
      mode = 0;
      eepromWrite(EEPROM_MODE, mode);
      motorState = true;
    }
  }
  if (needAction)
  {
    mode = 2;
    motorState = false;
  }
  if (mode == 1)
  {
    motorState = false;
  }
}

void handleNormalMessage()
{
  if (normalMessage)
  {
    if (dryRunTimerExpired)
    {
      if (motorState && !canFrame.data[5] && mode == 0)
      {
        needAction = true;
        normalBeep();
      }
      dryRunTimerExpired = false;
    }
    if (displayBacklightState)
    {
      lcdData.waterLevel = round((float)(canFrame.data[3] * 100 + canFrame.data[4]) / 100.0);
      lcdData.temperature = round((float)(canFrame.data[1] * 100 + canFrame.data[2]) / 100.0);
      lcdData.voltage = mainsVoltage;
      String firstRow, secondRow;
      if (lcdData.waterLevel > 0)
      {
        firstRow = "W L:" + (String)lcdData.waterLevel + "% VOL:" + (String)lcdData.voltage + "V";
        lcdFirstRow(firstRow, firstRow.length() != previousFirstRowLength);
        previousFirstRowLength = firstRow.length();
      }
      if (lcdData.temperature > 0)
      {
        secondRow = "TMP:" + (String)lcdData.temperature +
                    "\xDF" + "C T:" + (canFrame.data[5] ? "F" : "D") + " M:" +
                    (mode == 0 ? "A" : mode == 1 ? "M"
                                                 : "Q");
        lcdSecondRow(secondRow, secondRow.length() != previousSecondRowLength);
        previousSecondRowLength = secondRow.length();
      }
    }
    normalMessage = false;
  }
}

void handleTankState()
{
  if (tankState)
  {
    // First run
    if (canFrame.data[3])
    {
      // When tank is empty
      if (!canFrame.data[1] && canFrame.data[2])
      {
        motorState = true;
      }
    }
    else
    {
      // When tank is full
      if (canFrame.data[1] && !canFrame.data[2])
      {
        motorState = false;
      }
      // When tank is empty
      if (!canFrame.data[1] && canFrame.data[2])
      {
        motorState = true;
      }
    }
    tankState = false;
    digitalWrite(HALT_PIN, LOW);
  }
}

void handleButtonPress()
{
  if (displayOnOffButton)
  {
    displayOnOffButton = false;
    displayBacklightState = !displayBacklightState;
    lcd.setBacklight(displayBacklightState);
    eepromWrite(EEPROM_BACKLIGHT, displayBacklightState);
    beep();
  }
  else if (modeButton)
  {
    modeButton = false;
    if (mode == 0)
    {
      mode = 1;
    }
    else if (mode == 1)
    {
      mode = 0;
    }
    eepromWrite(EEPROM_MODE, mode);
    beep();
  }
  else if (actionButton)
  {
    needAction = false;
    actionButton = false;
    beep();
  }
}

// EEPROM functions
void eepromWrite(int idx, byte val)
{
  if (EEPROM.read(idx) != val)
  {
    EEPROM.write(idx, val);
  }
}

// ISRs
void buttonPress()
{
  if (millis() < 250 || interrupt_time < last_interrupt_time)
  {
    last_interrupt_time = 0;
  }
  interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 250)
  {
    last_interrupt_time = interrupt_time;
    if (digitalRead(ACTION_PIN) == LOW)
    {
      displayOnOffButton = false;
      modeButton = false;
      actionButton = true;
      return;
    }
    else if (analogRead(MODE_PIN) < 512)
    {
      displayOnOffButton = false;
      modeButton = true;
      actionButton = false;
      return;
    }
    else if (analogRead(DISPLAY_ON_OFF_PIN) < 512)
    {
      displayOnOffButton = true;
      modeButton = false;
      actionButton = false;
      return;
    }
  }
}

void canISR()
{
  if (CAN_BUS.readMsgBuf(&canFrame.id, &canFrame.length, canFrame.data) == CAN_OK)
  {
    if (canFrame.length == 5 && canFrame.id == CAN_IMP_ID && canFrame.data[0] == 240 && canFrame.data[4] == 240)
    {
      tankState = true;
      normalMessage = false;
      digitalWrite(HALT_PIN, HIGH);
    }
    else if (canFrame.length == 8 && canFrame.id == CAN_RECEIVE_ID && canFrame.data[0] == 170 && canFrame.data[7] == 170)
    {
      normalMessage = true;
      tankState = false;
    }
  }
  else
  {
    systemState = false;
  }
}

// LCD functions
void lcdFirstRow(String string = "", boolean clearScreen = true)
{
  if (clearScreen)
  {
    lcd.setCursor(0, 0);
    lcd.print("                ");
  }
  lcd.setCursor(0, 0);
  if (string.length())
  {
    lcd.print(string);
  }
}

void lcdSecondRow(String string = "", boolean clearScreen = true)
{
  if (clearScreen)
  {
    lcd.setCursor(0, 1);
    lcd.print("                ");
  }
  lcd.setCursor(0, 1);
  if (string.length())
  {
    lcd.print(string);
  }
}

void checkState(boolean system_state)
{
  lcd.clear();
  lcd.backlight();
  lcdFirstRow("CHECKING SYSTEM!");
  lcdSecondRow();
  int i = 0;
  while (i++ < 16)
  {
    lcd.print(char(255));
    delay(random(100, 500));
  }
  if (system_state)
  {
    lcdSecondRow("       OK       ");
  }
  else
  {
    lcdSecondRow("     FAILED     ");
    errorBeep();
    lcd.setBacklight(true);
    exit(0);
  }
  delay(1000);
  lcdFirstRow();
  lcdSecondRow();
}

// Buzzer functions
void errorBeep()
{
  int count = 0;
  while (count++ <= 30)
  {
    digitalWrite(BUZZER_PIN, HIGH);
    lcd.setBacklight(true);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    lcd.setBacklight(false);
    delay(100);
  }
}

void normalBeep()
{
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
  delay(50);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
}

void beep()
{
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
}

// Led Functions
void modeLED(short mode, boolean onOff)
{
  if (!onOff)
  {
    digitalWrite(MODE_GREEN_LED_PIN, LOW);
    digitalWrite(MODE_RED_LED_PIN, LOW);
    return;
  }
  switch (mode)
  {
  case 0:
    digitalWrite(MODE_GREEN_LED_PIN, HIGH);
    digitalWrite(MODE_RED_LED_PIN, LOW);
    break;
  case 1:
    digitalWrite(MODE_GREEN_LED_PIN, LOW);
    digitalWrite(MODE_RED_LED_PIN, HIGH);
    break;
  case 2:
    digitalWrite(MODE_GREEN_LED_PIN, HIGH);
    digitalWrite(MODE_RED_LED_PIN, HIGH);
    break;
  default:
    break;
  }
}

void motorLED(boolean onOff)
{
  if (onOff)
  {
    digitalWrite(MOTOR_LED_PIN, HIGH);
    if (!motorStateRunOnce)
    {
      normalBeep();
      motorStateRunOnce = true;
      motorStopOnce = false;
    }
  }
  else
  {
    digitalWrite(MOTOR_LED_PIN, LOW);
    if (!motorStopOnce)
    {
      normalBeep();
      motorStopOnce = true;
      motorStateRunOnce = false;
    }
  }
}

void mainsLED(boolean onOff)
{
  if (onOff)
  {
    digitalWrite(MAINS_LED_PIN, HIGH);
  }
  else
  {
    digitalWrite(MAINS_LED_PIN, LOW);
  }
}