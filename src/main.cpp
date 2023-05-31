// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.
#include <Arduino.h>
// Import libraries (BLEPeripheral depends on SPI)
#include <SPI.h>
#include <BLEPeripheral.h>

#include "nrf_soc.h"
#include "nrf_nvic.h"
#include "nrf_sdm.h"
extern "C"
{
#include "BLESerial.h"
#include "fstorage.h"
#include "softdevice_handler.h"
#include "app_error.h"
#include "nordic_common.h"
#include "ble_stack_handler_types.h"
#include "ant_stack_handler_types.h"
#include "nrf_assert.h"
#include "sdk_errors.h"
#include "section_vars.h"
// #include "fstorage_internal_defs.h"
#include "wiring_analog_nrf52.c"
}

#include <MyGenericSPI.h>
#include <RHGenericSPI.h>
#include <ISRMX.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "nrf_delay.h"
#include <arduino-timer.h>

// define pins (varies per shield/board)
#define BLE_REQ 10 // nrf518822 unused
#define BLE_RDY 2  // nrf518822 unused
#define BLE_RST 9  // nrf518822 unused
// LED pin
#define LED_PIN 18

#define ARM_MATH_CM0
#include "arm_math.h"

#include "arm_const_structs.h"

#define FFT_TEST_COMP_SAMPLES_LEN 256 // 256 complex samples
#define FFT_TEST_OUT_SAMPLES_LEN (FFT_TEST_COMP_SAMPLES_LEN / 2)
#define NIRQ 28

static const uint8_t _SS = 30;
// static const uint8_t _MOSI = PIN_SPI_MOSI;
//  static const uint8_t _MISO = 25;//PIN_SPI_MISO;
//  static const uint8_t _SCK = 23;  //PIN_SPI_SCK; on dev board
static const uint8_t FDATA = PIN_A3;
static const uint8_t ADATA = PIN_A2;
static const uint8_t BATTERY = PIN_A4;

// define motor pins
#define MOTOR1_PIN1 12
#define MOTOR1_PIN2 13
#define MOTOR2_PIN1 14
#define MOTOR2_PIN2 15
#define MOTORS_SLEEP 11
#define MOTORS_ERROR 10

#define MID_SENSOR1 16
#define MID_SENSOR2 8

MyGenericSPI hardware_spi2; // using default constructor spi settings
ISRMX isrmx(_SS, NIRQ, hardware_spi2);
BLESerial bleSerial = BLESerial(BLE_REQ, BLE_RDY, BLE_RST);

#define DO_SLEEP 1
#define RNG_ENABLED 1

static const uint8_t AES_KEY[16] = {'c', 'Q', 'f', 'T', 'j', 'W', 'n', 'Z', 'r', '4', 'u', '7', 'x', '!', 'z', '%'};
static nrf_ecb_hal_data_t m_ecb_data;
unsigned char challenge[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};
char solution[33]; // solution to challenge

uint8_t Auth = 0; // 0 = not authenticated, 1 = first challenge solved , 3 = authenticated / second challenge solved
uint8_t connected = 0;
float32_t *m_fft_input_f64;
float32_t *m_fft_output_f64;
int key_signal_tmp = 0;
static float32_t ftt_of_ref[64]; // reference fft
uint32_t max_val_index_ref = 0;
float32_t max_value_ref = 0;
float32_t hellinger_threshold = 0.7;
uint8_t cool_down = 0;
static uint32_t adcReference = ADC_CONFIG_REFSEL_SupplyOneThirdPrescaling;
static uint32_t adcPrescaling = ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling;
enum KeyCalibrationState
{
  KEY_CALIBRATION_STATE_NOT_CALIBRATED,
  KEY_CALIBRATION_STATE_CALIBRATING,
  KEY_CALIBRATION_STATE_CALIBRATING_STEP_1,
  KEY_CALIBRATION_STATE_CALIBRATING_STEP_2,
  KEY_CALIBRATION_STATE_CALIBRATED_STEP_DONE,
};

enum KeySensorStatus
{
  SENSOR_OK,
  SENSOR_ERROR
};

// manual motor control for both motors forwards and backwards
enum ManualMotorControl
{
  MOTOR1_FORWARD,
  MOTOR1_BACKWARD,
  MOTOR2_FORWARD,
  MOTOR2_BACKWARD,
};

enum KeybotState
{
  KEYBOT_STATE_IDLE,
  KEYBOT_PRESSING_LEFT,
  KEYBOT_PRESSING_RIGHT,
  KEYBOT_RETURNING_TO_CENTER_FROM_LEFT,
  KEYBOT_RETURNING_TO_CENTER_FROM_RIGHT,
  KEYBOT_ERROR_PRESSING_LEFT,
  KEYBOT_ERROR_PRESSING_RIGHT,
  KEYBOT_ERROR_RETURNING_TO_CENTER_FROM_LEFT,
  KEYBOT_ERROR_RETURNING_TO_CENTER_FROM_RIGHT,
  KEYBOT_STATE_EMERGENCY_RESET,
  KEYBOT_STATE_CENTERING,
  KEYBOT_ERROR_CENTERING,

};

enum KeyBotCommand
{
  KEYBOT_PRESS_LEFT,
  KEYBOT_PRESS_RIGHT,
  KEYBOT_EMERGENCY_STOP,
  KEYBOT_CENTER,
};

uint8_t mid_sensor1 = 0;
uint8_t mid_sensor2 = 0;

bool key_sensed = false;

KeyCalibrationState key_calibration_state = KEY_CALIBRATION_STATE_NOT_CALIBRATED;
KeySensorStatus key_sensor_status = SENSOR_ERROR;

BLEService keybotService = BLEService("00001815-0000-1000-8000-00805F9B34FB");
// create challenge characteristic
BLEFixedLengthCharacteristic challengeCharacteristic = BLEFixedLengthCharacteristic("00002a3d-0000-1000-8000-00805f9b34fa", BLERead | BLENotify, 20);
// add descriptor
BLEDescriptor challengeDescriptor = BLEDescriptor("2901", "Challenge");
// solution characteristic
BLEFixedLengthCharacteristic solutionCharacteristic = BLEFixedLengthCharacteristic("00002a3d-0000-1000-8000-00805f9b34fb", BLEWrite, 20);
// add descriptor
BLEDescriptor solutionDescriptor = BLEDescriptor("2901", "Solution");

// auth characteristic
BLECharCharacteristic authCharacteristic = BLECharCharacteristic("00002a3d-0000-1000-8000-00805f9b34fc", BLERead | BLENotify);
// add descriptor
BLEDescriptor authDescriptor = BLEDescriptor("2901", "Auth");

BLECharCharacteristic calibrateCharacteristic("00002a3d-0000-1000-8000-00805f9b34fe", BLERead | BLEWrite | BLENotify);

BLEDescriptor CalibrationDescriptor = BLEDescriptor("2901", "Used for calibration");

BLECharCharacteristic sensorStatusCharacteristic("00002a3d-0000-1000-8000-00805f9b34fd", BLERead | BLENotify);

BLEDescriptor sensorStatusDescriptor = BLEDescriptor("2901", "Sensor status");

// BLECharCharacteristic calibrateChangeModeCharacteristic("00002a3d-0000-1000-8000-00805f9b34ff", BLERead | BLEWrite | BLENotify);

// test characteristic with response
BLECharCharacteristic testCharacteristic("00002a3d-0000-1000-8000-00805f9b34f0", BLERead | BLEWrite | BLENotify);

BLEDescriptor testDescriptor = BLEDescriptor("2901", "Test");

// manaul motor control characteristic
BLECharCharacteristic manualMotorControlCharacteristic("00002a3d-0000-1000-8000-00805f9b34f1", BLERead | BLEWrite | BLENotify);

BLEDescriptor manualMotorControlDescriptor = BLEDescriptor("2901", "Manual motor control");

// mid sensor 1 characteristic
BLECharCharacteristic midSensorsCharacteristic("00002a3d-0000-1000-8000-00805f9b34f2", BLERead | BLENotify);

BLEDescriptor midSensorsDescriptor = BLEDescriptor("2901", "Mid sensor 1 and 2");

// unlock command characteristic
BLECharCharacteristic keyBotCommandCharacteristic("00002a3d-0000-1000-8000-00805f9b34f3", BLERead | BLEWrite | BLENotify);

BLEDescriptor keyBotCommandDescriptor = BLEDescriptor("2901", "Unlock command");

// unlock status characteristic
BLECharCharacteristic keyBotStateCharacteristic("00002a3d-0000-1000-8000-00805f9b34f4", BLERead | BLENotify);

BLEDescriptor keyBotStateStatusDescriptor = BLEDescriptor("2901", "Unlock status");

// b attery voltage characteristic
BLEFloatCharacteristic batteryVoltageCharacteristic("00002a3d-0000-1000-8000-00805f9b34f5", BLERead | BLENotify);

BLEDescriptor batteryVoltageDescriptor = BLEDescriptor("2901", "Battery voltage");

void blePeripheralDisconnectHandler(BLECentral &central);
void blePeripheralRemoteServicesDiscoveredHandler(BLECentral &central);
void blePeripheralConnectHandler(BLECentral &central);
void blinkLed(int times, int timedelay);
void solutionCharacteristicWritten(BLECentral &central, BLECharacteristic &characteristic);
void calibrateChangeModeWritten(BLECentral &central, BLECharacteristic &characteristic);
void testCharacteristicWritten(BLECentral &central, BLECharacteristic &characteristic);
void manualMotorControlCharacteristicWritten(BLECentral &central, BLECharacteristic &characteristic);
void unlockCommandCharacteristicWritten(BLECentral &central, BLECharacteristic &characteristic);
bool isLimitSensorTriggered(int sensor, int index);
void updateMidSensors();
uint32_t getRandom32();
void motor1Forward();
void motor1Reverse();
void motor2Forward();
void motor2Reverse();
void stopMotor1();
void stopMotor2();
void stopAllMotors();

float batteryVoltage();
void onBatteryVoltageCharacteristicSubscribed(BLECentral &central, BLECharacteristic &characteristic);

static void fft_process(float32_t *p_input,
                        const arm_cfft_instance_f32 *p_input_struct,
                        float32_t *p_output,
                        uint16_t output_size)
{
  // Use CFFT module to process the data.
  arm_cfft_f32(p_input_struct, p_input, 0, 1);
  // Calculate the magnitude at each bin using Complex Magnitude Module function.
  arm_cmplx_mag_f32(p_input, p_output, output_size);
}
bool fft(void *opaque);
uint32_t fastAnalogRead(uint32_t pin);

void roll_signal(q7_t *signal, int n, int shift);

// timer
Timer<> fft_timer;

Timer<> battery_timer;

const int debounceDelay = 100;                // Debounce delay in milliseconds
const unsigned long motorTimeout = 16000;     // Motor timeout in milliseconds (5 seconds)
const unsigned long centeringTimeout = 25000; // Motor timeout in milliseconds (5 seconds)
const unsigned long returnTimeout = 20000;    // Return timeout in milliseconds (5 seconds)
unsigned long lastLimitSensorChange = 0;

unsigned long startTime;
unsigned long forwardMovementDuration;

KeybotState currentState = KEYBOT_STATE_IDLE;

bool updateBatteryVoltage(void *opaque)
{
  batteryVoltageCharacteristic.setValueLE(batteryVoltage());
  return true;
}

void setup()
{
  Serial.begin(115200);

  bleSerial.setAppearance(BLE_APPEARANCE_GENERIC_REMOTE_CONTROL);
  bleSerial.setLocalName("KeyBot_000000000001");
  bleSerial.setAdvertisedServiceUuid(keybotService.uuid());

  bleSerial.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  bleSerial.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
  bleSerial.setEventHandler(BLERemoteServicesDiscovered, blePeripheralRemoteServicesDiscoveredHandler);
  batteryVoltageCharacteristic.setEventHandler(BLESubscribed, onBatteryVoltageCharacteristicSubscribed);
  // listen for write events
  solutionCharacteristic.setEventHandler(BLEWritten, solutionCharacteristicWritten);
  // calibrateChangeModeCharacteristic.setEventHandler(BLEWritten, calibrateChangeModeWritten);
  testCharacteristic.setEventHandler(BLEWritten, testCharacteristicWritten);
  manualMotorControlCharacteristic.setEventHandler(BLEWritten, manualMotorControlCharacteristicWritten);
  keyBotCommandCharacteristic.setEventHandler(BLEWritten, unlockCommandCharacteristicWritten);
  bleSerial.addAttribute(keybotService);
  bleSerial.addAttribute(challengeCharacteristic);
  bleSerial.addAttribute(challengeDescriptor);
  bleSerial.addAttribute(solutionCharacteristic);
  bleSerial.addAttribute(solutionDescriptor);
  bleSerial.addAttribute(authCharacteristic);
  bleSerial.addAttribute(authDescriptor);
  // bleSerial.addAttribute(calibrateCharacteristic);
  // bleSerial.addAttribute(CalibrationDescriptor);
  // bleSerial.addAttribute(sensorStatusCharacteristic);
  // bleSerial.addAttribute(sensorStatusDescriptor);
  // bleSerial.addAttribute(calibrateChangeModeCharacteristic);
  //  bleSerial.addAttribute(testCharacteristic);
  //  bleSerial.addAttribute(testDescriptor);
  bleSerial.addAttribute(manualMotorControlCharacteristic);
  bleSerial.addAttribute(manualMotorControlDescriptor);
  bleSerial.addAttribute(midSensorsCharacteristic);
  bleSerial.addAttribute(midSensorsDescriptor);
  bleSerial.addAttribute(keyBotCommandCharacteristic);
  bleSerial.addAttribute(keyBotCommandDescriptor);
  bleSerial.addAttribute(keyBotStateCharacteristic);
  bleSerial.addAttribute(keyBotStateStatusDescriptor);
  bleSerial.addAttribute(batteryVoltageCharacteristic);
  bleSerial.addAttribute(batteryVoltageDescriptor);

  authCharacteristic.setValue('0');
  calibrateCharacteristic.setValue(key_calibration_state + 48);
  sensorStatusCharacteristic.setValue(key_sensor_status + 48);
  // calibrateChangeModeCharacteristic.setValue('0');
  testCharacteristic.setValue('0');
  manualMotorControlCharacteristic.setValue('0');
  batteryVoltageCharacteristic.setValueLE(batteryVoltage());

  bleSerial.begin();

  // set key
  memcpy(m_ecb_data.key, AES_KEY, 16);

  pinMode(LED_PIN, OUTPUT);

  // setup motors
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
  pinMode(MOTOR2_PIN1, OUTPUT);
  pinMode(MOTOR2_PIN2, OUTPUT);
  pinMode(MOTORS_SLEEP, OUTPUT);

  // set all to low
  digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, LOW);
  digitalWrite(MOTOR2_PIN1, LOW);
  digitalWrite(MOTOR2_PIN2, LOW);
  digitalWrite(MOTORS_SLEEP, LOW);

  // MID SENSOR
  pinMode(MID_SENSOR1, INPUT_PULLDOWN);
  pinMode(MID_SENSOR2, INPUT_PULLDOWN);

  mid_sensor1 = digitalRead(MID_SENSOR1);
  mid_sensor2 = digitalRead(MID_SENSOR2);

  // set midSensorsCharacteristic 0 if both sensors are 0 and 1 if first is 1 and second is 0 and 2 if first is 0 and second is 1
  midSensorsCharacteristic.setValue(mid_sensor1 + mid_sensor2 * 2 + 48);

  // setup S3NSOR
  pinMode(FDATA, INPUT);
  pinMode(ADATA, INPUT);

  int ok = isrmx.init();

  if (ok)
  {
    key_sensor_status = SENSOR_OK;
    Serial.println("isrmx init ok");
  }
  else
  {
    key_sensor_status = SENSOR_ERROR;
    Serial.println("isrmx init failed");
  }
  sensorStatusCharacteristic.setValue(key_sensor_status + 48);

  // calib
  for (int i = 0; i < 64; i++)
  {
    ftt_of_ref[i] = 0.0;
  }

  m_fft_input_f64 = (float32_t *)malloc(FFT_TEST_COMP_SAMPLES_LEN * sizeof(float32_t));
  m_fft_output_f64 = (float32_t *)malloc(FFT_TEST_COMP_SAMPLES_LEN * sizeof(float32_t));

  analogReadResolution(9); // 8 bit resolution for fast reading

  // start timer
  fft_timer.every(20, fft, 0);

  battery_timer.every(20000, updateBatteryVoltage, 0);

  Serial.println(F("BLE ready"));

  key_calibration_state = KEY_CALIBRATION_STATE_CALIBRATING;
  calibrateCharacteristic.setValue(key_calibration_state + 48);

  blinkLed(1, 1000);
}

bool keySensed()
{
  return key_sensed;
}

void updateStateMachine()
{
  static KeybotState previousState = KEYBOT_STATE_IDLE;
  KeybotState newState = currentState;
  static int previousLimitSensor1State = -1;
  static int previousLimitSensor2State = -1;

  if (newState != previousState)
  {
    startTime = millis(); // Reset the timer

    if (newState == KEYBOT_STATE_CENTERING)
    {
      previousLimitSensor1State = -1;
      previousLimitSensor2State = -1;
    }
  }

  unsigned long elapsedTime = millis() - startTime;

  switch (currentState)
  {
  case KEYBOT_STATE_IDLE:
    break;

  case KEYBOT_PRESSING_LEFT:
    motor1Forward();

    if (keySensed())
    {
      currentState = KEYBOT_RETURNING_TO_CENTER_FROM_LEFT;

      startTime = millis(); // Reset the timer
      forwardMovementDuration = elapsedTime;
    }
    else if (elapsedTime >= motorTimeout)
    {
      currentState = KEYBOT_ERROR_PRESSING_LEFT;
      bleSerial.println("ERROR PRESSING LEFT");
      startTime = millis(); // Reset the timer
    }
    break;

  case KEYBOT_PRESSING_RIGHT:
    motor2Forward();
    if (keySensed())
    {
      currentState = KEYBOT_RETURNING_TO_CENTER_FROM_RIGHT;
      startTime = millis(); // Reset the timer
      forwardMovementDuration = elapsedTime;
    }
    else if (elapsedTime >= motorTimeout)
    {
      currentState = KEYBOT_ERROR_PRESSING_RIGHT;
      bleSerial.println("ERROR PRESSING RIGHT");
      startTime = millis(); // Reset the timer
    }
    break;

  case KEYBOT_RETURNING_TO_CENTER_FROM_LEFT:
    motor1Reverse();
    if (isLimitSensorTriggered(MID_SENSOR1, 0))
    {
      stopMotor1();
      currentState = KEYBOT_STATE_IDLE;
    }
    else if (elapsedTime >= returnTimeout)
    {
      currentState = KEYBOT_ERROR_RETURNING_TO_CENTER_FROM_LEFT;
      bleSerial.println("ERROR RETURNING TO CENTER FROM LEFT");
    }
    break;

  case KEYBOT_RETURNING_TO_CENTER_FROM_RIGHT:
    motor2Reverse();
    if (isLimitSensorTriggered(MID_SENSOR2, 1))
    {
      stopMotor2();
      currentState = KEYBOT_STATE_IDLE;
    }
    else if (elapsedTime >= returnTimeout)
    {
      currentState = KEYBOT_ERROR_RETURNING_TO_CENTER_FROM_RIGHT;
      bleSerial.println("ERROR RETURNING TO CENTER FROM RIGHT");
    }
    break;

  case KEYBOT_ERROR_PRESSING_LEFT:
    // keybot is returning to center from left
    motor1Reverse();
    if (isLimitSensorTriggered(MID_SENSOR1, 0))
    {
      stopMotor1();
      currentState = KEYBOT_STATE_IDLE;
    }
    else if (elapsedTime >= returnTimeout)
    {

      currentState = KEYBOT_ERROR_RETURNING_TO_CENTER_FROM_LEFT;
      bleSerial.println("ERROR RETURNING TO CENTER FROM LEFT");
    }
    break;

  case KEYBOT_ERROR_PRESSING_RIGHT:
    // keybot is returning to center from right
    motor2Reverse();
    if (isLimitSensorTriggered(MID_SENSOR2, 1))
    {
      stopMotor2();
      currentState = KEYBOT_STATE_IDLE;
    }
    else if (elapsedTime >= returnTimeout)
    {
      currentState = KEYBOT_ERROR_RETURNING_TO_CENTER_FROM_RIGHT;
      bleSerial.println("ERROR RETURNING TO CENTER FROM RIGHT");
    }
    break;

  case KEYBOT_ERROR_RETURNING_TO_CENTER_FROM_LEFT:
    // stop motors limit sesnor is not working
    stopMotor1();
    currentState = KEYBOT_STATE_IDLE;

    break;

  case KEYBOT_ERROR_RETURNING_TO_CENTER_FROM_RIGHT:
    // stop motors limit sesnor is not working
    stopMotor2();
    currentState = KEYBOT_STATE_IDLE;
    break;

  case KEYBOT_STATE_EMERGENCY_RESET:
    // stop motors limit sesnor is not working
    stopAllMotors();
    // reset
    currentState = KEYBOT_STATE_IDLE;
    break;

  case KEYBOT_STATE_CENTERING:
    // move both morots bacwards until both limit sensors are triggered

    if (previousLimitSensor1State == -1)
    {
      previousLimitSensor1State = isLimitSensorTriggered(MID_SENSOR1, 0);
      bleSerial.println("previousLimitSensor1State" + String(previousLimitSensor1State));
    }
    if (previousLimitSensor2State == -1)
    {
      previousLimitSensor2State = isLimitSensorTriggered(MID_SENSOR2, 1);
      bleSerial.println("previousLimitSensor2State" + String(previousLimitSensor2State));
    }

    if (previousLimitSensor1State == 0)
    {
      motor1Reverse();
    }
    if (previousLimitSensor2State == 0)
    {
      motor2Reverse();
    }
    if (previousLimitSensor1State == 1)
    {
      motor1Forward();
    }
    if (previousLimitSensor2State == 1)
    {
      motor2Forward();
    }

    if (isLimitSensorTriggered(MID_SENSOR1, 0) != previousLimitSensor1State)
    {
      delay(100);
      stopMotor1();
    }
    if (isLimitSensorTriggered(MID_SENSOR2, 1) != previousLimitSensor2State)
    {
      delay(100);
      stopMotor2();
    }
    // if both different from previous state then stop
    if (isLimitSensorTriggered(MID_SENSOR1, 0) != previousLimitSensor1State && isLimitSensorTriggered(MID_SENSOR2, 1) != previousLimitSensor2State)
    {
      delay(100);
      stopAllMotors();
      currentState = KEYBOT_STATE_IDLE;
    }

    // if (isLimitSensorTriggered(MID_SENSOR1,0) && previousLimitSensor1State == 0){
    //    delay(100);
    //   stopMotor1();

    // }
    // if (isLimitSensorTriggered(MID_SENSOR2,1) && previousLimitSensor2State == 0){
    //   delay(100);
    //   stopMotor2();
    // }
    // if (!isLimitSensorTriggered(MID_SENSOR1,0) && previousLimitSensor1State == 1){
    //   delay(100);
    //   stopMotor1();
    //   bleSerial.println("adjusting motor 1");
    //   previousLimitSensor1State = -1; //force to stop at transition from 0 to 1
    // }
    // if (!isLimitSensorTriggered(MID_SENSOR2,1) && previousLimitSensor2State == 1){
    //   delay(100);
    //   stopMotor2();
    //   bleSerial.println("adjusting motor 2");
    //   previousLimitSensor2State = -1; //force to stop at transition from 0 to 1
    // }

    // if((isLimitSensorTriggered(MID_SENSOR1,0) && previousLimitSensor1State == 0) && (isLimitSensorTriggered(MID_SENSOR2,1) && previousLimitSensor2State == 0)){
    //   currentState = KEYBOT_STATE_IDLE;
    //   previousLimitSensor1State = -1;
    //   previousLimitSensor2State = -1;
    // }

    if (elapsedTime >= centeringTimeout)
    {
      currentState = KEYBOT_ERROR_CENTERING;
      bleSerial.println("ERROR CENTERING");
    }
    break;

  case KEYBOT_ERROR_CENTERING:
    // stop motors limit sesnor is not working
    stopAllMotors();
    // reset
    currentState = KEYBOT_STATE_IDLE;
    break;
  }

  if (newState != previousState)
  {
    Serial.print("State changed from ");
    Serial.print(previousState);
    Serial.print(" to ");
    Serial.println(newState);
    keyBotStateCharacteristic.setValue(newState + 48);

    previousState = newState;
  }
}

void loop()
{
  fft_timer.tick();     // read data from sensor
  battery_timer.tick(); // read data from sensor
  updateMidSensors();
  updateStateMachine();

  // digitalWrite(LED_PIN, LOW);
  // digitalWrite(LED_PIN, HIGH);
  // digitalWrite(LED_PIN, LOW);//40us

  if (DO_SLEEP && !connected)
  {

    Serial.println(F("Sleep"));
    digitalWrite(MOTORS_SLEEP, LOW);
    isrmx.sleep_config();

    sd_app_evt_wait();
    Serial.println(F("WakeUp"));
    sd_nvic_ClearPendingIRQ(SWI2_IRQn);
    isrmx.default_cfg(); // todo FSK IF NECESSARY
  }

  bleSerial.poll();
}

typedef union
{
  uint8_t asBuf[4];
  uint32_t asInt;
} conv8_32;

uint32_t getRandom32()
{
  uint8_t bytes_available = 0;
  uint32_t err_code;
  while (bytes_available < 4)
  {
    err_code = sd_rand_application_bytes_available_get(&bytes_available);
    APP_ERROR_CHECK(err_code);
  }

  conv8_32 converter;
  err_code = sd_rand_application_vector_get(converter.asBuf, 4);
  APP_ERROR_CHECK(err_code);

  return converter.asInt;
};

void updateChallenge()
{
  uint32_t rnd = getRandom32();
  Serial.println("updateChallenge");
  Serial.println(rnd);

  char hexString[9];
  sprintf(hexString, "%08lX", rnd);
  Serial.println(hexString);
  challenge[0] = hexString[0];
  challenge[1] = hexString[1];
  challenge[2] = hexString[2];
  challenge[3] = hexString[3];
  challenge[4] = hexString[4];
  challenge[5] = hexString[5];
  challenge[6] = hexString[6];
  challenge[7] = hexString[7];
  // line terminator
  rnd = getRandom32();
  Serial.println(rnd);

  char hexString2[9];
  sprintf(hexString2, "%08lX", rnd);
  Serial.println(hexString2);
  challenge[8] = hexString2[0];
  challenge[9] = hexString2[1];
  challenge[10] = hexString2[2];
  challenge[11] = hexString2[3];
  challenge[12] = hexString2[4];
  challenge[13] = hexString2[5];
  challenge[14] = hexString2[6];
  challenge[15] = hexString2[7];

  Serial.println(String((char *)challenge));
  challengeCharacteristic.setValue(challenge, 16);
}

void updateMidSensors()
{
  static int previousLimitSensor1State = -1;
  static int previousLimitSensor2State = -1;

  int currentLimitSensor1State = isLimitSensorTriggered(MID_SENSOR1, 0);
  int currentLimitSensor2State = isLimitSensorTriggered(MID_SENSOR2, 1);

  if (previousLimitSensor1State != currentLimitSensor1State)
  {
    midSensorsCharacteristic.setValue(currentLimitSensor1State + currentLimitSensor2State * 2 + 48);
    bleSerial.println("mid_sensor1_changed" + String(currentLimitSensor1State));
    previousLimitSensor1State = currentLimitSensor1State;
  }

  if (previousLimitSensor2State != currentLimitSensor2State)
  {
    midSensorsCharacteristic.setValue(currentLimitSensor1State + currentLimitSensor2State * 2 + 48);
    bleSerial.println("mid_sensor2_changed" + String(currentLimitSensor2State));
    previousLimitSensor2State = currentLimitSensor2State;
  }
}

void calibrateChangeModeWritten(BLECentral &central, BLECharacteristic &characteristic)
{

  unsigned char buffer = characteristic.value()[0];
  Serial.println("calibrateChangeModeWritten");
  Serial.println(buffer);
  cool_down = 40;

  // reset calibration
  bool ok = isrmx.change_modulation(buffer == 48 ? 1 : 2);
  if (ok)
  {
    Serial.println("isrmx change_modulation ok");
  }
  else
  {
    Serial.println("isrmx change_modulation failed");
  }
}

void solutionCharacteristicWritten(BLECentral &central, BLECharacteristic &characteristic)
{
  Serial.println("solutionCharacteristicWritten");
  // check if first 16 chars are the same as challenge

  unsigned char buffer[17] = {'\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0'};

  for (int i = 0; i < 16; i++)
  {
    buffer[i] = characteristic.value()[i];
  }

  Serial.println("user solution: " + String((char *)buffer));
  Serial.println("correct solution: " + String((char *)solution));

  if (Auth == 0)
  {
    Serial.println("checking solution 1");

    for (int i = 0; i < 16; i++)
    {
      if (buffer[i] != solution[i])
      {
        Serial.println("wrong solution 1");
        return;
      }
    }

    Serial.println("correct solution 1");
    Auth = 1;
    return;
  }

  if (Auth == 1)
  {
    // check if second 16 chars are the same as challenge
    Serial.println("checking solution 2");
    for (int i = 16; i < 32; i++)
    {
      if (buffer[i - 16] != solution[i])
      {
        Serial.println("wrong solution 2");
        return;
      }
    }

    Serial.println("correct solution 2");
    Serial.println("Authenticated");
    authCharacteristic.setValue('1');
    Auth = 2;

    // wake up motors
    digitalWrite(MOTORS_SLEEP, HIGH);
  }
}

void setSolution()
{
  unsigned char buffer[16] = {'\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0'};

  for (int i = 0; i < 16; i++)
  {
    buffer[i] = challenge[i];
  }

  memcpy(m_ecb_data.cleartext, buffer, 16);
  sd_ecb_block_encrypt(&m_ecb_data);
  Serial.println("encrypting");

  for (int i = 0; i < 16; i++)
  {
    buffer[i] = m_ecb_data.cleartext[i];
  }
  Serial.println(String((char *)buffer));

  Serial.println("with key");
  for (int i = 0; i < 16; i++)
  {
    buffer[i] = m_ecb_data.key[i];
  }
  Serial.println(String((char *)buffer));

  Serial.println("result");

  char hexString[33];
  for (int i = 0; i < 16; i++)
  {
    sprintf(hexString + i * 2, "%02X", m_ecb_data.ciphertext[i]);
  }
  Serial.println(String((char *)hexString));

  // set solution
  memcpy(solution, hexString, 32);
  // set new line terminator
  solution[32] = '\0';

  Serial.println("solution set : " + String((char *)solution));
}

void blePeripheralConnectHandler(BLECentral &central)
{
  // central connected event handler
  Serial.print(F("Connected event, central: "));
  Serial.println(central.address());
  Auth = 0;
  updateChallenge();
  setSolution();

  connected = 1;

  blinkLed(5, 200);
}
void blePeripheralDisconnectHandler(BLECentral &central)
{
  Serial.print(F("Disconnected event, central: "));
  Serial.println(central.address());
  blinkLed(1, 200);

  Auth = 0;
  connected = 0;
  // sleep motors
  digitalWrite(MOTORS_SLEEP, LOW);
}

void blePeripheralRemoteServicesDiscoveredHandler(BLECentral &central)
{
  Serial.print(F("Remote Services Discovered event, central: "));
  Serial.println(central.address());
  blinkLed(1, 200);
}

void testCharacteristicWritten(BLECentral &central, BLECharacteristic &characteristic)
{

  bleSerial.println("testCharacteristicWritten");

  int h = isrmx.read(0x01);
  bleSerial.println(h);
  int l = isrmx.read(0x02);
  bleSerial.println(l);
}

void manualMotorControlCharacteristicWritten(BLECentral &central, BLECharacteristic &characteristic)
{
  // bleSerial.println("manualMotorControlCharacteristicWritten");
  unsigned char buffer = characteristic.value()[0] - 48;
  // bleSerial.println(buffer);

  if (buffer == MOTOR1_FORWARD)
  {

    // bleSerial.println("MOTOR1_FORWARD");

    // move motor 1 forward for 100ms
    motor1Forward();
    delay(400);
    stopMotor1();
  }
  else if (buffer == MOTOR1_BACKWARD)
  {

    // bleSerial.println("MOTOR1_BACKWARD");

    // move motor 1 backward for 100ms
    motor1Reverse();
    delay(400);
    stopMotor1();
  }
  else if (buffer == MOTOR2_FORWARD)
  {

    // bleSerial.println("MOTOR2_FORWARD");

    // move motor 2 forward for 100ms
    motor2Forward();
    delay(200);
    stopMotor2();
  }
  else if (buffer == MOTOR2_BACKWARD)
  {

    // bleSerial.println("MOTOR2_BACKWARD");

    // move motor 2 backward for 100ms
    motor2Reverse();
    delay(200);
    stopMotor2();
  }
}

void blinkLed(int times, int delaytime)
{
  for (int i = 0; i < times; i++)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(delaytime);
    digitalWrite(LED_PIN, LOW);
  }
}
int compare(const void *a, const void *b)
{
  float fa = *(const float *)a;
  float fb = *(const float *)b;
  return (fa > fb) - (fa < fb);
}

void median_filter(const float *input, float *output, size_t signal_length, size_t filter_size)
{
  size_t half_filter_size = filter_size / 2;
  float window[filter_size];

  for (size_t i = 0; i < signal_length; ++i)
  {
    size_t start = (i < half_filter_size) ? 0 : i - half_filter_size;
    size_t end = (i + half_filter_size >= signal_length) ? signal_length - 1 : i + half_filter_size;
    size_t window_length = end - start + 1;

    for (size_t j = 0; j < window_length; ++j)
    {
      window[j] = input[start + j];
    }

    qsort(window, window_length, sizeof(float), compare);
    output[i] = window[window_length / 2];
  }
}

void unlockCommandCharacteristicWritten(BLECentral &central, BLECharacteristic &characteristic)
{
  // bleSerial.println("CommandCharacteristicWritten");
  int buffer = characteristic.value()[0] - 48;
  bleSerial.println(buffer);

  if (buffer == KEYBOT_EMERGENCY_STOP)
  {
    // stop all motors
    currentState = KEYBOT_STATE_EMERGENCY_RESET;
  }

  // check if state is idle
  if (currentState == KEYBOT_STATE_IDLE)
  {
    // check if unlock command is received
    if (buffer == KEYBOT_PRESS_LEFT)
    {
      // stop all motors
      stopAllMotors();
      motor1Forward();
      currentState = KEYBOT_PRESSING_LEFT;
    }
    else if (buffer == KEYBOT_PRESS_RIGHT)
    {

      // stop all motors
      stopAllMotors();
      motor2Forward();
      currentState = KEYBOT_PRESSING_RIGHT;
    }
    else if (buffer == KEYBOT_CENTER)
    {
      // stop all motors
      stopAllMotors();
      currentState = KEYBOT_STATE_CENTERING;
    }
  }
}

bool fft(void *opaque)
{

  key_sensed = false;

  if (cool_down > 0)
  {
    cool_down--;
    return true;
  }

  int read1 = 0;
  unsigned long begin = millis();

  // digitalWrite(LED_PIN, HIGH);

  for (int i = 0; i < (256 - 1UL); i += 2)
  {

    // 4us for one loop iteration

    key_signal_tmp = fastAnalogRead(FDATA); // TODO switch to ADATA  #80us penalty at 10 bit resolution
    // key_signal_tmp = analogRead(FDATA); // TODO switch to ADATA  #40us penalty at 10 bit resolution

    m_fft_input_f64[(uint16_t)i] = (float32_t)key_signal_tmp; // 10us penalty
    m_fft_input_f64[(uint16_t)i + 1] = 0.0;

    if (key_signal_tmp > 256 / 2 - 20) // 300
    {
      read1++;
    }

    digitalWrite(LED_PIN, HIGH);
    // delayMicroseconds(4); //120 /115 //13
    digitalWrite(LED_PIN, LOW); // 90 us
  }

  // 25ms for 128 samples =

  unsigned long time = millis() - begin;
  // calculate sampling frequency
  float frequency = 128.0 / time;

  digitalWrite(LED_PIN, LOW);

  bool similar_num_of_1_0s = false;
  if (read1 > (FFT_TEST_OUT_SAMPLES_LEN / 2) - 10 && read1 < (FFT_TEST_OUT_SAMPLES_LEN / 2) + 10)
  {
    similar_num_of_1_0s = true;
  }

  // bleSerial.println("read ones: " + String(read1));

  // sampling at 5000 Hz

  for (int i = 0; i < (256 - 1UL); i += 2)
  {

    digitalWrite(LED_PIN, m_fft_input_f64[(uint16_t)i] > 256 / 2 - 20 ? HIGH : LOW);

    // delay for time / 128
    delayMicroseconds((int)(time / 128.0));

    // delayMicroseconds(120);
  }

  //   }
  //   Serial.println();

  digitalWrite(LED_PIN, LOW);

  if (similar_num_of_1_0s)
  {

    q7_t input_signal[256]; // save a copy of the input signal

    for (int i = 0; i < (256 - 1UL); i += 2)
    {
      input_signal[i / 2] = m_fft_input_f64[i] > (256 - 20) ? (q7_t)1 : (q7_t)0;
    }

    fft_process(m_fft_input_f64,
                &arm_cfft_sR_f32_len128,
                m_fft_output_f64,
                FFT_TEST_OUT_SAMPLES_LEN);

    m_fft_output_f64[0] = 0;

    float32_t max_value;
    uint32_t max_val_index;

    arm_max_f32(&m_fft_output_f64[0], FFT_TEST_OUT_SAMPLES_LEN / 2, &max_value, &max_val_index);

    float32_t frequency_of_max_bin = (float32_t)max_val_index * (float32_t)frequency * 1000.0 / (float32_t)FFT_TEST_OUT_SAMPLES_LEN;
    float32_t min_value, range;

    arm_min_f32(&m_fft_output_f64[0], FFT_TEST_OUT_SAMPLES_LEN / 2, &min_value, NULL);

    range = max_value - min_value;

    arm_offset_f32(&m_fft_output_f64[0], -min_value, &m_fft_output_f64[0], FFT_TEST_OUT_SAMPLES_LEN / 2);

    arm_scale_f32(&m_fft_output_f64[0], 1.0 / range, &m_fft_output_f64[0], FFT_TEST_OUT_SAMPLES_LEN / 2);

    float32_t taps[9] = {0.00013383061741478741168975830078,
                         0.00443186145275831222534179687500,
                         0.05399112775921821594238281250000,
                         0.24197144806385040283203125000000,
                         0.39894348382949829101562500000000,
                         0.24197144806385040283203125000000,
                         0.05399112775921821594238281250000,
                         0.00443186145275831222534179687500,
                         0.00013383061741478741168975830078};

    // use a fir filter to remove noise

    arm_fir_instance_f32 fir;

    arm_fir_init_f32(&fir, 9, taps, &m_fft_output_f64[64], FFT_TEST_OUT_SAMPLES_LEN / 2);

    arm_fir_f32(&fir, m_fft_output_f64, &m_fft_output_f64[64], FFT_TEST_OUT_SAMPLES_LEN / 2);

    for (int i = 0; i < (FFT_TEST_OUT_SAMPLES_LEN / 2) - 4; i++)
    {
      m_fft_output_f64[i + 64] = m_fft_output_f64[i + 64 + 4];
    }

    // add 4 zeros to the end

    for (int i = (FFT_TEST_OUT_SAMPLES_LEN)-4; i < (FFT_TEST_OUT_SAMPLES_LEN); i++)
    {
      m_fft_output_f64[i] = 0;
    }

    // 2500 hz is valid
    //  500 hz is valid
    //  1000 hz is valid
    //  2000 hz is valid
    // allowed error is 20 hz

    // Serial.println("poss " + String(frequency_of_max_bin) + " Hz ons" + String(read1));

    // check if most of the values are bellow 0.3
    uint32_t count = 0;

    for (int i = 64; i < (FFT_TEST_OUT_SAMPLES_LEN); i++)
    {
      if (m_fft_output_f64[i] < 0.2)
      {
        count++;
      }
    }

    // 70% of the values should be bellow 0.3
    float32_t percentage = (float32_t)count / (float32_t)64;

    float32_t max_value_scaled;
    uint32_t max_val_index_scaled;

    arm_max_f32(&m_fft_output_f64[64], 64, &max_value_scaled, &max_val_index_scaled);

    // if (percentage > 0.7 &&((frequency_of_max_bin >= 2480 && frequency_of_max_bin <= 2520) || (frequency_of_max_bin >= 480 && frequency_of_max_bin <= 520) ||
    // (frequency_of_max_bin >= 980 && frequency_of_max_bin <= 1020) || (frequency_of_max_bin >= 1980 && frequency_of_max_bin <= 2020)))
    //{

    if (percentage > 0.8)
    {

      // todo generate a new clean signal of the same frequency and compare it to the signal that was read
      // match both signals on the same edge and then compare the values
      //  scale them too so that they are the same size and then compare them
      // robut edge detection is needed
      // if the signal is not a square wave then it will not work
      // TODO

      //   for (int i = 0; i < 64; i++)
      //     {
      //       ftt_of_ref[i] = m_fft_output_f64[i + 64];
      //     }

      //     //write code that makes a square wave signal that is sampled at desired frequency in c

      int sample_rate = frequency * 1000; // Desired sampling rate (in Hz)
      int freq = frequency_of_max_bin;    // Frequency of the square wave (in Hz)
      int amplitude = 1;                  // Amplitude of the square wave

      double ti = 0.0; // Time index variable

      // Serial.println(key_signal_tmp);

      // take every second value of  m_fft_input_f64 and put it in the second half of the array
      //  int j = 0;
      //  for (int i = 2; i < (256 - 1UL); i += 2){
      //    m_fft_input_f64[j] = m_fft_input_f64[i] > 256/2 -20 ? 1 : 0;
      //    j++;
      //  }

      // bleSerial.print(String( m_fft_input_f64[0] ) + "rrr");

      // Generate and output square wave samples
      for (int i = 128; i < 256; i += 1)
      {

        // Increment time index by a single sample interval
        ti += 1.0 / sample_rate;

        // Compute the value of the current sample
        q7_t sample = sin(2 * PI * freq * ti) >= 0 ? (q7_t)1 : (q7_t)0;

        input_signal[i] = (q7_t)sample;
        // m_fft_input_f64[i + 1] = 0.0;

        // Serial.print(sample);
      }

      // FIRST half is the m_fft_input_f64 is the signal that was read
      // and the second half is the ideal signal that was generated
      // based on the frequency that was detected

      // q7_t result_corelation[32 * 2 - 1];

      q31_t max_dot_product = (q31_t)0;
      int max_dot_product_index = 0;
      for (int i = 0; i < 128; i++)
      {
        // do dot product
        q31_t res;
        roll_signal(&input_signal[128], FFT_TEST_COMP_SAMPLES_LEN / 2, 1);

        arm_dot_prod_q7(input_signal, &input_signal[128], 128, &res);
        if (res > max_dot_product)
        {
          max_dot_product = res;
          max_dot_product_index = i;
        }
      }

      // roll the ideal signal so that it matches the read signal
      roll_signal(&input_signal[128], FFT_TEST_COMP_SAMPLES_LEN / 2, max_dot_product_index);

      // arm_correlate_q7(input_signal, 32, &input_signal[128], 32, result_corelation);
      // uint32_t max_correlation_index;
      // q7_t max_correlation_value;
      // arm_max_q7(&result_corelation[0], 32, &max_correlation_value, &max_correlation_index);
      // for (int i = 0; i < 32 * 2 - 1; i++){
      //   bleSerial.print(result_corelation[i]);
      // }
      // bleSerial.println("");
      // roll the ideal signal so that it matches the read signal
      // roll_signal(&input_signal[128], FFT_TEST_COMP_SAMPLES_LEN/2, max_correlation_index);

      // print both signals side by side
      //  for (int i = 0; i < FFT_TEST_COMP_SAMPLES_LEN/2; i++){
      //    Serial.print(input_signal[i]);
      //    Serial.print(" ");
      //    Serial.println(input_signal[i + FFT_TEST_COMP_SAMPLES_LEN/2]);
      //  }

      // compare the two signals using arm amth
      int sum = 0;
      for (int i = 0; i < FFT_TEST_COMP_SAMPLES_LEN / 2; i++)
      {
        sum += abs(input_signal[i] - input_signal[i + FFT_TEST_COMP_SAMPLES_LEN / 2]);
      }
      bleSerial.println("possible:  sum " + String(sum) + ", " + String(frequency_of_max_bin) + " hz");

      if (sum < 57)
      {

        // 57 threshold passes through 20% noise
        bleSerial.println("valid key detected!");
        key_sensed = true;
      }
      else
      {
        bleSerial.println("not detected");
        key_sensed = false;
      }

      // Serial.println("poss2 " + String(frequency_of_max_bin) + " Hz ons" + String(read1) + "sample rate " + String(frequency * 1000) + " hz" + "percentage " + String(percentage) + "max value " + String(max_value) + "sum " + String(sum));

      // Serial.println(sum);

      // if(sum > 0.7){

      //   Serial.println("detected");

      // } else {

      //   Serial.println("not detected");

      // }

      Serial.println("poss " + String(frequency_of_max_bin) + " Hz ons" + String(read1) + "sample rate " + String(frequency * 1000) + " hz" + "percentage " + String(percentage) + "max value " + String(max_value));
      // this eqates to 1 second of cooldown
      // bleSerial.println("poss " + String(frequency_of_max_bin) + " Hz ons" + String(read1) + "sample rate " + String(frequency * 1000) + " hz");

      String log = "{\"f\":" + String(frequency_of_max_bin) + ",\"s\":" + String(frequency * 1000) + ",\"p\":" + String(percentage) + ",\"m\":" + String(max_value) + ",\"i\":" + String(max_val_index) + "}";
      // bleSerial.println(log);
      cool_down = 20;

      for (int i = 64; i < (FFT_TEST_OUT_SAMPLES_LEN); i++)
      {
        Serial.println(m_fft_output_f64[i]);
      }

      if (!key_calibration_state == KeyCalibrationState::KEY_CALIBRATION_STATE_CALIBRATING)
      {

        for (int i = 0; i < 64; i++)
        {
          ftt_of_ref[i] = m_fft_output_f64[i + 64];
        }

        key_calibration_state = KeyCalibrationState::KEY_CALIBRATION_STATE_CALIBRATING_STEP_1;
        calibrateCharacteristic.setValue(key_calibration_state + 48);
        Serial.println("calibrating 0");

        // add the current fft to the calibration data
      }
      else if (key_calibration_state == KeyCalibrationState::KEY_CALIBRATION_STATE_CALIBRATING_STEP_1)
      {
        for (int i = 0; i < 64; i++)
        {
          ftt_of_ref[i] = (m_fft_output_f64[i + 64] + ftt_of_ref[i]) / 2;
        }

        key_calibration_state = KeyCalibrationState::KEY_CALIBRATION_STATE_CALIBRATING_STEP_2;
        calibrateCharacteristic.setValue(key_calibration_state + 48);
        Serial.println("calibrating 1");
      }
      else if (key_calibration_state == KeyCalibrationState::KEY_CALIBRATION_STATE_CALIBRATING_STEP_2)
      {
        for (int i = 0; i < 64; i++)
        {
          ftt_of_ref[i] = (m_fft_output_f64[i + 64] + ftt_of_ref[i]) / 2.0;
        }

        // for (int i = 0; i < 64; i++)
        // {
        //   Serial.println(ftt_of_ref[i]);
        // }

        float32_t max_value_tmp;
        uint32_t max_val_index_tmp;

        arm_max_f32(&ftt_of_ref[0], 64, &max_value_tmp, &max_val_index_tmp);

        max_val_index_ref = max_val_index_tmp;
        max_value_ref = max_value_tmp;

        Serial.println("max value ref: " + String(max_value_ref));
        Serial.println("max index ref: " + String(max_val_index_ref));

        Serial.println("calibrated");

        key_calibration_state = KeyCalibrationState::KEY_CALIBRATION_STATE_CALIBRATED_STEP_DONE;
        calibrateCharacteristic.setValue(key_calibration_state + 48);
      }
      else if (key_calibration_state == KeyCalibrationState::KEY_CALIBRATION_STATE_CALIBRATED_STEP_DONE &&
               max_val_index_scaled >= (max_val_index_ref - 2) && max_val_index_scaled <= (max_val_index_ref + 2))

      {
        // Serial.println("FFT OUTPUT start");

        // for (int i = 0; i < 64; i++)
        // {
        //   Serial.println(m_fft_output_f64[i + 64]);
        // }

        float32_t sum = 0;

        for (int i = 0; i < 64; i++)
        {
          m_fft_output_f64[i + 64] = sqrt(m_fft_output_f64[i + 64]) - sqrt(ftt_of_ref[i]);

          // square the result

          m_fft_output_f64[i + 64] = m_fft_output_f64[i + 64] * m_fft_output_f64[i + 64];

          // sum the result

          sum = sum + m_fft_output_f64[i + 64];
        }

        // take the square root of the sum

        sum = sqrt(sum);

        // print

        // Serial.println("FFT OUTPUT end " + String(sum));

        if (sum <= hellinger_threshold)
        {

          Serial.println("VALID " + String(frequency_of_max_bin) + " Hz ons" + String(read1) + "sampled at " + String(frequency * 1000.0) + " hz");
          // print max bin and value
          Serial.println("Max bin: " + String(max_val_index_scaled) + " value: " + String(max_value_scaled));
          // print index of max bin and value

          // calculate the matching percentage with how close the sum is to 0

          float32_t matching_percent = (hellinger_threshold - sum) / hellinger_threshold * 100;

          Serial.println("Matching " + String(matching_percent) + "%");
        }
        else if (sum > hellinger_threshold)
        {

          Serial.println("INVALID 2");
        }
      }
    }
  }

  return true;
}

static inline uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to)
{
  if (from == to)
  {
    return value;
  }

  if (from > to)
  {
    return value >> (from - to);
  }
  else
  {
    return value << (to - from);
  }
}

uint32_t fastAnalogRead(uint32_t ulPin = 4)
{
  uint32_t pin = ADC_CONFIG_PSEL_AnalogInput5;
  uint32_t adcResolution;
  int16_t value;

  ulPin = g_ADigitalPinMap[ulPin];

  adcResolution = ADC_CONFIG_RES_9bit;

  NRF_ADC->ENABLE = 1;

  uint32_t config_reg = 0;

  config_reg |= ((uint32_t)adcResolution << ADC_CONFIG_RES_Pos) & ADC_CONFIG_RES_Msk;
  config_reg |= ((uint32_t)adcPrescaling << ADC_CONFIG_INPSEL_Pos) & ADC_CONFIG_INPSEL_Msk;
  config_reg |= ((uint32_t)adcReference << ADC_CONFIG_REFSEL_Pos) & ADC_CONFIG_REFSEL_Msk;

  if (adcReference & ADC_CONFIG_EXTREFSEL_Msk)
  {
    config_reg |= adcReference & ADC_CONFIG_EXTREFSEL_Msk;
  }

  NRF_ADC->CONFIG = ((uint32_t)pin << ADC_CONFIG_PSEL_Pos) | (NRF_ADC->CONFIG & ~ADC_CONFIG_PSEL_Msk);

  NRF_ADC->CONFIG = config_reg | (NRF_ADC->CONFIG & ADC_CONFIG_PSEL_Msk);

  NRF_ADC->TASKS_START = 1;

  while (!NRF_ADC->EVENTS_END)
    ;
  NRF_ADC->EVENTS_END = 0;

  value = (int32_t)NRF_ADC->RESULT;

  NRF_ADC->TASKS_STOP = 1;

  NRF_ADC->ENABLE = 0;

  return value;
}

void roll_signal(q7_t *signal, int n, int shift)
{
  q7_t temp[n];
  int abs_shift = abs(shift) % n;

  if (shift > 0)
  {
    memcpy(temp, signal + n - abs_shift, abs_shift * sizeof(q7_t));
    memmove(signal + abs_shift, signal, (n - abs_shift) * sizeof(q7_t));
    memcpy(signal, temp, abs_shift * sizeof(q7_t));
  }
  else if (shift < 0)
  {
    memcpy(temp, signal, abs_shift * sizeof(q7_t));
    memmove(signal, signal + abs_shift, (n - abs_shift) * sizeof(q7_t));
    memcpy(signal + n - abs_shift, temp, abs_shift * sizeof(q7_t));
  }
}

bool isLimitSensorTriggered(int pin, int sensorIndex)
{
  static bool lastState[2] = {HIGH, HIGH};
  static unsigned long lastChange[2] = {0, 0};
  bool currentState = digitalRead(pin);

  if (currentState != lastState[sensorIndex])
  {
    if (millis() - lastChange[sensorIndex] > debounceDelay)
    {
      lastState[sensorIndex] = currentState;
    }
    lastChange[sensorIndex] = millis();
  }

  return lastState[sensorIndex] == HIGH;
}

// void motor1Forward()
// {
//   digitalWrite(MOTOR1_PIN1, HIGH);
//   digitalWrite(MOTOR1_PIN2, LOW);
// }

// void motor1Reverse()
// {
//   digitalWrite(MOTOR1_PIN1, LOW);
//   digitalWrite(MOTOR1_PIN2, HIGH);
// }

// void motor2Forward()
// {
//   digitalWrite(MOTOR2_PIN1, HIGH);
//   digitalWrite(MOTOR2_PIN2, LOW);
// }

// void motor2Reverse()
// {
//   digitalWrite(MOTOR2_PIN1, LOW);
//   digitalWrite(MOTOR2_PIN2, HIGH);
// }
void motor1Forward()
{
  digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, HIGH);
}

void motor1Reverse()
{
  digitalWrite(MOTOR1_PIN1, HIGH);
  digitalWrite(MOTOR1_PIN2, LOW);
}

void motor2Forward()
{
  digitalWrite(MOTOR2_PIN1, LOW);
  digitalWrite(MOTOR2_PIN2, HIGH);
}

void motor2Reverse()
{
  digitalWrite(MOTOR2_PIN1, HIGH);
  digitalWrite(MOTOR2_PIN2, LOW);
}


void stopMotor1()
{
  digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, LOW);
}

void stopMotor2()
{
  digitalWrite(MOTOR2_PIN1, LOW);
  digitalWrite(MOTOR2_PIN2, LOW);
}

void stopAllMotors()
{
  stopMotor1();
  stopMotor2();
}

float batteryVoltage()
{
  return analogRead(BATTERY);
}

void onBatteryVoltageCharacteristicSubscribed(BLECentral &central, BLECharacteristic &characteristic)
{
  Serial.println("onBatteryVoltageCharacteristicSubscribed");
  batteryVoltageCharacteristic.setValueLE(batteryVoltage());
  bleSerial.println("Battery voltage: " + String(batteryVoltage()));
}
