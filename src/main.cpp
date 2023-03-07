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
static const uint8_t _MOSI = PIN_SPI_MOSI;
static const uint8_t _MISO = PIN_SPI_MISO;
static const uint8_t _SCK = PIN_SPI_SCK;
static const uint8_t FDATA = PIN_A2;
static const uint8_t ADATA = PIN_A3;

MyGenericSPI hardware_spi2; // using default constructor spi settings
ISRMX isrmx(_SS, NIRQ, hardware_spi2);
BLESerial bleSerial = BLESerial(BLE_REQ, BLE_RDY, BLE_RST);

#define DO_SLEEP 0
#define RNG_ENABLED 1

static const uint8_t AES_KEY[16] = {'c', 'Q', 'f', 'T', 'j', 'W', 'n', 'Z', 'r', '4', 'u', '7', 'x', '!', 'z', '%'};
static nrf_ecb_hal_data_t m_ecb_data;
unsigned char challenge[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};
char solution[33]; // solution to challenge

uint8_t Auth = 0; // 0 = not authenticated, 1 = first challenge solved , 3 = authenticated / second challenge solved

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

KeyCalibrationState key_calibration_state = KEY_CALIBRATION_STATE_NOT_CALIBRATED;

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
BLECharCharacteristic calibrateChangeModeCharacteristic("00002a3d-0000-1000-8000-00805f9b34ff", BLERead | BLEWrite | BLENotify);


void blePeripheralDisconnectHandler(BLECentral &central);
void blePeripheralRemoteServicesDiscoveredHandler(BLECentral &central);
void blePeripheralConnectHandler(BLECentral &central);
void blinkLed(int times, int timedelay);
void solutionCharacteristicWritten(BLECentral &central, BLECharacteristic &characteristic);
void calibrateChangeModeWritten(BLECentral &central, BLECharacteristic &characteristic);

uint32_t getRandom32();
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

// timer
Timer<> fft_timer;

void setup()
{
  Serial.begin(115200);

  bleSerial.setAppearance(BLE_APPEARANCE_GENERIC_REMOTE_CONTROL);
  bleSerial.setLocalName("KeyBot_000000000000");
  bleSerial.setAdvertisedServiceUuid(keybotService.uuid());

  bleSerial.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  bleSerial.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
  bleSerial.setEventHandler(BLERemoteServicesDiscovered, blePeripheralRemoteServicesDiscoveredHandler);
  // listen for write events
  solutionCharacteristic.setEventHandler(BLEWritten, solutionCharacteristicWritten);
  calibrateChangeModeCharacteristic.setEventHandler(BLEWritten, calibrateChangeModeWritten);
  bleSerial.addAttribute(keybotService);
  bleSerial.addAttribute(challengeCharacteristic);
  bleSerial.addAttribute(challengeDescriptor);
  bleSerial.addAttribute(solutionCharacteristic);
  bleSerial.addAttribute(solutionDescriptor);
  bleSerial.addAttribute(authCharacteristic);
  bleSerial.addAttribute(authDescriptor);
  bleSerial.addAttribute(calibrateCharacteristic);
  bleSerial.addAttribute(CalibrationDescriptor);
  bleSerial.addAttribute(calibrateChangeModeCharacteristic);


  authCharacteristic.setValue('0');
  calibrateCharacteristic.setValue(key_calibration_state+48);
  calibrateChangeModeCharacteristic.setValue('0');
  bleSerial.begin();

  // set key
  memcpy(m_ecb_data.key, AES_KEY, 16);

  pinMode(LED_PIN, OUTPUT);
  // setup S3NSOR
  pinMode(FDATA, INPUT);
  pinMode(ADATA, INPUT);

  int ok = isrmx.init();

  if (ok)
  {
    Serial.println("isrmx init ok");
  }
  else
  {
    Serial.println("isrmx init failed");
  }

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

  Serial.println(F("BLE ready"));

  key_calibration_state = KEY_CALIBRATION_STATE_CALIBRATING;
  calibrateCharacteristic.setValue(key_calibration_state+48);


  blinkLed(1, 1000);
}

void loop()
{
  fft_timer.tick(); // read data from sensor

// digitalWrite(LED_PIN, LOW);
// digitalWrite(LED_PIN, HIGH);
// digitalWrite(LED_PIN, LOW);//40us


  

  if (DO_SLEEP)
  {

    Serial.println(F("Sleep"));

    sd_app_evt_wait();
    Serial.println(F("WakeUp"));
    sd_nvic_ClearPendingIRQ(SWI2_IRQn);
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

void calibrateChangeModeWritten(BLECentral &central, BLECharacteristic &characteristic){

  unsigned char buffer = characteristic.value()[0];
  Serial.println("calibrateChangeModeWritten");
  Serial.println(buffer);
  cool_down = 40;

  //reset calibration
  bool ok = isrmx.change_modulation(buffer==48?1:2);
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

  blinkLed(5, 200);
}
void blePeripheralDisconnectHandler(BLECentral &central)
{
  Serial.print(F("Disconnected event, central: "));
  Serial.println(central.address());
  blinkLed(1, 200);

  Auth = 0;
}

void blePeripheralRemoteServicesDiscoveredHandler(BLECentral &central)
{
  Serial.print(F("Remote Services Discovered event, central: "));
  Serial.println(central.address());
  blinkLed(1, 200);
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

bool fft(void *opaque)
{

  if (cool_down > 0)
  {
    cool_down--;
    return true;
  }


  int read1 = 0;
  unsigned long begin = millis();

  //digitalWrite(LED_PIN, HIGH);

 

  for (int i = 0; i < (256 - 1UL); i += 2)
  {

    //4us for one loop iteration
  
    key_signal_tmp = fastAnalogRead(FDATA); // TODO switch to ADATA  #80us penalty at 10 bit resolution 
    //key_signal_tmp = analogRead(FDATA); // TODO switch to ADATA  #40us penalty at 10 bit resolution
  
   

    

    m_fft_input_f64[(uint16_t)i] = (float32_t)key_signal_tmp;  //10us penalty
    m_fft_input_f64[(uint16_t)i + 1] = 0.0;


    if (key_signal_tmp > 256/2 -20) //300
    {
      read1++;
    }
    

  digitalWrite(LED_PIN, HIGH);
    //delayMicroseconds(4); //120 /115 //13
    digitalWrite(LED_PIN, LOW);  //90 us

  }


  //25ms for 128 samples = 

  unsigned long time = millis() - begin;
  // calculate sampling frequency
  float frequency = 128.0 / time;

   digitalWrite(LED_PIN, LOW);

  bool similar_num_of_1_0s = false;
  if (read1 > (FFT_TEST_OUT_SAMPLES_LEN / 2) - 10 && read1 < (FFT_TEST_OUT_SAMPLES_LEN / 2) + 10)
  {
    similar_num_of_1_0s = true;
  }
  // sampling at 5000 Hz

   for (int i = 0; i < (256 - 1UL); i += 2)
  {

    digitalWrite(LED_PIN,    m_fft_input_f64[(uint16_t)i] > 256/2 -20 ? HIGH : LOW);

    //delay for time / 128
    delayMicroseconds((int) (time / 128.0));
    
   


   //delayMicroseconds(120);
  }
 

   
//   }
//   Serial.println();

      digitalWrite(LED_PIN, LOW);


  if (similar_num_of_1_0s)
  {

    // for (int i = 0; i < (256 - 1UL); i += 2)
    // {
    //   Serial.println(m_fft_input_f64[i]);
    // }


    
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

    //Serial.println("poss " + String(frequency_of_max_bin) + " Hz ons" + String(read1));

    //check if most of the values are bellow 0.3 
    uint32_t count = 0;


     for (int i = 64; i < (FFT_TEST_OUT_SAMPLES_LEN); i++)
      {
        if (m_fft_output_f64[i] < 0.2)
        {
          count++;
        }
      } 

      //70% of the values should be bellow 0.3
      float32_t percentage = (float32_t)count / (float32_t) 64;


      float32_t max_value_scaled;
      uint32_t max_val_index_scaled;

      arm_max_f32(&m_fft_output_f64[64], 64, &max_value_scaled, &max_val_index_scaled);




// if (percentage > 0.7 &&((frequency_of_max_bin >= 2480 && frequency_of_max_bin <= 2520) || (frequency_of_max_bin >= 480 && frequency_of_max_bin <= 520) ||
       // (frequency_of_max_bin >= 980 && frequency_of_max_bin <= 1020) || (frequency_of_max_bin >= 1980 && frequency_of_max_bin <= 2020)))
    //{


    if (percentage > 0.8)
    {

      //todo generate a new clean signal of the same frequency and compare it to the signal that was read
      //match both signals on the same edge and then compare the values
      // scale them too so that they are the same size and then compare them
      //robut edge detection is needed
      //if the signal is not a square wave then it will not work
      //TODO


      
    //   for (int i = 0; i < 64; i++)
    //     {
    //       ftt_of_ref[i] = m_fft_output_f64[i + 64];
    //     }

    //     //write code that makes a square wave signal that is sampled at desired frequency in c

  

    // int sample_rate =  frequency * 1000; // Desired sampling rate (in Hz)
    // int freq =  frequency_of_max_bin; // Frequency of the square wave (in Hz)
    // int amplitude = 1; // Amplitude of the square wave

    // double ti = 0.0; // Time index variable

    // //Serial.println(key_signal_tmp);

    // // Generate and output square wave samples
    //  for (int i = 0; i < (256 - 1UL); i += 2)
    // {

    //     // Increment time index by a single sample interval
    //     ti += 1.0 / sample_rate;

    //     // Compute the value of the current sample
    //     int sample = (sin(2 * PI * freq * ti) >= 0 ? 459 : 3);

    //     m_fft_input_f64[i] = (float32_t)sample;
    //     m_fft_input_f64[i + 1] = 0.0;

    //     //Serial.print(sample);
    // }
     

    //  fft_process(m_fft_input_f64,
    //             &arm_cfft_sR_f32_len128,
    //             m_fft_output_f64,
    //             FFT_TEST_OUT_SAMPLES_LEN);

    // m_fft_output_f64[0] = 0;

    // for (int i = 0; i < (256 - 1UL); i += 2)
    // {
    //   Serial.println(m_fft_input_f64[i]);
    // }


            
        //Serial.println("poss2 " + String(frequency_of_max_bin) + " Hz ons" + String(read1) + "sample rate " + String(frequency * 1000) + " hz" + "percentage " + String(percentage) + "max value " + String(max_value) + "sum " + String(sum));





        // Serial.println(sum);

        // if(sum > 0.7){

        //   Serial.println("detected");

        // } else {

        //   Serial.println("not detected");

        // }





      



      





      

      
      Serial.println("poss " + String(frequency_of_max_bin) + " Hz ons" + String(read1) + "sample rate " + String(frequency * 1000) + " hz" + "percentage " + String(percentage) + "max value " + String(max_value));
      // this eqates to 1 second of cooldown
      //bleSerial.println("poss " + String(frequency_of_max_bin) + " Hz ons" + String(read1) + "sample rate " + String(frequency * 1000) + " hz");

      String log = "{\"f\":" + String(frequency_of_max_bin) + ",\"s\":" + String(frequency * 1000) + ",\"p\":" + String(percentage) + ",\"m\":" + String(max_value) + ",\"i\":" + String(max_val_index) + "}";
      bleSerial.println(log);
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
        calibrateCharacteristic.setValue(key_calibration_state+48);
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
          calibrateCharacteristic.setValue(key_calibration_state+48);
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
          calibrateCharacteristic.setValue(key_calibration_state+48);

      }
      else if (key_calibration_state == KeyCalibrationState::KEY_CALIBRATION_STATE_CALIBRATED_STEP_DONE &&
      max_val_index_scaled >= (max_val_index_ref - 2) && max_val_index_scaled <= (max_val_index_ref + 2))
      
      {
        //Serial.println("FFT OUTPUT start");

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

        //Serial.println("FFT OUTPUT end " + String(sum));

        if (sum <= hellinger_threshold)
        {

          Serial.println("VALID " + String(frequency_of_max_bin) + " Hz ons" + String(read1) + "sampled at " + String(frequency *1000.0) + " hz");
          //print max bin and value
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

static inline uint32_t mapResolution( uint32_t value, uint32_t from, uint32_t to )
{
  if ( from == to )
  {
    return value ;
  }

  if ( from > to )
  {
    return value >> (from-to) ;
  }
  else
  {
    return value << (to-from) ;
  }
}

uint32_t fastAnalogRead( uint32_t ulPin=4 )
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

  while(!NRF_ADC->EVENTS_END);
  NRF_ADC->EVENTS_END = 0;

  value = (int32_t)NRF_ADC->RESULT;

  NRF_ADC->TASKS_STOP = 1;

  NRF_ADC->ENABLE = 0;

  return value;
}