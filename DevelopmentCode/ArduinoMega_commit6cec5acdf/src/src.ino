// TODO: add possibility of a small LCD screen

// TODO: find a solution to the slow data transfer arduino / RPi.
//       try to do by chunks of 256 bytes of data (if arduino tx is 512)
// TODO: use better methodology with the SD card: writing by chunks of data (see post google "fast sd card author")
// TODO: possible that some VN100 frames are lost; check that size of buffers is ok as set in the platformio.ini file.

// TODO: go once through the code to chek all
// TODO: change the SD card code: separate update of file number and opening of files
// TODO: add debugging to the SD card to indicate where in the process things are happening

// TODO: Avoid #define for constant
// TODO: function macros use () around
// TODO: rationalize debug messages
// TODO: macro define a debugprint and debugprintln function
// TODO: implement RPi interface
// TODO: got through all delays and clean / remove them
// TODO: add information about board status (battery) in SD log files
// go through watchdogs

// TODO: add a SD manager keeper, that erases old files to avoid to fill the memory

// TODO: attach a serial port for debugging
// TODO: interaction with Raspberry Pi
// TODO: Arduino board manager (sleeps, wake up, feedback)
// TODO: prepare VN100 IMU
// TODO: Iridium manager
// TODO: RaspberryPi manager
// TODO: to reduce energy consumption, make the board sleep if nothing to log
// TODO: take a look at the watchdogs
// TODO: check the ISBD callback routine wrt watchdog
// TODO: could use struct(char *, str_length) instead of plain char *. I do not want to use strings
// as dynamically allocated, ok on computers, bad on mC.

// TODO: could put the VN100, GPS, SD and Iridium on a MOSFET (physical change to board)

// NOTE: cut the capacitor that prevents reboot when RPi opens serial

#include "parameters.h"
#include "GPSManager.h"
#include "EEPROM_interaction.h"
#include "SDManager.h"
#include "BoardManager.h"
#include "IridiumManager.h"
#include "VN100Manager.h"
#include "RaspberryManager.h"

//////////////////////////////////////////////
// General
#include <Arduino.h>
#include <Wire.h>
uint32_t timer = millis();
int count = 0; //define counter

static const uint8_t Pin_reset = 3; //TCA9548A reset pin
const int count_nr = 90; //Number of sensor measurements

// Temperature sensors
#include "TSYS01.h"
TSYS01 sensor;
float T1;
float T2;
float T3;
float T1_ave = 0;
float T2_ave = 0;
float T3_ave = 0;
int T1_count = 0;
int T2_count = 0;
int T3_count = 0;

// Pressure sensor
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
Adafruit_BMP3XX bmp;

//#include <Adafruit_LPS35HW.h>
//Adafruit_LPS35HW LPS35HW = Adafruit_LPS35HW();
float P1;
float P1_ave;
int P1_count;
float P1T;
float P1T_ave;
int P1T_count;

// Wind anemometer
#define WindSensorPin (2) //define pin of wind sensor
uint32_t time1; //define start of measurement
uint32_t time2; //define end of measurement
volatile unsigned long Rotations; //measurement of nr of anemometer rotations
volatile unsigned long ContactBounceTime;
float WindSpeed; //Wind speed measurement

// Sonar ping
#include "ping1d.h"
#include "SoftwareSerial.h"
static const uint8_t arduinoRxPin = 10; //Serial1 rx (white)
static const uint8_t arduinoTxPin = 11; //Serial1 tx (green)
SoftwareSerial pingSerial = SoftwareSerial(arduinoRxPin, arduinoTxPin);

static Ping1D ping { pingSerial };
int count_sonar = 0; //define sonar counter
int S1; //sonar: distance
int S2; //sonar: confidence level
long S80_ave = 0L;
long S90_ave = 0L;
long S100_ave = 0L;
int S80_count = 0;
int S90_count = 0;
int S100_count = 0;
int Sonar_range = 0;
int Sonar_check;

//////////////////////////////////////////////

// board manager
BoardManager board_manager{};

// SD
SDManager sd_manager{};

// VN100
VN100Manager vn100_manager{&SERIAL_VN100, &sd_manager};

// GPS
GPSManager gps_controller{&SERIAL_GPS, &sd_manager};

// Iridium
IridiumManager iridium_manager{&SERIAL_IRIDIUM, &gps_controller, &board_manager, &sd_manager};

// RPi
RaspberryManager raspberry_manager{&board_manager, &sd_manager, &iridium_manager, &SERIAL_RASPBERRY};

// TODO: class for the VN100
// TODO: class for interaction with RaspberryPi
// TODO: not urgent: class for the LSM9DS0
// TODO: fix all names: name cpp file and header should be the same as class they contain

void setup() {

  // manage the board: decide if should wake up
  board_manager.start();
  wdt_reset();

  print_debug_status();

  // make SD card ready
  sd_manager.start_sd();
  wdt_reset();

  // make Iridium ready
  iridium_manager.start();
  wdt_reset();

  // make GPS ready
  gps_controller.start();
  wdt_reset();

  // make VN100 ready
  vn100_manager.start();
  wdt_reset();

  // raspberry Pi
  // will be started when needed

  ///////////////////////////////////////////////////

  Wire.begin();
  Wire.setClock(10000);
  Wire.setWireTimeout(10000, true);
  pinMode(Pin_reset, OUTPUT);
  digitalWrite(Pin_reset, HIGH);

  // set Sonar
  pingSerial.begin(9600);
  ping.set_speed_of_sound(1500000);
  if (ping.initialize() == true) {
    Sonar_check = 1;
  } else {
    Sonar_check = 0;
  }

  //set Pressure
  bmp.begin_I2C();
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
//  LPS35HW.begin_I2C();


  ///////////////////////////////////////////////////

  // start logging!
  board_manager.start_logging(DURATION_LOGGING_MS);
}

void loop() {

  wdt_reset();

#if DEBUG && DEBUG_SLOW
  SERIAL_DEBUG.println(F("calling loop"));
#endif

  // decide which step in the process at
  int board_status = board_manager.check_status();

  attachInterrupt(digitalPinToInterrupt(WindSensorPin), isr_rotation, FALLING); //initiate wind anemometer count
  time1 = millis();

  switch (board_status) {
    case BOARD_LOGGING:
      vn100_manager.perform_logging();
      gps_controller.perform_logging();
      break;
    case BOARD_DONE_LOGGING:
      // close SD card
      sd_manager.close_datafile();

      time2 = millis();
      WindSpeed = Rotations * (2.25 / ((25 * 60))) * 0.44704;
      detachInterrupt(digitalPinToInterrupt(WindSensorPin)); //close wind anemometer count

      ////////////////

      while (count < count_nr) {
        // take measurements every second (actually only necessary to reduce serial messages...)
        if (millis() - timer > 1000) {
          timer = millis();

          //Sensor readings:
          TCA9548A(0); sensor.init(); sensor.read();
          T1 = sensor.temperature();
//          dataFile.print(millis());
//          dataFile.print(F(","));
//          dataFile.print(T1);
          if (abs(sensor.temperature()) < 50) {
            T1 = sensor.temperature();
            T1_ave = T1_ave + T1;
            T1_count++;
          }
          ErrCheck();

          TCA9548A(1); sensor.init(); sensor.read();
          T2 = sensor.temperature();
//          dataFile.print(F(","));
//          dataFile.print(T2);
          if (abs(sensor.temperature()) < 50) {
            T2 = sensor.temperature();
            T2_ave = T2_ave + T2;
            T2_count++;
          }
          ErrCheck();

          TCA9548A(2); sensor.init(); sensor.read();
          T3 = sensor.temperature();
//          dataFile.print(F(","));
//          dataFile.print(T3);
          if (abs(sensor.temperature()) < 50) {
            T3 = sensor.temperature();
            T3_ave = T3_ave + T3;
            T3_count++;
          }
          ErrCheck();

          TCA9548A(3);
          bmp.performReading();
          P1 = bmp.pressure / 100.0;
          P1T = bmp.temperature;
//          dataFile.print(F(","));
//          dataFile.print(P1);
          if (P1 > 700 && P1 < 1200) {
            P1_ave = P1_ave + P1;
            P1_count++;
          }
          if (abs(P1T) < 50) {
            P1T_ave = P1T_ave + P1T;
            P1T_count++;
          }
          ErrCheck();

//          dataFile.print(F(","));
//          dataFile.print(Rotations);
//          dataFile.print(F(","));

//          dataFile.println(F(","));

          if (Sonar_check == 1) {
            while (count_sonar < 10) { //count_sonar can't be too big, or GPS will do difficult
              if (ping.update()) {
                S1 = ping.distance();
                S2 = ping.confidence();
//                dataFile.print(S1);
//                dataFile.print(F(","));
//                dataFile.println(S2);

                //Consider only data with confidence level of 80 and higher
                if (S2 >= 80 && S2 < 100 && S1 < 10000 && S1 > 0) {
                  S80_ave = S80_ave + S1;
                  S80_count++;
                }
                if (S2 == 100 && S1 < 10000 && S1 > 0) {
                  S100_ave = S100_ave + S1;
                  S100_count++;
                }
              }
              count_sonar++;
            }
            count_sonar = 0;
          }
          count++;
        }
        wdt_reset();
      }

      File dataFile;
      dataFile = SD.open("DATAFILE.txt", FILE_WRITE);
      dataFile.println(F("start"));

      dataFile.print(T1_ave / T1_count); dataFile.print(F(","));
      dataFile.print(T2_ave / T2_count); dataFile.print(F(","));
      dataFile.print(T3_ave / T3_count); dataFile.print(F(","));
      dataFile.print(P1_ave / P1_count); dataFile.print(F(","));
      dataFile.print(P1T_ave / P1T_count); dataFile.print(F(","));
      dataFile.print(WindSpeed); dataFile.println(F(","));

      if (S100_count > 30) {
        Sonar_range = 1;
      } else if (S80_count > 5) {
        Sonar_range = 2;
      }
//      dataFile.println(Sonar_range);
//      dataFile.println(Sonar_check);
      if (Sonar_range == 1) {
        dataFile.print(S100_ave / S100_count);dataFile.println(F(" S100"));
      } else if (Sonar_range == 2) {
        dataFile.print(S80_ave / S80_count);dataFile.println(F(" S80"));
      }
//      dataFile.print(S80_ave); dataFile.print(F(" ")); dataFile.println(S80_count);
//      dataFile.print(S100_ave); dataFile.print(F(" ")); dataFile.println(S100_count);

      long value_fileIndex = EEPROMReadlong(1);
      if (value_fileIndex < 10) {
        dataFile.print(F("F0000"));
        dataFile.println(value_fileIndex);
      }
      else if (value_fileIndex < 100) {
        dataFile.print(F("F000"));
        dataFile.println(value_fileIndex);
      }
      else if (value_fileIndex < 1000) {
        dataFile.print(F("F00"));
        dataFile.println(value_fileIndex);
      }
      else if (value_fileIndex < 1000) {
        dataFile.print(F("F0"));
        dataFile.println(value_fileIndex);
      }
      else {
        dataFile.print(F("F"));
        dataFile.println(value_fileIndex);
      }
      float battery_level_VV =  float(analogRead(PIN_MSR_BATTERY)) * 5.0 / 1024.0;
//      dataFile.print(F("Voltage"));
      dataFile.println(battery_level_VV);
      dataFile.println(" ");
      dataFile.close();
      digitalWrite(Pin_reset, LOW);

      wdt_reset();


      //////////////////

      // go through Iridium vital messages
      iridium_manager.send_receive_iridium_vital_information();
      wdt_reset();

      // go through Raspberry Pi interaction
      perform_raspberry_interaction();

      // ask to be put off
      board_manager.ask_to_be_off();

      // put to deep sleep
      board_manager.sleep_or_reboot();
      break;
  }
}

// TODO: put in raspberry_manager class
void perform_raspberry_interaction(void) {
  raspberry_manager.start();
  raspberry_manager.send_filename();
  raspberry_manager.file_content_to_raspberry();
  raspberry_manager.receive_processed_data();
  raspberry_manager.transmit_through_iridium();
  // note: crontab gets raspberry to automatically stop
}

void TCA9548A(uint8_t bus)
{
  Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}

void ErrCheck() //Check for errors
{
  if (Wire.getWireTimeoutFlag() == 1) {
    digitalWrite(Pin_reset, LOW);
    Wire.clearWireTimeoutFlag();
    digitalWrite(Pin_reset, HIGH);
  }
}

void isr_rotation () {
  if ((millis() - ContactBounceTime) > 15 ) { // debounce the switch contact.
    Rotations++;
    ContactBounceTime = millis();
  }
}
