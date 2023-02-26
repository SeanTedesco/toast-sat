/* 
 * Title:         Weather Systems Satellite 
 * Last Updated:  March 03rd, 2022
 * Author:        Sean Tedesco 
 * Brief:         Example of a toastSat payload that utilizes the BMP180 pressure sensor
 *                  and the TMP36 temperature sensor. The time, temperature from the TMP36,  
 *                  and the temperature and pressure from the BMP180 are measured and
 *                  recorded onto an SD card. 
 *                Note: The contents of the SD card is overwritten upon reset of the Arduino.             
 */

/*****************************************************************
 * NOTES / HARDWARE SETUP
 *  - PIN CONFIGURATION IS MEANT FOR ARDUINO NANO EVERY 
 *  - TMP36 TEMPERATURE SENSORS POWERED WITH 5.0 VOLTS
 *
 *  - TEMP_SENS1 --> ANALOG PIN A6 (as INPUT) 
 *  
 *  - BMP180 SCL --> ANALOG PIN A5
 *  - BMP180 SDA --> ANALOG PIN A4
 *  
 *  - THE SD CARD MUST BE CONNECTED TO A LOGIC LEVEL SHIFTER 
 *  - SD CARD CSN  --> LV1 --> HV1 --> DIGITAL PIN 10
 *  - SD CARD MOSI --> LV1 --> HV1 --> DIGITAL PIN 11
 *  - SD CARD MISO --> LV1 --> HV1 --> DIGITAL PIN 12
 *  - SD CARD SCLK --> LV1 --> HV1 --> DIGITAL PIN 13
 *  - ARDUINO 3.3V --> LV
 *  - ARDUINO 5.0V --> HV
 *  - ARDUINO GND  --> GND
 *
 *  - STATUS LED (GREEN) --> DIGITAL PIN 2
 *
 * Requirements:
 *  BMP180 Library: https://learn.sparkfun.com/tutorials/bmp180-barometric-pressure-sensor-hookup-/all#res
 */

/*****************************************************************
* USER INPUT
*/
#define DEBUG       1  // set to 0 to disable debug traces
#define OVERWRITE   1  // set to 0 to disable overwriting

/*****************************************************************
 * INCLUDES
 */
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <SFE_BMP180.h>

/*****************************************************************
 * PIN SETUP and GLOBAL VARIABLES
 */
#if DEBUG
  #define debug_serial_begin(...) Serial.begin(__VA_ARGS__);
  #define debug_print(...)    Serial.print(__VA_ARGS__)
  #define debug_write(...)    Serial.write(__VA_ARGS__)
  #define debug_println(...)  Serial.println(__VA_ARGS__)
#else
  #define debug_serial_begin(...)
  #define debug_print(...)
  #define debug_write(...)
  #define debug_println(...)
#endif

// pin specifications
const int led_green_pin = 2;  // output
const int sd_card_csn = 10;   // output
const int temp_sens_pin = A6; // input

// tmp36 temperature sensor
int analog_input;
float temp_average, raw_voltage, temp_celsius;

// bmp180 pressure sensor
SFE_BMP180 pressure_sensor;
char status;
double T, P, kpa, baseline;

// sd card reader / writer
File myFile;
unsigned long time_stamp; 
unsigned long seconds_since_boot;

/*****************************************************************
 * FUNCTION DECLARATIONS
 */
double getPressure(void);

/*****************************************************************
 * SETUP 
 */
void setup(){

  // open serial communications 
  debug_serial_begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  
  //turn on the status LEDs to let the user know the system is working
  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(led_green_pin, OUTPUT);
  digitalWrite(led_green_pin, HIGH);
  debug_println("*** ToastSat System Started ***");
  
  // set up the temperature sensor
  pinMode(temp_sens_pin, INPUT);
  debug_println("initializing tmp36 sensor... initialization done.");

  // set up the pressure sensor
  debug_print("initializing bmp180 sensor... ");
  if (!pressure_sensor.begin()){
    debug_println("initialization failed!");
    digitalWrite(led_green_pin, LOW);
    while(1);
  }
  debug_println("initialization done.");

  // Get the baseline pressure
  baseline = getPressure();

  // set up the sd card reader / writer  
  debug_print("Initializing SD card... ");
  if (!SD.begin(sd_card_csn)) {
    debug_println("initialization failed!");
    digitalWrite(led_green_pin, LOW);
    while (1);
  }
  debug_println("initialization done.");

  if (SD.exists("weather.csv") && OVERWRITE){
    SD.remove("weather.csv");
    debug_println("removed old weather file.");
  }

  myFile = SD.open("weather.csv", FILE_WRITE);
    // if the file opened properly, write to it:
    if (myFile) {
      debug_print("writing to weather.csv... ");
      myFile.println("time(s), TMP36(C),BMP180(C), Pressure(kPa)");
      // close the file:
      myFile.close();
      debug_println("done.");
    } else {
      // if the file didn't open, print an error:
      debug_println("error opening weather.csv");
      digitalWrite(led_green_pin, LOW);
      while (1);
    }
  delay(5000);
  digitalWrite(led_green_pin, LOW);
}


/*****************************************************************
 * LOOP
 */
void loop(){

  digitalWrite(led_green_pin, LOW);
  delay(50);
  digitalWrite(led_green_pin, HIGH);
  delay(50);
  digitalWrite(led_green_pin, LOW);
  delay(50);
  digitalWrite(led_green_pin, HIGH);
  delay(50);
  digitalWrite(led_green_pin, LOW);

  time_stamp = millis();
  seconds_since_boot = time_stamp / 1000;
    
  // read the analog input at each sensor
  analog_input = analogRead(temp_sens_pin);
  // convert analog input to a voltage value
  raw_voltage = analog_input * (5.0/1024.0); 
  // convert voltage value to a temperature value
  temp_celsius = (raw_voltage - 0.5) * 100;
  debug_print("temp1: ");
  debug_println(temp_celsius);

  delay(100);

  status = pressure_sensor.startTemperature();
  if (status != 0) {
    delay(status);
    status = pressure_sensor.getTemperature(T);
  }
  debug_print("temp2: ");
  debug_println(T);

  delay(100);
  P = getPressure();
  kpa = P*0.1;
  debug_print("press: ");
  debug_println(kpa);

  // save this loop's collected data to the sd-card by printing the data to in .csv formatting 
  myFile = SD.open("weather.csv", FILE_WRITE);
  if (myFile) {
    debug_print("writing to weather.csv... ");
    myFile.print(seconds_since_boot);
    myFile.print(",");
    myFile.print(temp_celsius);
    myFile.print(",");
    myFile.print(T);
    myFile.print(",");
    myFile.println(kpa);
    
    // close the file:
    myFile.close();
    debug_println("done.");
  } else {
    // if the file cannot be opened flash the LED and print an error message. 
    debug_println("error opening weather.csv");
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    delay(50);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
  }

  if (DEBUG){
      myFile = SD.open("weather.csv");
    if (myFile) {
      debug_println("contents of weather.csv:");
      while (myFile.available()) {
        debug_write(myFile.read());
      }
      myFile.close();
    } else {
      debug_println("error opening weather.csv");
      digitalWrite(LED_BUILTIN, HIGH);
      delay(50);
      digitalWrite(LED_BUILTIN, LOW);
      delay(50);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(50);
      digitalWrite(LED_BUILTIN, LOW);
    }
  }

  delay(1000);
}

/*****************************************************************
 * FUNCTIONS
 */

/*/*****************************************************************
 * @brief: gets the current pressure value as measured by the BMP180 sensor. 
 * @param: void
 * @return: pressure measurement in hPa, floating point number. 
 */
double getPressure(void) {
  status = pressure_sensor.startTemperature();
  if (status != 0) {
    delay(status);
    status = pressure_sensor.getTemperature(T);
    if (status != 0) {
      status = pressure_sensor.startPressure(3);
      if (status != 0) {
        delay(status);
        status = pressure_sensor.getPressure(P,T);
        if (status != 0) {
          return(P);
        } 
        else debug_println("error retrieving pressure measurement.");
      }
      else debug_println("error starting pressure measurement.");
    }
    else debug_println("error retrieving temperature measurement.");
  }
  else debug_println("error starting temperature measurement.");
}
