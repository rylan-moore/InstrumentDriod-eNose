/**
 * @file eNose.ino
 * @author Rylan Moore (rylan.moore@colorado.edu)
 * @brief Enose project
 * @version 1.1
 * @date 2022-11-7
 * 
 * @copyright Copyright (c) 2022
 * 
 * @note  Sensors:
 *        MQ-135/2/4/8/6
 *        REF - 20k Vishay Resistor
 *        BME688 - 0x76/0x77
 *        ADS1115 - 0x48, 0x49, 0x4A, 0x4B
 *        
 *      MCU:
 *        QT PY SAMD21-E18A
 * 
 * Current version collects data from all sensors and reports it over serial to a host. 
 */



#include "bsec.h"
#include <Adafruit_ADS1X15.h>

//test define 
#define enose_calib    true //This test will enable the serial output, basically all the code does now. 

// Helper functions declarations
void checkIaqSensorStatus(void);
void errLeds(void);

// Create an object of the class Bsec -- BME 688
Bsec iaqSensor;

String output; //used for serial output

Adafruit_ADS1115 ads1; //0x48
Adafruit_ADS1115 ads2; //0x49
Adafruit_ADS1115 ads3; //0x4b
Adafruit_ADS1115 ads4; //ox4a

/*Sensor Table ## This can be updated without needing to change any vars. The only thing to change is the Serial Print HEADER
s1 MQ135
s2 MQ2
s3 MQ4 
S4 MQ8 
s5 k20r
s6 MQ6 
*/


//info for the Sensor 1
  #define S1_ADC    ads1 //define which adc Sensor 1 is on
  #define S1_RS_PIN  1 //channel on the adc that the sense resistor is on
  #define S1_RH_PIN  0 //channel on the adc that the heat sense resistor is on

//info for the Sensor 2
  #define S2_ADC     ads1 //define which adc Sensor 2 is on
  #define S2_RS_PIN  3 //channel on the adc that the sense resistor is on
  #define S2_RH_PIN  2 //channel on the adc that the heat sense resistor is on

//info for the sensor 3
  #define S3_ADC     ads2 //define which adc the Sensor 3 is on
  #define S3_RS_PIN  1 //channel on the adc that the sense resistor is on
  #define S3_RH_PIN  0 //channel on the adc that the heat sense resistor is on

//info for the Sensor 4
  #define S4_ADC     ads2 //define which adc the Sensor 4 is on 
  #define S4_RS_PIN  3 //channel on the adc that the sense resistor is on
  #define S4_RH_PIN  2 //channel on the adc that the heat sense resistor is on

//info for the Sensor 5
  #define S5_ADC     ads3 //define which adc the Sensor 5 is on
  #define S5_RS_PIN  1 //channel on the adc that the sense resistor is on
  #define S5_RH_PIN  0 //channel on the adc that the heat sense resistor is on

//info for the Sensor 6
  #define S6_ADC     ads3 //define which adc the Sensor 6 is on
  #define S6_RS_PIN  3 //channel on the adc that the sense resistor is on
  #define S6_RH_PIN  2 //channel on the adc that the heat sense resistor is on

//end sensor define section
  #define VSS_ADC       ads4
  #define VSS_PIN       0

//i2c devices defines
#define ADC1_ADDR       0x48
#define ADC2_ADDR       0x49
#define ADC3_ADDR       0x4b
#define ADC4_ADDR       0x4a

//for heater use
int heat_i = 0; //NOT USED
#define RsH   1.5     //sense resistors on the gas sensor heaters. NO LONGER USED 
#define RsS   20000   //sense resistors on the gas sensors. 
float Vcc=5.0;       //heater voltage, will be re-calculated each run
float Vss=2.5;        //starting sense resistor voltage, will be calculated each run


const int REF_INTERVAL = 1000; //want a sample every 1000ms reported back over serial.
unsigned long lastRefresh = 0;
unsigned long test_start = 0;
const unsigned long test_duration = 10000000; //get 10k seconds of data from start of serial monitoring
const int num_averages = 10;

// Entry point for the example
void setup(void)
{
  Serial.begin(115200);

  Wire.begin();

  /*check i2c devices are on bus and addresses correctly*/

  //check status of the adcs 
  if (!ads1.begin(ADC1_ADDR, &Wire)){
    Serial.println("Failed to find ADC1");
    while(1){
      delay(10);
    }
  }
  if (!ads2.begin(ADC2_ADDR, &Wire)){
    Serial.println("Failed to find ADC2");
    while(1){
      delay(10);
    }
  }
  if (!ads3.begin(ADC3_ADDR, &Wire)){
    Serial.println("Failed to find ADC3");
    while(1){
      delay(10);
    }
  }
  if (!ads4.begin(ADC4_ADDR, &Wire)){
    Serial.println("Failed to find ADC4");
    while(1){
      delay(10);
    }
  }
  /*end checking i2c devices*/

  iaqSensor.begin(BME680_I2C_ADDR_SECONDARY, Wire);
  checkIaqSensorStatus();

  bsec_virtual_sensor_t sensorList[10] = {
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  };

  iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
  checkIaqSensorStatus();

  ads1.setGain(GAIN_TWOTHIRDS); //set original gain on each of the adc to avoid accidentally being in high gain
  ads2.setGain(GAIN_TWOTHIRDS); 
  ads3.setGain(GAIN_TWOTHIRDS); 
  ads4.setGain(GAIN_ONE);

  #if enose_calib
    while(!Serial)
  #endif
  /*Send the header row for excel import later*/
  output = "time, temp, humidity,VoC,Pressure,CO2, MQ135, MQ2, MQ4, MQ8, MQ6, 20k1, Vss, Sample_Time"; //Note: Update for sensor changes
  Serial.println(output);
  /* Set the gain values on all the ADC devices, change from default*/
  S1_ADC.setGain(GAIN_TWO);
  S3_ADC.setGain(GAIN_TWO);
  S5_ADC.setGain(GAIN_TWO);

  /*Start the test timer, which will then run for 10k seconds*/
  test_start = millis();
}

// Function that is looped forever
void loop(void)
{
  unsigned long current = millis(); //This will store the current time at the start of the loop
  
  /*Enter the if statement when it has been one second between sample runs, and less time than total test time*/
  if ( (current - lastRefresh >= REF_INTERVAL) && ((current - test_start) < test_duration)){

    iaqSensor.run(); //Tell the BME688 to collect data
    String output; //Output string to print
    float voltage; //Float var used for calculation throughout this loop

    float s1 = 0, s2 = 0, s3 = 0, s4 = 0, s5 = 0, s6 = 0; //Float vars used for sensors 1-6 these are changed throughout the loop
    Vss = 0;
    unsigned long avg_start = millis(); //collect the CPU time at the start of these measurements to count time to take 70 samples
    for (int i = 0; i< num_averages; i++){ //collect the three data points per sensor
      Vss += VSS_ADC.readADC_SingleEnded(VSS_PIN); //read the supply voltage at the start of each half second interval.
      s1 += S1_ADC.readADC_SingleEnded(S1_RS_PIN);
      s2 += S2_ADC.readADC_SingleEnded(S2_RS_PIN);
      s3 += S3_ADC.readADC_SingleEnded(S3_RS_PIN);
      s4 += S4_ADC.readADC_SingleEnded(S4_RS_PIN);
      s5 += S5_ADC.readADC_SingleEnded(S5_RS_PIN);
      s6 += S6_ADC.readADC_SingleEnded(S6_RS_PIN);
    }
    unsigned long avg_end = millis(); //collect the CPU time at the end of the for loop to count sample time. 

    s1 = s1/num_averages; //calculate the average for each sensor ADC value. 
    s2 = s2/num_averages;
    s3 = s3/num_averages;
    s4 = s4/num_averages;
    s5 = s5/num_averages;
    s6 = s6/num_averages;

    Vss = Vss /num_averages; //Compute the averaged VSS value from the 2.5V refrence
    Vss = VSS_ADC.computeVolts(Vss); //convert the measurement to volts

    output = String((current/1000))+ "," + String(iaqSensor.temperature)+ "," + String(iaqSensor.humidity)+","+ String(iaqSensor.breathVocEquivalent)+ "," + String(iaqSensor.pressure)+"," +String(iaqSensor.co2Equivalent);
    
    /*Compute Sensor 1 voltage*/
    voltage = S1_ADC.computeVolts(s1);
    voltage = ((Vss - voltage) * RsS )/ voltage;
    output += "," + String(voltage,3);

    /*Compute Sensor 2 voltage*/
    voltage = S2_ADC.computeVolts(s2);
    voltage = ((Vss - voltage) * RsS )/ voltage;
    output += "," + String(voltage,3);

    /*Compute Sesnor 3 voltage*/
    voltage = S3_ADC.computeVolts(s3);
    voltage = ((Vss - voltage) * RsS )/ voltage;
    output += "," + String(voltage,3);

    /*Compute Sensor 4 voltage*/
    voltage = S4_ADC.computeVolts(s4);
    voltage = ((Vss - voltage) * RsS )/ voltage;
    output += "," + String(voltage,3);

    /*Compute Sensor 5 voltage*/
    voltage = S5_ADC.computeVolts(s5);
    voltage = ((Vss - voltage) * RsS )/ voltage;
    output += "," + String(voltage,3);

    /*Compute Sensor 6 voltage*/
    voltage = S6_ADC.computeVolts(s6);
    voltage = ((Vss - voltage) * RsS )/ voltage;
    output += "," + String(voltage,3 ) + "," + String(Vss, 4);

    /*Compute the data collection time in ms*/
    output += "," + String((avg_end - avg_start));
    Serial.println(output);

    lastRefresh = current; 
  }
  else if ((current - test_start) > test_duration){ //if the alotted test time has stopped. 
    Serial.println("Done with test");
    while(1);
  }

  
}

// Helper function definitions
void checkIaqSensorStatus(void){
  if (iaqSensor.status != BSEC_OK) {
    if (iaqSensor.status < BSEC_OK) {
      output = "BSEC error code : " + String(iaqSensor.status);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      output = "BSEC warning code : " + String(iaqSensor.status);
      Serial.println(output);
    }
  }

  if (iaqSensor.bme680Status != BME680_OK) {
    if (iaqSensor.bme680Status < BME680_OK) {
      output = "BME680 error code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      output = "BME680 warning code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
    }
  }
}

void errLeds(void){} //Functioin not used anymore
