/**
 * @file eNose.ino
 * @author Rylan Moore (rylan.moore@colorado.edu)
 * @brief Enose project
 * @version 1.1
 * @date 2022-11-7
 * 
 * @copyright Copyright (c) 2022
 * 
 * @note Sensors:
 *        MQ-135/2/3/4/8/7
 *        BME688 - 0x76/0x77
 *        ADS1115 - 0x48, 0x49, 0x4A, 0x4B
 *        
 *      MCU:
 *        QT PY SAMD21-E18A
 * 
 * Current version collects data from all sensors and reports it over serial to a host. 
 * 
 * 
 * 
 */



#include "bsec.h"
#include <Adafruit_ADS1X15.h>

//test define 
//#define i2c_test    true
#define enose_calib    true

// Helper functions declarations
void checkIaqSensorStatus(void);
void errLeds(void);

// Create an object of the class Bsec
Bsec iaqSensor;

String output; //used for serial output

Adafruit_ADS1115 ads1; //0x48
Adafruit_ADS1115 ads2; //0x49
Adafruit_ADS1115 ads3; //0x4b
Adafruit_ADS1115 ads4; //ox4a
// Adafruit_MCP4728 mcp1; //0x60
// Adafruit_MCP4728 mcp2; //0x61 (this was set already)

//begin sensor defines 

//info for the MQ135 sensor
  #define MQ135_ADC     ads1 //define which adc the MQ135 is on
  #define MQ135_RS_PIN  1 //channel on the adc that the sense resistor is on
  #define MQ135_RH_PIN  0 //channel on the adc that the heat sense resistor is on

  // #define MQ135_DAC     mcp1 //define wich dac the MQ135 heater is on
  // #define MQ135_DAC_CH  MCP4728_CHANNEL_B //channel on the dac that the heater is on

//info for the MQ-2 sensor
  #define MQ2_ADC     ads1 //define which adc the MQ135 is on
  #define MQ2_RS_PIN  3 //channel on the adc that the sense resistor is on
  #define MQ2_RH_PIN  2 //channel on the adc that the heat sense resistor is on

  // #define MQ2_DAC     mcp1 //define wich dac the MQ135 heater is on
  // #define MQ2_DAC_CH MCP4728_CHANNEL_A //channel on the dac that the heater is on

//info for the MQ-4 sensor
  #define MQ8_ADC     ads2 //define which adc the MQ135 is on
  #define MQ8_RS_PIN  1 //channel on the adc that the sense resistor is on
  #define MQ8_RH_PIN  0 //channel on the adc that the heat sense resistor is on

  // #define MQ8_DAC     mcp1 //define wich dac the MQ135 heater is on
  // #define MQ8_DAC_CH  MCP4728_CHANNEL_D //channel on the dac that the heater is on

//info for the MQ-8 sensor
  #define MQ4_ADC     ads2 //define which adc the MQ135 is on
  #define MQ4_RS_PIN  3 //channel on the adc that the sense resistor is on
  #define MQ4_RH_PIN  2 //channel on the adc that the heat sense resistor is on

  // #define MQ4_DAC     mcp1 //define wich dac the MQ135 heater is on
  // #define MQ4_DAC_CH  MCP4728_CHANNEL_C //channel on the dac that the heater is on

//info for the 20k ref sensor
  #define MQ3_ADC     ads3 //define which adc the MQ135 is on
  #define MQ3_RS_PIN  1 //channel on the adc that the sense resistor is on
  #define MQ3_RH_PIN  0 //channel on the adc that the heat sense resistor is on

  // #define MQ3_DAC     mcp2 //define wich dac the MQ135 heater is on
  // #define MQ3_DAC_CH  MCP4728_CHANNEL_A //channel on the dac that the heater is on

//info for the MQ-6 sensor
  #define MQ7_ADC     ads3 //define which adc the MQ135 is on
  #define MQ7_RS_PIN  3 //channel on the adc that the sense resistor is on
  #define MQ7_RH_PIN  2 //channel on the adc that the heat sense resistor is on

  // #define MQ7_DAC     mcp2 //define wich dac the MQ135 heater is on
  // #define MQ7_DAC_CH  MCP4728_CHANNEL_B //channel on the dac that the heater is on

//info for the IR CO2 sensor
  // #define IR_ADC        adsX //define which adc the IR CO2 sensor is on
  // #define IR_PIN        2 //channel on the adc where the IR analog in is located. 
  // #define InfaredIn     2 //This sensor is installed on the Input A2 on the ADS1115 with default address in single ended mode

//end sensor define section
  #define VSS_ADC       ads4
  #define VSS_PIN       0

//i2c devices defines
// #define DAC1_ADDR       0x60
// #define DAC2_ADDR       0x61
#define ADC1_ADDR       0x48
#define ADC2_ADDR       0x49
#define ADC3_ADDR       0x4b
#define ADC4_ADDR       0x4a

//for heater use
int heat_i = 0;
#define RsH   1.5     //sense resistors on the gas sensor heaters. 
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
  #if i2c_test
    while(!Serial) //wait for the serial connection 
  #endif

  Wire.begin();

  /*check i2c devices are on bus and addresses correctly*/
  //check the status of the dac  
  // if (!mcp1.begin(DAC1_ADDR)) {
  //   Serial.println("Failed to find DAC1");
  //   while (1) {
  //     delay(10);
  //   }
  // }
  //   if (!mcp2.begin(DAC2_ADDR)) { //start the second dac with other address
  //   Serial.println("Failed to find DAC2");
  //   while (1) {
  //     delay(10);
  //   }
  // }
  //set all of the dac channels to 0 to avoid over current. 
    // mcp1.setChannelValue(MQ135_DAC_CH, (0)); 
    // mcp1.setChannelValue(MQ2_DAC_CH, (0)); 
    // mcp1.setChannelValue(MQ8_DAC_CH, (0)); 
    // mcp1.setChannelValue(MQ4_DAC_CH, (0)); 
    // mcp2.setChannelValue(MQ3_DAC_CH, (0)); 
    // mcp2.setChannelValue(MQ7_DAC_CH, (0)); 
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
  //pinMode(A0, INPUT);

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
  //ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/false);
  // Print the header
  #if enose_calib
    while(!Serial)
  #endif
    output = "time, temp, humidity,VoC,Pressure,CO2, MQ135, MQ2, MQ4, MQ8, MQ6, 20k1, Vss, Sample_Time";

  //output = "time, temp, humidity, RH135, RS135, RH2, RS2, RH8, RS8, RH4, RS4, RH3, RS3, RH7, RS7, Vss, co2_30, co2_41, co241temp, co241humid";
  Serial.println(output);
  MQ135_ADC.setGain(GAIN_TWO); //reset gains This could likely move to void setup
  MQ8_ADC.setGain(GAIN_TWO);
  MQ3_ADC.setGain(GAIN_TWO);

  //i2c unit test. 
  #if i2c_test
    Serial.println("Finsihed i2c device test check!");
    while(1){
      delay(10);
    }
  #endif

  test_start = millis();
  //calibrate the MQ sensor. 
}

// Function that is looped forever
void loop(void)
{
  // //MQ135 gasSensor = MQ135(A0, 10.91, 22);
  // unsigned long time_trigger = millis();
  // if (iaqSensor.run()) { // If new data is available
  //   output = String(time_trigger/1000);
  //   // output += ", " + String(iaqSensor.rawTemperature);
  //   // output += ", " + String(iaqSensor.pressure);
  //   // // output += ", " + String(iaqSensor.rawHumidity);
  //   // // output += ", " + String(iaqSensor.gasResistance);
  //   // // output += ", " + String(iaqSensor.iaq);
  //   // // output += ", " + String(iaqSensor.iaqAccuracy);
  //   output += ", " + String(iaqSensor.temperature);
  //   output += ", " + String(iaqSensor.humidity);
  //   // output += ", " + String(iaqSensor.staticIaq);
  //   output += ", " + String(iaqSensor.co2Equivalent); //here!!!
  //   output += ", " + String(getCorrectedPPM(iaqSensor.temperature, iaqSensor.humidity));
  //   //output += ", " + String(getPPM());
  //   output += ", " + String(getPPMgravity()); //get the value from the Infared Sensor
  //   // output += ", " + String(iaqsSensor.breathVocEquivalent);
  //   // int16_t results = ads.getLastConversionResults();
  //   // output += ", " + String( ads.computeVolts(results)); //test the adc output 
  //   // output += ", " + String( results); //test the adc output 
  //   Serial.println(output);
  // } else {
  //   checkIaqSensorStatus();
  // }
  // //int i = ads.readADC_SingleEnded(0);
  // //ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/false);
  // mcp1.setChannelValue(MCP4728_CHANNEL_A, (2800)); //set the output of the opamp to 5v, calibrated
  unsigned long current = millis();
  if ( (current - lastRefresh >= REF_INTERVAL) && ((current - test_start) < test_duration)){

    iaqSensor.run();
    String output;
    float voltage;

    float s135 = 0, s2 = 0, s8 = 0, s4 = 0, s3 = 0, s7 = 0;
    Vss = 0;
    unsigned long avg_start = millis();
    for (int i = 0; i< num_averages; i++){ //collect the three data points per sensor
      Vss += VSS_ADC.readADC_SingleEnded(VSS_PIN); //read the supply voltage at the start of each half second interval.
      s135 += MQ135_ADC.readADC_SingleEnded(MQ135_RS_PIN);
      s2 += MQ2_ADC.readADC_SingleEnded(MQ2_RS_PIN);
      s8 += MQ8_ADC.readADC_SingleEnded(MQ8_RS_PIN);
      s4 += MQ4_ADC.readADC_SingleEnded(MQ4_RS_PIN);
      s3 += MQ3_ADC.readADC_SingleEnded(MQ3_RS_PIN);
      s7 += MQ7_ADC.readADC_SingleEnded(MQ7_RS_PIN);
    }
    unsigned long avg_end = millis();
    s135 = s135/num_averages; //calculate the average
    s2 = s2/num_averages;
    s8 = s8/num_averages;
    s4 = s4/num_averages;
    s3 = s3/num_averages;
    s7 = s7/num_averages;

    Vss = Vss /num_averages;
    Vss = VSS_ADC.computeVolts(Vss); //convert the measurement to volts
    //String(iaqSensor.temperature)
    output = String((current/1000))+ "," + String(iaqSensor.temperature)+ "," + String(iaqSensor.humidity)+","+ String(iaqSensor.breathVocEquivalent)+ "," + String(iaqSensor.pressure)+"," +String(iaqSensor.co2Equivalent);
    //output +=  "," + String(h135,3); //output the heater resistance and the sense resistance for all sensors
    voltage = MQ135_ADC.computeVolts(s135);
    voltage = ((Vss - voltage) * RsS )/ voltage;
    output += "," + String(voltage,3);

    //output += "," +String(h2,3);
    voltage = MQ2_ADC.computeVolts(s2);
    voltage = ((Vss - voltage) * RsS )/ voltage;
    output += "," + String(voltage,3);

    //output += "," + String(h8,3);
    voltage = MQ8_ADC.computeVolts(s8);
    voltage = ((Vss - voltage) * RsS )/ voltage;
    output += "," + String(voltage,3);

    //output += "," +String(h4,3);
    voltage = MQ4_ADC.computeVolts(s4);
    voltage = ((Vss - voltage) * RsS )/ voltage;
    output += "," + String(voltage,3);

    //output += "," +String(h3,3);
    voltage = MQ3_ADC.computeVolts(s3);
    voltage = ((Vss - voltage) * RsS )/ voltage;
    output += "," + String(voltage,3);

    //output += "," + String(h7,3);
    voltage = MQ7_ADC.computeVolts(s7);
    //Serial.println(String(voltage,4)+ "," + s7 + "," + String(Vss,4));
    voltage = ((Vss - voltage) * RsS )/ voltage;
    
    output += "," + String(voltage,3 ) + "," + String(Vss, 4);
    output += "," + String((avg_end - avg_start));
    Serial.println(output);
    //delay(1000);
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

void errLeds(void){
  // pinMode(LED_BUILTIN, OUTPUT);
  // digitalWrite(LED_BUILTIN, HIGH);
  // delay(100);
  // digitalWrite(LED_BUILTIN, LOW);
  // delay(100);
}
