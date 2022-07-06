

/**
 * @file eNose.ino
 * @author Rylan Moore (rylan.moore@colorado.edu)
 * @brief Enose project
 * @version 0.1
 * @date 2022-06-23
 * 
 * @copyright Copyright (c) 2022
 * 
 * @note Sensors:
 *        MQ-135
 *        BME688
 *        Gravity Infared V1.1
 *      MCU:
 *        QT PY SAMD21
 */



#include "bsec.h"
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MCP4728.h>
//#include "MQ135.h"

// Helper functions declarations
void checkIaqSensorStatus(void);
void errLeds(void);

float getCorrectionFactor(float t, float h);
float getCorrectedResistance(float t, float h);
float getResistance(void);
float getCorrectedPPM(float t, float h);
float getCorrectedRZero(float t, float h);
float getPPM();

float getPPMgravity(void);

// Create an object of the class Bsec
Bsec iaqSensor;

String output;

Adafruit_ADS1115 ads; //declare the adc for use over i2c
Adafruit_ADS1115 ads2;
Adafruit_ADS1115 ads3;
Adafruit_MCP4728 mcp; //declare the dac for use over i2c


//for MQ
#define PARA 116.6020682
#define PARB -2.769034857

/// Parameters to model temperature and humidity dependence
#define CORA .00035
#define CORB .02718
#define CORC 1.39538
#define CORD .0018
#define CORE -.003333333
#define CORF -.001923077
#define CORG 1.130128205

/// Atmospheric CO2 level for calibration purposes
#define ATMOCO2 420 //Global CO2 Aug 2021
#define _rload  20.1
#define _rzero  8.1//10.91

#define InfaredIn 2 //This sensor is installed on the Input A2 on the ADS1115 with default address in single ended mode

//info for the MQ135 sensor
#define MQ135_ADC ads //define which adc the MQ135 is on
#define MQ135_DAC mcp //define wich dac the MQ135 heater is on
#define MQ135_RS_PIN  0 //channel on the adc that the sense resistor is on
#define MQ135_RH_PIN  3 //channel on the adc that the heat sense resistor is on
#define MQ135_VC_PIN  1 //channel on the adc where the supply voltage is read. 

#define IR_ADC  ads //define which adc the IR CO2 sensor is on
#define IR_PIN  2 //channel on the adc where the IR analog in is located. 


// Entry point for the example
void setup(void)
{
  Serial.begin(115200);
  Wire.begin();

  

  iaqSensor.begin(BME680_I2C_ADDR_SECONDARY, Wire);
  output = "\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
  Serial.println(output);
  checkIaqSensorStatus();
  //pinMode(A0, INPUT);

  //check the status of the dac  
  if (!mcp.begin()) {
    Serial.println("Failed to find MCP4728 chip");
    while (1) {
      delay(10);
    }
  }

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


  ads2.begin(0x49, &Wire); //init the 2nd and 3rd ads1115 with different addresses jumpered, start ads with the default address as well.  
  ads3.begin(0x4B, &Wire);
  ads.begin();
  ads.setGain(GAIN_ONE); //this will set the range to 0-4.1 V with 12mV resolution
  //ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/false);
  // Print the header
  output = "Timestamp [ms], raw temperature [°C], pressure [hPa], raw relative humidity [%], gas [Ohm], IAQ, IAQ accuracy, temperature [°C], relative humidity [%], Static IAQ, CO2 equivalent, breath VOC equivalent";
  Serial.println(output);

  //calibrate the MQ sensor. 
  //_rzero = getCorrectedRZero(iaqSensor.temperature, iaqSensor.humidity);
}

// Function that is looped forever
void loop(void)
{
  //MQ135 gasSensor = MQ135(A0, 10.91, 22);
  unsigned long time_trigger = millis();
  if (iaqSensor.run()) { // If new data is available

    output = String(time_trigger/1000);
    // output += ", " + String(iaqSensor.rawTemperature);
    // output += ", " + String(iaqSensor.pressure);
    // // output += ", " + String(iaqSensor.rawHumidity);
    // // output += ", " + String(iaqSensor.gasResistance);
    // // output += ", " + String(iaqSensor.iaq);
    // // output += ", " + String(iaqSensor.iaqAccuracy);
    output += ", " + String(iaqSensor.temperature);
    output += ", " + String(iaqSensor.humidity);
    // output += ", " + String(iaqSensor.staticIaq);
    output += ", " + String(iaqSensor.co2Equivalent); //here!!!
    output += ", " + String(getCorrectedPPM(iaqSensor.temperature, iaqSensor.humidity));
    //output += ", " + String(getPPM());
    output += ", " + String(getPPMgravity()); //get the value from the Infared Sensor
    // output += ", " + String(iaqSensor.breathVocEquivalent);
    // int16_t results = ads.getLastConversionResults();
    // output += ", " + String( ads.computeVolts(results)); //test the adc output 
    // output += ", " + String( results); //test the adc output 
    Serial.println(output);
  } else {
    checkIaqSensorStatus();
  }
  //int i = ads.readADC_SingleEnded(0);
  //ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/false);
  mcp.setChannelValue(MCP4728_CHANNEL_A, (3000)); //set the output to 3v3 on the va on the dac

  
}

// Helper function definitions
void checkIaqSensorStatus(void)
{
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

void errLeds(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}

/**************************************************************************/
/*!
@brief  Get the correction factor to correct for temperature and humidity

@param[in] t  The ambient air temperature
@param[in] h  The relative humidity

@return The calculated correction factor
*/
/**************************************************************************/
float getCorrectionFactor(float t, float h) {
    // Linearization of the temperature dependency curve under and above 20 degree C
    // below 20degC: fact = a * t * t - b * t - (h - 33) * d
    // above 20degC: fact = a * t + b * h + c
    // this assumes a linear dependency on humidity
    if(t < 20){
        return CORA * t * t - CORB * t + CORC - (h-33.)*CORD;
    } else {
        return CORE * t + CORF * h + CORG;
    }
}

/**************************************************************************/
/*!
@brief  Get the resistance of the sensor, ie. the measurement value corrected
        for temp/hum

@param[in] t  The ambient air temperature
@param[in] h  The relative humidity

@return The corrected sensor resistance kOhm
*/
/**************************************************************************/
float getCorrectedResistance(float t, float h) {
  return getResistance()/getCorrectionFactor(t, h);
}

/**************************************************************************/
/*!
@brief  Get the resistance of the sensor, ie. the measurement value
		Known issue: If the ADC resolution is not 10-bits, this will give
		back garbage values!

@return The sensor resistance in kOhm
*/
/**************************************************************************/
float getResistance(void) { 
    ads.setGain(GAIN_ONE);
    //int  val = ads.readADC_Differential_0_1();
    int val = ads.readADC_SingleEnded(0);
    float fval = val * (0.125/1000);

    ads.setGain(GAIN_TWOTHIRDS);
    val = ads.readADC_SingleEnded(1); //read the refrence voltage
    float rval = val *(0.1875/1000);
    //Serial.println(val);
    //val = (val*4.1)/5;
    //Serial.println(val);
  //this will need to be changed in order to collect data from the external ADC
  //return ((32768./(float)val) - 1.)*_rload;
  return _rload*((rval - fval)/ fval);
  //return _rload* ((1 - ((float)val / 32768.)) / ((float)val / 32768.) );

}

/**************************************************************************/
/*!
@brief  Get the ppm of CO2 sensed (assuming only CO2 in the air), corrected
        for temp/hum

@param[in] t  The ambient air temperature
@param[in] h  The relative humidity

@return The ppm of CO2 in the air
*/
/**************************************************************************/
float getCorrectedPPM(float t, float h) {
  float store;
  for (int i = 0; i < 32; i++){
    store += PARA * pow((getCorrectedResistance(t, h)/_rzero), -PARB);
  }
  return (store/32);
  
}

/**************************************************************************/
/*!
@brief  Get the ppm of CO2 sensed (assuming only CO2 in the air)

@return The ppm of CO2 in the air
*/
/**************************************************************************/
float getPPM() {
  //Serial.println(getResistance());
  return PARA * pow((getResistance()/_rzero), -PARB);

}

/**************************************************************************/
/*!
@brief  Get the resistance RZero of the sensor for calibration purposes
  This will be done once per code run. 

@return The sensor resistance RZero in kOhm
*/
/**************************************************************************/
float getCorrectedRZero(float t, float h) { 
  return getCorrectedResistance(t, h) * pow((ATMOCO2/PARA), (1./PARB));
}


/**
 * @brief Will read the adc computed result from the Infared CO2 sensor. 
 * 
 * @note Will need to implement some sort of loop that will average data based on noise analysis. 
 * 
 * @return float 
 * Return CO2 in PPM
 */
float getPPMgravity(void){
  ads.setGain(GAIN_TWO); //change the gain to make this reading more accurate. 
  int rawd = ads.readADC_SingleEnded(InfaredIn); //Read the raw data. 
  float rawf = ads.computeVolts(rawd); //convert to volts. 
  if(rawf < 0.4){
    return 0;
  }
  else{
    return (3125*rawf)-1250;
  }
  return -1;
}