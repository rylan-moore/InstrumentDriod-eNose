//Rylan Moore
//Enose project
/*
Sensor List:
    MQ-135
    BME688

MCU:
    QT PY SAMD21
*/

#include "MQ135.h"
#include "bsec.h"

#define RZero       758.7   //The rzero from calibration for MQ135
#define MQ135Pin    A0      //pin that the MQ sensor is attached to

//function declarations
void BSEC_Init(void);
void checkIaqSensorStatus(void);
void errLeds(void);

//globals for sensor output
Bsec iaqSensor;
String output;


void setup(void){
    Serial.begin(115200);
    delay(1000);
    Wire.begin();
    pinMode(A0, INPUT); //pin for MQ input 

    BSEC_Init();
}

void loop(void){
    //for now just want to print out useful values:
    //Temp, humid, voc, co2 MQ, co2 BME
    unsigned long time_trigger = millis();
    MQ135 gasSensor = MQ135(MQ135Pin, RZero);
    if (iaqSensor.run()) { // If new data is available
        output = "T + H + Co2 + VoC + Co2e + Co2 \n";
        //output = String(time_trigger);
        //output += ", " + String(iaqSensor.rawTemperature);
        //output += ", " + String(iaqSensor.pressure);
        //output += ", " + String(iaqSensor.rawHumidity);
        //output += ", " + String(iaqSensor.gasResistance);
        //output += ", " + String(iaqSensor.iaq);
        //output += ", " + String(iaqSensor.iaqAccuracy);
        output += ", " + String(iaqSensor.temperature);
        output += ", " + String(iaqSensor.humidity);
        //output += ", " + String(iaqSensor.staticIaq);
        output += ", " + String(iaqSensor.co2Equivalent);
        output += ", " + String(iaqSensor.breathVocEquivalent);
        output += ", " + String(gasSensor.getCorrectedPPM(iaqSensor.temperature, iaqSensor.humidity));
        output += ", " + String(gasSensor.getPPM());
        Serial.println(output);
    } else {
        checkIaqSensorStatus();
    }
}

//Functions
//Initialize all of the BME688 sensor function
void BSEC_Init(void){
iaqSensor.begin(BME680_I2C_ADDR_SECONDARY, Wire);
  output = "\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
  Serial.println(output);
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

  // Print the header
  output = "Timestamp [ms], raw temperature [°C], pressure [hPa], raw relative humidity [%], gas [Ohm], IAQ, IAQ accuracy, temperature [°C], relative humidity [%], Static IAQ, CO2 equivalent, breath VOC equivalent";
  Serial.println(output);
}

//this will check the status of the BME688 sensor
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

//Will display sensor BME688 error
void errLeds(void){
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}