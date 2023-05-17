/* Includes */
//#include <LiquidCrystal.h>
#include "LiquidCrystal_I2C.h"
#include "SparkFun_SGP30_Arduino_Library.h"  
#include <Wire.h>
#include "DHT.h"

/* Defines */
#define DHT_Pin1 7
#define DHT_Pin2 8
#define DHTTYPE DHT22
//const int ciRegisterSelectLCD = 12, ciEnableLCD = 9, ciDataLCD_Pin4 = 5, ciDataLCD_Pin5 = 4, ciDataLCD_Pin6 = 3, ciDataLCD_Pin7 = 2;
//LiquidCrystal lcd(ciRegisterSelectLCD, ciEnableLCD, ciDataLCD_Pin4, ciDataLCD_Pin5, ciDataLCD_Pin6, ciDataLCD_Pin7);

/* Type Declarations */
typedef struct MoistureSensorInfo_t
{
  int iMoistureSensorInput;
  int iMoistureSensorPower;
} SensorInfo_t;

/* Function Declarations */
LiquidCrystal_I2C lcd(0x27, 20, 4);  // set the LCD address to 0x3F for a 16 chars and 2 line display//

/* Variables */
DHT dht[] = { { DHT_Pin1, DHT22 }, { DHT_Pin2, DHT22 } };
DHT DHT_H;
SGP30 AirQualitySensor;  
SensorInfo_t asMoistureSensors[2];
float afHumiditySensor[2];
float afTemperatureSensor[2];
int iFanPWM_Tent1 = 5;
int iFanPWM_Tent2 = 6;
int iHeatingPad_Tent1 = 2;  // tent2 = pin4
//int iTemperatureMin_Sensor2 = 20;

//Rather than powering the soil moisture sensor through the 3.3V or 5V pins,
//A digital pin will be used to power the sensor. This will
//prevent corrosion of the sensor as it sits in the soil.

/* Function Definitions */
void setup() {
  const int ciMoistureSensor1 = A0;
  const int ciMoistureSensor2 = A1;
  const int ciMoistureOutput1 = 12;
  const int ciMoistureOutput2 = 13;
  pinMode(iFanPWM_Tent1, OUTPUT);
  pinMode(iFanPWM_Tent2, OUTPUT);
  pinMode(iHeatingPad_Tent1, OUTPUT);
  pinMode(ciMoistureOutput1, OUTPUT);    //Set D7 as an OUTPUT
  digitalWrite(ciMoistureOutput1, LOW);  //Set to LOW so no power is flowing through the sensor
  InitiateReadSensor();
  lcd.init();
  //lcd.begin(16, 4);
  lcd.backlight();
  Serial.begin(9600);
  Wire.begin();
  //Initialize sensor
  if (AirQualitySensor.begin() == false) {
    Serial.println("No SGP30 Detected. Check connections.");
    while (1)
      ;
  }

  asMoistureSensors[0] = {
    .iMoistureSensorInput = ciMoistureSensor1,
    .iMoistureSensorPower = ciMoistureOutput1
  };
  asMoistureSensors[1] = {
    .iMoistureSensorInput = ciMoistureSensor2,
    .iMoistureSensorPower = ciMoistureOutput2
  };

  //Initializes sensor for air quality readings
  //measureAirQuality should be called in one second increments after a call to initAirQuality
  AirQualitySensor.initAirQuality();

  for (auto& sensor : dht) { /* This is a c++ specific function, lookup c++ forloop to read more https://en.cppreference.com/w/cpp/language/range-for */
    sensor.begin();
  }
}

void loop() { /* Nice */
  InitiateReadSensor();
  AirQualitySensor.measureAirQuality();
  vLCD_Tent_DisplayFunction(0);
  vLCD_Tent_DisplayFunction(1);
  delay(1);
}

void vLCD_Tent_DisplayFunction(int iTentNo) {
  char acScreenOutputBuffer[30];
  snprintf(acScreenOutputBuffer, sizeof(acScreenOutputBuffer), "");  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  vTemperatureController(afTemperatureSensor[iTentNo]);
  lcd.print(afTemperatureSensor[iTentNo], 1);
  lcd.print(" \xDF""C  Tent");
  lcd.print(iTentNo +1);            
  lcd.setCursor(0, 1);
  lcd.print("Humidity: ");
  HumidityController();
  lcd.print(afHumiditySensor[iTentNo]);
  lcd.setCursor(0, 2);
  lcd.print("Moisture: ");
  lcd.print(ReadSoil(asMoistureSensors[iTentNo].iMoistureSensorInput, asMoistureSensors[iTentNo].iMoistureSensorPower));
  lcd.setCursor(0, 3);
  lcd.print("pH: 0.0");
  snprintf(acScreenOutputBuffer, sizeof(acScreenOutputBuffer), "");
  if(iTentNo == 0)
  {
    snprintf(acScreenOutputBuffer, sizeof(acScreenOutputBuffer), " CO2: %dppm", AirQualitySensor.CO2);
    lcd.print(acScreenOutputBuffer);
  }
  else
  {
    snprintf(acScreenOutputBuffer, sizeof(acScreenOutputBuffer), " CO2: 0000ppm");
    lcd.print(acScreenOutputBuffer);
  }
  delay(3000);
}

int ReadSoil(int iMoistureSensorPinInput, int iMoistureSensorPinOutput) {
  int iStoreMoistureVal = 0;
  digitalWrite(iMoistureSensorPinOutput, HIGH);
  delay(10);
  iStoreMoistureVal = analogRead(iMoistureSensorPinInput);
  digitalWrite(iMoistureSensorPinOutput, LOW);
  return iStoreMoistureVal;
}

void InitiateReadSensor() {
  for (int i = 0; i < 2; i++) {
    afTemperatureSensor[i] = dht[i].readTemperature();
    afHumiditySensor[i] = dht[i].readHumidity();
  }
}

void HumidityController() 
{
  /* Due to polarity of the fan, the on and off state is defined like this: */
  const uint8_t ON  = 0;
  const uint8_t OFF = 1;
  static int _iHumidityMin_Sensor1 = 62; // normally same as tent 2 
  static int _iHumidityMax_Sensor1 = 66; // 62-64 range for bud drying (make button for this shit)
  static int _iHumidityMin_Sensor2 = 60;
  static int _iHumidityMax_Sensor2 = 67;
  #warning Testing below!
  if (afHumiditySensor[0] < _iHumidityMin_Sensor1) {
    digitalWrite(iFanPWM_Tent1, ON);
  } else {
    digitalWrite(iFanPWM_Tent1, ON);
  }

  if (afHumiditySensor[1] < _iHumidityMin_Sensor2) {
    digitalWrite(iFanPWM_Tent2, ON);
  } else {
    digitalWrite(iFanPWM_Tent2, ON);
  }
}

void vTemperatureController(float fTemperatureVal) {
  const uint8_t ON = 1;
  const uint8_t OFF = 0;
  const float cfTemperatureMin_Sensor = 19.0;
  const float cfTemperatureMax_Sensor = 20.0;

  if      (fTemperatureVal <= cfTemperatureMin_Sensor) 
  {
    digitalWrite(iHeatingPad_Tent1, ON);
  }
  else if (fTemperatureVal >= cfTemperatureMax_Sensor) 
  {
    digitalWrite(iHeatingPad_Tent1, OFF);
  }
}

/* Write Light functions - 80% and 50% */

// pH sensors - WAITING 