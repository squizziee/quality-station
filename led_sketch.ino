#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <LiquidCrystal.h>

// SPI pins for BME280
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

// two BME280 sensors
Adafruit_BME280 bme1; // I2C
Adafruit_BME280 bme2(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // SPI

// other sensor pins
#define VUMETER_PIN A1
#define LDR_PIN A0
#define MQ135_PIN A2

// 40x2 screen pins
const int rs = 2, en = 3, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// delay times for loop function
unsigned long delayTime;
unsigned long displayDelayTime;

// expected range of measurements in {min, max} format
const int   GAS_EXPECTED[2] =         {100, 1000};
const float LUMINOSITY_EXPECTED[2] =  {0, 500};
const int   SOUND_EXPECTED[2] =       {25, 200};
const float TEMPERATURE_EXPECTED[2] = {5, 50};
const float PRESSURE_EXPECTED[2] =    {950, 1100};
const float HUMIDITY_EXPECTED[2] =    {10, 100};

// normal range of measurements for indoor environment in {min, max} format
const int   GAS_NORMAL[2] =         {400, 750};
const float LUMINOSITY_NORMAL[2] =  {50, 150};
const int   SOUND_NORMAL[2] =       {25, 55};
const float TEMPERATURE_NORMAL[2] = {16, 22};
const float PRESSURE_NORMAL[2] =    {1000, 1050};
const float HUMIDITY_NORMAL[2] =    {30, 60};

// struct for measurement storage
struct Measurements {
  int gas;
  float luminosity; 
  int decibels;
  float degrees; 
  float hPascals; 
  float percentage;
};

// class for RGB LED color storage
class Color {
  public:
    int red;
    int green;
    int blue;

    Color(int red, int green, int blue) : red(red), green(green), blue(blue) {}

    Color() {

    }
};

class Quality {
  public:
    Color qualityColor;
    int qualityArr[6];

    int getOverallQuality() {
      return qualityArr[0] + qualityArr[1] + qualityArr[2] +
             qualityArr[3] + qualityArr[4] + qualityArr[5];
    }

    String getLackingMeasurements() {
      String result = "";
      bool resultEmpty = true;
      if (qualityArr[0] == 1) {
        result += "gas";
        resultEmpty = false;
      }
      if (qualityArr[1] == 1) {
        if (!resultEmpty) result += "," ;
        result += "light" ;
        resultEmpty = false;
      }
      if (qualityArr[2] == 1) {
        if (!resultEmpty) result += "," ;
        result += "noise" ;
        resultEmpty = false;
      }
      if (qualityArr[3] == 1) {
        if (!resultEmpty) result += "," ;
        result += "temp." ;
        resultEmpty = false;
      }
      if (qualityArr[4] == 1) {
        if (!resultEmpty) result += "," ;
        result += "pres." ;
        resultEmpty = false;
      }
      if (qualityArr[5] == 1) {
        if (!resultEmpty) result += "," ;
        result += "humid. " ;
        resultEmpty = false;
      }
      if (resultEmpty) {
        return "None";
      }
      return result;
    }
};

void setup() {
  Serial.begin(9600);

  bool status;
  status = bme1.begin();   
  bool status2 = bme2.begin();
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  
  Serial.println("Started monitoring...");
  delayTime = 1000;
  displayDelayTime = 2000;

  Serial.println();

  pinMode(VUMETER_PIN, INPUT);

  lcd.begin(40, 2);

  lcd.setCursor(0, 0);
}

void loop() {

  lcd.clear();

  Measurements calibrated = calibrateMeasurements(
    measureGasLevel(), 
    measureLuminosity(), 
    measureSoundLevel(), 
    measureTemperature(false), 
    measurePressure(false),
    measureHumidity(false)
  );

  Quality quality = calculateQuality(
    calibrated.gas, 
    calibrated.luminosity, 
    calibrated.decibels, 
    calibrated.degrees, 
    calibrated.hPascals, 
    calibrated.percentage
  );

  setColor(quality.qualityColor);

  // Serial.print("Quality color: ");
  // Serial.print(quality.red);
  // Serial.print(",");
  // Serial.print(quality.green);
  // Serial.print(",");
  // Serial.println(quality.blue);

  gasToLCD(calibrated.gas);
  lcd.setCursor(0, 1);
  luminosityToLCD(calibrated.luminosity);
  delay(displayDelayTime);
  lcd.setCursor(0, 0);
  lcd.clear();

  soundToLCD(calibrated.decibels);
  lcd.setCursor(0, 1);
  temperatureToLCD(calibrated.degrees);
  delay(displayDelayTime);
  lcd.setCursor(0, 0);
  lcd.clear();

  pressureToLCD(calibrated.hPascals);
  lcd.setCursor(0, 1);
  humidityToLCD(calibrated.percentage);
  delay(displayDelayTime - delayTime);
  lcd.setCursor(0, 0);
  lcd.clear();

  lcd.print("Overall quality: ");
  lcd.print(6 - quality.getOverallQuality());
  lcd.print("/6");
  lcd.setCursor(0, 1);
  lcd.print("Bad: ");
  lcd.print(quality.getLackingMeasurements());
  delay(displayDelayTime);
  lcd.setCursor(0, 0);

  delay(delayTime);
}

String compare(int val, int min, int max) {
  if (val >= min && val <= max) return "OK";
  if (val < min) return "LOW";
  return "HIGH";
}

String compareF(float val, float min, float max) {
  if (val >= min && val <= max) return "OK";
  if (val < min) return "LOW";
  return "HIGH";
}

// display data to 40x2 LCD

void gasToLCD(int gas) {
  lcd.print("Gas: ");
  if (gas < 0) {
    lcd.print("no valid sensor data");
    return;
  }
  lcd.print(gas);
  lcd.print(" ppm (");
  lcd.print(compare(gas, GAS_NORMAL[0], GAS_NORMAL[1]));
  lcd.print(")");
}

void luminosityToLCD(float luminosity) {
  lcd.print("Luminosity: ");
  if (luminosity < 0) {
    lcd.print("no valid sensor data");
    return;
  }
  lcd.print(luminosity);
  lcd.print(" lux (");
  lcd.print(compareF(luminosity, LUMINOSITY_NORMAL[0], LUMINOSITY_NORMAL[1]));
  lcd.print(")");
}

void soundToLCD(int decibels) {
  lcd.print("Sound level: ");
  if (decibels < 0) {
    lcd.print("no valid sensor data");
    return;
  }
  lcd.print(decibels);
  lcd.print(" db (");
  lcd.print(compare(decibels, SOUND_NORMAL[0], SOUND_NORMAL[1]));
  lcd.print(")");
}

void temperatureToLCD(float degrees) {
  lcd.print("Temperature: ");
  if (degrees < 0) {
    lcd.print("no valid sensor data");
    return;
  }
  lcd.print(degrees);
  lcd.print(" *C (");
  lcd.print(compareF(degrees, TEMPERATURE_NORMAL[0], TEMPERATURE_NORMAL[1]));
  lcd.print(")");
}

void pressureToLCD(float hPascals) {
  lcd.print("Pressure: ");
  if (hPascals < 0) {
    lcd.print("no valid sensor data");
    return;
  }
  lcd.print(hPascals);
  lcd.print(" hPa (");
  lcd.print(compareF(hPascals, PRESSURE_NORMAL[0], PRESSURE_NORMAL[1]));
  lcd.print(")");
}

void humidityToLCD(float percentage) {
  lcd.print("Humidity: ");
   if (percentage < 0) {
    lcd.print("no valid sensor data");
    return;
  }
  lcd.print(percentage);
  lcd.print(" % (");
  lcd.print(compareF(percentage, HUMIDITY_NORMAL[0], HUMIDITY_NORMAL[1]));
  lcd.print(")");
}

// measurement functions

float measureTemperature(bool useSecondary) {
  return useSecondary ? bme2.readTemperature() : bme1.readTemperature();
}

float measurePressure(bool useSecondary) {
  return useSecondary ? bme2.readPressure() / 100.0F : bme1.readPressure() / 100.0F;
}

float measureHumidity(bool useSecondary) {
  return useSecondary ? bme2.readHumidity() : bme1.readHumidity();
}

int measureSoundLevel() {
  int analogValue = analogRead(VUMETER_PIN);
  float voltage = analogValue * (5.0 / 1023.0);
  int db = 20 * log10(voltage / 0.000092) ; 
  return db;
}

float measureLuminosity() {
  int value_idr = analogRead(LDR_PIN);
  float converted = map(value_idr, 20, 1005, 0, 100);
  return converted;
}

int measureGasLevel() {
  int value_idr = analogRead(MQ135_PIN);
  return value_idr;
}

// relay control functions

void switchToSecondarySensors() {
  pinMode(0, OUTPUT);
  digitalWrite(0, HIGH);
  delay(100);
}

void switchToPrimarySensors() {
  pinMode(0, OUTPUT);
  digitalWrite(0, LOW);
  delay(100);
}

// measurement calibration function. Accepts measurements from
// primary set of sensors, then switches to secondary and 
// acquires second list of measurements. After analysis of values
// from both sets of sensors returns calibrated results
Measurements calibrateMeasurements(
  int gas, 
  float luminosity, 
  int decibels, 
  float degrees, 
  float hPascals, 
  float percentage) {
  
  switchToSecondarySensors();

  int gas2 = measureGasLevel();
  float luminosity2 = measureLuminosity();
  int decibels2 = measureSoundLevel();
  float degrees2 = measureTemperature(true);
  float hPascals2 = measurePressure(true);
  float percentage2 = measureHumidity(true);

  switchToPrimarySensors();

  Measurements m;
  m.gas = assessCorrectness(gas, gas2, GAS_EXPECTED[0], GAS_EXPECTED[1]);
  m.luminosity = assessCorrectnessF(luminosity, luminosity2, LUMINOSITY_EXPECTED[0], LUMINOSITY_EXPECTED[1]);
  m.decibels = assessCorrectness(decibels, decibels2, SOUND_EXPECTED[0], SOUND_EXPECTED[1]);
  m.degrees = assessCorrectnessF(degrees, degrees2, TEMPERATURE_EXPECTED[0], TEMPERATURE_EXPECTED[1]);
  m.hPascals = assessCorrectnessF(hPascals, hPascals2, PRESSURE_EXPECTED[0], PRESSURE_EXPECTED[1]);
  m.percentage = assessCorrectnessF(percentage, percentage2, HUMIDITY_EXPECTED[0], HUMIDITY_EXPECTED[1]);

  return m;
}

// measurement comparison function for int
int assessCorrectness(int measurement1, int measurement2, int lowest, int highest) {
  int correct;
  if (measurement1 > lowest && measurement1 < highest && measurement2 > lowest && measurement1 < highest) {
    correct = (measurement1 + measurement2) / 2;
  }
  else if(measurement1 > lowest && measurement1 < highest) {
    correct = measurement1;
  }
  else if(measurement2 > lowest && measurement2 < highest) {
    correct = measurement2;
  }
  else {
    correct = -1;
  }
  return correct;
}

// measurement comparison function for float
float assessCorrectnessF(float measurement1, float measurement2, float lowest, float highest) {
  float correct;
  if (measurement1 > lowest && measurement1 < highest && measurement2 > lowest && measurement1 < highest) {
    correct = (measurement1 + measurement2) / 2.0;
  }
  else if(measurement1 > lowest && measurement1 < highest) {
    correct = measurement1;
  }
  else if(measurement2 > lowest && measurement2 < highest) {
    correct = measurement2;
  }
  else {
    correct = -1;
  }
  return correct;
}

// writing quality color to RGB LED
void setColor(Color color) {
  int redPin = 8;
  int greenPin = 9;
  analogWrite(redPin, color.red);
  analogWrite(greenPin, color.green);
};

// calculation of quality quotient according to given calibrated measurements
Quality calculateQuality(
  int gas, 
  float luminosity, 
  int decibels, 
  float degrees, 
  float hPascals, 
  float percentage) {
  int code1 = 1;
  int code2 = 1;
  int code3 = 1;
  int code4 = 1;
  int code5 = 1;
  int code6 = 1;

  if (gas <= 750 && gas >= 400) code1 = 0;
  else code1 = 1;

  if (luminosity >= 50 && luminosity <= 150) code2 = 0;
  else code2 = 1;

  if (decibels <= 55) code3 = 0;
  else code3 = 1;

  if (degrees >= 16 && degrees <= 22) code4 = 0;
  else code4 = 1;

  if (hPascals >= 1000 && hPascals <= 1050) code5 = 0;
  else code5 = 1;

  if (percentage >= 30 && percentage <= 60) code6 = 0;
  else code6 = 1;

  int total = code1 + code2 + code3 + code4 + code5 + code6;

  Quality result;
  result.qualityArr[0] = code1;
  result.qualityArr[1] = code2;
  result.qualityArr[2] = code3;
  result.qualityArr[3] = code4;
  result.qualityArr[4] = code5;
  result.qualityArr[5] = code6;

  switch(total) {
    case 6:
      result.qualityColor = Color(255, 0, 0);
      break;
    case 5:
      result.qualityColor = Color(212, 42, 0);
      break;
    case 4:
      result.qualityColor = Color(170, 85, 0);
      break;
    case 3:
      result.qualityColor = Color(127, 127, 0);
      break;
    case 2:
      result.qualityColor = Color(85, 170, 0);
      break;
    case 1:
      result.qualityColor = Color(42, 212, 0);
      break;
    case 0:
      result.qualityColor = Color(0, 255, 0);
      break;
  }
  Serial.print(result.qualityColor.red);
  Serial.print(result.qualityColor.green);

  return result;
}