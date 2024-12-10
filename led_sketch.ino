#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
//#include <LiquidCrystal.h>
#include <ssd1306.h>

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

#define DEBUG_PIN 4
#define SIGNAL_PIN 8

// delay times for loop function
unsigned long delayTime;
unsigned long displayDelayTime;
bool isDebug = true;

// expected range of measurements in {min, max} format
const int   GAS_EXPECTED[2] =         {100, 1000};
const float LUMINOSITY_EXPECTED[2] =  {0, 500};
const int   SOUND_EXPECTED[2] =       {25, 100};
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

// integrity map in {primary, secondary} format
int GAS_SENSORS[2] =         {0, 0};
int LUMINOSITY_SENSORS[2] =  {0, 0};
int SOUND_SENSORS[2] =       {0, 0};
int TEMPERATURE_SENSORS[2] = {0, 0};
int PRESSURE_SENSORS[2] =    {0, 0};
int HUMIDITY_SENSORS[2] =    {0, 0};
int SCREENS[1] =             {0};

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

// class for measurement quality storage
class Quality {
  public:
    Color qualityColor;
    int qualityArr[6];

    int getOverallQuality() {
      int total = 0;
      for (int i = 0; i < 6; i++) {
        if (qualityArr[i] != -1 && qualityArr[i] != 1) {
          ++total;
        }
      }
      return total;
    }

    int getCountOfAllMeasurementsWithValidSensors() {
      int total = 0;
      for (int i = 0; i < 6; i++) {
        if (qualityArr[i] != -1) {
          ++total;
        }
      }
      return total;
    }

    String getLackingMeasurements() {
      String result = "";
      bool resultEmpty = true;
      if (qualityArr[0] == 1) {
        result += "CO2";
        resultEmpty = false;
      }
      if (qualityArr[1] == 1) {
        if (!resultEmpty) result += ", " ;
        result += "LUMINOSITY" ;
        resultEmpty = false;
      }
      if (qualityArr[2] == 1) {
        if (!resultEmpty) result += ", " ;
        result += "NOISE" ;
        resultEmpty = false;
      }
      if (qualityArr[3] == 1) {
        if (!resultEmpty) result += ", " ;
        result += "TEMPERATURE" ;
        resultEmpty = false;
      }
      if (qualityArr[4] == 1) {
        if (!resultEmpty) result += ", " ;
        result += "PRESSURE" ;
        resultEmpty = false;
      }
      if (qualityArr[5] == 1) {
        if (!resultEmpty) result += ", " ;
        result += "HUMIDITY" ;
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
  bool status= bme1.begin();   
  bool status2 = bme2.begin();
  if (!status && !status2) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while(true) {
      pulseSignal();
      status = bme1.begin();
      status2 = bme2.begin();
    }
  }
  
  delayTime = 1000;
  displayDelayTime = 200;

  pinMode(VUMETER_PIN, INPUT);
  pinMode(LDR_PIN, INPUT);
  pinMode(MQ135_PIN, INPUT);
  pinMode(DEBUG_PIN, INPUT);

  pinMode(SIGNAL_PIN, OUTPUT);

  ssd1306_setFixedFont(ssd1306xled_font6x8);
  ssd1306_128x64_i2c_init();
  ssd1306_clearScreen();

  Wire.begin();

  ssd1306_printFixed(0,  0, "Initializing...", STYLE_NORMAL);

  Serial.println("Started monitoring...");
  checkRelays();
}

void pulseSignal() {
  for (int i = 0; i < 10; i++) {
    digitalWrite(SIGNAL_PIN, HIGH);
    delay(100);
    digitalWrite(SIGNAL_PIN, LOW);
    delay(100);
  }
}

// find out if screen is connected via I2C
bool checkScreen() {
  Serial.println("Scanning for I2C devices...");
  byte error, address;

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      if (address == 60) {
        Serial.println("Found screen");
        return true;
      } 
    }
  }
  Serial.println("Error: no display found");
  return false;
}

bool inRange(int val, int min, int max) {
  return val > min && val < max;
}

// find out if relays are connected
bool checkRelays() {
  Serial.println("Scanning for relays...");

  pinMode(VUMETER_PIN, INPUT_PULLUP);
  pinMode(LDR_PIN, INPUT_PULLUP);
  pinMode(MQ135_PIN, INPUT_PULLUP);

  int relay1 = analogRead(VUMETER_PIN);
  int relay2 = analogRead(LDR_PIN);
  int relay3 = analogRead(MQ135_PIN);

  switchToSecondarySensors();

  int relay1_ = analogRead(VUMETER_PIN);
  int relay2_ = analogRead(LDR_PIN);
  int relay3_ = analogRead(MQ135_PIN);

  switchToPrimarySensors();

  pinMode(VUMETER_PIN, INPUT);
  pinMode(LDR_PIN, INPUT);
  pinMode(MQ135_PIN, INPUT);

Serial.println(relay1);
  Serial.println(relay2);
  Serial.println(relay3);
  Serial.println(relay1_);
  Serial.println(relay2_);
  Serial.println(relay3_);

  if (
    (inRange(relay1, 0, 20) || inRange(relay1_, 0, 20)) && 
    (inRange(relay2, 0, 1000) || inRange(relay2_, 0, 1000)) && 
    (inRange(relay3, 0, 1000) || inRange(relay3_, 0, 1000))
  ) {
    if (!inRange(relay1, 0, 1000)) SOUND_SENSORS[0] = 1;
    if (!inRange(relay1_, 0, 1000)) SOUND_SENSORS[1] = 1;
    if (!inRange(relay2, 0, 1000)) LUMINOSITY_SENSORS[1] = 1;
    if (!inRange(relay2_, 0, 1000)) LUMINOSITY_SENSORS[0] = 1;
    if (!inRange(relay3, 0, 1000)) GAS_SENSORS[1] = 1;
    if (!inRange(relay3_, 0, 1000)) GAS_SENSORS[0] = 1;

    Serial.println("All relays connected");
    return true;
  }

  Serial.println("Error: system integrity fault: ");
  Serial.println("Either relay is disconnected or two sensors of the same type are not connected to relay.");
  Serial.println("[System analysis]");
  Serial.println("- VUMETER RELAY: " + String(inRange(relay1, 0, 1000) || inRange(relay1_, 0, 1000) ? "connected" : "N/A"));
  Serial.println("- LDR RELAY: " + String(inRange(relay2, 0, 1000) || inRange(relay2_, 0, 1000) ? "connected" : "N/A"));
  Serial.println("- MQ135 RELAY: " + String(inRange(relay3, 0, 1000) || inRange(relay3_, 0, 1000) ? "connected" : "N/A"));
  // Serial.println(relay1);
  // Serial.println(relay2);
  // Serial.println(relay3);
  // Serial.println(relay1_);
  // Serial.println(relay2_);
  // Serial.println(relay3_);

  ssd1306_printFixed(0,  0, "Fatal error: insufficient integrity", STYLE_NORMAL);
  ssd1306_printFixed(0,  20, (String("- VUMETER RELAY: ") + String((inRange(relay1, 0, 1000) || inRange(relay1_, 0, 1000)) ? "OK" : "N/A")).c_str(), STYLE_NORMAL);
  ssd1306_printFixed(0,  30, (String("- LDR RELAY: ") + String((inRange(relay2, 0, 1000) || inRange(relay2_, 0, 1000)) ? "OK" : "N/A")).c_str(), STYLE_NORMAL);
  ssd1306_printFixed(0,  35, (String("- MQ135 RELAY: ") + String((inRange(relay3, 0, 1000) || inRange(relay3_, 0, 1000)) ? "OK" : "N/A")).c_str(), STYLE_NORMAL);

  return false;
}

// calculate total system integrity (13 components)
float calculateIntegrity() {
  int intact = 13 -
    (GAS_SENSORS[0] + GAS_SENSORS[1] + LUMINOSITY_SENSORS[0] + LUMINOSITY_SENSORS[1] +
    SOUND_SENSORS[0] + SOUND_SENSORS[1] + TEMPERATURE_SENSORS[0] + TEMPERATURE_SENSORS[1] +
    PRESSURE_SENSORS[0] + PRESSURE_SENSORS[1] + HUMIDITY_SENSORS[0] + HUMIDITY_SENSORS[1] + SCREENS[0]);
  return intact / 13.0 * 100.0;
}

void showDebug() {
    String r1 = "INTEGRITY: " + String(calculateIntegrity()) + "%";

    String r2_1 = "PRIMARY";
    String r2_2 = "SECONDARY";

    String r3_1 = "G";
    String r3_2 = String((GAS_SENSORS[0] ? "INVALID" : "VALID"));
    String r3_3 = String((GAS_SENSORS[1] ? "INVALID" : "VALID"));

    String r4_1 = "L";
    String r4_2 = String((LUMINOSITY_SENSORS[0] ? "INVALID" : "VALID"));
    String r4_3 = String((LUMINOSITY_SENSORS[1] ? "INVALID" : "VALID"));

    String r5_1 = "N";
    Serial.print(SOUND_SENSORS[0]);
    Serial.print(SOUND_SENSORS[1]);
    String r5_2 = String((SOUND_SENSORS[1] ? "INVALID" : "VALID"));
    String r5_3 = String((SOUND_SENSORS[0] ? "INVALID" : "VALID"));

    String r6_1 = "T";
    String r6_2 = String((TEMPERATURE_SENSORS[0] ? "INVALID" : "VALID"));
    String r6_3 = String((TEMPERATURE_SENSORS[1] ? "INVALID" : "VALID"));

    String r7_1 = "P";
    String r7_2 = String((PRESSURE_SENSORS[0] ? "INVALID" : "VALID"));
    String r7_3 = String((PRESSURE_SENSORS[1] ? "INVALID" : "VALID"));

    String r8_1 = "H";
    String r8_2 = String((HUMIDITY_SENSORS[0] ? "INVALID" : "VALID"));
    String r8_3 = String((HUMIDITY_SENSORS[1] ? "INVALID" : "VALID"));
    ssd1306_printFixed(0,  0, r1.c_str(), STYLE_NORMAL);

    ssd1306_printFixed(10,  10, r2_1.c_str(), STYLE_NORMAL);
    ssd1306_printFixed(65,  10, r2_2.c_str(), STYLE_NORMAL);

    ssd1306_printFixed(0,  20, r3_1.c_str(), STYLE_NORMAL);
    ssd1306_printFixed(10,  20, r3_2.c_str(), STYLE_NORMAL);
    ssd1306_printFixed(65,  20, r3_3.c_str(), STYLE_NORMAL);

    ssd1306_printFixed(0,  25, r4_1.c_str(), STYLE_NORMAL);
    ssd1306_printFixed(10,  30, r4_2.c_str(), STYLE_NORMAL);
    ssd1306_printFixed(65,  30, r4_3.c_str(), STYLE_NORMAL);

    ssd1306_printFixed(0,  35, r5_1.c_str(), STYLE_NORMAL);
    ssd1306_printFixed(10,  35, r5_2.c_str(), STYLE_NORMAL);
    ssd1306_printFixed(65,  35, r5_3.c_str(), STYLE_NORMAL);

    ssd1306_printFixed(0,  45, r6_1.c_str(), STYLE_NORMAL);
    ssd1306_printFixed(10,  45, r6_2.c_str(), STYLE_NORMAL);
    ssd1306_printFixed(65,  45, r6_3.c_str(), STYLE_NORMAL);

    ssd1306_printFixed(0,  55, r7_1.c_str(), STYLE_NORMAL);
    ssd1306_printFixed(10,  55, r7_2.c_str(), STYLE_NORMAL);
    ssd1306_printFixed(65,  55, r7_3.c_str(), STYLE_NORMAL);

    ssd1306_printFixed(0,  60, r8_1.c_str(), STYLE_NORMAL);
    ssd1306_printFixed(10,  60, r8_2.c_str(), STYLE_NORMAL);
    ssd1306_printFixed(65,  60, r8_3.c_str(), STYLE_NORMAL);
}

void loop() {
  // checks
  bool screenPresent = checkScreen();
  if (!screenPresent) {
    while(1) {
      delay(1000);
      screenPresent = checkScreen();
    }
  }

  SCREENS[0] = !screenPresent;

  bool relaysConnected = checkRelays();
  if (!relaysConnected) {
    while(1) {
      pulseSignal();
      relaysConnected = checkRelays();
    }
    
  }

  isDebug = (bool) digitalRead(DEBUG_PIN); 
  int value = analogRead(DEBUG_PIN);

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

  ssd1306_clearScreen();

  gasToLCD(calibrated.gas);
  luminosityToLCD(calibrated.luminosity);

  soundToLCD(calibrated.decibels);

  delay(displayDelayTime);
  ssd1306_clearScreen();

  temperatureToLCD(calibrated.degrees);
  pressureToLCD(calibrated.hPascals);
  humidityToLCD(calibrated.percentage);

  delay(displayDelayTime);
  ssd1306_clearScreen();

  String row1 = "OVERALL QUALITY: " + String(quality.getOverallQuality()) + "/" + String(quality.getCountOfAllMeasurementsWithValidSensors());
  String row2 = "LACKING: " + quality.getLackingMeasurements();
  ssd1306_printFixed(0,  0, row1.c_str(), STYLE_NORMAL);
  ssd1306_printFixed(0,  10, row2.c_str(), STYLE_NORMAL);

  delay(displayDelayTime);

  if (isDebug) {
    ssd1306_clearScreen();
    showDebug();
    delay(displayDelayTime);
  }
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

// display data to OLED display

void gasToLCD(int gas) {
  String row1 = "CO2: " + String(gas) + " PPM";
  if (gas < 0) {
    ssd1306_printFixed(0,  0, "CO2: ", STYLE_NORMAL);
    ssd1306_printFixed(0,  10, "NO VALID DATA", STYLE_NORMAL);
    return;
  }
  String row2 = compare(gas, GAS_NORMAL[0], GAS_NORMAL[1]) + " (" + String(GAS_NORMAL[0]) + "-" + String(GAS_NORMAL[1]) + ")";
  ssd1306_printFixed(0,  0, row1.c_str(), STYLE_NORMAL);
  ssd1306_printFixed(0,  10, row2.c_str(), STYLE_NORMAL);
}

void luminosityToLCD(float luminosity) {
  String row1 = "LUMINOSITY: " + String(luminosity) + " LUX";
  if (luminosity < 0) {
    ssd1306_printFixed(0,  25, "LUMINOSITY: ", STYLE_NORMAL);
    ssd1306_printFixed(0,  35, "NO VALID DATA", STYLE_NORMAL);
    return;
  }
  String row2 = compare(luminosity, LUMINOSITY_NORMAL[0], LUMINOSITY_NORMAL[1]) + " (" + String(LUMINOSITY_NORMAL[0]) + "-" + String(LUMINOSITY_NORMAL[1]) + ")";
  ssd1306_printFixed(0,  25, row1.c_str(), STYLE_NORMAL);
  ssd1306_printFixed(0,  35, row2.c_str(), STYLE_NORMAL);
}

void soundToLCD(int decibels) {
  String row1 = "NOISE: " + String(decibels) + " DB";
  if (decibels < 0) {
    ssd1306_printFixed(0,  50, "NOISE: ", STYLE_NORMAL);
    ssd1306_printFixed(0,  60, "NO VALID DATA", STYLE_NORMAL);
    return;
  }
  String row2 = compare(decibels, SOUND_NORMAL[0], SOUND_NORMAL[1]) + " (" + String(SOUND_NORMAL[0]) + "-" + String(SOUND_NORMAL[1]) + ")";
  ssd1306_printFixed(0,  50, row1.c_str(), STYLE_NORMAL);
  ssd1306_printFixed(0,  60, row2.c_str(), STYLE_NORMAL);
}

void temperatureToLCD(float degrees) {
  String row1 = "TEMPERATURE: " + String(degrees) + " *C";
  if (degrees < 0) {
    ssd1306_printFixed(0,  0, "TEMPERATURE: ", STYLE_NORMAL);
    ssd1306_printFixed(0,  10, "NO VALID DATA", STYLE_NORMAL);
    return;
  }
  String row2 = compare(degrees, TEMPERATURE_NORMAL[0], TEMPERATURE_NORMAL[1]) + " (" + String(TEMPERATURE_NORMAL[0]) + "-" + String(TEMPERATURE_NORMAL[1]) + ")";
  ssd1306_printFixed(0,  0, row1.c_str(), STYLE_NORMAL);
  ssd1306_printFixed(0,  10, row2.c_str(), STYLE_NORMAL);
}

void pressureToLCD(float hPascals) {
  String row1 = "PRESSURE: " + String(hPascals) + " HPA";
  if (hPascals < 0) {
    ssd1306_printFixed(0,  25, "PRESSURE: ", STYLE_NORMAL);
    ssd1306_printFixed(0,  35, "NO VALID DATA", STYLE_NORMAL);
    return;
  }
  String row2 = compare(hPascals, PRESSURE_NORMAL[0], PRESSURE_NORMAL[1]) + " (" + String(PRESSURE_NORMAL[0]) + "-" + String(PRESSURE_NORMAL[1]) + ")";
  ssd1306_printFixed(0,  25, row1.c_str(), STYLE_NORMAL);
  ssd1306_printFixed(0,  35, row2.c_str(), STYLE_NORMAL);
}

void humidityToLCD(float percentage) {
  String row1 = "HUMIDITY: " + String(percentage) + "%";
  if (percentage < 0) {
    ssd1306_printFixed(0,  50, "HUMIDITY: ", STYLE_NORMAL);
    ssd1306_printFixed(0,  60, "NO VALID DATA", STYLE_NORMAL);
    return;
  }
  String row2 = compare(percentage, HUMIDITY_NORMAL[0], HUMIDITY_NORMAL[1]) + " (" + String(HUMIDITY_NORMAL[0]) + "-" + String(HUMIDITY_NORMAL[1]) + ")";
  ssd1306_printFixed(0,  50, row1.c_str(), STYLE_NORMAL);
  ssd1306_printFixed(0,  60, row2.c_str(), STYLE_NORMAL);
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
  Serial.print("Sound: ");
  Serial.println(db);
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
  delay(500);
}

void switchToPrimarySensors() {
  pinMode(0, OUTPUT);
  digitalWrite(0, LOW);
  delay(500);
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
  m.gas = assessCorrectness(gas, gas2, GAS_EXPECTED[0], GAS_EXPECTED[1], GAS_SENSORS[0], GAS_SENSORS[1]);
  m.luminosity = assessCorrectnessF(luminosity, luminosity2, LUMINOSITY_EXPECTED[0], LUMINOSITY_EXPECTED[1], LUMINOSITY_SENSORS[0], LUMINOSITY_SENSORS[1]);
  m.decibels = assessCorrectness(decibels, decibels2, SOUND_EXPECTED[0], SOUND_EXPECTED[1], SOUND_SENSORS[0], SOUND_SENSORS[1]);
  m.degrees = assessCorrectnessF(degrees, degrees2, TEMPERATURE_EXPECTED[0], TEMPERATURE_EXPECTED[1], TEMPERATURE_SENSORS[0], TEMPERATURE_SENSORS[1]);
  m.hPascals = assessCorrectnessF(hPascals, hPascals2, PRESSURE_EXPECTED[0], PRESSURE_EXPECTED[1], PRESSURE_SENSORS[0], PRESSURE_SENSORS[1]);
  m.percentage = assessCorrectnessF(percentage, percentage2, HUMIDITY_EXPECTED[0], HUMIDITY_EXPECTED[1], HUMIDITY_SENSORS[0], HUMIDITY_SENSORS[1]);

  return m;
}

// measurement comparison function for int
int assessCorrectness(int measurement1, int measurement2, int lowest, int highest, int& ref1, int& ref2) {
  int correct;
  int ref1Copy = ref1;
  int ref2Copy = ref2;
  if (measurement1 > lowest && measurement1 < highest && measurement2 > lowest && measurement2 < highest && !ref1Copy && !ref2Copy) {
    correct = (measurement1 + measurement2) / 2;
    ref1 = 0;
    ref2 = 0;
  }
  else if(measurement1 > lowest && measurement1 < highest && !ref1Copy) {
    correct = measurement1;
    ref2 = 1;
    ref1 = 0;
  }
  else if(measurement2 > lowest && measurement2 < highest && !ref2Copy) {
    correct = measurement2;
    ref1 = 1;
    ref2 = 0;
  }
  else {
    correct = -1;
    ref1 = 1;
    ref2 = 1;
  }
  if (ref1Copy == 1) {
    ref1 = 1;
  }
  if (ref2Copy == 1) {
    ref2 = 1;
  }
  return correct;
}

// measurement comparison function for float
float assessCorrectnessF(float measurement1, float measurement2, float lowest, float highest, int& ref1, int& ref2) {
  float correct;
  int ref1Copy = ref1;
  int ref2Copy = ref2;
  if (measurement1 > lowest && measurement1 < highest && measurement2 > lowest && measurement2 < highest && !ref1Copy && !ref2Copy) {
    correct = (measurement1 + measurement2) / 2.0;
    ref1 = 0;
    ref2 = 0;
  }
  else if(measurement1 > lowest && measurement1 < highest && !ref1Copy) {
    correct = measurement1;
    ref2 = 1;
    ref1 = 0;
  }
  else if(measurement2 > lowest && measurement2 < highest && !ref2Copy) {
    correct = measurement2;
    ref1 = 1;
    ref2 = 0;
  }
  else {
    correct = -1;
    ref1 = 1;
    ref2= 1;
  }
  if (ref1Copy == 1) {
    ref1 = 1;
  }
  if (ref2Copy == 1) {
    ref2 = 1;
  }
  return correct;
}

// writing quality color to RGB LED
void setColor(Color color) {
  int redPin = 2;
  int greenPin = 3;
  analogWrite(redPin, color.red);
  analogWrite(greenPin, color.green);
};

// calculation of quality quotient according to given calibrated measurements.
// 1 is for abnormal measurement, 0 is for normal, -1 is for incorrect.
Quality calculateQuality(
  int gas, 
  float luminosity, 
  int decibels, 
  float degrees, 
  float hPascals, 
  float percentage) {

  int codes[6] = {1, 1, 1, 1, 1, 1};

  // set to 0 if is in normal range or -1 if incorrect
  if (gas <= 750 && gas >= 400) codes[0] = 0;
  else if (gas < 0) codes[0] = -1;

  if (luminosity >= 50 && luminosity <= 150) codes[1] = 0;
  else if (luminosity < 0) codes[1] = -1;

  if (decibels > 0 && decibels <= 55) codes[2] = 0;
  else if (decibels <= 0) codes[2] = -1;

  if (degrees >= 16 && degrees <= 22) codes[3] = 0;
  else if (degrees < 0) codes[3] = -1;

  if (hPascals >= 1000 && hPascals <= 1050) codes[4] = 0;
  else if (hPascals < 0) codes[4] = -1;

  if (percentage >= 30 && percentage <= 60) codes[5] = 0;
  else if (percentage < 0) codes[5] = -1;

  Quality result;

  int total = 0;
  int totalMax = 0;

  for (int i = 0; i < 6; i++) {
    if (codes[i] != -1) totalMax++;
    if (codes[i] == 0) total++;
    result.qualityArr[i] = codes[i];
  }

  result.qualityColor = calculateQualityColor(total, totalMax);

  return result;
}

// divide the spectrum in equal fragments depending on total count
// of valid measurements
Color calculateQualityColor(int total, int totalMax) {
  int step = 255 / totalMax;
  return Color(step * (totalMax - total), step * total, 0);
}