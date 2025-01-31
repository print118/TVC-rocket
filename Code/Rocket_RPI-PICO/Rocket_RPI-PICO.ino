/*

Keep in Google Drive for easy access.

*/

// Connections:

  // BMP180 (3.3V):
  const int bmpSDA = 20;
  const int bmpSCL = 21;

  // MPU9250 (5V):
  const int mpuSDA = 18;
  const int mpuSCL = 19;

  // HC-12 Radio module (5V):
  const int radioTX = 0; // RXD
  const int radioRX = 1; // TXD

  // GPS (5V):
  const int gpsTX = 4;
  const int gpsRX = 5;

  // Servo (7.4v Battery):
  const int botServoPin = 6;
  const int topServoPin = 7;

  const int ServoMin = 500; // In servo steps
  const int ServoMax = 2500; // ^

  // SD card adapter (5V):
  const int sdCS = 12;
  const int sdSCK = 10;
  const int sdTX = 11; // MOSI
  const int sdRX = 8; // MISO

  // Ignition MOSFETs (7.4v Battery):
  const int motorPin = 2;
  const int chutePin = 3;

  // Buzzer:
  const int buzzerPin = 9; // 9 original

//

// Variables:

  // Gyroscope variables
  float gyroX; // Raw data
  float gyroY; // ^
  float gyroZ; // ^
  float yaw; // Calculated angle
  float pitch; // ^
  float roll; // ^
  float gyroAngleZ; // ^
  float tempMPU; // Temperature

  // Accelerometer variables
  float accX; // Raw data
  float accY; // ^
  float accZ; // ^

  float linear_accX; // Linear
  float linear_accY; // ^
  float linear_accZ; // ^

  float accTotal;
  float accLinearTotal;

  // BMP variables
  float pressure;
  float tempBMP;
  float altitudeBMP;
  float prevAltitude;
  float groundPressure;
  float deltaAltitude;

  // GPS variables
  float altitudeGPS;
  float latitudeGPS;
  float longitudeGPS;

  // HC12
  char endMark = '\n';

  // Battery info
  const int battADC_pin = A2;
  int battADC_val;
  float battVoltage;

  // System variables
  bool sysLock = true; // If true, prevents launch

  int criticalErrorFreq = 400; 
  int successFreq = 2800;

  //unsigned long millis() = 0;
  unsigned long prevTime = 0;
  int timeDelay = 0;

  int folderAmount = 0;

  bool stateBMP;
  bool stateIMU;
  bool stateSD;

  unsigned long apogeeTime;
  unsigned long landTime;
  unsigned long ignitionTime;
  unsigned long liftoffTime;
  unsigned long chuteTime;
  unsigned long pintestTime;
  unsigned long buzzerTime;
  unsigned long ledTime;
  unsigned long startTime;

  bool counterState5 = false;

  bool ignition = false;
  bool pintest = false;
  bool chuteDeployed = false;
  bool oneTimeRun = false;

  int datatestCounter = 20;
  unsigned long motortestTime;

  // PID Controller variables:
  float prevPitch = 0.0;
  float prevRoll = 0.0;

  float deltaPitch;
  float deltaRoll;

  float pPitch;
  float iPitch;
  float dPitch;

  float pRoll;
  float iRoll;
  float dRoll;

  float pitchPID;
  float rollPID;

  int bottomServoSteps;
  int topServoSteps;

  unsigned long prevTimePID;

  bool disableBuzzer = false;

  int state = 0; // Indicates rocket state:
                 // -1 - Motor test
                 //  0 - Idle
                 //  1 - Rocket motor ignition
                 //  2 - Lift-off and/or Flight
                 //  3 - Apogee
                 //  4 - Chutes deployed
                 //  5 - Landing
                 //  6 - Emergency stop (TVC lock, chute deployment, beep and broadcast GPS)

//

#include <Servo.h>
#include "Wire.h"
#include "MPU9250.h"
#include <Adafruit_BMP085.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPSPlus.h>

// Create servo objects
Servo bottomServo;
Servo topServo;

// IMU object
MPU9250 mpu;

// Pressure sensor object
Adafruit_BMP085 bmp;

// SD objects
File root;
File dataFile;

// GPS object
TinyGPSPlus gps;

// Connection pin constants
const int chipSelect = 9; // SD card adapter pin


// Data logging function, takes data log destination as an argument
// "SD" --> Logs to sd card
// "Serial" --> Logs to USB Serial
// "Serial1" --> Logs to Radio
void logSystemDataTo(String destination) {

  if (destination == "SD") {
    dataFile.print(millis());
    dataFile.print(",");
    dataFile.print(tempBMP);
    dataFile.print(",");
    dataFile.print(pressure);
    dataFile.print(",");
    dataFile.print(altitudeBMP);
    dataFile.print(",");
    dataFile.print(latitudeGPS);
    dataFile.print(",");
    dataFile.print(longitudeGPS);
    dataFile.print(",");
    dataFile.print(altitudeGPS);
    dataFile.print(",");
    dataFile.print(tempMPU);
    dataFile.print(",");
    dataFile.print(accX);
    dataFile.print(",");
    dataFile.print(accY);
    dataFile.print(",");
    dataFile.print(accZ);
    dataFile.print(",");
    dataFile.print(accTotal);
    dataFile.print(",");
    dataFile.print(linear_accX);
    dataFile.print(",");
    dataFile.print(linear_accY);
    dataFile.print(",");
    dataFile.print(linear_accZ);
    dataFile.print(",");
    dataFile.print(accLinearTotal);
    dataFile.print(",");
    dataFile.print(pitch);
    dataFile.print(",");
    dataFile.print(yaw);
    dataFile.print(",");
    dataFile.print(roll);
    dataFile.print(",");
    dataFile.print(pitchPID);
    dataFile.print(",");
    dataFile.print(rollPID);
    dataFile.print(",");
    dataFile.print(pPitch);
    dataFile.print(",");
    dataFile.print(iPitch);
    dataFile.print(",");
    dataFile.print(dPitch);
    dataFile.print(",");
    dataFile.print(pRoll);
    dataFile.print(",");
    dataFile.print(iRoll);
    dataFile.print(",");
    dataFile.print(dRoll);
    dataFile.print(",");
    dataFile.print(prevPitch);
    dataFile.print(",");
    dataFile.print(prevRoll);
    dataFile.print(",");
    dataFile.print(deltaPitch);
    dataFile.print(",");
    dataFile.print(deltaRoll);
    dataFile.print(",");
    dataFile.print(topServoSteps);
    dataFile.print(",");
    dataFile.print(bottomServoSteps);
    dataFile.print(",");
    dataFile.print(ignition);
    dataFile.print(",");
    dataFile.print(datatestCounter);
    dataFile.print(",");
    dataFile.print(state);
    dataFile.print(",");
    dataFile.println(sysLock);
    dataFile.flush();
    return;
  }
  else if (destination == "Serial") {
    Serial.print(millis());
    Serial.print(",");
    Serial.print(tempBMP);
    Serial.print(",");
    Serial.print(pressure);
    Serial.print(",");
    Serial.print(altitudeBMP);
    Serial.print(",");
    Serial.print(latitudeGPS);
    Serial.print(",");
    Serial.print(longitudeGPS);
    Serial.print(",");
    Serial.print(altitudeGPS);
    Serial.print(",");
    Serial.print(tempMPU);
    Serial.print(",");
    Serial.print(accX);
    Serial.print(",");
    Serial.print(accY);
    Serial.print(",");
    Serial.print(accZ);
    Serial.print(",");
    Serial.print(accTotal);
    Serial.print(",");
    Serial.print(linear_accX);
    Serial.print(",");
    Serial.print(linear_accY);
    Serial.print(",");
    Serial.print(linear_accZ);
    Serial.print(",");
    Serial.print(accLinearTotal);
    Serial.print(",");
    Serial.print(pitch);
    Serial.print(",");
    Serial.print(yaw);
    Serial.print(",");
    Serial.print(roll);
    Serial.print(",");
    Serial.print(pitchPID);
    Serial.print(",");
    Serial.print(rollPID);
    Serial.print(",");
    Serial.print(pPitch);
    Serial.print(",");
    Serial.print(iPitch);
    Serial.print(",");
    Serial.print(dPitch);
    Serial.print(",");
    Serial.print(pRoll);
    Serial.print(",");
    Serial.print(iRoll);
    Serial.print(",");
    Serial.print(dRoll);
    Serial.print(",");
    Serial.print(prevPitch);
    Serial.print(",");
    Serial.print(prevRoll);
    Serial.print(",");
    Serial.print(deltaPitch);
    Serial.print(",");
    Serial.print(deltaRoll);
    Serial.print(",");
    Serial.print(topServoSteps);
    Serial.print(",");
    Serial.print(bottomServoSteps);
    Serial.print(",");
    Serial.print(ignition);
    Serial.print(",");
    Serial.print(datatestCounter);
    Serial.print(",");
    Serial.print(state);
    Serial.print(",");
    Serial.println(sysLock);
    return;
  }
  else if (destination == "Serial1") {
    Serial1.print(millis());
    Serial1.print(",");
    Serial1.print(tempBMP);
    Serial1.print(",");
    Serial1.print(pressure);
    Serial1.print(",");
    Serial1.print(altitudeBMP);
    Serial1.print(",");
    Serial1.print(latitudeGPS);
    Serial1.print(",");
    Serial1.print(longitudeGPS);
    Serial1.print(",");
    Serial1.print(altitudeGPS);
    Serial1.print(",");
    Serial1.print(tempMPU);
    Serial1.print(",");
    Serial1.print(accX);
    Serial1.print(",");
    Serial1.print(accY);
    Serial1.print(",");
    Serial1.print(accZ);
    Serial1.print(",");
    Serial1.print(accTotal);
    Serial1.print(",");
    Serial1.print(linear_accX);
    Serial1.print(",");
    Serial1.print(linear_accY);
    Serial1.print(",");
    Serial1.print(linear_accZ);
    Serial1.print(",");
    Serial1.print(accLinearTotal);
    Serial1.print(",");
    Serial1.print(pitch);
    Serial1.print(",");
    Serial1.print(yaw);
    Serial1.print(",");
    Serial1.print(roll);
    Serial1.print(",");
    Serial1.print(pitchPID);
    Serial1.print(",");
    Serial1.print(rollPID);
    Serial1.print(",");
    Serial1.print(pPitch);
    Serial1.print(",");
    Serial1.print(iPitch);
    Serial1.print(",");
    Serial1.print(dPitch);
    Serial1.print(",");
    Serial1.print(pRoll);
    Serial1.print(",");
    Serial1.print(iRoll);
    Serial1.print(",");
    Serial1.print(dRoll);
    Serial1.print(",");
    Serial1.print(prevPitch);
    Serial1.print(",");
    Serial1.print(prevRoll);
    Serial1.print(",");
    Serial1.print(deltaPitch);
    Serial1.print(",");
    Serial1.print(deltaRoll);
    Serial1.print(",");
    Serial1.print(topServoSteps);
    Serial1.print(",");
    Serial1.print(bottomServoSteps);
    Serial1.print(",");
    Serial1.print(ignition);
    Serial1.print(",");
    Serial1.print(datatestCounter);
    Serial1.print(",");
    Serial1.print(state);
    Serial1.print(",");
    Serial1.println(sysLock);
    return;
  }
  else {
    Serial.print("Given destination not correct: (");
    Serial.print(destination);
    Serial.print(")");
    Serial.println("");
  }


}

void criticalError(bool stopBool) {
  if (!disableBuzzer) {
    tone(buzzerPin, criticalErrorFreq);
    delay(300);
    noTone(buzzerPin);
    delay(50);
    tone(buzzerPin, criticalErrorFreq);
    delay(300);
    noTone(buzzerPin);
    delay(1000);
  }
  if (stopBool) {
    while (1) {}
  }
}

void successBeep() {
  if (disableBuzzer) {return;}
  tone(buzzerPin, successFreq);
  delay(200);
  noTone(buzzerPin);
  delay(50);
  tone(buzzerPin, successFreq);
  delay(200);
  noTone(buzzerPin);
}

float calcAltitude() {
  float alt;
  //alt = 44330 * (1.0 - pow(pressure / groundAirPressure, 0.1903));
  alt = ( (pow((groundPressure/pressure), (1/5.257)) - 1)*(tempBMP+273.15) )/0.0065;
  return alt;
  
}

void printDirectory(File dir, int numTabs) {
  while (true) {

    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    /*for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('a');
    }*/
    //Serial.print(entry.name());
    if (entry.isDirectory()) {
      //Serial.println(" ");
      folderAmount += 1;
      printDirectory(entry, numTabs + 1);
    } 
    
    entry.close();
  }
}

void calcPID() {

  // Define gain values 
  float pGain = 0.5;
  float iGain = 0.02;
  float dGain = 0.15;

  // Calculate PID angle
  deltaPitch = pitch - prevPitch;
  deltaRoll = roll - prevRoll;

  pPitch = pitch*pGain;
  iPitch += pitch*iGain;
  dPitch = (deltaPitch/(millis()-prevTimePID))*dGain;

  pRoll = roll*pGain;
  iRoll += roll*iGain;
  dRoll = (deltaRoll/(millis()-prevTimePID))*dGain;

  // Resulting required angle
  pitchPID = pPitch + iPitch + dPitch; // Bottom servo
  rollPID = pRoll + iRoll + dRoll; // Top servo

  // Flip pitch and roll directions
  pitchPID *= -1;
  rollPID *= -1;

  prevPitch = pitch;
  prevRoll = roll;
  prevTimePID = millis();

  if (state == -1) {
    pitchPID = pitch;
    rollPID = roll;
  }

  // Set angle
  bottomServoSteps = bottomAngletoStep(pitchPID);
  topServoSteps = topAngletoStep(rollPID);

  return;

}

int topAngletoStep(float x) {

  struct data {
    float angle;
    int servo;
  } topCalibrationData[] = {
    {15.31,2500},
    {14.40,2490},
    {14.22,2480},
    {14.12,2470},
    {13.79,2460},
    {13.10,2450},
    {12.58,2440},
    {12.00,2430},
    {10.98,2420},
    {10.60,2410},
    {9.94,2400},
    {9.36,2390},
    {8.27,2380},
    {7.44,2370},
    {7.05,2360},
    {6.26,2350},
    {5.73,2340},
    {5.35,2330},
    {4.81,2320},
    {4.12,2310},
    {3.55,2300},
    {3.10,2290},
    {2.49,2280},
    {1.51,2270},
    {1.12,2260},
    {0.68,2250},
    {0.22,2240},
    {-0.27,2230},
    {-0.72,2220},
    {-1.36,2210},
    {-2.01,2200},
    {-2.38,2190},
    {-2.65,2180},
    {-3.26,2170},
    {-3.78,2160},
    {-4.23,2150},
    {-4.65,2140},
    {-4.98,2130},
    {-5.39,2120},
    {-5.77,2110},
    {-6.18,2100},
    {-6.48,2090},
    {-6.90,2080},
    {-7.31,2070},
    {-7.81,2060},
    {-8.07,2050},
    {-8.36,2040},
    {-8.67,2030},
    {-8.99,2020},
    {-9.37,2010},
    {-9.72,2000},
    {-10.07,1990},
    {-10.39,1980},
    {-10.69,1970},
    {-10.90,1960},
    {-11.22,1950},
    {-11.41,1940},
    {-11.62,1930},
    {-11.85,1920},
    {-12.00,1910},
    {-12.20,1900}
  };

  int length = sizeof topCalibrationData / sizeof topCalibrationData[0];

  // Check if given x value is outside data range and snap to closest value if outside
  if (x >= topCalibrationData[0].angle) {
    Serial.println("topAngletoStep input angle out of bounds");
    Serial.println("");
    return topCalibrationData[0].servo;
  }
  if (x <= topCalibrationData[length-1].angle) {
    Serial.println("topAngletoStep input angle out of bounds");
    Serial.println("");
    return topCalibrationData[length-1].servo;
  }

  for (int i = 0; i <= length-1; i++ ) {
    if (x <= topCalibrationData[i].angle && x > topCalibrationData[i+1].angle) {

      int steps = (map(x*100, topCalibrationData[i].angle*100, topCalibrationData[i+1].angle*100, topCalibrationData[i].servo, topCalibrationData[i+1].servo));

      return steps;
    }
  }

  Serial.print("topAngletoStep function error, value of x: ");
  Serial.println(x);
  Serial.println("");
  Serial1.print("topAngletoStep function error, value of x: ");
  Serial.print("topAngletoStep function error, value of x: ");
  Serial1.println(x);
  Serial.println(x);
  Serial1.println("");
  Serial.println("");
  return topServoSteps;

}

int bottomAngletoStep(float x) {

  struct data {
    float angle;
    int servo;
  } bottomCalibrationData[] = {
    {11.00,2350},
    {10.81,2340},
    {10.43,2330},
    {10.45,2320},
    {9.00,2310},
    {8.42,2300},
    {8.57,2290},
    {7.64,2280},
    {7.20,2270},
    {6.46,2260},
    {6.28,2250},
    {5.60,2240},
    {5.04,2230},
    {4.58,2220},
    {4.04,2210},
    {3.63,2200},
    {3.03,2190},
    {2.52,2180},
    {1.90,2170},
    {1.57,2160},
    {1.01,2150},
    {0.55,2140},
    {0.08,2130},
    {-0.35,2120},
    {-0.70,2110},
    {-1.09,2100},
    {-1.56,2090},
    {-1.92,2080},
    {-2.35,2070},
    {-2.87,2060},
    {-3.36,2050},
    {-3.72,2040},
    {-4.09,2030},
    {-4.44,2020},
    {-4.70,2010},
    {-4.99,2000},
    {-5.34,1990},
    {-5.77,1980},
    {-6.19,1970},
    {-6.60,1960},
    {-6.89,1950},
    {-4.70,2010},
    {-4.99,2000},
    {-5.34,1990},
    {-5.77,1980},
    {-6.19,1970},
    {-6.60,1960},
    {-6.89,1950},
    {-7.32,1940},
    {-7.76,1930},
    {-8.07,1920},
    {-8.44,1910},
    {-8.75,1900},
    {-9.04,1890},
    {-9.40,1880},
    {-9.79,1870},
    {-10.15,1860},
    {-10.38,1850},
    {-10.69,1840},
    {-10.93,1830},
    {-11.28,1820},
    {-11.51,1810},
    {-11.64,1800},
    {-11.95,1790},
    {-12.17,1780},
    {-12.34,1770},
    {-12.53,1760},
    {-12.75,1750},
    {-13.05,1740},
    {-13.17,1730},
    {-13.35,1720},
    {-13.43,1710},
    {-13.43,1700}
  };

  int length = sizeof bottomCalibrationData / sizeof bottomCalibrationData[0];

  // Check if given x value is outside data range and snap to closest value if outside
  if (x >= bottomCalibrationData[0].angle) {
    Serial.println("bottomAngletoStep input angle out of bounds");
    Serial.println("");
    return bottomCalibrationData[0].servo;
  }
  if (x <= bottomCalibrationData[length-1].angle) {
    Serial.println("bottomAngletoStep input angle out of bounds");
    Serial.println("");
    return bottomCalibrationData[length-1].servo;
  }

  for (int i = 0; i <= length-1; i++ ) {
    if (x <= bottomCalibrationData[i].angle && x > bottomCalibrationData[i+1].angle) {

      int steps = (map(x*100, bottomCalibrationData[i].angle*100, bottomCalibrationData[i+1].angle*100, bottomCalibrationData[i].servo, bottomCalibrationData[i+1].servo));

      return steps;
    }
  }

  Serial.print("bottomAngletoStep function error, value of x: ");
  Serial.println(x);
  Serial.println("");
  Serial1.print("bottomAngletoStep function error, value of x: ");
  Serial.print("bottomAngletoStep function error, value of x: ");
  Serial1.println(x);
  Serial.println(x);
  Serial1.println("");
  Serial.println("");
  return bottomServoSteps;

}

void setup() {
  
  // USB serial
  Serial.begin(115200);

  delay(3000);

  // Radio module
  Serial1.setTX(radioTX);
  Serial1.setRX(radioRX);
  Serial1.begin(9600);

  // GPS module
  Serial2.setTX(gpsTX);
  Serial2.setRX(gpsRX);
  Serial2.begin(9600);

  // Set the servo pins
  bottomServo.attach(botServoPin, ServoMin, ServoMax);
  topServo.attach(topServoPin, ServoMin, ServoMax);
  bottomServo.writeMicroseconds(bottomAngletoStep(0.0));
  topServo.writeMicroseconds(topAngletoStep(0.0));

  // Define correct pins for Wire (Barometer) and Wire1 (IMU), start connections
  //Wire.setClock(3400000);
  Wire.setSDA(bmpSDA);
  Wire.setSCL(bmpSCL);
  Wire1.setSDA(mpuSDA);
  Wire1.setSCL(mpuSCL);
  
  //Wire.setClock(3400000); //??
  Wire1.begin();

  // Define SD Card adapter pins
  SPI1.setSCK(sdSCK);
  //SPI1.setCS(sdCS);
  //Serial.println("h");
  SPI1.setTX(sdTX);
  SPI1.setRX(sdRX);

  // pinModes
  Serial.println("Setting pins");
  Serial.println("");
  pinMode(buzzerPin, OUTPUT); // Buzzer pin
  pinMode(motorPin, OUTPUT);
  pinMode(chutePin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  stateBMP = bmp.begin(0);
  stateSD = SD.begin(sdCS, SPI1);
  stateIMU = mpu.setup(0x68, Wire1);

  Serial.println("Testing sensor validity");
  Serial.println("");

  if (!stateBMP) {
    Serial.println("BMP failure, is the BMP connected properly?");
    Serial1.println("BMP failure, is the BMP connected properly?");
  }
  if (!stateSD) {
    Serial.println("SD card reader failure, is the SD card plugged in?");
    Serial1.println("SD card reader failure, is the SD card plugged in?");
  }
  if (!stateIMU) {
    Serial.println("IMU failure, is the IMU connected properly?");
    Serial1.println("IMU failure, is the IMU connected properly?");
  }

  if (!stateBMP || !stateSD || !stateIMU) {
    Serial.println("Critical Error");
    Serial1.println("Critical Error");
    criticalError(1);
  }

  // Prepare SD Card, create data folder, etc.
  Serial.println("Accessing SD card and creating datalog...");
  root = SD.open("/Logs");
  printDirectory(root, 0);


  // Create string for directory location and creates it
  String createDir = "/Logs/" + String(folderAmount+1);
  SD.mkdir(createDir);
  
  createDir += "/datalog.csv";
  dataFile = SD.open(createDir, FILE_WRITE);

  // Checks of dataFile could be opened, if not --> criticalError()
  if (!dataFile) {
    Serial1.println("datalog.csv file couldn't be opened,");
    Serial.println("datalog.csv file couldn't be opened,");
    Serial1.println("");
    Serial.println("");
    criticalError(1);
  }

  // Writes headings for sensor data
    dataFile.print("currentTime");
    dataFile.print(",");
    dataFile.print("tempBMP");
    dataFile.print(",");
    dataFile.print("pressure");
    dataFile.print(",");
    dataFile.print("altitudeBMP");
    dataFile.print(",");
    dataFile.print("latitude");
    dataFile.print(",");
    dataFile.print("longitude");
    dataFile.print(",");
    dataFile.print("altitudeGPS");
    dataFile.print(",");
    dataFile.print("tempMPU");
    dataFile.print(",");
    dataFile.print("accX");
    dataFile.print(",");
    dataFile.print("accY");
    dataFile.print(",");
    dataFile.print("accZ");
    dataFile.print(",");
    dataFile.print("accTotal");
    dataFile.print(",");
    dataFile.print("linear_accX");
    dataFile.print(",");
    dataFile.print("linear_accY");
    dataFile.print(",");
    dataFile.print("linear_accZ");
    dataFile.print(",");
    dataFile.print("accLinearTotal");
    dataFile.print(",");
    dataFile.print("pitch");
    dataFile.print(",");
    dataFile.print("yaw");
    dataFile.print(",");
    dataFile.print("roll");
    dataFile.print(",");
    dataFile.print("pitchPID");
    dataFile.print(",");
    dataFile.print("rollPID");
    dataFile.print(",");
    dataFile.print("pPitch");
    dataFile.print(",");
    dataFile.print("iPitch");
    dataFile.print(",");
    dataFile.print("dPitch");
    dataFile.print(",");
    dataFile.print("pRoll");
    dataFile.print(",");
    dataFile.print("iRoll");
    dataFile.print(",");
    dataFile.print("dRoll");
    dataFile.print(",");
    dataFile.print("prevPitch");
    dataFile.print(",");
    dataFile.print("prevRoll");
    dataFile.print(",");
    dataFile.print("deltaPitch");
    dataFile.print(",");
    dataFile.print("deltaRoll");
    dataFile.print(",");
    dataFile.print("topServoSteps");
    dataFile.print(",");
    dataFile.print("bottomServoSteps");
    dataFile.print(",");
    dataFile.print("ignition");
    dataFile.print(",");
    dataFile.print("datatestCounter");
    dataFile.print(",");
    dataFile.print("state");
    dataFile.print(",");
    dataFile.println("sysLock");
  

  Serial.println("SD card ready");
  Serial.println("");


  // Tone to indicate ground placement

  if (!disableBuzzer) {
    tone(buzzerPin, successFreq);
    delay(400);
    noTone(buzzerPin);
  }

  // Gyroscope calculating offsets
  Serial1.println("Gyroscope and Accelerometer calibration in 5s.");
  Serial.println("Gyroscope and Accelerometer calibration in 5s.");
  Serial1.println("Leave rocket in upright position on steady ground...");
  Serial.println("Leave rocket in upright position on steady ground...");
  mpu.verbose(true);
  delay(5000);
  mpu.calibrateAccelGyro();
  Serial1.println("Gyro and Accelerometer calibrated.");
  Serial.println("Gyro and Accelerometer calibrated.");
  Serial1.println("");
  Serial.println("");

  /*// Magnetometer calibration
  Serial1.println("Magnetometer calibration in 5s.");
  Serial.println("Magnetometer calibration in 5s.");
  Serial1.println("Wave rocket around in all directions, trying to look in every direction with it...");
  Serial.println("Wave rocket around in all directions, trying to look in every direction with it...");
  delay(3000);
  mpu.calibrateMag();
  Serial1.println("Magnetometer calibrated.");
  Serial.println("Magnetometer calibrated.");
  Serial1.println("");
  Serial.println("");

  mpu.verbose(false);
  mpu.setFilterIterations(10);
  mpu.setMagneticDeclination(12.47);*/
  Serial1.println("IMU calibration complete.");
  Serial.println("IMU calibration complete.");
  Serial1.println("");
  Serial.println("");

  Serial1.println("Sampling air pressure average for altitude calculation...");
  Serial.println("Sampling air pressure average for altitude calculation...");
  // Take average of current ground air pressure
  for (int i = 0; i < 10; i++) {
    groundPressure += bmp.readPressure();
    delay(10);   
  }
  Serial1.println("Pressure sampling complete.");
  Serial.println("Pressure sampling complete.");
  Serial1.println("");
  Serial.println("");
  
  groundPressure = (groundPressure/10);
  Serial1.println("Rocket setup complete, waiting for commands...");
  Serial1.println("");
  Serial.println("Rocket setup complete, waiting for commands...");
  Serial.println("");
  
  successBeep();

  startTime = millis();
  

}

void loop() {
  //millis() = millis();

  while (Serial2.available()) {
    gps.encode(Serial2.read());
  }

  // Update MPU-9250
  mpu.update();

  // Radio communication
  byte rxByte;
  String rxData;
  if (Serial1.available()) {
    Serial.println("Receiving radio data...");
    while (char(rxByte) != endMark) {
      if (Serial1.available()) {
        rxByte = Serial1.read();
        rxData += char(rxByte);
      }
      //digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
    Serial.print("Radio data received: ");
    Serial.println(rxData);
    Serial.println("");
  }
  rxData.trim();

  // Disable safety
  if (rxData == "safetyOFF" && state == 0) {
    Serial1.println("Disable safety command received");
    Serial1.println("");
    sysLock = false;
  }

  // Enable safety
  if (rxData == "safetyON" && state == 0) {
    Serial1.println("Enable safety command received");
    Serial1.println("");
    sysLock = true;
  }

  // Activate on launch command
  if (rxData == "launch" && state == 0 && !sysLock) {
    Serial1.println("Launch command received");
    Serial1.println("");
    Serial.println("Launch command received");
    Serial.println("");
    ignition = true;
  }
  else if (rxData == "launch" && state != 0) {
    Serial1.println("Error: Launch command received while rocket was not idle");
    Serial1.println("");
    Serial.println("Error: Launch command received while rocket was not idle");
    Serial.println("");
  }
  else if (rxData == "launch" && sysLock) {
    Serial1.println("Can't ignite rocket motor, if safety is not disengaged!");
    Serial1.println("");
    Serial.println("Can't ignite rocket motor, if safety is not disengaged!");
    Serial.println("");
  }

  // Run once after time period, when rocket start finished
  /*if (millis()-startTime >= 5000 && !oneTimeRun) {
    rxData = "pintest";
    oneTimeRun = true;
  }*/

  if (rxData == "pintest" && state == 0) {
    Serial1.println("MOSFET pin test command received");
    Serial1.println("");
    Serial.println("MOSFET pin test command received");
    Serial.println("");
    //delay(500);
    digitalWrite(motorPin, HIGH);
    digitalWrite(chutePin, HIGH);
    pintest = true;
    pintestTime = millis();
  }
  if (pintest && ((millis()-pintestTime) >= 2000)) {
    Serial1.println("MOSFET pin test finished");
    Serial1.println("");
    Serial.println("MOSFET pin test finished");
    Serial.println("");
    //delay(500);
    digitalWrite(motorPin, LOW);
    digitalWrite(chutePin, LOW);
    pintest = false;
  }

  if (rxData == "chute") {
    Serial1.println("Manual parachute activation command received");
    Serial1.println("Switching state");
    Serial1.println("");
    
    state = 3; // Set to apogee, which triggers parachute
  }

  if (rxData == "datatest" && state == 0) {
    datatestCounter = 0;
    logSystemDataTo("Serial1");
  }
  if (rxData == "datatest" || (datatestCounter < 10 && state == 0)) {
    logSystemDataTo("Serial1");
    datatestCounter += 1;
  }

  if (rxData == "motortest" && state == 0) {
    Serial1.println("Motortest command received");
    Serial1.println("");
    state = -1;
    motortestTime = millis();
  }
  if (state == -1 && (millis()-motortestTime) > 5000 ) {
    Serial1.println("Motortest done");
    Serial1.println("");
    state = 0;
  }

  if (rxData == "calibrateIMU" && state == 0) {
    Serial1.println("Calibrating gyroscope and accelerometer, keep the rocket still...");
    mpu.calibrateAccelGyro();
    Serial1.println("Done");
    Serial1.println("");
  }
  else if (rxData == "calibrateIMU") {
    Serial1.println("Error: Can't calibrate gyro while not idle");
    Serial1.println("");
  }

  if (rxData == "calibrateMAG" && state == 0) {
    Serial1.println("Calibrating magnetometer, code will be stuck until complete, rotate rocket in all directions...");
    mpu.calibrateMag();
    Serial1.println("Done");
    Serial1.println("");
  }
  else if (rxData == "calibrateMAG") {
    Serial1.println("Error: Can't calibrate magnetometer while not idle");
    Serial1.println("");
  }




  // Data logging and motor control
  if (millis()-prevTime >= timeDelay) {

    // Process barometer data
    tempBMP = bmp.readTemperature();
    pressure = bmp.readPressure();
    altitudeBMP = calcAltitude();

    if (millis()-prevTime >= 1000) {
      deltaAltitude = altitudeBMP - prevAltitude;
      prevAltitude = altitudeBMP;
    }


    // Process GPS data
    while (Serial2.available()) {
      gps.encode(Serial2.read());
    }
    if (gps.altitude.isValid()) {
      altitudeGPS = gps.altitude.meters();
    }
    if (gps.location.isValid()) {
      latitudeGPS = gps.location.lat();
      longitudeGPS = gps.location.lng();
    }  

    // Process IMU data
    if (mpu.update()) {
      yaw = mpu.getYaw();
      pitch = mpu.getPitch();
      roll = mpu.getRoll();

      linear_accX = mpu.getLinearAccX();
      linear_accY = mpu.getLinearAccY();
      linear_accZ = mpu.getLinearAccZ();

      accX = mpu.getAccX();
      accY = mpu.getAccY();
      accZ = mpu.getAccZ();

      tempMPU = mpu.getTemperature();

      accTotal = sqrt( sq(accX) + sq(accY) + sq(accZ) );
      accLinearTotal = sqrt( sq(linear_accX) + sq(linear_accY) + sq(linear_accZ) );

    }

    /*Serial.print(yaw);
    Serial.print(",");
    Serial.print(pitch);
    Serial.print(",");
    Serial.println(roll);*/


    // Get battery voltage
    /*battADC_val = analogRead(battADC_pin);
    battVoltage = battADC_val*(3.3/1023)*(4/3);*/

    // ---------------------------------------- //



    if (state == 0 && ignition && !sysLock) {
      state = 1; // Motor ignition state
      digitalWrite(motorPin, HIGH);
      ignitionTime = millis();
      Serial.println("Rocket motor ignited");
      Serial.println("");
    }
    else if (state == 0 && ignition && sysLock) {
      Serial1.println("Safety not disengaged, can't fire!");
      Serial1.println("");
      ignition = false;
    }
    
    // Check for lift-off after motor ignition
    if (state == 1 && linear_accZ >= 1) { // Value of 1 comes from the rockets predicted acceleration in g's with a little headroom
      liftoffTime = millis();             // Remove "&& linear_accZ >= 1" to switch state without liftoff detection
      state = 2; // Flight state
      digitalWrite(motorPin, LOW);
      Serial.println("Liftoff detected");
      Serial.println("");
    }


    if (state == 2 || state == -1) {
      // Calculate servo angles
      calcPID();
      
      // Move rocket motor
      bottomServo.write(bottomServoSteps);
      topServo.write(topServoSteps);

    }

    // Report if at apogee
    if ( deltaAltitude < -3.0 && state == 2) { // If change in altitude is below -3m
      state = 3; // Apogee state
      apogeeTime = millis();
      Serial.println("Apogee reached");
      Serial.println("");
    }

    // Deploy parachute if at apogee
    if (state == 3) {
      chuteTime = millis();
      digitalWrite(chutePin, HIGH);
      state = 4; // Parachute deployed state
      chuteDeployed = true;
      Serial.println("Chutes deployed");
      Serial.println("");
    }

    if ( ((millis()-chuteTime) >= 2000) && chuteDeployed) {
      digitalWrite(chutePin, LOW);
      chuteDeployed = false;
    }

    if (state == 4 && accLinearTotal <= 0.3) { // If total acceleration (not linear) is 1.5g!!! or below
      if ( (millis()-landTime) >= 2000) {                                   //REMEMBER THE G'SS!!
        state = 5; // Landing confirmed state
        Serial.println("Rocket landed");
        Serial.println("");
      }
    }
    else if (state == 4) {
      landTime = millis();
    }

    if (state > 0) {
      // Broadcast GPS location to radio
      Serial1.print(latitudeGPS);
      Serial1.print(", ");
      Serial1.println(longitudeGPS);
      /*Serial.print(latitudeGPS);
      Serial.print(", ");
      Serial.println(longitudeGPS);*/
    }

    // Store data on SD card
    if (state != 5) {
      logSystemDataTo("SD");
    }

    if (state == 5 && !counterState5) {
      dataFile.println("");
      dataFile.println("Flight end.");
      dataFile.print("Landing time: ");
      dataFile.println(landTime);
      dataFile.print("Flight duration: ");
      dataFile.print((landTime-ignitionTime)/1000);
      dataFile.println("s");
      dataFile.flush();
      counterState5 = true;
    }

    prevTime = millis();
  }

  // Buzzer timer
  if ((millis()-buzzerTime) >= 1000 && !sysLock && !disableBuzzer) {
    tone(buzzerPin, 2000, 200);
    buzzerTime = millis();
  }

  // LED blink activity indicator
  if (millis()-ledTime >= 10) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    ledTime = millis();
  }


}
