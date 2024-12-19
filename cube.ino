/************************** Neopixels **************************/
#include <Adafruit_NeoPixel.h>
#define LED_PIN 0
#define LED_COUNT 5
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
long firstPixelHue = 0;
unsigned long previousTimeLeds = millis();
long timeIntervalLeds = 10;

/************************** IMU **************************/
#include "LSM6DS3.h"
#include "Wire.h"
#include "SPI.h"

//Create instance of LSM6DS3Core
LSM6DS3Core imu(I2C_MODE, 0x6A);    //I2C device address 0x6A
unsigned long previousTimeIMU = 0;

int previous_x, previous_y, previous_z;

// Enable IMU 6d orientation detection.
int imu_enable_6d_orientation()
{
  uint8_t dataToWrite = 0;
  //Setup the accelerometer******************************
  dataToWrite = 0; //Start Fresh!
  //dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_100Hz;
  dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_2g;
  dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_416Hz;

  uint16_t err = 0;
  err += imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);
  if (err){
    Serial.print("Error while setting XL: ");
    Serial.println(err);
    return err;
  }

  dataToWrite = 0; //Start Fresh!
  dataToWrite |= LSM6DS3_ACC_GYRO_SIXD_THS_70_degree;
  err += imu.writeRegister(LSM6DS3_ACC_GYRO_TAP_THS_6D, dataToWrite);
  if (err){
    Serial.print("Error while setting 6d threshold: ");
    Serial.println(err);
    return err;
  }

  dataToWrite = 0; //Start Fresh!
  dataToWrite |= 0x80;
  err += imu.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, dataToWrite);
  if (err){
    Serial.print("Error while setting interrupts: ");
    Serial.println(err);
    return err;
  }

  dataToWrite = 0; //Start Fresh!
  dataToWrite |= LSM6DS3_ACC_GYRO_INT1_6D_ENABLED;
  err += imu.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, dataToWrite);
  if (err){
    Serial.print("Error while routing 6D to int1: ");
    Serial.println(err);
    return err;
  }
}

// Read 6D orientation from IMU.
int read_6d(int *x, int *y, int *z)
{
  int err = 0;
  uint8_t reg;
  err += imu.readRegister(&reg, LSM6DS3_ACC_GYRO_D6D_SRC);
  if (err){
    Serial.print("Error while reading 6d orientation: ");
    Serial.println(err);
  }

  *x = 0;
  *y = 0;
  *z = 0;

  switch (reg & LSM6DS3_ACC_GYRO_DSD_XL_DETECTED){
    case LSM6DS3_ACC_GYRO_DSD_XL_DETECTED:
      *x = -1;
      break;
  }

  switch (reg & LSM6DS3_ACC_GYRO_DSD_XH_DETECTED){
    case LSM6DS3_ACC_GYRO_DSD_XH_DETECTED:
      *x = 1;
      break;
  }

  switch (reg & LSM6DS3_ACC_GYRO_DSD_YL_DETECTED){
    case LSM6DS3_ACC_GYRO_DSD_YL_DETECTED:
      *y = -1;
      break;
  }

  switch (reg & LSM6DS3_ACC_GYRO_DSD_YH_DETECTED){
    case LSM6DS3_ACC_GYRO_DSD_YH_DETECTED:
      *y = 1;
      break;
  }

  switch (reg & LSM6DS3_ACC_GYRO_DSD_ZL_DETECTED){
    case LSM6DS3_ACC_GYRO_DSD_ZL_DETECTED:
      *z = -1;
      break;
  }

  switch (reg & LSM6DS3_ACC_GYRO_DSD_ZH_DETECTED){
    case LSM6DS3_ACC_GYRO_DSD_ZH_DETECTED:
      *z = 1;
      break;
  }
}

/************************** MP3 Player **************************/
#include "DFRobotDFPlayerMini.h"
#define FPSerial Serial1
#define VOLUME_PIN A3
int volume = 15;

DFRobotDFPlayerMini myDFPlayer;

// Print DFPlayer communication details.
void printDetail(uint8_t type, int value) {
  switch (type) {
    case TimeOut:
      Serial.println(F("Time Out!"));
      break;
    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;
    case DFPlayerUSBInserted:
      Serial.println("USB Inserted!");
      break;
    case DFPlayerUSBRemoved:
      Serial.println("USB Removed!");
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          Serial.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}

/************************** Setup **************************/
void setup()
{
  // Setting up built-in red LED.
  pinMode(LED_BUILTIN, OUTPUT);

  // Setting up neopixel strip.
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)

  //Init Serial port
  Serial.begin(9600);
//  while (!Serial);

  // Setting up MP3 player.
  FPSerial.begin(9600);
  if (!myDFPlayer.begin(FPSerial, /*isACK = */ true, /*doReset = */ true)) {  //Use serial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    digitalWrite(LED_BUILTIN, LOW);   
    while (true) {
      delay(0);  // Code to compatible with ESP8266 watch dog.
    }
  }
  Serial.println(F("DFPlayer Mini online."));

  myDFPlayer.volume(volume);  //Set volume value. From 0 to 30

  // Setting up IMU.
  //Call .beginCore() to configure the IMU
  if (imu.beginCore() != 0) {
      Serial.println("Device Error!");
  } else {
      Serial.println("Device OK!");
  }

  imu_enable_6d_orientation();
}

void loop() {
  // Check if volume needs to be changed
  int new_volume = analogRead(VOLUME_PIN);  // read the input pin
  if (new_volume>=1010) {
    // Prevent max value to oscillate between 29 and 30.
    new_volume = 1023;
  }
  new_volume = map(new_volume, 0, 1023, 1, 30);
  if (new_volume != volume)
  {
    volume = new_volume;
    Serial.print("New volume: ");
    Serial.println(volume);
    myDFPlayer.volume(volume);  //Set volume value. From 0 to 30
  }

  // Update neopixels color.
  // Took the rainbow function of the strandtest sample from the Adafruit Neopixel library.
  if (firstPixelHue >= 5*65536) {
    firstPixelHue=0;
  }
  unsigned long currentTime = millis();
  if(currentTime - previousTimeLeds > timeIntervalLeds) {
    previousTimeLeds = currentTime;
    strip.rainbow(firstPixelHue);
    strip.show(); // Update strip with new contents
    firstPixelHue+=256;
  }

  // Check if music needs to be changed.
  int val = digitalRead(PIN_LSM6DS3TR_C_INT1);
  if (val){
    int err = 0;
    uint8_t status, md1_cfg;
    err += imu.readRegister(&status, LSM6DS3_ACC_GYRO_D6D_SRC);
    if (err){
      Serial.print("Error while reading 6d status: ");
      Serial.println(err);
    }

    err += imu.readRegister(&md1_cfg, LSM6DS3_ACC_GYRO_MD1_CFG);
    if (err){
      Serial.print("Error while reading 6d status: ");
      Serial.println(err);
    }

    if (md1_cfg & LSM6DS3_ACC_GYRO_INT1_6D_ENABLED) {
      if (status & LSM6DS3_ACC_GYRO_D6D_EV_STATUS_DETECTED) {
        int x, y, z;
        read_6d(&x, &y, &z);
        Serial.print("X: ");
        Serial.print(x);
        Serial.print(" Y: ");
        Serial.print(y);
        Serial.print(" Z: ");
        Serial.println(z);

        if (previous_x != x || previous_y != y || previous_z != z){
          unsigned long currentTimeIMU = millis();
          // Prevent changing multiple tracks at once because several interrupts are often triggered at once.
          if (currentTimeIMU - previousTimeIMU >= 1000)
          {
            myDFPlayer.next();
            previousTimeIMU = currentTimeIMU;
          }
        }

        previous_x = x;
        previous_y = y;
        previous_z = z;
      }
    }
  }

  // Check for error in DFPlayer libary.
  if (myDFPlayer.available()) {
    printDetail(myDFPlayer.readType(), myDFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states.
  }
}
