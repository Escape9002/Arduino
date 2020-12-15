/*
   seems like the protocol got problems with every characteristic besides string. test it. rewrite APP inventor and this shit
*/
//-------------------------------------------------------------------------------------BIBLIOTHEK
//---------------------------------------------------BLE
#include <ArduinoBLE.h>
//---------------------------------------------------MPU9250
#include "MPU9250.h"
// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);
int status;
//-------------------------------------------------------------------------------------BIBLIOTHEK
//-------------------------------------------------------------------------------------VARIABLEN
//---------------------------------------------------Acclereator
int accelX = 1;
int accelY = 1;
int accelZ = 1;
//---------------------------------------------------Gyroscope
int gyroX = 1;
int gyroY = 1;
int gyroZ = 1;
//---------------------------------------------------magnetic sensor
int magnetX = 1;
int magnetY = 1;
int magnetZ = 1;
//---------------------------------------------------Temperatur
int temp = 1;
int PotiValue = 1;
//-------------------------------------------------------------------------------------VARIABLEN
//-------------------------------------------------------------------------------------BLE_SETUP
BLEService SendingService("1101");
BLEUnsignedIntCharacteristic accelXChar("2101", BLERead | BLENotify);
BLEUnsignedIntCharacteristic accelYChar("2102", BLERead | BLENotify);
BLEUnsignedIntCharacteristic accelZChar("2103", BLERead | BLENotify);
//-------------------------------------------------------------------------------------BLE_SETUP
//-------------------------------------------------------------------------------------//-------------------------------------------------------------------------------------//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------SETUP()
void setup() {
  Serial.begin(115200);

  //---------------------------------------------------MPU9250 Setup

  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }

  //---------------------------------------------------BLE Setup

  if (!BLE.begin()) {
    Serial.println("BLE failed to Initiate");
    delay(500);
    while (1);
  }

  BLE.setLocalName("BLöäheee");
  BLE.setAdvertisedService(SendingService);
  
  customService.addCharacteristic(accelXChar);
  customService.addCharacteristic(accelYChar);
  customService.addCharacteristic(accelZChar);
  
  BLE.addService(SendingService);
  
  accelXChar.writeValue(accelX);
  accelYChar.writeValue(accelY);
  accelZChar.writeValue(accelZ);

  BLE.advertise();

  Serial.println("Bluetooth device is now active, waiting for connections...");

  //---------------------------------------------------Intern LED Setup

  pinMode(LED_BUILTIN, OUTPUT);    //B
  pinMode(LEDR, OUTPUT);           //R
  pinMode(LEDG, OUTPUT);           //G
}
//-------------------------------------------------------------------------------------//-------------------------------------------------------------------------------------//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------LOOP()
void loop() {

  BLEDevice central = BLE.central();

  if (central) {
    connectedLight();
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    digitalWrite(LED_BUILTIN, HIGH);

    while (central.connected()) {
      read_accel();
      accelXChar.writeValue(accelX);
  accelYChar.writeValue(accelY);
  accelZChar.writeValue(accelZ);
    }
  }
  disconnectedLight();
  Serial.print("Disconnected from central: ");
  Serial.println(central.address());
}

//-------------------------------------------------------------------------------------//-------------------------------------------------------------------------------------//-------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------PROGRAMMS

//--------------------------------------------------- Accelerator READ
void read_Accel() {
  Serial.print("AccelX: ");
  Serial.print(IMU.getAccelX_mss(), 6);
  Serial.print("  ");
  Serial.print("AccelY: ");
  Serial.print(IMU.getAccelY_mss(), 6);
  Serial.print("  ");
  Serial.print("AccelZ: ");
  Serial.println(IMU.getAccelZ_mss(), 6);

  accelX = IMU.getAccelX_mss();
  accelY = IMU.getAccelY_mss();
  accelZ = IMU.getAccelZ_mss();
}
/*
//--------------------------------------------------- Gyroscope READ
void read_gyro() {
  Serial.print("GyroX: ");
  Serial.print(IMU.getGyroX_rads(), 6);
  Serial.print("  ");
  Serial.print("GyroY: ");
  Serial.print(IMU.getGyroY_rads(), 6);
  Serial.print("  ");
  Serial.print("GyroZ: ");
  Serial.println(IMU.getGyroZ_rads(), 6);

  gyroX = IMU.getGyroX_rads();
  gyroY = IMU.getGyroY_rads();
  gyroZ = IMU.getGyroZ_rads();
}

//--------------------------------------------------- Magentic READ
void read_magnetic() {
  Serial.print("MagX: ");
  Serial.print(IMU.getMagX_uT(), 6);
  Serial.print("  ");
  Serial.print("MagY: ");
  Serial.print(IMU.getMagY_uT(), 6);
  Serial.print("  ");
  Serial.print("MagZ: ");
  Serial.println(IMU.getMagZ_uT(), 6);

  magnetX = IMU.getMagX_uT();
  magnetY = IMU.getMagY_uT();
  magnetZ = IMU.getMagZ_uT();
}
//--------------------------------------------------- Temperatur READ
void read_temp() {
  Serial.print("Temperature in C: ");
  Serial.println(IMU.getTemperature_C(), 6);
  Serial.println();

  temp = IMU.getTemperature_C();
}
*/
//--------------------------------------------------- LED ( Connection status)
void connectedLight() {
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDG, HIGH);
}
void disconnectedLight() {
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, LOW);
}
