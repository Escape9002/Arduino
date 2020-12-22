/*
   Written: Antonio Rehwinkel [2020]
   Usage: Programm for JuFo Project
        [Movement tracker for archers]

   Used Devices:
    -Arduino Nano 33 BLE
    -MPU9250

   Device Name: MPU9250

   Service Uuid: c54beb4a-40c7-11eb-b378-0242ac130002

   charactersitics Uuid [String]:
   d6b78de4-40c7-11eb-b378-0242ac130002

   Note:
   - no need for multiple characteristics
   - only sends Strings(decoding by Receiver needed)

  current Errors:
   none

  compatible with BLETestVol6.apk

  Datenübertragung:

    "[Wert(accelX)]|[Wert(accelY)]|[Wert(accelZ)]|
     [Wert(gyroX)]|[Wert(gyroY)]|[Wert(gyroZ)]|
     [Wert(magnetX)]|[Wert(magnetY)]|[Wert(magnetZ)]
    "

    contained in [Lvel_String]
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
/*
  volatile short accelXInt = 1;
  volatile short accelYInt = 1;
  volatile short accelZInt = 1;

  String accelXStr = " ";
  String accelYStr = " ";
  String accelZStr = " ";

  byte accelXbyte = 1;
  byte accelYbyte = 1;
  byte accelZbyte = 1;
*/
float ac_x, ac_y, ac_z;

//---------------------------------------------------Gyroscope
/*
  volatile int gyroXInt = 1;
  volatile int gyroYInt = 1;
  volatile int gyroZInt = 1;

  String gyroXStr = " ";
  String gyroYStr = " ";
  String gyroZStr = " ";
*/
float gy_x, gy_y, gy_z;

//---------------------------------------------------magnetic sensor
/*
  volatile int magnetXInt = 1;
  volatile int magnetYInt = 1;
  volatile int magnetZInt = 1;

  String magnetXStr = " ";
  String magnetYStr = " ";
  String magnetZStr = " ";
*/
float ma_x, ma_y, ma_z;

//---------------------------------------------------Temperatur
/*
  volatile int tempInt = 1;
  String tempStr = " ";
*/
//---------------------------------------------------Place for all data
String Level_String;
//-------------------------------------------------- Timer
long newTime = 0;
long oldTime = 0;
//-------------------------------------------------------------------------------------VARIABLEN
//-------------------------------------------------------------------------------------BLE_SETUP
BLEService SendingService("c54beb4a-40c7-11eb-b378-0242ac130002");
BLEStringCharacteristic accelXChar("d6b78de4-40c7-11eb-b378-0242ac130002", BLERead | BLENotify, 40);
//-------------------------------------------------------------------------------------BLE_SETUP
//-------------------------------------------------------------------------------------//-------------------------------------------------------------------------------------//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------SETUP()
void setup() {
  Serial.begin(115200);
  while (!Serial)
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


  BLE.setLocalName("MPU9250");
  BLE.setAdvertisedService(SendingService);

  //BLE.setAdvertisedServiceUuid("c54beb4a-40c7-11eb-b378-0242ac130002");

  SendingService.addCharacteristic(accelXChar);
  // SendingService.addCharacteristic(accelYChar);
  // SendingService.addCharacteristic(accelZChar);

  BLE.addService(SendingService);

  BLE.advertise();

  //accelXChar.writeValue(accelXInt);
  // accelYChar.writeValue(accelYInt);
  // accelZChar.writeValue(accelZInt);


  String address = BLE.address();
  Serial.print("Local address is: ");
  Serial.println(address);
  delay(1000);

  Serial.println("Bluetooth device is now active, waiting for connections...");


  //---------------------------------------------------Intern LED Setup

  pinMode(LED_BUILTIN, OUTPUT);    //B
  pinMode(LEDR, OUTPUT);           //R
  pinMode(LEDG, OUTPUT);           //G
}
//-------------------------------------------------------------------------------------//-------------------------------------------------------------------------------------//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------LOOP()
void loop() {

  read_accel();
  read_gyro();
  read_magnetic();
  debug_Level_String();

  BLEDevice central = BLE.central();

  if (central) {
    connectedLight();
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    digitalWrite(LED_BUILTIN, HIGH);

    while (central.connected()) {
      read_accel();
      read_gyro();
      read_magnetic();
      send_String();
      debug_Level_String();


    }
  }
  disconnectedLight();
  //Serial.print("Disconnected from central: ");
  //Serial.println(central.address());
}

//-------------------------------------------------------------------------------------//-------------------------------------------------------------------------------------//-------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------PROGRAMMS
//--------------------------------------------------- Send Int
/*
  void send_Int() {
  accelXChar.writeValue(accelXInt);
  }*/
//--------------------------------------------------- Send String (in ASCII)        !!!Charactersitics needs to be declared as String!!!

void send_String() {
  Level_String = "ST"+String(ac_x)+"/"+String(ac_y)+"/"+String(ac_z)+"/"+String(gy_x)+"/"+String(gy_y)+"/"+String(gy_z)+"/"+String(ma_x)+"/"+String(ma_y)+"/"+String(ma_z);

  accelXChar.writeValue(Level_String);

}

//--------------------------------------------------- Send Bytes
/*
  void send_byte() {
  accelXChar.writeValue(accelXbyte);
  //accelYChar.writeValue(accelYbyte);
  //accelZChar.writeValue(accelZbyte);
  }
*/
//--------------------------------------------------- Accelerator READ
void read_accel() {
  IMU.readSensor();
  /*
    accelXInt = IMU.getAccelX_mss();
    accelYInt = IMU.getAccelY_mss();
    accelZInt = IMU.getAccelZ_mss();

    accelXbyte = IMU.getAccelX_mss();
    accelYbyte = IMU.getAccelY_mss();
    accelZbyte = IMU.getAccelZ_mss();
  */
  ac_x = IMU.getAccelX_mss();
  ac_y = IMU.getAccelY_mss();
  ac_z = IMU.getAccelZ_mss();
}

//--------------------------------------------------- Gyroscope READ
void read_gyro() {
  IMU.readSensor();
  /*
    gyroXInt = IMU.getGyroX_rads();
    gyroYInt = IMU.getGyroY_rads();
    gyroZInt = IMU.getGyroZ_rads();
  */
  gy_x = IMU.getGyroX_rads();
  gy_y = IMU.getGyroY_rads();
  gy_z = IMU.getGyroZ_rads();
}

//--------------------------------------------------- Magentic READ
void read_magnetic() {
  IMU.readSensor();
  /*
    magnetXInt = IMU.getMagX_uT();
    magnetYInt = IMU.getMagY_uT();
    magnetZInt = IMU.getMagZ_uT();
  */
  ma_x = IMU.getMagX_uT();
  ma_y = IMU.getMagY_uT();
  ma_z = IMU.getMagZ_uT();

}
//--------------------------------------------------- Temperatur READ
/*
  void read_tempInt() {
  IMU.readSensor();

  tempInt = IMU.getTemperature_C();

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
//--------------------------------------------------- Timer for transmitting Rate
void transRate() {
  newTime = millis();
  Serial.println(newTime - oldTime);
  oldTime = newTime;
}
//------------------------------------------------------------------------------------------------------ Debugging
//--------------------------------------------------- accelerator
/*
  void debug_accel() {

  Serial.print("AccelX: ");
  Serial.print(IMU.getAccelX_mss(), 6);
  Serial.print("  ");
  Serial.print("AccelY: ");
  Serial.print(IMU.getAccelY_mss(), 6);
  Serial.print("  ");
  Serial.print("AccelZ: ");
  Serial.println(IMU.getAccelZ_mss(), 6);

    Serial.print("AccelX: ");
    Serial.print(accelXInt);
    Serial.print("  ");
    Serial.print("AccelY: ");
    Serial.print(accelYInt);
    Serial.print("  ");
    Serial.print("AccelZ: ");
    Serial.println(accelZInt);

  }
*/
//--------------------------------------------------- gyroscope
/*
  void debug_gyro() {

  Serial.print("GyroX: ");
  Serial.print(IMU.getGyroX_rads(), 6);
  Serial.print("  ");
  Serial.print("GyroY: ");
  Serial.print(IMU.getGyroY_rads(), 6);
  Serial.print("  ");
  Serial.print("GyroZ: ");
  Serial.println(IMU.getGyroZ_rads(), 6);
  }
*/
//--------------------------------------------------- magnetic sensor
/*
  void debug_magnet() {
  Serial.print("MagX: ");
  Serial.print(IMU.getMagX_uT(), 6);
  Serial.print("  ");
  Serial.print("MagY: ");
  Serial.print(IMU.getMagY_uT(), 6);
  Serial.print("  ");
  Serial.print("MagZ: ");
  Serial.println(IMU.getMagZ_uT(), 6);
  }
*/
//--------------------------------------------------- temperatur
/*
  void debug_temp(){
  Serial.print("Temperature in C: ");
  Serial.println(IMU.getTemperature_C(), 6);
  Serial.println();
  }
*/
//-------------------------------------------------- Level_String
void debug_Level_String() {
  Serial.print("ich lauf");
  Serial.println(Level_String);
  
}
