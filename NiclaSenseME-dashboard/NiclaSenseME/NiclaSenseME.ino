/*

Arduino Nano BLE Sense dashboard demo


Hardware required: https://store.arduino.cc/nano-33-ble-sense

1) Upload this sketch to the Arduino Nano BLE sense board

2) Open the following web page in the Chrome browser:
https://arduino.github.io/ArduinoAI/BLESense-test-dashboard/

3) Click on the green button in the web page to connect the browser to the board over BLE


Web dashboard by D Pajak

Device sketch based on example by Sandeep Mistry

*/

#include "Arduino_BHY2.h"

#include <ArduinoBLE.h>

#define BLE_SENSE_UUID(val) ("6fbe1da7-" val "-44de-92c4-bb6e04fb0212")

const int VERSION = 0x00000000;

BLEService service(BLE_SENSE_UUID("0000"));
BLEUnsignedIntCharacteristic versionCharacteristic(BLE_SENSE_UUID("1001"), BLERead);
BLEShortCharacteristic gyroX(BLE_SENSE_UUID("2001"), BLERead); // 16-bit

// String to calculate the local and device name
String name;

// buffer to read samples into, each sample is 16-bits
//short sampleBuffer[256];

// number of samples read
//volatile int samplesRead;

SensorXYZ gyro(SENSOR_ID_GYRO);

void setup()
{
  Serial.begin(115200);

  // while (!Serial);
  Serial.println("Started");

  BHY2.begin();
  gyro.begin();

  if (!BLE.begin())
  {
    Serial.println("Failled to initialized BLE!");

    while (1)
      ;
  }

  String address = BLE.address();

  Serial.print("address = ");
  Serial.println(address);

  address.toUpperCase();

  name = "BLESense-";
  name += address[address.length() - 5];
  name += address[address.length() - 4];
  name += address[address.length() - 2];
  name += address[address.length() - 1];

  Serial.print("name = ");
  Serial.println(name);

  BLE.setLocalName(name.c_str());
  BLE.setDeviceName(name.c_str());
  BLE.setAdvertisedService(service);

  service.addCharacteristic(versionCharacteristic);
  service.addCharacteristic(gyroX);

  versionCharacteristic.setValue(VERSION);

  gyroX.setEventHandler(BLERead, onGyroXRead);
  //gyroX.setValue(55);

  BLE.addService(service);

  BLE.advertise();
}

void loop()
{
  int16_t x = 0;

  Serial.println(x);
  while (BLE.connected())
  {
    BHY2.update(100);
  }
}

void onGyroXRead(BLEDevice central, BLECharacteristic characteristic)
{
  BHY2.update(1);
  int16_t x = gyro.x();
  Serial.println(x);
  gyroX.writeValue((int16_t)x);
}