/*

Arduino Nicla Sense ME BLE Sense dashboard demo


Hardware required: https://store.arduino.cc/nicla-sense-me

1) Upload this sketch to the Arduino Nano BLE sense board

2) Open the following web page in the Chrome browser:
https://arduino.github.io/ArduinoAI/BLESense-test-dashboard/

3) Click on the green button in the web page to connect the browser to the board over BLE


Web dashboard by D Pajak

Device sketch based on example by Sandeep Mistry
Sketch fixed to be used with the Nicla Sense ME by Pablo Marquínez

*/
#include "Nicla_System.h"
#include "Arduino_BHY2.h"
#include <ArduinoBLE.h>

#define BLE_SENSE_UUID(val) ("6fbe1da7-" val "-44de-92c4-bb6e04fb0212")

const int VERSION = 0x00000000;

BLEService service(BLE_SENSE_UUID("0000"));
BLEUnsignedIntCharacteristic versionCharacteristic(BLE_SENSE_UUID("1001"), BLERead);
BLEFloatCharacteristic temperatureCharacteristic(BLE_SENSE_UUID("2001"), BLERead);
BLEUnsignedIntCharacteristic humidityCharacteristic(BLE_SENSE_UUID("3001"), BLERead);
BLEUnsignedIntCharacteristic pressureCharacteristic(BLE_SENSE_UUID("4001"), BLERead);

BLECharacteristic accelerometerCharacteristic(BLE_SENSE_UUID("5001"), BLERead | BLENotify, 3 * sizeof(float)); // Array of 3 bytes, RGB
BLECharacteristic gyroscopeCharacteristic(BLE_SENSE_UUID("6001"), BLERead | BLENotify, 3 * sizeof(int16_t));   // Array of 3 bytes, RGB

BLECharacteristic rgbLedCharacteristic(BLE_SENSE_UUID("7001"), BLERead | BLEWrite, 3 * sizeof(byte)); // Array of 3 bytes, RGB

// String to calculate the local and device name
String name;

Sensor temperature(SENSOR_ID_TEMP);
Sensor humidity(SENSOR_ID_HUM);
Sensor pressure(SENSOR_ID_BARO);
SensorXYZ gyroscope(SENSOR_ID_GYRO);
SensorXYZ accelerometer(SENSOR_ID_ACC);

void setup()
{
  Serial.begin(115200);

  // while (!Serial);
  Serial.println("Started");

  nicla::begin();
  nicla::leds.begin();
  nicla::leds.setColor(green);
  BHY2.begin();
  temperature.begin();
  humidity.begin();
  pressure.begin();
  gyroscope.begin();
  accelerometer.begin();

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

  service.addCharacteristic(temperatureCharacteristic);
  service.addCharacteristic(humidityCharacteristic);
  service.addCharacteristic(pressureCharacteristic);
  service.addCharacteristic(versionCharacteristic);
  service.addCharacteristic(accelerometerCharacteristic);
  service.addCharacteristic(gyroscopeCharacteristic);
  service.addCharacteristic(rgbLedCharacteristic);

  temperatureCharacteristic.setEventHandler(BLERead, onTemperatureCharacteristicRead);
  humidityCharacteristic.setEventHandler(BLERead, onHumidityCharacteristicRead);
  pressureCharacteristic.setEventHandler(BLERead, onPressureCharacteristicRead);

  rgbLedCharacteristic.setEventHandler(BLEWritten, onRgbLedCharacteristicWrite);

  versionCharacteristic.setValue(VERSION);

  BLE.addService(service);

  BLE.advertise();
}

void loop()
{

  while (BLE.connected())
  {
    BHY2.update(100);
    if (gyroscopeCharacteristic.subscribed())
    {
      float x, y, z;

      x = gyroscope.x();
      y = gyroscope.y();
      z = gyroscope.z();

      float gyroscopeValues[3] = {x, y, z};

      gyroscopeCharacteristic.writeValue(gyroscopeValues, sizeof(gyroscopeValues));
    }
    if (accelerometerCharacteristic.subscribed())
    {
      float x, y, z;
      x = accelerometer.x();
      y = accelerometer.y();
      z = accelerometer.z();

      float accelerometerValues[] = {x, y, z};
      accelerometerCharacteristic.writeValue(accelerometerValues, sizeof(accelerometerValues));
    }
  }
}

void onTemperatureCharacteristicRead(BLEDevice central, BLECharacteristic characteristic)
{
  float temperatureValue = temperature.value();
  temperatureCharacteristic.writeValue(temperatureValue);
}
void onHumidityCharacteristicRead(BLEDevice central, BLECharacteristic characteristic)
{
  uint8_t humidityValue = humidity.value();
  humidityCharacteristic.writeValue(humidityValue);
}
void onPressureCharacteristicRead(BLEDevice central, BLECharacteristic characteristic)
{
  uint8_t pressureValue = pressure.value();
  pressureCharacteristic.writeValue(pressureValue);
}

void onRgbLedCharacteristicWrite(BLEDevice central, BLECharacteristic characteristic)
{
  byte r = rgbLedCharacteristic[0];
  byte g = rgbLedCharacteristic[1];
  byte b = rgbLedCharacteristic[2];

  nicla::leds.setColor(b, g, r);
}