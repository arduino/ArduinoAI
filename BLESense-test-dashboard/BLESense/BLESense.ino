#include <arm_math.h>

#include <Arduino_APDS9960.h>
#include <Arduino_HTS221.h>
#include <Arduino_LPS22HB.h>
#include <Arduino_LSM9DS1.h>
#include <PDM.h>

#include <ArduinoBLE.h>

#define BLE_SENSE_UUID(val) ("6fbe1da7-" val "-44de-92c4-bb6e04fb0212")

const int VERSION = 0x00000000;

BLEService                     service                       (BLE_SENSE_UUID("0000"));
BLEUnsignedIntCharacteristic   versionCharacteristic         (BLE_SENSE_UUID("1001"), BLERead);
BLEUnsignedShortCharacteristic ambientLightCharacteristic    (BLE_SENSE_UUID("2001"), BLENotify); // 16-bit
BLECharacteristic              colorCharacteristic           (BLE_SENSE_UUID("2002"), BLENotify, 3 * sizeof(unsigned short)); // Array of 16-bit, RGB
BLEUnsignedCharCharacteristic  proximityCharacteristic       (BLE_SENSE_UUID("2003"), BLENotify); // Byte, 0 - 255 => close to far
BLEByteCharacteristic          gestureCharacteristic         (BLE_SENSE_UUID("2004"), BLENotify); // NONE = -1, UP = 0, DOWN = 1, LEFT = 2, RIGHT = 3
BLECharacteristic              accelerationCharacteristic    (BLE_SENSE_UUID("3001"), BLENotify, 3 * sizeof(float)); // Array of 3 floats, G
BLECharacteristic              gyroscopeCharacteristic       (BLE_SENSE_UUID("3002"), BLENotify, 3 * sizeof(float)); // Array of 3 floats, dps
BLECharacteristic              magneticFieldCharacteristic   (BLE_SENSE_UUID("3003"), BLENotify, 3 * sizeof(float)); // Array of 3 floats, uT

BLEFloatCharacteristic         pressureCharacteristic        (BLE_SENSE_UUID("4001"), BLERead); // Float, kPa
BLEFloatCharacteristic         temperatureCharacteristic     (BLE_SENSE_UUID("4002"), BLERead); // Float, Celcius
BLEFloatCharacteristic         humidityCharacteristic        (BLE_SENSE_UUID("4003"), BLERead); // Float, Percentage
BLECharacteristic              microphoneLevelCharacteristic (BLE_SENSE_UUID("5001"), BLENotify, 32); // Int, RMS of audio input
BLECharacteristic              rgbLedCharacteristic          (BLE_SENSE_UUID("6001"), BLERead | BLEWrite, 3 * sizeof(byte)); // Array of 3 bytes, RGB

// String to calculate the local and device name
String name;

// buffer to read samples into, each sample is 16-bits
short sampleBuffer[256];

arm_rfft_instance_q15 FFT;

// number of samples read
volatile int samplesRead;

void setup() {
  Serial.begin(9600);

  //while (!Serial);
  Serial.println("Started");


  if (!APDS.begin()) {
    Serial.println("Failled to initialized APDS!");

    while (1);
  }

  if (!HTS.begin()) {
    Serial.println("Failled to initialized HTS!");

    while (1);
  }

  if (!BARO.begin()) {
    Serial.println("Failled to initialized BARO!");

    while (1);
  }

  if (!IMU.begin()) {
    Serial.println("Failled to initialized IMU!");

    while (1);
  }

  // configure the data receive callback
  PDM.onReceive(onPDMdata);

  // initialize PDM with:
  // - one channel (mono mode)
  // - a 16 kHz sample rate
  if (!PDM.begin(1, 16000)) {
    Serial.println("Failed to start PDM!");
    while (1);
  }

  if (!BLE.begin()) {
    Serial.println("Failled to initialized BLE!");

    while (1);
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
  service.addCharacteristic(ambientLightCharacteristic);
  service.addCharacteristic(colorCharacteristic);
  service.addCharacteristic(proximityCharacteristic);
  service.addCharacteristic(gestureCharacteristic);
  service.addCharacteristic(accelerationCharacteristic);
  service.addCharacteristic(gyroscopeCharacteristic);
  service.addCharacteristic(magneticFieldCharacteristic);

  service.addCharacteristic(pressureCharacteristic);
  service.addCharacteristic(temperatureCharacteristic);
  service.addCharacteristic(humidityCharacteristic);
  service.addCharacteristic(microphoneLevelCharacteristic);
  service.addCharacteristic(rgbLedCharacteristic);

  versionCharacteristic.setValue(VERSION);
  pressureCharacteristic.setEventHandler(BLERead, onPressureCharacteristicRead);
  temperatureCharacteristic.setEventHandler(BLERead, onTemperatureCharacteristicRead);
  humidityCharacteristic.setEventHandler(BLERead, onHumidityCharacteristicRead);
  rgbLedCharacteristic.setEventHandler(BLEWritten, onRgbLedCharacteristicWrite);

  BLE.addService(service);


  BLE.advertise();
}

void loop() {
  while (BLE.connected()) {
    if ((ambientLightCharacteristic.subscribed() || colorCharacteristic.subscribed()) && APDS.colorAvailable()) {
      int r, g, b, ambientLight;

      APDS.readColor(r, g, b, ambientLight);

      ambientLightCharacteristic.writeValue(ambientLight);

      unsigned short colors[3] = { r, g, b };

      colorCharacteristic.writeValue(colors, sizeof(colors));
    }

    if (proximityCharacteristic.subscribed() && APDS.proximityAvailable()) {
      int proximity = APDS.readProximity();

      proximityCharacteristic.writeValue(proximity);
    }

    if (gestureCharacteristic.subscribed() && APDS.gestureAvailable()) {
      int gesture = APDS.readGesture();

      gestureCharacteristic.writeValue(gesture);
    }

    if (accelerationCharacteristic.subscribed() && IMU.accelerationAvailable()) {
      float x, y, z;

      IMU.readAcceleration(x, y, z);

      float acceleration[3] = { x, y, z };

      accelerationCharacteristic.writeValue(acceleration, sizeof(acceleration));
    }

    if (gyroscopeCharacteristic.subscribed() && IMU.gyroscopeAvailable()) {
      float x, y, z;

      IMU.readGyroscope(x, y, z);

      float dps[3] = { x, y, z };

      gyroscopeCharacteristic.writeValue(dps, sizeof(dps));
    }

    if (magneticFieldCharacteristic.subscribed() && IMU.magneticFieldAvailable()) {
      float x, y, z;

      IMU.readMagneticField(x, y, z);

      float magneticField[3] = { x, y, z };

      magneticFieldCharacteristic.writeValue(magneticField, sizeof(magneticField));
    }

    if (microphoneLevelCharacteristic.subscribed() && samplesRead) {
      short micLevel;
     // arm_rms_q15 (sampleBuffer, samplesRead, &micLevel);

	  static arm_rfft_instance_q15 fft_instance;
	  static q15_t fftoutput[256*2]; //has to be twice FFT size
	  static byte spectrum[32];
    arm_rfft_init_q15(&fft_instance, 256/*bin count*/, 0/*forward FFT*/, 1/*output bit order is normal*/);
    arm_rfft_q15(&fft_instance, (q15_t*)sampleBuffer, fftoutput);
		arm_abs_q15(fftoutput, fftoutput, 256);

		float temp = 0;
    for (int i = 1; i < 256; i++) {
      temp = temp + fftoutput[i];
      if ((i &3) == 2){
        if (temp>1023) {temp=1023;};
        spectrum[i>>3] = (byte)(temp/2);
        temp = 0;
      }
  }
      microphoneLevelCharacteristic.writeValue((byte *) &spectrum, 32);
      samplesRead = 0;
    }
  }
}

void onPressureCharacteristicRead(BLEDevice central, BLECharacteristic characteristic) {
  float pressure = BARO.readPressure();

  pressureCharacteristic.writeValue(pressure);
}

void onTemperatureCharacteristicRead(BLEDevice central, BLECharacteristic characteristic) {
  float temperature = HTS.readTemperature()-5;

  temperatureCharacteristic.writeValue(temperature);
}

void onHumidityCharacteristicRead(BLEDevice central, BLECharacteristic characteristic) {
  float humidity = HTS.readHumidity();

  humidityCharacteristic.writeValue(humidity);
}

void onRgbLedCharacteristicWrite(BLEDevice central, BLECharacteristic characteristic) {
  byte r = rgbLedCharacteristic[0];
  byte g = rgbLedCharacteristic[1];
  byte b = rgbLedCharacteristic[2];

  setLedPinValue(LEDR, r);
  setLedPinValue(LEDG, g);
  setLedPinValue(LEDB, b);
}

void setLedPinValue(int pin, int value) {
  // RGB LED's are pulled up, so the PWM needs to be inverted

  if (value == 0) {
    // special hack to clear LED
    analogWrite(pin, 256);
  } else {
    analogWrite(pin, 255 - value);
  }
}

void onPDMdata() {
  // query the number of bytes available
  int bytesAvailable = PDM.available();

  // read into the sample buffer
  PDM.read(sampleBuffer, bytesAvailable);

  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;
}
