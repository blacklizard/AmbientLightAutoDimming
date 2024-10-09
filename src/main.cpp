#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>
#include <BLE2902.h>

#define GPIO_PIN 2

struct DeviceConfig {
  std::string serviceUUID;
  std::string characteristicUUID;
  std::vector<uint8_t> (*brightness)(int brightness);
};

BLEClient* pClient = nullptr;
BLERemoteCharacteristic* pRemoteCharacteristic = nullptr;
DeviceConfig device;

std::vector<uint8_t> brightnessLEDCAR(int brightness) {
  brightness = std::max(1, std::min(brightness, 100));
  return {
    123, 
    255, 
    1, 
    static_cast<uint8_t>((brightness * 32) / 100), 
    static_cast<uint8_t>(brightness),             
    2, 
    255, 
    255, 
    191
  };
}

std::vector<uint8_t> brightnessMELKO(int brightness) {
  brightness = std::max(1, std::min(brightness, 100));
  return {
    126, 
    4, 
    1, 
    static_cast<uint8_t>(brightness),
    1, 
    255, 
    255, 
    0, 
    239
  };
}

std::map<std::string, DeviceConfig> config = {
  {"MELK-O", {"fff0", "fff3", brightnessMELKO}},
  {"LEDCAR-01-", {"ffe0", "ffe1", brightnessLEDCAR}}
};

void connectToDevice() {
  BLEScan* pBLEScan = BLEDevice::getScan();
  BLEScanResults foundDevices = pBLEScan->start(2, false);

  for (int i = 0; i < foundDevices.getCount(); i++) {
    BLEAdvertisedDevice advertisedDevice = foundDevices.getDevice(i);
    std::string localName = advertisedDevice.getName();

    for (const auto& kv : config) {
      if (localName.find(kv.first) != std::string::npos) {
        device = kv.second;
        pClient = BLEDevice::createClient();
        try{
          if (!pClient->connect(&advertisedDevice)) {
            Serial.println("Failed to connect to device!");
            return;
          }
        } catch (const std::exception& e) {
          Serial.print("Error connecting to device: ");
          Serial.println(e.what());
          return;
        } catch (...) {
          Serial.println("Unknown error occurred while connecting to device.");
          return;
        }

        BLERemoteService* pRemoteService = pClient->getService(device.serviceUUID);
        if (pRemoteService != nullptr) {
          pRemoteCharacteristic = pRemoteService->getCharacteristic(device.characteristicUUID);
          if (pRemoteCharacteristic != nullptr) {
            Serial.println("Connected to device and characteristic found!");
          } else {
            Serial.println("Failed to find the characteristic!");
          }
        } else {
          Serial.println("Failed to find the service!");
        }
        return;
      }
    }
  }
}

bool isDeviceConnected() {
  return pClient != nullptr && pClient->isConnected();
}

void setBrightness(int brightness) {
  if(!isDeviceConnected()) {
    Serial.println("Device disconnected, attempting to reconnect...");
    connectToDevice();
  }

  if (pRemoteCharacteristic != nullptr) {
    try {
      std::vector<uint8_t> brightnessBuffer = device.brightness(brightness);
      pRemoteCharacteristic->writeValue(brightnessBuffer.data(), brightnessBuffer.size(), false);
      Serial.println("Brightness set!");
      Serial.print("Brightness set to: ");
      Serial.println(brightness);
      // pClient->disconnect();
    } catch (const std::exception& e) {
      Serial.print("Error setting brightness: ");
      Serial.println(e.what());
    } catch (...) {
      Serial.println("Unknown error occurred while setting brightness.");
    }
  } else {
    Serial.println("pRemoteCharacteristic is null. Cannot set brightness.");
  }
}

const int lightOnBrightness = 40;
const int lightOffBrightness = 100;
static int lastPinState = LOW;
// bool on = false;
unsigned long previousMillis = 0; 
unsigned long interval = 0;


void setup() {
  Serial.begin(115200);
  pinMode(GPIO_PIN, INPUT);
  BLEDevice::init("");
  delay(10000);
  int currentPinState = !digitalRead(GPIO_PIN);
  if (currentPinState == HIGH) {
    setBrightness(lightOnBrightness);
  } else if (currentPinState == LOW) {
    setBrightness(lightOffBrightness);
  }
}

void loop() {
  try {
    unsigned long currentMillis = millis();
    int currentPinState = !digitalRead(GPIO_PIN);
    if(currentMillis - interval >= 60000) {
      interval = currentMillis;
      Serial.println("60 seconds passed");  
      if(isDeviceConnected()) {
        pClient->disconnect();
        Serial.println("Disconnected from device.");
      }
    }
    if (currentPinState != lastPinState) {
      if (currentPinState == HIGH) {
        setBrightness(lightOnBrightness);
      } else if (currentPinState == LOW) {
        setBrightness(lightOffBrightness);
      }
      lastPinState = currentPinState;
      interval = currentMillis;
    }

    delay(100);
  } catch (const std::exception& e) {
    Serial.print("Error in loop: ");
    Serial.println(e.what());
  } catch (...) {
    Serial.println("Unknown error occurred in loop.");
  }
}
