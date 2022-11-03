#include <ESP32Servo.h>
#include <Keypad.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEBeacon.h>
#include <BLEAdvertisedDevice.h>
#include <BLEScan.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <Redis.h>
#include "EEPROM.h"
// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define ENDIAN_CHANGE_U16(x) ((((x)&0xFF00) >> 8) + (((x)&0xFF) << 8))
#define ALPHA 0.7
#define READ 34
#define DETECT 2670
#define MAX_SIZE 500
#define SERVICE_UUID_WIFI "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_WIFI "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define REDIS_PORT 6379
#define ROWS 4
#define COLS 3
#define MYPOS "1"
#define RANGE 62
#define R(i, a) for (int i = 0; i < a; ++i)

int scanTime = 2;  //In seconds
BLEScan *pBLEScan;

//define the cymbols on the buttons of the keypads
char hexaKeys[ROWS][COLS] = {
  { '1', '2', '3' },
  { '4', '5', '6' },
  { '7', '8', '9' },
  { '*', '0', '#' }
};
byte rowPins[ROWS] = { 4, 21, 19, 5 };  //connect to the row pinouts of the keypad
byte colPins[COLS] = { 16, 0, 18 };     //connect to the column pinouts of the keypad
Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);
Servo myservo;  // create servo object to control a servo
Redis *redis;
// AESLib aesLib;

// Possible PWM GPIO pins on the ESP32: 0(used by on-board button),2,4,5(used by on-board LED),12-19,21-23,25-27,32-33
// Possible PWM GPIO pins on the ESP32-S2: 0(used by on-board button),1-17,18(used by on-board LED),19-21,26,33-42
byte servoPin = 32;  // GPIO pin used to connect the servo control (digital out)
byte beepPin = 33;
// Possible ADC pins on the ESP32: 0,2,4,12-15,32-39; 34-39 are recommended for analog input
// Possible ADC pins on the ESP32-S2: 1-20 are recommended for analog input
String input = "";
String identifier;
const char *cert = "";
const char *passwd = "1234";
const char *redisIP;
TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;
TaskHandle_t Task4;
unsigned short int dist = 0;
bool openSignal = false;
bool isSetup = false;
bool redisConnected = false;
bool deviceConnected = false;
bool isUWBSetup = false;
bool firstOpened = false;
DynamicJsonDocument jsonData(MAX_SIZE);

enum State {
  CONNECTED,
  DOOR_OPENED,
  DOOR_CLOSED,
  INVALID_OPEN,
  SENSOR_CRUSHED
};

String randomID() {
  String s;
  R(i, 4) {
    if (random(1, 3) == 1) {
      s += String(random(48, 58));
    } else {
      s += String(random(65, 91));
    }
  }
  return s;
}

bool sendState(State state) {
  int failCount = 0;
  if (!redisConnected) {
    Serial.println("you should connect redis first!");
    return false;
  }
  Serial.print("send data: ");
  Serial.println(state);
  while (!redis->publish("local", (identifier + ":" + String(state) + ":" + MYPOS).c_str())) {
    if (failCount < 10) {
      Serial.println("send failed");
    }
    Serial.print(".");
    delay(500);
    failCount++;
  }
}

bool loadSetup() {
  Serial.println(F("Reading initial setup..."));
  if (!EEPROM.begin(MAX_SIZE)) {
    Serial.println(F("Failed to initialise EEPROM"));
    Serial.println(F("Restarting..."));
    delay(1000);
    ESP.restart();
  }
  identifier = EEPROM.readString(MAX_SIZE - 10);
  if (identifier.isEmpty()) {
    identifier = randomID();
    EEPROM.writeString(MAX_SIZE - 10, identifier);
    EEPROM.commit();
    Serial.print(F("Init identifier: "));
    Serial.println(identifier);
  }
  String setup = EEPROM.readString(0);
  if (setup.isEmpty()) {
    Serial.println(F("Setting is empty..."));
    return false;
  }
  deserializeJson(jsonData, setup);
  redisIP = jsonData["host"];
  if (jsonData["cert"]) {
    cert = jsonData["cert"];
    Serial.println(cert);
  }
  Serial.print(F("setup complete with: "));
  Serial.print(passwd);
  Serial.print(F(", "));
  Serial.println(redisIP);
  return true;
}

void saveValue() {
  Serial.println(F("Saving Setup..."));
  String save;
  serializeJson(jsonData, save);
  EEPROM.writeString(0, save);
  EEPROM.commit();
}

void flushAll() {
  R(i, MAX_SIZE)
  EEPROM.write(i, 0);
  EEPROM.commit();
  delay(500);
}

void setupWifi() {
  Serial.println(F("Setting Up wifi"));
  const char *ssid = jsonData["ssid"];
  const char *psk = jsonData["psk"];
  Serial.print(ssid);
  Serial.print(": ");
  Serial.println(psk);
  WiFi.begin(ssid, psk);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(2000);
  }
}

class AutoCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    Serial.println(F("Writing..."));
    String data = pCharacteristic->getValue().c_str();
    Serial.println(data);
    if (!data.isEmpty()) {
      deserializeJson(jsonData, data);
      saveValue();
      isSetup = true;
    }
  }
};

class UWBCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (!strlen(cert)) return;
    if (advertisedDevice.haveManufacturerData() == true) {
      std::string strManufacturerData = advertisedDevice.getManufacturerData();

      uint8_t cManufacturerData[100];
      strManufacturerData.copy((char *)cManufacturerData, strManufacturerData.length(), 0);
      BLEBeacon oBeacon = BLEBeacon();
      oBeacon.setData(strManufacturerData);
      BLEUUID oUUID = BLEUUID(cManufacturerData + 4, 16, true);
      oBeacon.setProximityUUID(oUUID);
      if (strManufacturerData.length() == 25 && cManufacturerData[0] == 0x4C && cManufacturerData[1] == 0x00) {
        short int major = ENDIAN_CHANGE_U16(oBeacon.getMajor());
        short int minor = ENDIAN_CHANGE_U16(oBeacon.getMinor());
        const char *uuid = oBeacon.getProximityUUID().toString().c_str();
        if (!strcmp(uuid, cert) && advertisedDevice.getRSSI() > -RANGE && major == 10 && minor == 150) {
          firstOpened = openSignal = true;
        }
      }
      return;
    }
  }
};

class ServerCallback : public BLEServerCallbacks {

  void onConnect(BLEServer *pServer, esp_ble_gatts_cb_param_t *param) {
    Serial.println(F("Connected"));

    deviceConnected = true;
  }

  void onDisconnect(BLEServer *pServer) {
    Serial.println(F("Disconnected"));
    deviceConnected = false;
  }
};

void initWifiProtocol() {
  BLEDevice::init(String("LockScale_Wifi_" + identifier).c_str());
  esp_ble_gap_config_local_privacy(true);
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID_WIFI);
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_WIFI, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

  pCharacteristic->setValue("None");
  pService->start();
  pServer->setCallbacks(new ServerCallback());
  pCharacteristic->setCallbacks(new AutoCallback());
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID_WIFI);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println(F("Advertising Started..."));
  while (!isSetup) {
    Serial.print(F("."));
    delay(2000);
  }
  pAdvertising->stop();
  pService->stop();
  Serial.println(F("Stop Wifi Auto Coupling..."));
  delay(1000);
  ESP.restart();
}

void initUWB() {
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();  //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new UWBCallbacks());
  pBLEScan->setActiveScan(true);  //active scan uses more power, but get results faster
  pBLEScan->setInterval(1000);
  pBLEScan->setWindow(999);  // less or equal setInterval value
  xTaskCreatePinnedToCore(
    &scanUWB,  // 태스크 함수
    "Task4",   // 테스크 이름
    5000,      // 스택 크기(워드단위)
    NULL,      // 태스크 파라미터
    3,         // 태스크 우선순위
    &Task4,    // 태스크 핸들
    0);        // 실행될 코어
}

void scanUWB(void *param) {
  while (true) {
    pBLEScan->start(scanTime, false);
    pBLEScan->clearResults();
    delay(500);
  }
}

bool doorClosed() {
  bool v = dist > DETECT;
  if (!v) return v;

  // ledcWriteNote(beepPin, NOTE_D, 4);
  tone(beepPin, 440, 500);
  // ledcWrite(0, 0);
  delay(2500);
  v = dist > DETECT;
  if (v) {
    // ledcWriteNote(beepPin, NOTE_D, 4);
    tone(beepPin, 440, 200);
    delay(100);
    tone(beepPin, 440, 200);
    // ledcWrite(0, 0);
  }
  return v;
}

void communicator(void *param) {
  Serial.print(F("# Task 1 running on core "));
  Serial.println(xPortGetCoreID());
  Serial.println("communicating start");

  setupWifi();
  delay(500);
  initUWB();
  delay(500);

  WiFiClient redisConn;
  while (!redisConn.connect(redisIP, REDIS_PORT)) {
    Serial.println("Failed to connect to the Redis server!");
    delay(2000);
  }
  redis = new Redis(redisConn);
  auto connRet = redis->authenticate("");
  if (connRet == RedisSuccess) {
    Serial.println("Connected to the Redis server!");
  } else {
    Serial.printf("Failed to authenticate to the Redis server! Errno: %d\n", (int)connRet);
    return;
  }
  delay(2000);
  redisConnected = true;
  sendState(CONNECTED);
  Serial.println("start subscribing: " + identifier);

  Serial.println("loop start");
  while (true) {
    String val = redis->get(identifier.c_str());
    if (!val.equals("")) {
      Serial.println(val);
      if (val.equals("open")) {
        firstOpened = openSignal = true;
      } else if (val.startsWith("aes:")) {
        jsonData["cert"] = val.substring(4);
        cert = jsonData["cert"];
        saveValue();
        redis->set(identifier.c_str(), "");
        ESP.restart();
      } else if (val.equals("purge")) {
        jsonData.remove("cert");
        cert = "";
        saveValue();
        redis->set(identifier.c_str(), "");
        ESP.restart();
      }
      redis->set(identifier.c_str(), "");
    }
    delay(100);
  }
}

void sensorRead(void *param) {
  Serial.print(F("# Task 2 running on core "));
  Serial.println(xPortGetCoreID());
  byte errorCount = 0;
  while (true) {
    dist = analogRead(READ);
    if (dist == 0) {
      sendState(SENSOR_CRUSHED);
      delay(10000);
    }
    if (dist <= DETECT && !openSignal) {
      errorCount++;
      if (errorCount == 10) {
        sendState(INVALID_OPEN);
        tone(beepPin, 440, 1000);
        errorCount = 0;
      }
    } else {
      errorCount = 0;
    }
    delay(500);
  }
}

void motorHandler(void *param) {
  Serial.print(F("# Task 3 running on core "));
  Serial.println(xPortGetCoreID());
  bool formarState = false;
  while (true) {
    if (openSignal) {
      myservo.write(0);
      if (formarState != openSignal) {
        tone(beepPin, 440, 200);
        sendState(DOOR_OPENED);
      }
      Serial.println(firstOpened);
      if (firstOpened) {
        firstOpened = false;
        delay(3000);
        Serial.println(firstOpened);

      } else if (doorClosed()) {
        firstOpened = openSignal = false;
        sendState(DOOR_CLOSED);
      }
    } else {
      myservo.write(70);
    }
    formarState = openSignal;
    delay(200);
  }
}

void setup() {
  Serial.begin(115200);
  // flushAll();
  if (!loadSetup()) {
    Serial.println(F("Starting wifi Auto Coupling protocol..."));
    initWifiProtocol();
  }
  Serial.println(F("Starting Task Daemon..."));
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);           // Standard 50hz servo
  myservo.attach(servoPin, 500, 2400);  // attaches the servo on pin 18 to the servo object
                                        // using SG90 servo min/max of 500us and 2400us
                                        // for MG995 large servo, use 1000us and 2000us,
                                        // which are the defaults, so this line could be
                                        // "myservo.attach(servoPin);"
  xTaskCreatePinnedToCore(
    &communicator,  // 태스크 함수
    "Task1",        // 테스크 이름
    15000,          // 스택 크기(워드단위)
    NULL,           // 태스크 파라미터
    0,              // 태스크 우선순위
    &Task1,         // 태스크 핸들
    0);             // 실행될 코어
  xTaskCreatePinnedToCore(
    &motorHandler,  // 태스크 함수
    "Task2",        // 테스크 이름
    4096,           // 스택 크기(워드단위)
    NULL,           // 태스크 파라미터
    1,              // 태스크 우선순위
    &Task2,         // 태스크 핸들
    1);             // 실행될 코어

  xTaskCreatePinnedToCore(
    &sensorRead,  // 태스크 함수
    "Task3",      // 테스크 이름
    4096,         // 스택 크기(워드단위)
    NULL,         // 태스크 파라미터
    2,            // 태스크 우선순위
    &Task3,       // 태스크 핸들
    1);           // 실행될 코어
}

void loop() {
  char customKey = customKeypad.getKey();
  if (customKey && !openSignal) {
    tone(beepPin, 440, 100);
    Serial.print("key: ");
    Serial.println(customKey);
    input += String(customKey);
    if (input.length() == 4) {
      if (input.equals(passwd)) {
        delay(100);
        firstOpened = openSignal = true;
      } else if (input.equals("**##")) {
        flushAll();
        ESP.restart();
      } else if (input.equals("####")) {
        ESP.restart();
      } else {
        R(i, 2) {
          tone(beepPin, 440, 200);
          delay(50);
        }
      }
      input = "";
    }
  }
  delay(50);
}