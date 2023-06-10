#include <Wire.h> 
#include "WiFi.h"
#include <esp_now.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_FeatherOLED.h>

//MAC address: 0C:DC:7E:CD:B8:94

#define PERIPHERAL_NAME "Monitoring Station"
#define SERVICE_UUID "EBC0FCC1-2FC3-44B7-94A8-A08D0A0A5079"
#define CHARACTERISTIC_INPUT_UUID   "C1AB2C55-7914-4140-B85B-879C5E252FE5"
#define CHARACTERISTIC_OUTPUT_UUID  "643954A4-A6CC-455C-825C-499190CE7DB0"
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

uint8_t selfAddress[6] = {0x0c, 0xdc, 0x7e, 0xcd, 0xb8, 0x94};
uint8_t sensorPackageAddress[6] = {0x0c, 0xdc, 0x7e, 0xcd, 0x09, 0x70};

typedef struct resume_message {
  char resume;
} resume_message;

typedef struct sensor_message {
  int hydrogen;
  float temperature;
  float humidity;
} sensor_message;

//pins
char buzzerPin = 21;
char buttonPin = 19;
char redPin = 18;
char yellowPin = 5;
char greenPin = 4;
uint8_t* ESP1_address;
uint8_t* ESP4_address;
uint8_t* broadcastAddress;
int temperatureThreshold = 150;
int hydrogenThreshold = 800;
int humidityThreshold = 99;
bool paused;
esp_now_peer_info_t peerInfo;

sensor_message sensorData; // sensor readings in a struct
resume_message resumeData; // resume message struct
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// Output characteristic is used to send the response back to the connected phone
BLECharacteristic *pOutputChar;
// Current value of output characteristic persisted here
static uint8_t outputData[1];

float getBatteryVoltage() {
#if defined(ARDUINO_FEATHER52)
  int raw;

  // Set the analog reference to 3.0V (default = 3.6V)
  analogReference(AR_INTERNAL_3_0);

  // Set the resolution to 12-bit (0..4095)
  analogReadResolution(12); // Can be 8, 10, 12 or 14

  // Let the ADC settle
  delay(1);

  // Get the raw 12-bit, 0..3000mV ADC value
  raw = analogRead(VBATPIN);

  // Set the ADC back to the default settings
  analogReference(AR_DEFAULT);
  analogReadResolution(10);

  // Convert the raw value to compensated mv, taking the resistor-
  // divider into account (providing the actual LIPO voltage)
  // ADC range is 0..3000mV and resolution is 12-bit (0..4095),
  return ((float)raw * REAL_VBAT_MV_PER_LSB) / 1000.0;
#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
  if (!lc)
    return 0;
  return lc->cellVoltage();
#else
  return analogRead(VBATPIN) * VBAT_MULTIPLIER;
#endif
}

void renderBattery(float _battery) {
    // Render the voltage in text
    display.setCursor(BATTTEXT_STARTX - 25, BATTTEXT_STARTY);
    const float voltageDifference = 4.17F - 3.2F;
    float batteryPercentage = ((_battery - 3.2F) / voltageDifference) * 100;
    if(batteryPercentage < 100 && batteryPercentage > 0) {
      display.print(" ");
      if(batteryPercentage < 10) display.print(" ");
    }
    if(batteryPercentage > 100) batteryPercentage = 100;
    display.print(int(batteryPercentage));
    display.print("%");
    display.setCursor(BATTTEXT_STARTX, BATTTEXT_STARTY);
    display.print(_battery, 2);
    display.println("V");
      // Draw the base of the battery
      display.drawLine(BATTICON_STARTX + 1, BATTICON_STARTY,
               BATTICON_STARTX + BATTICON_WIDTH - 4, BATTICON_STARTY,
               SH110X_WHITE);
      display.drawLine(BATTICON_STARTX, BATTICON_STARTY + 1, BATTICON_STARTX,
               BATTICON_STARTY + 5, SH110X_WHITE);
      display.drawLine(BATTICON_STARTX + 1, BATTICON_STARTY + 6,
               BATTICON_STARTX + BATTICON_WIDTH - 4, BATTICON_STARTY + 6,
               SH110X_WHITE);
      display.drawPixel(BATTICON_STARTX + BATTICON_WIDTH - 3, BATTICON_STARTY + 1,
                SH110X_WHITE);
      display.drawPixel(BATTICON_STARTX + BATTICON_WIDTH - 2, BATTICON_STARTY + 1,
                SH110X_WHITE);
      display.drawLine(BATTICON_STARTX + BATTICON_WIDTH - 1, BATTICON_STARTY + 2,
               BATTICON_STARTX + BATTICON_WIDTH - 1, BATTICON_STARTY + 4,
               SH110X_WHITE);
      display.drawPixel(BATTICON_STARTX + BATTICON_WIDTH - 2, BATTICON_STARTY + 5,
                SH110X_WHITE);
      display.drawPixel(BATTICON_STARTX + BATTICON_WIDTH - 3, BATTICON_STARTY + 5,
                SH110X_WHITE);
      display.drawPixel(BATTICON_STARTX + BATTICON_WIDTH - 3, BATTICON_STARTY + 6,
                SH110X_WHITE);

      // Draw the appropriate number of bars
      if (_battery > 4.26F) {
        // USB (Solid Rectangle)
        display.fillRect(BATTICON_STARTX + 2,    // X
                 BATTICON_STARTY + 2,    // Y
                 BATTICON_BARWIDTH3 * 3, // W
                 3,                      // H
                 SH110X_WHITE);
      } else if ((_battery <= 4.26F) && (_battery >= 4.1F)) {
        // Three bars
        for (uint8_t i = 0; i < 3; i++) {
          display.fillRect(BATTICON_STARTX + 2 + (i * BATTICON_BARWIDTH3),
                   BATTICON_STARTY + 2, BATTICON_BARWIDTH3 - 1, 3,
                   SH110X_WHITE);
        }
      } else if ((_battery < 4.1F) && (_battery >= 3.8F)) {
        // Two bars
        for (uint8_t i = 0; i < 2; i++) {
          display.fillRect(BATTICON_STARTX + 2 + (i * BATTICON_BARWIDTH3),
                   BATTICON_STARTY + 2, BATTICON_BARWIDTH3 - 1, 3,
                   SH110X_WHITE);
        }
      } else if ((_battery < 3.8F) && (_battery >= 3.4F)) {
        // One bar
        display.fillRect(BATTICON_STARTX + 2, BATTICON_STARTY + 2,
                 BATTICON_BARWIDTH3 - 1, 3, SH110X_WHITE);
      } else {
        // No bars
      }
}
void setLED(char mode) {
    if(mode == 'G') {
      digitalWrite(redPin, HIGH);
      digitalWrite(yellowPin, HIGH);
      digitalWrite(greenPin, LOW);
    }
    else if (mode == 'Y') {
      digitalWrite(redPin, HIGH);
      digitalWrite(yellowPin, LOW);
      digitalWrite(greenPin, HIGH);
    }
    else if (mode == 'R') {
      digitalWrite(redPin, LOW);
      digitalWrite(yellowPin, HIGH);
      digitalWrite(greenPin, HIGH);
    }
    else {
      digitalWrite(redPin, HIGH);
      digitalWrite(yellowPin, HIGH);
      digitalWrite(greenPin, HIGH);
    }
}

// callback function that will be executed when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void printScreen() {
  float battery = getBatteryVoltage();
  display.clearDisplay();
  renderBattery(battery);
  display.setCursor(0, 16);
  display.print("Temperature: ");
  display.print(sensorData.temperature, 2);
  display.print("K");
  if(sensorData.temperature <= temperatureThreshold) {
    //lcd.print("YES     ");
  }
  else {
    //lcd.print("NO     ");
  }
  display.setCursor(0, 32);
  display.print("Humidity: ");
  display.print(sensorData.humidity, 2);
  display.print("%");
  if(sensorData.humidity >= humidityThreshold) {
    //lcd.print("YES     ");
  }
  else {
    //lcd.print("NO     ");
  }
  display.setCursor(0, 48);
  display.print("Hydrogen: ");
  display.print(sensorData.hydrogen);
  display.print("ppm");
  if(sensorData.hydrogen >= hydrogenThreshold) {
    //lcd.print("YES     ");
  }
  else {
    //lcd.print("NO     ");
  }
  //lcd.setCursor(0, 4);
  if(sensorData.hydrogen >= hydrogenThreshold || (sensorData.temperature <= temperatureThreshold && sensorData.humidity >= humidityThreshold)) {
    //lcd.print("Press button to resume activity");
  }
  else {
    //lcd.print("                               ");
  }
  display.display();
}

void handleData() {
  if(sensorData.hydrogen >= hydrogenThreshold || (sensorData.temperature <= temperatureThreshold && sensorData.humidity >= humidityThreshold)) {
    setLED('R');
    Serial.println("Red Mode");
  }
  else if (sensorData.hydrogen >= hydrogenThreshold || sensorData.temperature <= temperatureThreshold || sensorData.humidity >= humidityThreshold) {
    setLED('Y');
    Serial.println("Yellow Mode");
  }
  else {
    setLED('G');
    Serial.println("Green Mode");
  }
  delay(100);
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&sensorData, incomingData, sizeof(sensorData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Hydrogen: ");
  Serial.println(sensorData.hydrogen);
  Serial.print("Temperature: ");
  Serial.println(sensorData.temperature);
  Serial.print("Humidity: ");
  Serial.println(sensorData.humidity);
  Serial.println();
  handleData();
  printScreen();
}

// Class defines methods called when a device connects and disconnects from the service
class ServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        Serial.println("BLE Client Connected");
    }
    void onDisconnect(BLEServer* pServer) {
        BLEDevice::startAdvertising();
        Serial.println("BLE Client Disconnected");
    }
};

class InputReceivedCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharWriteState) {
        uint8_t *inputValues = pCharWriteState->getData();
        outputData[0] = inputValues[0];
        Serial.printf("Sending response:   %02x\r\n", outputData[0]);  
        
        pOutputChar->setValue((uint8_t *)outputData, 1);
        pOutputChar->notify();
    }
};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(250);
  Serial.println("Starting monitoring system");
  pinMode(buzzerPin, OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(yellowPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  setLED('N');
  digitalWrite(buzzerPin, HIGH);
  paused = digitalRead(buttonPin);
  display.begin(SCREEN_ADDRESS, true); // Address 0x3C default
  display.display();
  delay(2000);
  // Clear the buffer.
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
    // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  Serial.println("Successfully connected with ESP-NOW");
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  // Configure thes server

  BLEDevice::init(PERIPHERAL_NAME);
  BLEServer *pServer = BLEDevice::createServer();

  // Create the service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a characteristic for the service
  BLECharacteristic *pInputChar = pService->createCharacteristic(
                              CHARACTERISTIC_INPUT_UUID,                                        
                              BLECharacteristic::PROPERTY_WRITE_NR | BLECharacteristic::PROPERTY_WRITE);

  pOutputChar = pService->createCharacteristic(
                              CHARACTERISTIC_OUTPUT_UUID,
                              BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

                                       

  // Hook callback to report server events
  pServer->setCallbacks(new ServerCallbacks());
  pInputChar->setCallbacks(new InputReceivedCallbacks());

  // Initial characteristic value
  outputData[0] = 0x00;
  pOutputChar->setValue((uint8_t *)outputData, 1);

  // Start the service
  pService->start();

  // Advertise the service
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  esp_now_register_recv_cb(OnDataRecv);
  //esp_now_register_send_cb(OnDataSent);
}

void loop() {
}
