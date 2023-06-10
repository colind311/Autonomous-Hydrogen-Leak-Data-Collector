#include "WiFi.h"
#include <SPI.h>
#include <Wire.h>
#include "DHT.h"
#include <esp_now.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_FeatherOLED.h>

//MAC address: 0C:DC:7E:CD:09:70

#define DHTTYPE DHT22   // Sensor DHT22
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

const int MAX_ANALOG_VAL = 4095;
const float MAX_BATTERY_VOLTAGE = 4.2; // Max LiPoly voltage of a 3.7 battery is 4.2

//Format of the "resume" data
typedef struct resume_message {
  char resume;
} resume_message;

//Format of the sensor data
typedef struct sensor_message {
  int hydrogen;
  float temperature;
  float humidity;
} sensor_message; 

int hydrogenPin = 15;
int DHTPin = 21;
bool resume;
int temperatureThreshold = 150; //Temperature threshold for a hydrogen leak (K)
int hydrogenThreshold = 800; //Hydrogen concentration threshold for a hydrogen leak (ppm)
int humidityThreshold = 99; //Humidity threshold for a hydrogen leak (%)
uint8_t ESP0Address[6] = {0x0c, 0xdc, 0x7e, 0xcd, 0xb8, 0x94}; //MAC address of the monitoring system's ESP32
DHT dht(DHTPin, DHTTYPE);
Adafruit_SSD1306 display = Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
sensor_message sensorData; //Object to store sensor data
resume_message resumeData; //Object to store "resume" data
esp_now_peer_info_t peerInfo;

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
               SSD1306_WHITE);
      display.drawLine(BATTICON_STARTX, BATTICON_STARTY + 1, BATTICON_STARTX,
               BATTICON_STARTY + 5, SSD1306_WHITE);
      display.drawLine(BATTICON_STARTX + 1, BATTICON_STARTY + 6,
               BATTICON_STARTX + BATTICON_WIDTH - 4, BATTICON_STARTY + 6,
               SSD1306_WHITE);
      display.drawPixel(BATTICON_STARTX + BATTICON_WIDTH - 3, BATTICON_STARTY + 1,
                SSD1306_WHITE);
      display.drawPixel(BATTICON_STARTX + BATTICON_WIDTH - 2, BATTICON_STARTY + 1,
                SSD1306_WHITE);
      display.drawLine(BATTICON_STARTX + BATTICON_WIDTH - 1, BATTICON_STARTY + 2,
               BATTICON_STARTX + BATTICON_WIDTH - 1, BATTICON_STARTY + 4,
               SSD1306_WHITE);
      display.drawPixel(BATTICON_STARTX + BATTICON_WIDTH - 2, BATTICON_STARTY + 5,
                SSD1306_WHITE);
      display.drawPixel(BATTICON_STARTX + BATTICON_WIDTH - 3, BATTICON_STARTY + 5,
                SSD1306_WHITE);
      display.drawPixel(BATTICON_STARTX + BATTICON_WIDTH - 3, BATTICON_STARTY + 6,
                SSD1306_WHITE);

      // Draw the appropriate number of bars
      if (_battery > 4.26F) {
        // USB (Solid Rectangle)
        display.fillRect(BATTICON_STARTX + 2,    // X
                 BATTICON_STARTY + 2,    // Y
                 BATTICON_BARWIDTH3 * 3, // W
                 3,                      // H
                 SSD1306_WHITE);
      } else if ((_battery <= 4.26F) && (_battery >= 4.1F)) {
        // Three bars
        for (uint8_t i = 0; i < 3; i++) {
          display.fillRect(BATTICON_STARTX + 2 + (i * BATTICON_BARWIDTH3),
                   BATTICON_STARTY + 2, BATTICON_BARWIDTH3 - 1, 3,
                   SSD1306_WHITE);
        }
      } else if ((_battery < 4.1F) && (_battery >= 3.8F)) {
        // Two bars
        for (uint8_t i = 0; i < 2; i++) {
          display.fillRect(BATTICON_STARTX + 2 + (i * BATTICON_BARWIDTH3),
                   BATTICON_STARTY + 2, BATTICON_BARWIDTH3 - 1, 3,
                   SSD1306_WHITE);
        }
      } else if ((_battery < 3.8F) && (_battery >= 3.4F)) {
        // One bar
        display.fillRect(BATTICON_STARTX + 2, BATTICON_STARTY + 2,
                 BATTICON_BARWIDTH3 - 1, 3, SSD1306_WHITE);
      } else {
        // No bars
      }
}

//Print to the I2C oLED display
void printScreen() {
  float battery = getBatteryVoltage();
  display.clearDisplay();
  renderBattery(battery);
  display.setCursor(0, 16);
  display.print("Temperature: ");
  display.print(sensorData.temperature, 2);
  display.print("K");
  display.setCursor(0, 32);
  display.print("Humidity: ");
  display.print(sensorData.humidity, 2);
  display.print("%");
  display.setCursor(0, 48);
  display.print("Hydrogen: ");
  display.print(sensorData.hydrogen);
  display.print("ppm");
  display.display();
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&resumeData, incomingData, sizeof(sensorData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  resume = resumeData.resume; //Resume activity when a packet is received
}

// callback function that will be executed when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.begin(115200);
  //Let oLED display start up
  delay(250);

  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  dht.begin();
  pinMode(hydrogenPin, INPUT_PULLUP);
  pinMode(DHTPin, INPUT_PULLUP);
  resume = true;
  display.begin(SCREEN_ADDRESS, true); // Address 0x3C default
  display.display();
  delay(2000);
  // Clear the buffer.
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

   // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  //Register peer
  memcpy(peerInfo.peer_addr, ESP0Address, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  //Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register callbacks
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

}

void loop() {
  // put your main code here, to run repeatedly:
  while(resume) {
    delay(500);
    //Read data from sensors
    sensorData.hydrogen = analogRead(hydrogenPin);
    sensorData.humidity = dht.readHumidity();
    sensorData.temperature = dht.readTemperature(true);
    Serial.print("Hydrogen value: ");
    Serial.print(sensorData.hydrogen);
    Serial.println(" ppm");
    Serial.print("Temperature: ");
    Serial.print(sensorData.temperature);
    Serial.println(" F");
    Serial.print("Humidity: ");
    Serial.print(sensorData.humidity);
    Serial.println("%");
    //Print to the screen
    printScreen();
    //If a leak has been detected, stop activity
    if(sensorData.hydrogen > hydrogenThreshold || (sensorData.temperature < temperatureThreshold && sensorData.humidity > humidityThreshold)) {
      resume = false;
      break;
    }
    //Send sensor data over Wi-Fi
    esp_now_send(ESP0Address, (uint8_t *) &sensorData, sizeof(sensorData));    
  }
}
