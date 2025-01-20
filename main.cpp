#include <WiFi.h>             // For WiFi connectivity
#include <PubSubClient.h>     // For MQTT communication
#include <DHT.h>              // For DHT sensor

// ====== 1. Pin Definitions ======
#define DHTPIN 4        // DHT11 data pin
#define PIRPIN 23       // PIR sensor data pin
#define IR1PIN 14       // IR sensor 1 data pin
#define IR2PIN 33       // IR sensor 2 data pin
#define RED_LED 18      // Red LED
#define BLUE_LED 19     // Blue LED
#define BUTTON_PIN 32   // Button for manual alarm trigger
#define RELAY_PIN 15    // Relay control pin

// ====== 2. Create DHT Object ======
DHT dht(DHTPIN, DHT11);

// ====== 3. WiFi and MQTT Credentials ======
const char* WIFI_SSID = "Omar’s iPhone";           
const char* WIFI_PASSWORD = "12345679";   
const char* MQTT_SERVER = "34.69.128.36";  
const char* MQTT_TOPIC = "iot";           
const int MQTT_PORT = 1883;

// ====== 4. Create WiFi and MQTT Client Objects ======
WiFiClient espClient;
PubSubClient mqttClient(espClient);

void setup() {
  Serial.begin(115200);

  // Initialize DHT sensor
  dht.begin();

  // Initialize pin modes
  pinMode(PIRPIN, INPUT);
  pinMode(IR1PIN, INPUT);
  pinMode(IR2PIN, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT);

  // Set initial states
  digitalWrite(RED_LED, LOW); // Turn off the red LED
  digitalWrite(BLUE_LED, HIGH); // Turn on the blue LED
  digitalWrite(RELAY_PIN, HIGH); // Keep the relay off

  // Connect to WiFi
  connectToWiFi();

  // Configure MQTT
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
}

void loop() {
  // Ensure MQTT connection
  if (!mqttClient.connected()) {
    connectToMQTT();
  }
  mqttClient.loop();

  // Read temperature and humidity
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Read PIR and IR sensor states
  bool motionDetected = digitalRead(PIRPIN);
  bool proximity1 = digitalRead(IR1PIN);
  bool proximity2 = digitalRead(IR2PIN);

  // Publish sensor data to MQTT topic
  publishMQTTData(temperature, humidity, motionDetected, proximity1, proximity2);

  // Determine alarm condition
  bool alarmCondition = false;
  if (temperature > 35 || motionDetected || !proximity1 || !proximity2) {
    alarmCondition = true;
  }

  // Check if the button is pressed to manually trigger the alarm
  if (digitalRead(BUTTON_PIN) == LOW) {
    alarmCondition = true;
    Serial.println("Manual override activated!");
  }

  // Control LEDs and relay based on alarm condition
  if (alarmCondition) {
    digitalWrite(RED_LED, HIGH);
    digitalWrite(BLUE_LED, LOW);
    digitalWrite(RELAY_PIN, LOW); // Activate relay
    Serial.println("Alarm triggered!");
  } else {
    digitalWrite(RED_LED, LOW);
    digitalWrite(BLUE_LED, HIGH);
    digitalWrite(RELAY_PIN, HIGH); // Deactivate relay
    Serial.println("System normal.");
  }

  delay(10000); // Wait 10 seconds before next loop iteration
}

void connectToWiFi() {
  Serial.print("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void connectToMQTT() {
  Serial.print("Connecting to MQTT...");
  while (!mqttClient.connected()) {
    if (mqttClient.connect("ESP32Client")) { // Client ID
      Serial.println("Connected to MQTT!");
      mqttClient.subscribe(MQTT_TOPIC);
    } else {
      Serial.print("Failed to connect. Error: ");
      Serial.println(mqttClient.state());
      delay(2000); // Retry in 2 seconds
    }
  }
}

void publishMQTTData(float temp, float hum, bool motion, bool prox1, bool prox2) {
  char payload[128];
  snprintf(payload, sizeof(payload), "{\"temperature\": %.2f, \"humidity\": %.2f, \"motion\": %d, \"proximity1\": %d, \"proximity2\": %d}",
           temp, hum, motion, prox1, prox2);
  mqttClient.publish(MQTT_TOPIC, payload);
  Serial.println("Data published to MQTT:");
  Serial.println(payload);
}
