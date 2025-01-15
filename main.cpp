// Include required libraries
#include <DHT.h>
#include <Servo.h>

// Define pins for components
#define DHTPIN 4  // DHT11 data pin
#define PIRPIN 23 // PIR sensor pin
#define IR1PIN 14 // IR sensor 1 pin
#define IR2PIN 33 // IR sensor 2 pin
#define RED_LED 18
#define GREEN_LED 19
#define BUTTON_PIN 32
#define SERVO_PIN 15

// Create objects
DHT dht(DHTPIN, DHT11);
Servo servo;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize DHT sensor
  dht.begin();

  // Set pin modes
  pinMode(PIRPIN, INPUT);
  pinMode(IR1PIN, INPUT);
  pinMode(IR2PIN, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Attach servo
  servo.attach(SERVO_PIN);
  servo.write(0); // Set servo to initial position

  // Initial LED states
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
}

void loop() {
  // Read DHT sensor data
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  // Read motion and proximity sensors
  bool motionDetected = digitalRead(PIRPIN);
  bool proximity1 = digitalRead(IR1PIN);
  bool proximity2 = digitalRead(IR2PIN);

  // Print sensor values to the serial monitor
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" °C, Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");

  Serial.print("Motion Detected: ");
  Serial.println(motionDetected);
  Serial.print("Proximity 1: ");
  Serial.println(proximity1);
  Serial.print("Proximity 2: ");
  Serial.println(proximity2);

  // Handle alarms based on sensor values
  if (temperature > 30 || motionDetected || !proximity1 || !proximity2) {
    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, LOW);
    servo.write(90); // Trigger servo action
  } else {
    digitalWrite(RED_LED, LOW);
    digitalWrite(GREEN_LED, HIGH);
    servo.write(0); // Reset servo
  }

  // Manual override with button
  if (digitalRead(BUTTON_PIN) == LOW) {
    Serial.println("Manual override activated");
    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, LOW);
    servo.write(90);
    delay(2000); // Hold position for 2 seconds
    servo.write(0);
  }

  // Delay for stability
  delay(1000);
}
