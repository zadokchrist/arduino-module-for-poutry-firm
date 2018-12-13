/********************
 * Program:  DHT22 humidity-temperature sensor tester
 * Description: print humidity and temperature to serial
 ********************/


#include <DHT.h>
//*** DECLARATION FOR HUMID AND TEMPERATURE SENSOR
#define DHTPIN 2     // what pin on the arduino is the DHT22 data line connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)
//#define DHTTYPE DHT11   // DHT 11
DHT dht(DHTPIN, DHTTYPE); // Initialize DHT sensor for normal 16mhz Arduino
//*** END OF DECLARATION FOR HUMID AND TEMPERATURE SENSOR

//*** DECLARATION FOR UTRASONIC SENSOR
#define ECHO_PIN 6 
#define TRIG_PIN 7 
float v=331.5+0.6*20; // m/s
//*** END OF DECLARATION FOR UTRASONIC SENSOR

//Constants
const int ledPin = 3; // led pin to detect change in temperature

// Light sensor declaration 
int photocellPin = A0; // the cell and 10K pulldown are connected to A0
int photocellReading; // the analog reading from the analog resistor divider

void setup() { // to run once
  InitUtrasonicSensor();
  InitTemperatureHumidSensor();
  InitUtrasonicSensor();
}

void loop() {
  DetectLight();
  ReadTemperatureAndHumidity();
  ReadUtrasonicSensor();
}   

// **************  READING UTRASONIC *************************
void InitUtrasonicSensor(){
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);
  }
void ReadUtrasonicSensor(){
  Serial.print(distance_centimetre(), DEC);
  Serial.println("cm");
  delay(1000); // ms 
}

// Define a new function that reads and converts the raw reading to distance (cm)
float distance_centimetre() {
  // Send sound pulse
  digitalWrite(TRIG_PIN, HIGH); // pulse started
  delayMicroseconds(12);
  digitalWrite(TRIG_PIN, LOW); // pulse stopped

  // listen for echo 
  float tUs = pulseIn(ECHO_PIN, HIGH); // microseconds
  float distance = tUs / 58; // cm 
  return distance;
}
// **************  END READING UTRASONIC *********************
// **************  READING LIGHT *****************************
void DetectLight(){
  photocellReading = analogRead(photocellPin);
  Serial.print("Analog reading = ");
  Serial.print(photocellReading); // the raw analog reading
  // We'll have a few threshholds, qualitatively determined
  if (photocellReading < 10) {
    Serial.println(" - Black");
  } else if (photocellReading < 200) {
    Serial.println(" - Dark");
  } else if (photocellReading < 500) {
    Serial.println(" - Light");
  } else if (photocellReading < 800) {
    Serial.println(" - Luminous");
  } else {
    Serial.println(" - Bright");
  }
  delay(2000);
  }
// **************  END OF READING LIGHT **********************

// **************  READING TEMPERATURE AND HUMIDITY *****************************
void InitTemperatureHumidSensor(){
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  Serial.begin(38400); // Initialize the serial port
  Serial.println("DHT22 Humidity - Temperature Sensor");
  Serial.println("RH (%)\t\t Temp (°C)");
  dht.begin();
  }

void ReadTemperatureAndHumidity(){
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT22 sensor!");
    return;
  }

  if(t>=20.0)
  {
    digitalWrite(ledPin, LOW);
    }else
    {
      digitalWrite(ledPin, HIGH);
      }
  Serial.print(h); 
  Serial.print(" \t\t");
  Serial.println(t); 

  // Wait a few seconds between measurements. The DHT22 should not be read at a higher frequency of
  // about once every 2 seconds. So we add a 3 second delay to cover this.
  delay(3000);
  }
//******************* END OF READING HUMIDITY AND TEMPERATURE *******************
