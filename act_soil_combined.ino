#include "Adafruit_seesaw.h"

// Soil Sensor
Adafruit_seesaw ss;

// Actuator Pins
#define IN1 D1    // GPIO5
#define IN2 D2    // GPIO4

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Combined Soil Sensor + Actuator Control");

  // Setup actuator pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  Serial.println("Actuator Control Ready.");
  Serial.println("Send 'e' to extend, 'r' to retract, 's' to stop.");
/*
  // Setup soil sensor
  Serial.println("Initializing seesaw Soil Sensor...");
  if (!ss.begin(0x36)) {
    Serial.println("ERROR! seesaw not found");
    while(1) delay(1);
  } else {
    Serial.print("seesaw started! version: ");
    Serial.println(ss.getVersion(), HEX);
  }*/
}
/*
void loop() {
  // === Read soil sensor ===
 
  float tempC = ss.getTemp();
  uint16_t capread = ss.touchRead(0);

  Serial.print("Temperature: "); Serial.print(tempC); Serial.println("*C");
  Serial.print("Capacitive: "); Serial.println(capread);
*/

void loop() {

  Serial.print("Temperature: "); Serial.print("23"); Serial.println("*C");
  Serial.print("Capacitive: "); Serial.println("115");
  


  // === Check actuator control via serial ===
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'e') {
      extendActuator();
      Serial.println("Extending actuator...");
    }
    else if (c == 'r') {
      retractActuator();
      Serial.println("Retracting actuator...");
    }
    else if (c == 's') {
      stopActuator();
      Serial.println("Stopping actuator...");
    }
  }

  delay(500);  // Slightly slower than original soil reading for readability
}

// === Actuator control functions ===
void extendActuator() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

void retractActuator() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}

void stopActuator() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}
