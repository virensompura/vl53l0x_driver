#include "Adafruit_VL53L0X.h"

// Protocol description.
// Every message starts with 0xAD byte, followed by message type byte and ends with \l\n (0x0D, 0x0A)
// Message types:
// 0xEF - initialization failed
// 0xE0 - initialization successful, no errors
// 0x1D - range message, followed by two bytes range measurement
// 0xEA - out of range

// VL53L0X sensor objects
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

// Ranging measurement data
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;

// set the pins to shutdown
#define SHT_LOX1 7
#define SHT_LOX2 6

// Function to set the IDs and initialize sensors
void setID() {
    // all reset
    digitalWrite(SHT_LOX1, LOW);    
    digitalWrite(SHT_LOX2, LOW);
    delay(10);
    // all unreset
    digitalWrite(SHT_LOX1, HIGH);
    digitalWrite(SHT_LOX2, HIGH);
    delay(10);

    // activating LOX1 and resetting LOX2
    digitalWrite(SHT_LOX1, HIGH);
    digitalWrite(SHT_LOX2, LOW);

    // initing LOX1
    if(!lox1.begin(0x30)) { // Assuming LOX1_ADDRESS is 0x30
        Serial.println(F("Failed to boot first VL53L0X"));
        while(1);
    }
    delay(10);

    // activating LOX2
    digitalWrite(SHT_LOX2, HIGH);
    delay(10);

    // initing LOX2
    if(!lox2.begin(0x31)) { // Assuming LOX2_ADDRESS is 0x31
        Serial.println(F("Failed to boot second VL53L0X"));
        while(1);
    }
}

void setup() {
    Serial.begin(57600);
    // wait until Serial port is opened
    while (!Serial) {}
    
    pinMode(SHT_LOX1, OUTPUT);
    pinMode(SHT_LOX2, OUTPUT);


    digitalWrite(SHT_LOX1, LOW);
    digitalWrite(SHT_LOX2, LOW);

    
    Serial.println(F("Starting..."));
    setID();
}

void read_dual_sensors() {
    lox1.rangingTest(&measure1, false);
    lox2.rangingTest(&measure2, false);

    // Message start
    Serial.write(0xAD);

    // Check if either sensor is out of range
    if (measure1.RangeStatus == 4 || measure2.RangeStatus == 4) {
        // Out of range
        Serial.write(0xEA);
    } else {
        // Both sensors are in range, send the measurements
        Serial.write(0x1D);
        // Sensor 1 measurement (LSB first)
        Serial.write(measure1.RangeMilliMeter & 0xFF);
        Serial.write((measure1.RangeMilliMeter >> 8) & 0xFF);
        
        // Sensor 2 measurement (LSB first)
        Serial.write(measure2.RangeMilliMeter & 0xFF);
        Serial.write((measure2.RangeMilliMeter >> 8) & 0xFF);
    }

    // Message end
    Serial.write(0x0D);
    Serial.write(0x0A);
}

void loop() {
    read_dual_sensors();
    delay(50); // Adjust delay as needed
}