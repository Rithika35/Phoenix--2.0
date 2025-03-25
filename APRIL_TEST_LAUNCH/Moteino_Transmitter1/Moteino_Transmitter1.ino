#include <RFM69_ATC.h>
#include <RFM69.h>
#include <SPI.h>
#include <SPIFlash.h>

// Moteino configuration
#define NODEID        2    // Unique ID of this node
#define NETWORKID     100  // Network ID
#define GATEWAYID     1    // ID of the receiver/gateway Moteino
#define FREQUENCY     RF69_915MHZ  // Match frequency to your region
#define ENCRYPTKEY    "TOPSECRETPASSWRD" // 16 bytes encryption key
#define IS_RFM69HW_HCW  // Uncomment if using RFM69HW/HCW
#define LED           9   // Moteino LED pin
#define ENABLE_ATC    // Comment out to disable ATC
#define ATC_RSSI      -75  // Target RSSI for ATC (default is -80)

// Serial configuration for ESP32 communication
#define SERIAL_BAUD   115200
#define MAX_BUFFER    120  // Buffer size for incoming ESP32 data

// Initialize radio
#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

SPIFlash flash(8, 0xEF30); // Flash chip on Moteino

// Variables for parsing ESP32 data
char buffer[MAX_BUFFER];
int statusValue;
float altitude, temp, velx, vely, velz;
float lat, lng, speed, altitude_m;
bool gpsAvailable = false;

void setup() {
  // Initialize LED
  pinMode(LED, OUTPUT);
  
  // Initialize serial communication with ESP32
  Serial.begin(SERIAL_BAUD);
  
  // Initialize radio
  radio.initialize(FREQUENCY, NODEID, NETWORKID);
  
#ifdef IS_RFM69HW_HCW
  radio.setHighPower(); // Only for RFM69HW/HCW
#endif
  
#ifdef ENABLE_ATC
  radio.enableAutoPower(ATC_RSSI);
  Serial.println("ATC Enabled - Auto Transmission Control");
#endif
  
  radio.encrypt(ENCRYPTKEY);
  
  // Initialize flash memory if available
  if (flash.initialize()) {
    flash.sleep();
  }
  
  // Signal setup complete
  blinkLED(3);
  
  Serial.println("Moteino Node ready for ESP32 data");
}

void loop() {
  // Check for incoming serial data from ESP32
  if (Serial.available() > 0) {
    // Read the incoming line into buffer
    int len = Serial.readBytesUntil('\n', buffer, MAX_BUFFER - 1);
    buffer[len] = '\0';  // Null-terminate the string
    
    // Blink LED to indicate data received
    digitalWrite(LED, HIGH);
    
    // Parse the incoming data
    gpsAvailable = (sscanf(buffer, "%d,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                           &statusValue, &altitude, &temp, &velx, &vely, &velz,
                           &lat, &lng, &speed, &altitude_m) == 10);
    
    if (gpsAvailable) {
      // 10 values received (GPS available)
      Serial.print("GPS Data: ");
      Serial.print(statusValue); Serial.print(",");
      Serial.print(altitude); Serial.print(",");
      Serial.print(temp); Serial.print(",");
      Serial.print(velx); Serial.print(",");
      Serial.print(vely); Serial.print(",");
      Serial.print(velz); Serial.print(",");
      Serial.print(lat, 6); Serial.print(",");
      Serial.print(lng, 6); Serial.print(",");
      Serial.print(speed); Serial.print(",");
      Serial.println(altitude_m);
    } else if (sscanf(buffer, "%d,%f,%f,%f,%f,%f",
                      &statusValue, &altitude, &temp, &velx, &vely, &velz) == 6) {
      // 6 values received (no GPS)
      Serial.print("No GPS Data: ");
      Serial.print(statusValue); Serial.print(",");
      Serial.print(altitude); Serial.print(",");
      Serial.print(temp); Serial.print(",");
      Serial.print(velx); Serial.print(",");
      Serial.print(vely); Serial.print(",");
      Serial.println(velz);
    } else {
      Serial.println("Invalid data format");
    }
    
    // Optional: Forward data via RFM69 to gateway
    if (radio.sendWithRetry(GATEWAYID, buffer, strlen(buffer), 3, 50)) {
      Serial.println("Data sent to Gateway");
    } else {
      Serial.println("Failed to send to Gateway");
    }
    
    // Turn off LED after processing
    digitalWrite(LED, LOW);
  }
}

// Helper function to blink LED
void blinkLED(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED, HIGH);
    delay(200);
    digitalWrite(LED, LOW);
    delay(200);
  }
}