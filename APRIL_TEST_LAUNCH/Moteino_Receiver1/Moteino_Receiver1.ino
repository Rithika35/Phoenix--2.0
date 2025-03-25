#include <RFM69_ATC.h>
#include <RFM69.h>
#include <SPI.h>
#include <SPIFlash.h>

// Gateway configuration
#define NODEID        1    // Unique ID of this gateway node
#define NETWORKID     100  // Network ID (must match the Node)
#define FREQUENCY     RF69_915MHZ  // Match frequency to your region
#define ENCRYPTKEY    "TOPSECRETPASSWRD" // 16 bytes encryption key
#define IS_RFM69HW_HCW  // Uncomment if using RFM69HW/HCW
#define LED           9   // Moteino LED pin
#define ENABLE_ATC    // Comment out to disable ATC

// Buffer sizes
#define MAX_BUFFER    160  // Large enough for full reassembled GPS data
#define RFM69_MAX_PAYLOAD 61  // Max payload per packet

// Initialize radio
#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

SPIFlash flash(8, 0xEF30); // Flash chip on Moteino

// Variables for parsing received data
int statusValue;
float altitude, temp, velx, vely, velz;
float lat, lng, speed, altitude_m;
char fullBuffer[MAX_BUFFER];  // Buffer to reassemble split packets
bool waitingForSecondPacket = false;

void setup() {
  pinMode(LED, OUTPUT);
  Serial.begin(115200);
  
  radio.initialize(FREQUENCY, NODEID, NETWORKID);
#ifdef IS_RFM69HW_HCW
  radio.setHighPower();
#endif
#ifdef ENABLE_ATC
  radio.enableAutoPower(-75);
  Serial.println("ATC Enabled - Auto Transmission Control");
#endif
  radio.encrypt(ENCRYPTKEY);
  
  if (flash.initialize()) {
    flash.sleep();
  }
  
  Serial.println("Moteino Gateway Receiver Ready");
  blinkLED(5); // Signal setup complete
}

void loop() {
  if (radio.receiveDone()) {
    char packet[RFM69_MAX_PAYLOAD + 1];
    memcpy(packet, radio.DATA, radio.DATALEN);
    packet[radio.DATALEN] = '\0'; // Null-terminate
    
    // Blink LED to indicate packet reception
    blinkLED(2);
    
    // Check if this is a single packet (no GPS) or part of a split packet (GPS)
    int commaCount = countCommas(packet);
    if (commaCount == 5) {
      // Single packet (6 values, no GPS)
      strcpy(fullBuffer, packet);
      parseAndPrintData(fullBuffer);
    } else if (commaCount < 5 && !waitingForSecondPacket) {
      // First packet of GPS data (6 values)
      strcpy(fullBuffer, packet);
      waitingForSecondPacket = true;
    } else if (waitingForSecondPacket) {
      // Second packet of GPS data (4 values)
      strcat(fullBuffer, ",");  // Add the missing comma between packets
      strcat(fullBuffer, packet);
      parseAndPrintData(fullBuffer);
      waitingForSecondPacket = false;
    } else {
      Serial.println("Unexpected packet format");
    }
    
    // Acknowledge the packet if requested
    if (radio.ACKRequested()) {
      radio.sendACK();
      Serial.println("ACK Sent");
    }
  }
}

// Parse and print the received data
void parseAndPrintData(char* data) {
  bool gpsAvailable = (sscanf(data, "%d,%f,%f,%f,%f,%f,%f,%f,%f,%f",
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
  } else if (sscanf(data, "%d,%f,%f,%f,%f,%f",
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
}

// Count commas in a string
int countCommas(char* str) {
  int count = 0;
  for (int i = 0; str[i] != '\0'; i++) {
    if (str[i] == ',') count++;
  }
  return count;
}

// Blink LED helper function
void blinkLED(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED, HIGH);
    delay(200);
    digitalWrite(LED, LOW);
    delay(200);
  }
}