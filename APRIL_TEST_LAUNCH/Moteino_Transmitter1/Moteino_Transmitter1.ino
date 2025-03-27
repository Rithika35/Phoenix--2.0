#include <RFM69_ATC.h>
#include <RFM69.h>
#include <SPI.h>
#include <SPIFlash.h>

// Moteino configuration
#define NODEID        2
#define NETWORKID     100
#define GATEWAYID     1
#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "TOPSECRETPASSWRD"
#define IS_RFM69HW_HCW
#define LED           9
#define ENABLE_ATC
#define ATC_RSSI      -75

// Serial configuration
#define SERIAL_BAUD   115200
#define MAX_BUFFER    130
#define RFM69_MAX_PAYLOAD 61  // RFM69 max data payload (after headers)

// Initialize radio
#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

SPIFlash flash(8, 0xEF30);

// Variables for parsing ESP32 data
int statusValue;
float altitude, temp, velx, vely, velz;
float lat, lng, speed, altitude_m;
bool gpsAvailable = false;

void setup() {
  pinMode(LED, OUTPUT);
  Serial.begin(SERIAL_BAUD);
  
  radio.initialize(FREQUENCY, NODEID, NETWORKID);
#ifdef IS_RFM69HW_HCW
  radio.setHighPower();
#endif
#ifdef ENABLE_ATC
  radio.enableAutoPower(ATC_RSSI);
  Serial.println("ATC Enabled");
#endif
  radio.encrypt(ENCRYPTKEY);
  
  if (flash.initialize()) {
    flash.sleep();
  }
  
  blinkLED(3);
  Serial.println("Moteino Node ready for ESP32 data");
}

void loop() {
  if (Serial.available() > 0) {
    char buffer[MAX_BUFFER];
    int len = Serial.readBytesUntil('\n', buffer, MAX_BUFFER - 1);
    buffer[len] = '\0';

    Serial.print("Length: "); Serial.println(len);
    Serial.println("Raw data"); Serial.println(buffer);
    
    digitalWrite(LED, HIGH);

    // Parse the incoming data
    // Trim buffer (remove trailing newline or spaces)
    for (int i = len - 1; i >= 0; i--) {
    if (buffer[i] == '\n' || buffer[i] == '\r' || buffer[i] == ' ') {
      buffer[i] = '\0';
      len = i;
    } else {
      break;
    }
    }

    int fields = 1;
    for (int i = 0; i < len; i++) {
      if (buffer[i] == ',') fields++;
    }

    char *token = strtok(buffer, ",");
    int tokenCount = 0;

    while (token != NULL && tokenCount < 10) {
      switch (tokenCount) {
        case 0: statusValue = atoi(token); break;
        case 1: altitude = atof(token); break;
        case 2: temp = atof(token); break;
        case 3: velx = atof(token); break;
        case 4: vely = atof(token); break;
        case 5: velz = atof(token); break;
        case 6: lat = atof(token); break;
        case 7: lng = atof(token); break;
        case 8: speed = atof(token); break;
        case 9: altitude_m = atof(token); break;
      }
      token = strtok(NULL, ",");
      tokenCount++;
    }

    //gpsAvailable = (fields == 10);

    if (fields == 6 && tokenCount == 6) {
      Serial.print("No GPS Data: ");
      Serial.print(statusValue); Serial.print(",");
      Serial.print(altitude); Serial.print(",");
      Serial.print(temp); Serial.print(",");
      Serial.print(velx); Serial.print(",");
      Serial.print(vely); Serial.print(",");
      Serial.println(velz);
    } else if (fields == 10 && tokenCount == 10) {
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
    } else {
      Serial.println("Invalid data format");
    }
      
    // Send data via RFM69
    if (len <= RFM69_MAX_PAYLOAD) {
      // Single packet (no GPS, fits in 61 bytes)
      if (radio.sendWithRetry(GATEWAYID, buffer, len, 3, 500)) {
        Serial.println("Single packet sent to Gateway");
      } else {
        Serial.println("Single packet send failed");
      }
    } else {
      // Split into two packets (with GPS, exceeds 61 bytes)
      char packet1[RFM69_MAX_PAYLOAD + 1];
      char packet2[RFM69_MAX_PAYLOAD + 1];
      
      // Find the 5th comma to split after velz
      int commaCount = 0;
      int splitPos = 0;
      for (int i = 0; i < len && commaCount < 6; i++) {
        if (buffer[i] == ',') commaCount++;
        if (commaCount == 5) {
          splitPos = i + 1;  // After 5th comma (before lat)
          break;
        }
      }
      
      // Packet 1: statusValue to velz
      strncpy(packet1, buffer, splitPos);
      packet1[splitPos] = '\0';
      
      // Packet 2: lat to altitude_m
      strcpy(packet2, buffer + splitPos);
      
      // Send Packet 1
      if (radio.sendWithRetry(GATEWAYID, packet1, strlen(packet1), 3, 50)) {
        Serial.println("Packet 1 sent to Gateway");
      } else {
        Serial.println("Packet 1 send failed");
      }
      
      // Send Packet 2
      if (radio.sendWithRetry(GATEWAYID, packet2, strlen(packet2), 3, 50)) {
        Serial.println("Packet 2 sent to Gateway");
      } else {
        Serial.println("Packet 2 send failed");
      }
    }
    
    digitalWrite(LED, LOW);
  }
}

void blinkLED(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED, HIGH);
    delay(200);
    digitalWrite(LED, LOW);
    delay(200);
  }
}