#include <RFM69_ATC.h>
#include <RFM69.h>
#include <SPI.h>
#include <SPIFlash.h>
#include "CRC16.h"

#define NODEID        2
#define NETWORKID     100
#define GATEWAYID     1
#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "TOPSECRETPASSWRD"
#define IS_RFM69HW_HCW
#define LED           9
#define ENABLE_ATC
#define ATC_RSSI      -75
#define SERIAL_BAUD   115200
#define MAX_BUFFER    140
#define RFM69_MAX_PAYLOAD 61

#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

SPIFlash flash(8, 0xEF30);
CRC16 crc;

int statusValue;
float altitude, temp, pitch, roll, velz;
float lat, lng, speed, altitude_m;
char serialBuffer[MAX_BUFFER];
int serialIndex = 0;

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
  if (flash.initialize()) flash.sleep();
  blinkLED(3);
  Serial.println("Moteino Node ready for ESP32 data");
}

void loop() {
  if (Serial.available() > 40) {
    Serial.println("Warning: Serial buffer filling up!");
  }
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      serialBuffer[serialIndex] = '\0';
      Serial.print("Received raw: '"); // Debug raw buffer
      Serial.print(serialBuffer);
      Serial.println("'");
      //if (verifyUARTCRC(serialBuffer)) {
        parseAndPrintData(serialBuffer); // Print data before sending
        processAndSend(serialBuffer);
      // } else {
      //   Serial.println("UART CRC failed - packet discarded");
      // }
      serialIndex = 0;
    } else if (serialIndex < MAX_BUFFER - 1) {
      serialBuffer[serialIndex++] = c;
    } else {
      serialIndex = 0;
      Serial.println("Buffer overflow - packet discarded");
    }
  }
}

bool verifyUARTCRC(char* buffer) {
  // Remove trailing newline if present
  int len = strlen(buffer);
  if (len > 0 && buffer[len - 1] == '\n') {
    buffer[len - 1] = '\0';
  }

  char* crcPos = strrchr(buffer, ',');
  if (!crcPos || strlen(crcPos + 1) != 4) {
    Serial.println("Invalid CRC format"); // Debug
    return false;
  }

  *crcPos = '\0'; // Split data and CRC
  uint16_t receivedCRC;
  sscanf(crcPos + 1, "%4X", &receivedCRC);
  crc.restart();
  crc.add((uint8_t*)buffer, strlen(buffer));
  uint16_t calculatedCRC = crc.calc();

  // Debug CRC values
  Serial.print("Calculated CRC: "); Serial.println(calculatedCRC, HEX);
  Serial.print("Received CRC: "); Serial.println(receivedCRC, HEX);

  return (calculatedCRC == receivedCRC);
}

void processAndSend(char* buffer) {
  digitalWrite(LED, HIGH);
  int len = strlen(buffer);
  Serial.print("Length: "); Serial.println(len);
  Serial.print("Raw data: "); Serial.println(buffer);

  crc.restart();
  crc.add((uint8_t*)buffer, len);
  uint16_t crcValue = crc.calc();

  char fullPacket[MAX_BUFFER + 6];
  sprintf(fullPacket, "%s,%04X", buffer, crcValue);
  int fullLen = strlen(fullPacket);

  if (fullLen <= RFM69_MAX_PAYLOAD) {
    if (radio.sendWithRetry(GATEWAYID, fullPacket, fullLen, 3, 50)) {
      Serial.println("Single packet with CRC sent to Gateway");
    } else {
      Serial.println("Single packet send failed");
    }
  } else {
    char packet1[RFM69_MAX_PAYLOAD + 1];
    char packet2[RFM69_MAX_PAYLOAD + 1];
    int splitPos = RFM69_MAX_PAYLOAD - 5;
    if (splitPos > len) splitPos = len;
    while (splitPos > 0 && buffer[splitPos - 1] != ',') splitPos--;

    if (splitPos > 0) {
      strncpy(packet1, buffer, splitPos);
      packet1[splitPos] = '\0';
      crc.restart();
      crc.add((uint8_t*)packet1, strlen(packet1));
      sprintf(packet1 + strlen(packet1), ",%04X", crc.calc());

      strcpy(packet2, buffer + splitPos);
      crc.restart();
      crc.add((uint8_t*)packet2, strlen(packet2));
      sprintf(packet2 + strlen(packet2), ",%04X", crc.calc());

      if (radio.sendWithRetry(GATEWAYID, packet1, strlen(packet1), 3, 50)) {
        Serial.println("Packet 1 with CRC sent to Gateway");
      } else {
        Serial.println("Packet 1 send failed");
      }
      
      if (radio.sendWithRetry(GATEWAYID, packet2, strlen(packet2), 3, 50)) {
        Serial.println("Packet 2 with CRC sent to Gateway");
      } else {
        Serial.println("Packet 2 send failed");
      }
    } else {
      Serial.println("Unable to split packet cleanly");
    }
  }
  digitalWrite(LED, LOW);
}

void parseAndPrintData(char* data) {
  int fields = 0;
  for (int i = 0; data[i]; i++) if (data[i] == ',') fields++;
  fields++;

  char tempBuffer[MAX_BUFFER];
  strcpy(tempBuffer, data); // Copy to avoid modifying original
  char *token = strtok(tempBuffer, ",");
  int tokenCount = 0;
  while (token != NULL && tokenCount < 10) {
    switch (tokenCount) {
      case 0: statusValue = atoi(token); break;
      case 1: altitude = atof(token); break;
      case 2: temp = atof(token); break;
      case 3: pitch = atof(token); break;
      case 4: roll = atof(token); break;
      case 5: velz = atof(token); break;
      case 6: lat = atof(token); break;
      case 7: lng = atof(token); break;
      case 8: speed = atof(token); break;
      case 9: altitude_m = atof(token); break;
    }
    token = strtok(NULL, ",");
    tokenCount++;
  }

  if (fields == 6 && tokenCount == 6) {
    Serial.println("No GPS Data:");
    Serial.print("Status: "); Serial.println(statusValue);
    Serial.print("Altitude: "); Serial.println(altitude);
    Serial.print("Temperature: "); Serial.println(temp);
    Serial.print("Pitch: "); Serial.println(pitch);
    Serial.print("Roll: "); Serial.println(roll);
    Serial.print("Velocity Z: "); Serial.println(velz);
  } else if (fields == 10 && tokenCount == 10) {
    Serial.println("GPS Data:");
    Serial.print("Status: "); Serial.println(statusValue);
    Serial.print("Altitude: "); Serial.println(altitude);
    Serial.print("Temperature: "); Serial.println(temp);
    Serial.print("Pitch: "); Serial.println(pitch);
    Serial.print("Roll: "); Serial.println(roll);
    Serial.print("Velocity Z: "); Serial.println(velz);
    Serial.print("Latitude: "); Serial.println(lat, 6);
    Serial.print("Longitude: "); Serial.println(lng, 6);
    Serial.print("Speed: "); Serial.println(speed);
    Serial.print("GPS Altitude: "); Serial.println(altitude_m);
  } 
  else {
    Serial.print("Merged packet data:");
    Serial.println(data);
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