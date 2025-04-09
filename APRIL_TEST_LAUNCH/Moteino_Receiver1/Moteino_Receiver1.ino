#include <RFM69_ATC.h>
#include <RFM69.h>
#include <SPI.h>
#include <SPIFlash.h>
#include "CRC16.h"

#define NODEID        1
#define NETWORKID     100
#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "TOPSECRETPASSWRD"
#define IS_RFM69HW_HCW
#define LED           9
#define ENABLE_ATC
#define MAX_BUFFER    160
#define RFM69_MAX_PAYLOAD 61

#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

SPIFlash flash(8, 0xEF30);
CRC16 crc;

int statusValue;
float altitude, temp, velx, vely, velz;
float lat, lng, speed, altitude_m;
char fullBuffer[MAX_BUFFER];
bool waitingForMore = false;

void setup() {
  pinMode(LED, OUTPUT);
  Serial.begin(115200);
  radio.initialize(FREQUENCY, NODEID, NETWORKID);
#ifdef IS_RFM69HW_HCW
  radio.setHighPower();
#endif
#ifdef ENABLE_ATC
  radio.enableAutoPower(-75);
  Serial.println("ATC Enabled");
#endif
  radio.encrypt(ENCRYPTKEY);
  if (flash.initialize()) flash.sleep();
  Serial.println("Moteino Gateway Receiver Ready");
  blinkLED(5);
}

void loop() {
  if (radio.receiveDone()) {
    char packet[RFM69_MAX_PAYLOAD + 1];
    memcpy(packet, radio.DATA, radio.DATALEN);
    packet[radio.DATALEN] = '\0';
    
    blinkLED(2);

    char* crcPos = strrchr(packet, ',');
    if (!crcPos || strlen(crcPos + 1) != 4) {
      Serial.println("Invalid CRC format - packet discarded:");
      Serial.println(packet);
      if (radio.ACKRequested()) radio.sendACK();
      return;
    }

    *crcPos = '\0';
    uint16_t receivedCRC;
    sscanf(crcPos + 1, "%4X", &receivedCRC);
    crc.restart();
    crc.add((uint8_t*)packet, strlen(packet));
    if (crc.calc() != receivedCRC) {
      Serial.println("RFM69 CRC check failed - packet discarded:");
      Serial.println(packet);
      if (radio.ACKRequested()) radio.sendACK();
      return;
    }

    int dataLen = strlen(packet);
    if (dataLen <= 56 && !waitingForMore) {
      strcpy(fullBuffer, packet);
      Serial.print("Received full packet: "); Serial.println(fullBuffer);
      parseAndPrintData(fullBuffer);
    } else if (!waitingForMore) {
      strcpy(fullBuffer, packet);
      waitingForMore = true;
    } else {
      strcat(fullBuffer, ",");
      strcat(fullBuffer, packet);
      if (strlen(fullBuffer) + 5 >= MAX_BUFFER) {
        Serial.println("Buffer overflow - resetting");
        waitingForMore = false;
      } else if (dataLen <= 56) {
        Serial.print("Received full packet: "); Serial.println(fullBuffer);
        parseAndPrintData(fullBuffer);
        waitingForMore = false;
      } else {
        waitingForMore = true;
      }
    }

    if (radio.ACKRequested()) {
      radio.sendACK();
      Serial.println("ACK Sent");
    }
  }
}

void parseAndPrintData(char* data) {
  int fields = 0;
  for (int i = 0; data[i]; i++) if (data[i] == ',') fields++;
  fields++;

  char *token = strtok(data, ",");
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

  if (fields == 6 && tokenCount == 6) {
    Serial.println("No GPS Data:");
    Serial.print("Status: "); Serial.println(statusValue);
    switch (statusValue) {
      case 0: Serial.println("Launch Pad"); break;
      case 1: Serial.println("Lift Off!"); break;
      case 2: Serial.println("Lockout period over"); break;
      case 3: Serial.println("Lockout Period"); break;
      case 4: Serial.println("Apogee Detected!"); break;
      case 5: Serial.println("Looking for apogee.."); break;
      case 6: Serial.println("Initiator Off"); break;
      case 7: Serial.println("Pumps Off"); break;
      case 8: Serial.println("Payload Deployed"); break;
      case 9: Serial.println("Touchdown"); break;
      case 10: Serial.println("..."); break;
      default: Serial.println("Unknown status"); break;
    }

    Serial.print("Altitude: "); Serial.println(altitude);
    Serial.print("Temperature: "); Serial.println(temp);
    Serial.print("Pitch: "); Serial.println(velx);
    Serial.print("Roll: "); Serial.println(vely);
    Serial.print("Yaw: "); Serial.println(velz);
  } else if (fields == 10 && tokenCount == 10) {
    Serial.println("GPS Data:");
    switch (statusValue) {
      case 0: Serial.println("Launch Pad"); break;
      case 1: Serial.println("Lift Off!"); break;
      case 2: Serial.println("Lockout period over"); break;
      case 3: Serial.println("Lockout Period"); break;
      case 4: Serial.println("Apogee Detected!"); break;
      case 5: Serial.println("Looking for apogee.."); break;
      case 6: Serial.println("Initiator Off"); break;
      case 7: Serial.println("Pumps Off"); break;
      case 8: Serial.println("Payload Deployed"); break;
      case 9: Serial.println("Touchdown"); break;
      case 10: Serial.println("..."); break;
      default: Serial.println("Unknown status"); break;
    }
    Serial.print("Altitude: "); Serial.println(altitude);
    Serial.print("Temperature: "); Serial.println(temp);
    Serial.print("Pitch: "); Serial.println(velx);
    Serial.print("Roll: "); Serial.println(vely);
    Serial.print("Yaw: "); Serial.println(velz);
    Serial.print("Latitude: "); Serial.println(lat, 6);
    Serial.print("Longitude: "); Serial.println(lng, 6);
    Serial.print("Speed: "); Serial.println(speed);
    Serial.print("GPS Altitude: "); Serial.println(altitude_m);
  } else {
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