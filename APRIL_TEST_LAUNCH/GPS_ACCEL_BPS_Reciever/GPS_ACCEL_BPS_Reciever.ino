#include <esp_now.h>
#include <WiFi.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <TinyGPS++.h>

// Define the RX and TX pins for Serial 2
#define RXD0 16
#define TXD2 17
#define GPS_BAUD 9600
#define NUMREADINGS 7

// Define UART pins for ESP32 to Moteino communication
#define MOTEINO_RX_PIN 32  // ESP32 RX pin, connects to Moteino TX
#define MOTEINO_TX_PIN 33  // ESP32 TX pin, connects to Moteino RX
#define MOTEINO_BAUD 115200 // Baud rate for Moteino communication
#define MOTEINO_BUFFER 128   // Buffer size for Moteino communication


// The TinyGPS++ object
TinyGPSPlus gps;

// Create an instance of the HardwareSerial class for Serial 2
HardwareSerial gpsSerial(2);  // UART2 for GPS
HardwareSerial MoteinoSerial(1);  // UART1 for Moteino

unsigned long lastMoteinoUpdate = 0;
const unsigned long MOTEINO_UPDATE_INTERVAL = 500; // Send to Moteino every 0.5 seconds

const int CS_PIN = 5; // SD card chip select
File file;

bool logging = false; // Flag to control logging

// REPLACE WITH THE MAC Address of your receiver 
// 08:d1:f9:c8:e2:94 // 03/26/2025
uint8_t broadcastAddress[] = {0x08, 0xd1, 0xf9, 0xc8, 0xe2, 0x94};

//Liftoff
float threshold = 70.0; // Altitude in ft needed to detect lift off
int liftoff = 0;// liftoff = flase
unsigned long LiftOffcount = 0; //countr after liftoff
unsigned long LOcount; //countstamp
unsigned long LaunchPadTime = 300;//2 seconds

// Lockoutperiod 
int lockout = 1; //lockout = true
float SuperSonicMin = 0.0; //numer of minutes of lockout duration
float SuperSonicSec = 2.0; // number of seconds of lockout duration
float SuperSonicCount = (SuperSonicSec/60.0 +SuperSonicMin)*6000.0;
const int consecutiveReadings = NUMREADINGS; // number of readings needed to exit lockout

// Apogee
int Apogee = 0;//Apogee = false
float lastAltitude = 0;// Variable to store the last altitude reading to check if lockout is over
float ApogeePressure = 0; // Variable to store the last pressure reading
unsigned long Apogeecount;//count stamp of apogee
//Water Ballast pins
const int SOLENOID1 = 7;  //first set
const int SOLENOID2 = 5;  //second set
unsigned long Draincount = 300;//seconds to drain ballast 30 seconds
int TurnOffValve = 0;//flag to turn off valve
unsigned long AfterApogee;//keep track of count after apogee

//initiator
const int Initiator = 9;//set initiator to pin 8
int InitiatorCount = 200;//turn of initiator after 2 seconds
int InitiatorOn = 0;

//Actuator 
int DroneDeploy = 0;//if solenoid valve is closed drone is ready to deploy
float DroneDeployment = 300 ;//altitude of drone deployment in meters
const int Actuator = 3; //Actuator pin
int TouchDown =0;
unsigned long Landing = 40; //altitude to terminate program

// Define variables to store incoming readings
float incomingpitch;
float incomingroll;
float incomingyaw;
float incomingTemp;
float incomingAlt;
float incomingPres;
float incoming_ax;
float incoming_ay;
float incoming_az;
int incomingStatus;
float incoming_vx;
float incoming_vy;
float incoming_vz;
char command;
uint32_t incomingcount;

//Timer Variables
hw_timer_t * timer = NULL;
uint32_t count;
int HundrethSec;
int TenthSec;
int OneSec;
int TenSec;
int iHundrethSec;
int iTenthSec;
int iOneSec;
int iTenSec;

// Variable to store if sending data was successful
String success;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    float Temp;
    float Altitude;
    float Pres;
    uint32_t count;
    float accelx;
    float accely;
    float accelz;
    float pitch;
    float roll;
    float yaw;
    int Status;
    float velx;
    float vely;
    float velz;
    char command;
    char logStatus;
} struct_message;

// Create a struct_message called SensorReadings to hold sensor readings
struct_message com;

// Create a struct_message to hold incoming sensor readings
struct_message incomingReadings;

esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint32_t * mac, const uint32_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  // Serial.print("Bytes received: ");
  // Serial.println(len);
  incomingTemp = incomingReadings.Temp;
  incomingAlt = incomingReadings.Altitude;
  incomingPres = incomingReadings.Pres;
  incomingcount = incomingReadings.count;
  incoming_ax = incomingReadings.accelx;
  incoming_ay = incomingReadings.accely;
  incoming_az = incomingReadings.accelz;
  incomingpitch = incomingReadings.pitch;
  incomingroll = incomingReadings.roll;
  incomingyaw = incomingReadings.yaw;
  incomingStatus = incomingReadings.Status;
  incoming_vx = incomingReadings.velx;
  incoming_vy = incomingReadings.vely;
  incoming_vz = incomingReadings.velz;
  incomingcount = incomingReadings.count;
  command = incomingReadings.command;

}

volatile bool TimerFlag = false; // Flag to indicate sensor reading
void IRAM_ATTR onTimer(){
  TimerFlag = true;
  count ++;

}
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  delay(3000);//delay 3 seconds to allow time 

  // Start Serial 2 with the defined RX and TX pins and a baud rate of 9600
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD0, TXD2);
  Serial.println("Serial 2 started at 9600 baud rate");

  // Start Moteino Serial with updated baud rate
  MoteinoSerial.begin(MOTEINO_BAUD, SERIAL_8N1, MOTEINO_RX_PIN, MOTEINO_TX_PIN);
  Serial.println("Moteino communication initialized");

  // Initialize the SD card
  if (!SD.begin(CS_PIN)) {
    Serial.println("SD card initialization failed!");
    return;
  }
  Serial.println("SD card initialized.");

//Timer ISR Configuration
  Serial.println("start timer ");
  timer = timerBegin(0, 80, true);  // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer, &onTimer, true); // edge (not level) triggered 
  timerAlarmWrite(timer, 10000, true); // 10000 * 1 us = 10 ms, autoreload true
  timerAlarmEnable(timer); // enable

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  Serial.println("Type 's' to start logging and 'f' to stop logging.");
}

void loop() {
  HundrethSec = (count) % 10;
  TenthSec = (count/10) % 10;
  OneSec = (count/ 100) % 10;
  TenSec = (count/ 1000) % 6; 
  iHundrethSec =  incomingcount % 10;
  iTenthSec = (incomingcount/10) % 10;
  iOneSec = (incomingcount/ 100) % 10;
  iTenSec = (incomingcount/ 1000)% 6;
  if (TimerFlag ) {
    TimerFlag = false; // Reset flag
     // Debug timer
    Serial.println(count);

    // Serial.print("T+");Serial.print(count/6000);Serial.print(":");Serial.print(TenSec);Serial.print(OneSec);Serial.print(":");Serial.print(TenthSec);Serial.print(HundrethSec);Serial.print(" ");
    //Serial.println(incomingAlt);
    //Start or stop logging based on serial input
    // if (Serial.available() > 0) {
    //   char command = Serial.read();

    if (command == 's' && !logging) {
      logging = true;
      // Send message via ESP-NOW
      com.logStatus = 'Y';
      
      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &com, sizeof(com));
      file = SD.open("/imu_data_withTele.csv", FILE_WRITE);
      if (file) {
        file.println("Time,sensor time,Tempature(F),Altitude(ft),preassure(hPa),x-accel,y-accel,z-accel,pitch,roll,yaw,velx, vely,velz,Latitude,Longitude,Velocity(ft/s), Altitude(m),status");
      } 
      else {
        logging = false;
      }
    } 
    else if (command == 'f' && logging) {
      logging = false;
      file.close();
      com.logStatus = 'N';
      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &com, sizeof(com));
    }
    if(liftoff == 0){
      Serial.println("Launch Pad with timer");
      if (count % 50 ==  0) {      // Send every 50th tick (500ms)
      SendMoteino(0);
      }
      if (logging ) {
        if (file) {
          //Timestamp
          file.print(count/6000);file.print(":");file.print(TenSec);file.print(OneSec);file.print(".");file.print(TenthSec);file.print(HundrethSec); file.print(",");
          file.print(incomingcount/6000);file.print(":");file.print(iTenSec);file.print(iOneSec);
          file.print(".");file.print(iTenthSec);file.print(iHundrethSec);file.print(",");
          PrintMeassurments();
          file.print("LAUNCH PAD"); file.print(",");
          file.println();
          file.flush();
        } 
      }  
    }
    //Check for liftoff
    if(count> LaunchPadTime && incomingAlt >= threshold && liftoff == 0 ) {
      if (logging ) {
        if (file) {
          LOcount = count;
          liftoff = 1;
          file.print(",");file.print(",");file.print(",");file.print(",");file.print(",");file.print(",");file.print(",");file.print(",");
          file.print(",");file.print(",");file.print(",");file.print(",");file.print(",");file.print(",");file.print(",");file.print(",");file.print(",");file.print(",");
          file.print("Lift Off"); file.print(",");
          file.println();
          file.flush();
        }
      }  
    }
    //Lockout Period
    if ((liftoff == 1) && (lockout == 1) ){
      if (logging ) {
        if (file) {
          PrintLiftOffTime(count);
          PrintMeassurments();
          if( LiftOffcount > SuperSonicCount && isLockoutOver(incomingAlt)){//if simulated lockout period is over begin to check for 10 consectutive increasing values
            file.print("Lockout period over");file.print(",");
            lockout = 0;
            file.println();
            file.flush(); 
          }
          else{
            file.print("LockOut Period"); file.print(",");
            file.println();
            file.flush(); 
          }
        }
      }    
    }
    //if lockout period is over startlooking for apogee
    if(liftoff == 1 && lockout == 0 && Apogee ==0 ){
      if (logging ) {
        if (file) {
          PrintLiftOffTime(count);
          PrintMeassurments();
          if(isApogee(incomingPres)){
            file.print("apogee detected");
            Apogee = 1;
            // digitalWrite(SOLENOID1, HIGH);
            // digitalWrite(SOLENOID2, HIGH);
            TurnOffValve = 1;
            Apogeecount = LiftOffcount;  
            InitiatorOn = 1;
            // digitalWrite(Initiator, HIGH);
            file.println();
            file.flush(); 
          }
          else{
            file.print("Looking for apogee");
            file.println();
            file.flush(); 
          }
        }
      }
    }
        //After apogee wait 45 seconds to turn of solenoid, and open actuator at 400 ft/121.92 meters
    if(Apogee == 1 && liftoff==1 && lockout ==0){
      
      PrintLiftOffTime(count);
      PrintMeassurments();
      AfterApogee = LiftOffcount - Apogeecount;
      // int solenoidcountTM = (AfterApogee) % 10;
      // int solenoidcountH = (AfterApogee/10) % 10;
      // int solenoidcountO = (AfterApogee/ 100) % 10;
      // int solenoidcountT = (AfterApogee/ 1000) % 6;
      // Serial.print(" | Descend: ");Serial.print(AfterApogee/6000);Serial.print(":");Serial.print(solenoidcountT);Serial.print(solenoidcountO);Serial.print(":");Serial.print(solenoidcountH);Serial.println(solenoidcountTM);
      //Turn off initiator 
      if(InitiatorCount < AfterApogee && InitiatorOn == 1){
        // digitalWrite(Initiator, LOW);//turn off ignitor pin
        file.print("Initiator Off");
        InitiatorOn = 0;
        file.println();
        file.flush(); 
      }
      //turn off solenoid valve
      else if((TurnOffValve == 1) && (Draincount < AfterApogee)){
        file.print("pumps off");
        // digitalWrite(SOLENOID1, LOW);
        // digitalWrite(SOLENOID2, LOW);
        TurnOffValve = 0; // solenoid and pumps have been turned off
        DroneDeploy = 1;//Ready to search drone deployment altitude
        file.println();
        file.flush(); 
      }
      //deploy payload
      else if((incomingAlt < DroneDeployment) && (DroneDeploy ==1)){   
        file.print("payload deployed");
        // digitalWrite(Actuator, HIGH);
        DroneDeploy = 0;
        TouchDown = 1;
        file.println();
        file.flush(); 
      }  
      //end program
      else if((incomingAlt < Landing) && (TouchDown ==1)){
        file.print("touchdown");
        TouchDown = 0;
        file.println();
        file.flush(); 
        while(1){};
      }else
      file.println();
      file.flush(); 
    }
  }
 // }
  // // Flight state logic with conditional prints and SendMoteino
  // if (liftoff == 0) {
  //   Serial.print("Launch Pad");Serial.println(count);
  //   SendMoteino(0);
  //}
}
//function to check if apogee has been reach using pressure
bool isApogee(float Pressure){
  static int consecutiveCount = 0; 
  bool status;
  // Check if altitude is greater than the last reading
  if (Pressure > ApogeePressure) {
    // Increase consecutive readings count
    consecutiveCount++;  
    // Check if consecutive readings count reaches the threshold
    if (consecutiveCount >= consecutiveReadings) {
      return true; // Altitude is increasing for consecutive readings
    }
  } else {
    // Reset consecutive readings count if altitude is not increasing
    consecutiveCount = 0;
    status = false;
  }  
  // Update the last altitude reading
  ApogeePressure = Pressure;
  
  return false; // Altitude is not increasing for consecutive readings
}  


bool isLockoutOver(float altitude){
  static int consecutiveCount = 0; 
  bool status;
  // Check if altitude is greater than the last reading
  if (altitude > lastAltitude) {
    // Increase consecutive readings count
    consecutiveCount++;  
    // Check if consecutive readings count reaches the threshold
    if (consecutiveCount >= consecutiveReadings) {
      return true; // Altitude is increasing for consecutive readings
    }
  } else {
    // Reset consecutive readings count if altitude is not increasing
    consecutiveCount = 0;
    status = false;
  }
  
  // Update the last altitude reading
  lastAltitude = altitude;
  
  return false; // Altitude is not increasing for consecutive readings
}


void PrintMeassurments() {
  //write temperature, altitude and pressure
  file.print(incomingReadings.Temp,2); file.print(",");
  file.print(incomingReadings.Altitude,2); file.print(",");
  file.print(incomingReadings.Pres,2); file.print(",");
  file.print(incomingReadings.accelx,2); file.print(",");
  file.print(incomingReadings.accely,2); file.print(",");
  file.print(incomingReadings.accelz,2); file.print(",");
  file.print(incomingReadings.pitch,2); file.print(",");
  file.print(incomingReadings.roll,2); file.print(",");
  file.print(incomingReadings.yaw,2); file.print(",");
  file.print(incomingReadings.velx,2); file.print(",");
  file.print(incomingReadings.vely,2); file.print(",");
  file.print(incomingReadings.velz,2); file.print(",");
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
  if (gps.location.isUpdated()) {
    file.print(gps.location.lat(), 6); file.print(",");
    file.print(gps.location.lng(), 6); file.print(",");
    file.print(gps.speed.kmph()*0.911344); file.print(",");
    file.print(gps.altitude.meters()*3.28084); file.print(",");
  }
  else{
    file.print(",");file.print(",");file.print(",");file.print(",");
  }
}

//Funtion that will keep track of time after lift off
void PrintLiftOffTime(int counter) {
  LiftOffcount = counter - LOcount;
  HundrethSec = LiftOffcount % 10;
  TenthSec = (LiftOffcount/10) % 10;
  OneSec = (LiftOffcount/ 100) % 10;
  TenSec = (LiftOffcount/ 1000)%6;
  file.print(LiftOffcount/6000);file.print(":");file.print(TenSec);file.print(OneSec);file.print(".");file.print(TenthSec);file.print(HundrethSec); file.print(",");
  file.print(incomingcount/6000);file.print(":");file.print(iTenSec);file.print(iOneSec);
  file.print(".");file.print(iTenthSec);file.print(iHundrethSec);file.print(",");
}


void SendMoteino(float statusValue){
 
  char buffer[128];
 // Only format and send if GPS data is updated
  if (gps.location.isUpdated()) {       //gpsSerial.available might send zeroed values
  sprintf(buffer, "%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.6f,%.6f,%.2f,%.2f",
          statusValue,              // int
          incomingReadings.Altitude, // float
          incomingReadings.Temp,     //  float
          incomingReadings.velx,     //  float
          incomingReadings.vely,     //  float
          incomingReadings.velz,     //  float
          gps.location.lat(),       // float (latitude)
          gps.location.lng(),       // float (longitude)
          gps.speed.kmph(),         // float (speed in km/h)
          gps.altitude.meters()     // float (altitude in meters)
  );
}else {
  sprintf(buffer, "%d,%.2f,%.2f,%.2f,%.2f,%.2f",
          statusValue,              // int
          incomingReadings.Altitude, //  float
          incomingReadings.Temp,     //  float
          incomingReadings.velx,     //  float
          incomingReadings.vely,     //  float
          incomingReadings.velz     //  float 
       );
      }
 MoteinoSerial.println(buffer);
 MoteinoSerial.flush();
 Serial.println("Data sent to Moteino:");
 Serial.println(buffer);
}