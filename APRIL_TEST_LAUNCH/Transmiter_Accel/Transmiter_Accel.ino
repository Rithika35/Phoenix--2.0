#include <esp_now.h> // BT
#include <WiFi.h> //BT
#include <SPI.h> //ALTI
#include <Adafruit_LSM9DS1.h> //used for gyro and magnetometer
#include <Wire.h>
#include <SparkFunLSM9DS1.h> //used for accelerometer
#include "Adafruit_BMP3XX.h"
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <math.h>


#define BMP_SCK 18
#define BMP_MISO 19
#define BMP_MOSI 23
#define BMP_CS 5
#define SEALEVELPRESSURE_HPA (1013.2)
Adafruit_BMP3XX bmp;
// i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
LSM9DS1 imu;

//timer interrupt
hw_timer_t * timer = NULL;


//Calibration offsets
float AccelOffset [3] = {-0.029132, 0.016325, -0.044878};
float AccelSlope [3] = {0.999205, 0.996043, 0.998175};

//Hard-iron Calibration settings
const float hard_iron[3] = {
  40.87,26.67,27.76
};

//soft-iron calibration settings
const float soft_iron[3][3] = {
  { 1.019, 0.052, 0.052 },
  { 0.052, 0.932, 0.011 },
  { 0.052, 0.011, 1.059 }
};
//magnetic declination from https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm mag_dcl = (+/-)deg+ min/60 + sec/3600 set to 0 to get magnteic field instead of geo heading. wellington = -7. 20.17 = -6.662
const float mag_decl = -5.031; //replace with palm bay at site (-6.791)
Adafruit_Sensor *mag = NULL;
sensors_event_t mag_event;
int loopcount = 0; 


//for blutooth
// REPLACE WITH THE MAC Address of your receiver 
// 48:e7:29:b4:ef:04 // 03/26/2025
uint8_t broadcastAddress[] = {0x48, 0xe7, 0x29, 0xb4, 0xef, 0x04};

// Define variables to store Sensor and integration readings
float Temperature;
float Altitude;
float Pressure;
float ax;
float ay;
float az;
float gx;
float gy;
float gz,mx,my,mz;
float vxm, vym, vzm,vzbps;
float amag = 0, vmag=0;
float dt = 0.01;//change in time
float pitchm, rollm, yawm; // measured from accel

//Filter variables
float mxfn, myfn, mzfn, mxfo = 0, myfo=0,mzfo =0;
float vzfo = 0, vzfn; // LPF vertical velocity from bps 
float pitchfo = 0, yawfo = 0, rollfo=0, pitchfn, yawfn, rollfn; //LPF angles from accel
float Pressfo = Pressure;
float Pressurefn;
float Altitudefo,Altitudefn;
float axfn,axfo,ayfn,ayfo,azfn,azfo;

//velocity vectors
float vx =0, vy = 0, vz = 0;//velocity vectors
float previousAltitude;
float theta = 0,phi = 0, psi = 0,thetag = 0,phig = 0, psig = 0;
float x,y,z;
float xo=0;
float yo=0;
float zo=0;
uint32_t count = 0;


//timer variables
int HundrethSec;
int TenthSec;
int OneSec;
int TenSec;

float degToRad = PI / 180;

// Variable to store if sending data was successful
String success;
int Status; // 1-lift off, 2 -lockoutis over, 3 - apogee

//everything below is for bluetooth
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
struct_message SensorReadings;

// Create a struct_message to hold incoming sensor readings
struct_message incomingReadings;


esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("\r\nLast Packet Send Status:\t");
  // Serial.print(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}




volatile bool sensorReadFlag = false; // Flag to indicate sensor reading

void setupAccel()
{
  imu.settings.accel.scale = 16; // Set accel scale to +/-16g.
  imu.settings.accel.sampleRate = 6; // Set accel to 954Hz.

}

//timer interrupt function
void IRAM_ATTR onTimer(){
 sensorReadFlag = true; // Set flag for main loop
  count ++;
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  delay(3000);
  
  //Timer ISR Configuration
  Serial.println("start timer ");
  timer = timerBegin(0, 80, true);  // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer, &onTimer, true); // edge (not level) triggered 
  timerAlarmWrite(timer, 10000, true); // 10000 * 1 us = 10 ms, autoreload true
  timerAlarmEnable(timer); // enable

  

  // Altimeter Sensor sensor
  while (!Serial);
  Serial.println("Adafruit BMP388 / Sensor test");
  //if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
  //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
  if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }
  //}
  
  //setup accelerometer
  Wire.begin();
  if (imu.begin() == false) // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                   "work for an out of the box LSM9DS1 " \
                   "Breakout, but may need to be modified " \
                   "if the board jumpers are.");
    while (1);
  }
  setupAccel();//sparkfun library
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);//adafruit library

   // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  Serial.println("Found LSM9DS1 9DOF");



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

}
 
void loop() {
  if (sensorReadFlag ) {
  HundrethSec = count % 10;
  TenthSec = (count/10) % 10;
  OneSec = (count/ 100) % 10;
  TenSec = (count/ 1000)%6;
  sensorReadFlag = false; // Reset flag
  getReadings();
  
  // Set values to send
  ValuesToSend();

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &SensorReadings, sizeof(SensorReadings));
   
  // if (result == ESP_OK) {
  //   Serial.print(" Sent with success ");
  // }
  // else {
  //   Serial.println("Error sending the data");
  // }
  }
}
void getReadings(){


  /* Get a new sensor event */ 
  imu.readAccel();
  Temperature = bmp.temperature*1.8+32;
  Altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA)*3.28084;
  Pressure = (bmp.pressure / 100.0);
  ax = (AccelSlope[0]*(imu.calcAccel(imu.ax)-AccelOffset[0]))*3.28;// acceleration in x
  ay = (AccelSlope[1]*(imu.calcAccel(imu.ay)-AccelOffset[1]))*3.28;// acceleration in y
  az = (AccelSlope[2]*(imu.calcAccel(imu.az)-AccelOffset[2]))*3.28;

  /* Get a new sensor event */ 
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp); 

  imu.readGyro();
  gx = (g.gyro.x-.085)*(180/(22/7));
  gy = (g.gyro.y+.0321)*(180/(22/7));
  gz = (g.gyro.z-.008)*(180/(22/7));
    //put reaw magnetometer readings into an array
  float mag_data[] = {
  m.magnetic.x,
  m.magnetic.y,
  m.magnetic.z
  };
  static float hi_cal[3];
  //apply hard-iron offsets
  for(uint8_t i = 0; i < 3; i++){
    hi_cal[i] = mag_data[i] - hard_iron[i];
  }
  //applying soft-iron offsets
  for(uint8_t i = 0; i < 3; i++){
    mag_data[i] = (soft_iron[i][0] * hi_cal[0])+(soft_iron[i][1] * hi_cal[1])+(soft_iron[i][2] * hi_cal[2]);
  }
  mx = mag_data[0];
  my = mag_data[1];
  mz = mag_data[2];

 //filters
  mxfn = mxfo*.9 +mx*.1;
  myfn = myfo *.9 +my*.1;
  mzfn = mzfo *.9 +mz*.1;
  
  amag = sqrt(ay*ay+ax*ax+az*az);
  pitchm = atan2(ax,sqrt(az*az+ay*ay))*(180/(22/7));
  rollm = atan2(ay,sqrt(az*az+ax*ax))*(180/(22/7));
  
  //low pass filter for pitch and roll
  // pitchfn = .9*pitchfo + .1*pitchm;
  // rollfn = .9*rollfo + .1*rollm;

  theta = (theta+ gx *.01)*.9+ pitchm*.1;
  phi = (phi + gy *.01)*.9+ rollm* .1;
  ax = (ax - sin(theta*(22/7)/180));
  ay = (ay - sin(phi * (22/7)/180));
  az = (az - cos(theta* (22/7)/180)*cos(phi * (22/7)/180));
  mx = mxfn*cos(theta)+mzfn*sin(theta);
  my = myfn*cos(phi)+mxfn*sin(theta)*sin(phi)-mzfn*cos(theta)*sin(phi);


  yawm = atan2(my,mx)*(180/(22/7));
  yawm += mag_decl;
  //convert to 0-360 degrees
  if (yawm < 0){
  yawm += 360;

  }
  yawfn = yawfo *.9 + yawm*.1;
  psi = (psi+ gz *.01)*.9+ yawfn*.1;

  axfn = axfo* .9+ ax*.1;
  ayfn = ayfo* .9+ ay*.1;
  azfn = azfo* .9+ az*.1;
  

  // vertical position
  Altitudefn = Altitudefo * .9 + Altitude *.1;
  z = Altitude - previousAltitude;

  //vertical velocity
  vzbps = z/dt;
  //low pass filter for vertical velocity
  vzfn = .99*vzfo+.01*vzbps;
  //replace previous altitude with new altitude
  previousAltitude = Altitude;
  //fuse bps and accel vertical velocity

  vxm = sin(theta*PI/180)*cos(phi*PI/180)*vzfn;
  vym = sin(phi*PI/180)*sin(theta*PI/180)*vzfn;
  vzm = cos(theta* (22/7)/180)*cos(phi * (22/7)/180)*vzfn;

  vx = (vx + axfn*dt)*.9+ vxm*.1;
  vy = (vy + ayfn*dt)*.9+ vym*.1;
  vz = (vz + azfn*dt)*.9+ vzm*.1;
  // vx = (vx + axfn*dt);
  // vy = (vy + ayfn*dt);
  // vz = (vz + azfn*dt);

  Pressurefn = Pressfo * .9 + Pressure *.1;

  // Serial.print(" | Temperature: ");
  Serial.print(Pressurefn);
  Serial.print("|  ");
  Serial.print(Altitudefn);
  Serial.print("|  ");
  Serial.print(vz);
  Serial.print("|  ");
  Serial.print(theta);
  Serial.print("|  ");  
  Serial.print(phi);
  Serial.print("|  ");
  Serial.print(axfn);
  Serial.print("|  ");  
  Serial.print(ayfn);
  Serial.print("|  ");
  Serial.println(azfn);
  

  // Serial.println(" deg |");
  rollfo = rollfn;
  pitchfo = pitchfn;
  vzfo = vzfn;
  yawfo = yawfn;
  Altitudefo = Altitudefn;
  mxfo =mxfn;
  myfo =myfn;
  mzfo =mzfn;
  axfo =axfn;
  ayfo =ayfn;
  azfo =azfn;
  Pressfo = Pressurefn;
}
void ValuesToSend(){
  SensorReadings.Temp = Temperature;
  SensorReadings.Altitude =  Altitudefn;
  SensorReadings.Pres = Pressurefn;
  SensorReadings.count = count;
  SensorReadings.accelx = axfn;
  SensorReadings.accely = ayfn;
  SensorReadings.accelz = azfn;
  SensorReadings.pitch = theta;
  SensorReadings.roll = phi;
  SensorReadings.yaw = psi;
  SensorReadings.Status = Status;
  SensorReadings.velx = vx;
  SensorReadings.vely = vy;
  SensorReadings.velz = vz;
  
  
  // Send message via ESP-NOW
  // esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &SensorReadings, sizeof(SensorReadings));
}

