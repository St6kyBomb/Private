// ================================================== ( 1 - PZEM-017 DC Energy Meter ) ====================================================================//
#include <ModbusMaster.h>
#define MAX485_DE      3   // pin 14 (Tx) and pin 15 (Rx)//
#define MAX485_RE      2
static uint8_t pzemSlaveAddr = 0x01;
static uint16_t NewshuntAddr = 0x0003;
ModbusMaster node;
unsigned long time_old;
float PZEMVoltage =0;
float PZEMCurrent =0;
float PZEMPower =0;
float PZEMEnergy=0;
unsigned long startMillisPZEM;
unsigned long currentMillisPZEM;
const unsigned long periodPZEM = 1000;
// ================================================== (2 - LCD Screen ) ==================================================================== //
#include <LiquidCrystal_I2C.h>
const int lcdRows = 2;
const int lcdColumns = 16;
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);
// ================================================== (3 - SD Card ) ==================================================================== //
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
const int chipSelect = 53;  // MOSI - pin 51, MISO - pin 50, sck - pin 52, CS - pin 53 //
// ================================================== (4 - Bluetooth ) ==================================================================== // 
#include <SoftwareSerial.h>
SoftwareSerial bluetooth(10,11); /* RX, TX */
int inbyte;
int i=0;
int j=0;
int l=0;// สร้างตัวแปร Loop //
// ================================================== (5 - GPS ) ==================================================================== // 
#include <TinyGPS++.h>
#define GPS_RX_PIN 13
#define GPS_TX_PIN 12
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN); //RX,TX//
TinyGPSPlus gps;
float Lat,Lon,b,Eot,Delta,Omega,Alpha,t,ts,Gamma,Zeta ;
double Jd ;
int year;
byte day,month,hour,minute,second;
// ================================================== (6 - Compass (GY-273) ) ==================================================================== // 
#include <MechaQMC5883.h>
MechaQMC5883 qmc;
int Windows,azimuthCal,azimuthMax,azimuthMin,azimuthStart,RotateSensor,Final;
int x, y, z;
int azimuth;
int k = 0;
// ================================================== (7 - Motor ) ==================================================================== // 
#include <L298N.h>
float GammaN,WindowsN,AngleMax,AngleMin,ActualAngle,RotateAngle;
const unsigned int IN1 = 7;
const unsigned int IN2 = 8;
const unsigned int EN = 9;
L298N motor(EN, IN1, IN2);

// ||||||||||||||||||||||||||||||||||||||||||||||||||| (Setup Phase ) ||||||||||||||||||||||||||||||||||||||||||||||||||| //
void setup(){
  // ================================================== ( 1 - PZEM-017 DC Energy Meter ) ====================================================================//
  Serial.begin(9600);
  startMillisPZEM = millis();
  Serial3.begin(9600,SERIAL_8N2);   
  node.begin(pzemSlaveAddr, Serial3);  /* By default communicate via Serial3 port: pin 14 (Tx) and pin 15 (Rx)*/
  pinMode(MAX485_RE, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_RE, 0);
  digitalWrite(MAX485_DE, 0);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
  changeAddress(0XF8, 0x01);

  // ================================================== (2 - LCD Screen ) ==================================================================== //
  lcd.begin();
  lcd.setCursor(0,0);
  Wire.begin();
  lcd.backlight(); 
  // ================================================== (3 - SD Card ) ==================================================================== //
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization failed!");
    lcd.print("SD not found");
  }
  else {
    Serial.println("SD card initialized successfully!");
    lcd.print("SD Card OK");
  }
  delay(500);
  lcd.clear();
  // ================================================== (4 - Bluetooth ) ==================================================================== //
  bluetooth.begin(9600);
  // ================================================== (5 - GPS ) ==================================================================== //
  gpsSerial.begin(9600);
   if (gpsSerial.available()) {
    lcd.print("GPS : OK");
   }
   else{
    lcd.print("GPS : ERROR");
   }
   delay(500);
   lcd.clear();
  // ================================================== (6 - Compass (GY-273) ) ==================================================================== // 
  MechaQMC5883 qmc;
 // ================================================== (7 - Motor ) ==================================================================== // 
  motor.setSpeed(50);
}
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| ( เข้า Main Program Loop ) ||||||||||||||||||||||||||||||||||||||||//
void loop(){ 
  PZEMMod();
  Compass();
  GPSLoop();
  SunPos();
  if(i=0){
    Motor();
  }
  BT();
  SDCARD();
}
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| ( จบ Main Program Loop ) ||||||||||||||||||||||||||||||||||||||||//

//===================================================================================================================================//
void PZEMMod(){
  currentMillisPZEM = millis();                                                                     /* count time for program run every second (by default)*/
        if (currentMillisPZEM - startMillisPZEM >= periodPZEM)                                            /* for every x seconds, run the codes below*/
        {    
          uint8_t result;                                                                                 /* Declare variable "result" as 8 bits */   
          result = node.readInputRegisters(0x0000, 6);                                                    /* read the 9 registers (information) of the PZEM-014 / 016 starting 0x0000 (voltage information) kindly refer to manual)*/
          if (result == node.ku8MBSuccess)                                                                /* If there is a response */
            {
              uint32_t tempdouble = 0x00000000;                                                           /* Declare variable "tempdouble" as 32 bits with initial value is 0 */ 
              PZEMVoltage = node.getResponseBuffer(0x0000) / 100.0;                                       /* get the 16bit value for the voltage value, divide it by 100 (as per manual) */
                                                                                                          // 0x0000 to 0x0008 are the register address of the measurement value
              PZEMCurrent = node.getResponseBuffer(0x0001) / 100.0;                                       /* get the 16bit value for the current value, divide it by 100 (as per manual) */
              
              tempdouble =  node.getResponseBuffer(0x0003)  + node.getResponseBuffer(0x0002);      /* get the power value. Power value is consists of 2 parts (2 digits of 16 bits in front and 2 digits of 16 bits at the back) and combine them to an unsigned 32bit */
              PZEMPower = tempdouble / 10.0;                                                              /* Divide the value by 10 to get actual power value (as per manual) */
              
              tempdouble =  node.getResponseBuffer(0x0005)  + node.getResponseBuffer(0x0004);      /* get the energy value. Energy value is consists of 2 parts (2 digits of 16 bits in front and 2 digits of 16 bits at the back) and combine them to an unsigned 32bit */
              PZEMEnergy = tempdouble;                                                                             
  
              Serial.print(PZEMVoltage, 1);                                                               /* Print Voltage value on Serial Monitor with 1 decimal*/
              Serial.print("V   ");
              Serial.print(PZEMCurrent, 3);
              Serial.print("A   ");
              Serial.print(PZEMPower, 1);
              Serial.print("W  ");
              Serial.print(PZEMEnergy, 0);
              Serial.print("Wh  ");
              Serial.println();
              
              if (pzemSlaveAddr==2)                                                                       /* just for checking purpose to see whether can read modbus*/
                {
                  Serial.println(); 
                }
            } 
              else
                {
                  Serial.println("Failed to read modbus");
                  lcd.print("PZEM Error");  // ต่อตัววัดไฟผิด อย่าลืมทำใน Manual ด้วย Error Code
                }
              startMillisPZEM = currentMillisPZEM ;                                                       /* Set the starting point again for next counting time */
        }
  lcd.clear();
  lcd.print("Voltage: ");
  lcd.print(PZEMVoltage, 1);
  lcd.print("V");
  lcd.setCursor(0, 1);
  lcd.print("Current: ");
  lcd.print(PZEMCurrent, 3);
  lcd.print("A");
  delay(500);
  lcd.clear();
  lcd.print("Power: ");
  lcd.print(PZEMPower, 0);
  lcd.print("W");
  delay(500);
}
//=================================================================================================================================//
void Compass(){
  lcd.clear();
  qmc.read(&x, &y, &z, &azimuth);

  float magneticFieldStrength = sqrt(x * x + y * y + z * z); // คำนวณค่าแรงสนามแม่เหล็ก

  // แปลงค่าเมตริกซ์เป็นแกรม
  float magneticFieldStrength_gauss = magneticFieldStrength / 10.0;
  // แปลงค่าแรงสนามแม่เหล็กในแกน x และ y เป็นมุม (องศา)
  float angle_radians = atan2(static_cast<float>(y), static_cast<float>(x));
  if (angle_radians < 0) {
    angle_radians += 2 * PI;
  }

  // แปลงมุมจาก radians เป็น degrees
  float angle_degrees = angle_radians * 180.0 / PI;

  Serial.println();
  Serial.print("x: ");
  Serial.print(x);
  Serial.print(" y: ");
  Serial.print(y);
  Serial.print(" z: ");
  Serial.print(z);
  Serial.print(" Azimuth: ");
  Serial.print(azimuth);
  Serial.print(" Magnetic Field Strength (Gauss): ");
  Serial.print(magneticFieldStrength_gauss);
  Serial.print(" Angle (Degrees): ");
  Serial.print(angle_degrees);
  Serial.println();
  delay(1000);
  /*qmc.read(&x, &y, &z, &azimuth);
  float angle_radians = atan2(static_cast<float>(y), static_cast<float>(x));
  if (angle_radians < 0) {
    angle_radians += 2 * PI;
  }
   // แปลงมุมจาก radians เป็น degrees
  int angle_degrees = angle_radians * 180.0 / PI;
  Serial.print("Azimuth N: ");
  Serial.print(azimuth);*/
  if ((k==0) && (azimuth!=0)){
    azimuthStart=azimuth;
    k=1;
   Serial.print("Start :");
  Serial.print(azimuthStart);
  }
 
  if((azimuthStart>=0)&&(azimuthStart<6)){
    WindowsN=60;
  }
  if((azimuthStart>5)&&(azimuthStart<16)){
    WindowsN=50;
  }
  if((azimuthStart>15)&&(azimuthStart<26)){
    WindowsN=40;
  }
  if((azimuthStart>25)&&(azimuthStart<36)){
    WindowsN=30;
  }  
  if((azimuthStart>35)&&(azimuthStart<46)){
    WindowsN=20;
  }
  if((azimuthStart>45)&&(azimuthStart<56)){
    WindowsN=10;
  }
  if((azimuthStart>55)&&(azimuthStart<66)){
    WindowsN=0;
  }
  if((azimuthStart>65)&&(azimuthStart<76)){
    WindowsN=350;
  }
  if((azimuthStart>75)&&(azimuthStart<86)){
    WindowsN=340;
  }
  if((azimuthStart>85)&&(azimuthStart<96)){
    WindowsN=330;
  }
  if((azimuthStart>95)&&(azimuthStart<106)){
    WindowsN=320;
  }
  if((azimuthStart>105)&&(azimuthStart<116)){
    WindowsN=310;
  }
  if((azimuthStart>115)&&(azimuthStart<126)){
    WindowsN=300;
  }
  if((azimuthStart>125)&&(azimuthStart<136)){
    WindowsN=290;
  }
  if((azimuthStart>135)&&(azimuthStart<146)){
    WindowsN=280;
  }
  if((azimuthStart>145)&&(azimuthStart<156)){
    WindowsN=270;
  }
  if((azimuthStart>155)&&(azimuthStart<166)){
    WindowsN=260;
  }
  if((azimuthStart>165)&&(azimuthStart<176)){
    WindowsN=250;
  }
  if((azimuthStart>175)&&(azimuthStart<186)){
    WindowsN=240;
  }
  if((azimuthStart>185)&&(azimuthStart<196)){
    WindowsN=230;
  }
  if((azimuthStart>195)&&(azimuthStart<206)){
    WindowsN=220;
  }
  if((azimuthStart>205)&&(azimuthStart<216)){
    WindowsN=210;
  }
  if((azimuthStart>215)&&(azimuthStart<226)){
    WindowsN=200;
  }
  if((azimuthStart>225)&&(azimuthStart<236)){
    WindowsN=190;
  }
  if((azimuthStart>235)&&(azimuthStart<246)){
    WindowsN=180;
  }
  if((azimuthStart>245)&&(azimuthStart<256)){
    WindowsN=170;
  }
  if((azimuthStart>255)&&(azimuthStart<266)){
    WindowsN=160;
  }
  if((azimuthStart>265)&&(azimuthStart<276)){
    WindowsN=150;
  }
  if((azimuthStart>275)&&(azimuthStart<286)){
    WindowsN=140;
  }
  if((azimuthStart>285)&&(azimuthStart<296)){
    WindowsN=130;
  }
  if((azimuthStart>295)&&(azimuthStart<306)){
    WindowsN=120;
  }
  if((azimuthStart>305)&&(azimuthStart<316)){
    WindowsN=110;
  }
  if((azimuthStart>315)&&(azimuthStart<326)){
    WindowsN=100;
  }
  if((azimuthStart>325)&&(azimuthStart<336)){
    WindowsN=90;
  }
  if((azimuthStart>335)&&(azimuthStart<346)){
    WindowsN=80;
  }
  if((azimuthStart>345)&&(azimuthStart<356)){
    WindowsN=70;
  }
  if((azimuthStart>355)&&(azimuthStart<360)){
    WindowsN=60;
  }
  Serial.println("WindowsNorth : ");
Serial.print(WindowsN);
}
//=================================================================================================================================//
void BT(){
  float voltage = PZEMVoltage;
  float current = PZEMCurrent;
  float power = PZEMPower;
  // แปลงพลังงานเป็นสตริงที่มีทศนิยม 1 ตำแหน่ง
  char voltageStr[10];
  dtostrf(voltage, 5, 1, voltageStr);
  char currentStr[10];
  dtostrf(current, 5, 1, currentStr);
  char powerStr[10];
  dtostrf(power, 5, 1, powerStr);
  // ส่งข้อมูลแรงดันผ่านทาง Bluetooth
  bluetooth.print("Voltage: ");
  bluetooth.println(voltageStr);
  // ส่งข้อมูลกระแสผ่านทาง Bluetooth
  bluetooth.print("Current: ");
  bluetooth.println(currentStr);
  // ส่งข้อมูลพลังงานผ่านทาง Bluetooth
  bluetooth.print("Power: ");
  bluetooth.println(powerStr);
   // ตรวจสอบว่ามีข้อมูลที่พร้อมสำหรับอ่านจาก Bluetooth หรือไม่
 while (bluetooth.available()) {
        inbyte = bluetooth.read();
          Serial.println(inbyte);
           // อ่านข้อมูลที่ได้รับจาก Bluetooth
if (inbyte==1)   // คำสั่งหมุน 45
  {
   i=1 ;
   j=1 ;
  }     
if (inbyte==2) // คำสั่งหมุน 90
{
  j=2;        
  i=1 ; 
  }
if (inbyte==3)// คำสั่งหมุน 135   
  {
    j=3;
       i=1 ;
  }        
if (inbyte==4)// คำสั่งหมุนซ้าย
  {
   motor.forward();
       i=1 ; 
       j=0;
  }
if (inbyte==5) // คำสั่งหมุนขวา
  {
   motor.backward();
       i=1;
       j=0; 
  }
if (inbyte==6)   //stop
  {
   motor.stop();
       i=1 ;
       j=0; 
  }
if (inbyte==0)//Auto
  {
       i=0 ;
       j=0;
  }
 }
 Serial.print(" Final = ");
 Serial.println(Final);
if (j==1){
   AngleMin = WindowsN-90;
   if(AngleMin < 0 ){
    AngleMin = AngleMin+360;// เช็คด้วยว่า อย่าเกิน 0 --- 360   gammaN-anglemin  = องศาที่แผงต้องหมุนจริง เอาค่านี้ไป *1.11 จะได้ค่าเซ็นเซอร์ เอาไปบวกกะ ค่าต่ำสุดเซ็นเซอร์
   }
   AngleMax = WindowsN+90;
   if(AngleMax > 360 ){
    AngleMax=AngleMax-360;
   }
   RotateAngle = 45;
   RotateSensor = RotateAngle*1.11;
   azimuthMax = azimuthStart +100;
   if(azimuthMax > 360){
    azimuthMax = azimuthMax-360;
   }
   azimuthMin = azimuthStart-100;
   if(azimuthMin < 0){
    azimuthMin = azimuthMin+360;
   }
   Final=RotateSensor+azimuthMin;
    if(azimuth<Final){
      Serial.println("azimuth J: ");
      Serial.print(azimuth);
      motor.forward();
      Serial.println("Foward");
       if((Final+5>azimuth)&&(Final-5<azimuth)){
      motor.stop();
      Serial.println("stop");
    }
    }
    if(azimuth>Final){
      motor.backward();
      Serial.println("azimuth J: ");
      Serial.print(azimuth);
      Serial.println("back");
      //Serial.println(Final+10);
      Serial.println(azimuth);   
        if((Final+5>azimuth)&&(Final-5<azimuth)){
      motor.stop();
      Serial.print("Stop");
      }
  }
}
if (j==2){
   AngleMin = WindowsN-90;
   if(AngleMin < 0 ){
    AngleMin = AngleMin+360;// เช็คด้วยว่า อย่าเกิน 0 --- 360   gammaN-anglemin  = องศาที่แผงต้องหมุนจริง เอาค่านี้ไป *1.11 จะได้ค่าเซ็นเซอร์ เอาไปบวกกะ ค่าต่ำสุดเซ็นเซอร์
   }
   AngleMax = WindowsN+90;
   if(AngleMax > 360 ){
    AngleMax=AngleMax-360;
   }
   RotateAngle = 90;
   RotateSensor = RotateAngle*1.11;
   azimuthMax = azimuthStart+100;
   if(azimuthMax > 360){
    azimuthMax = azimuthMax-360;
   }
   azimuthMin = azimuthStart-100;
   if(azimuthMin < 0){
    azimuthMin = azimuthMin+360;
   }
   Final=RotateSensor+azimuthMin;
    if(azimuth<Final){
      Serial.println("azimuth J: ");
      Serial.print(azimuth);
      motor.forward();
      Serial.println("Foward");
       if((Final+5>azimuth)&&(Final-5<azimuth)){
      motor.stop();
      Serial.println("stop");
    }
    }
    if(azimuth>Final){
      motor.backward();
      Serial.println("azimuth J: ");
      Serial.print(azimuth);
      Serial.println("back");
     // Serial.println(Final+10);
      Serial.println(azimuth);   
        if((Final+5>azimuth)&&(Final-5<azimuth)){
      motor.stop();
      Serial.print("Stop");
      }
  }
}
if (j==2){
   AngleMin = WindowsN-90;
   if(AngleMin < 0 ){
    AngleMin = AngleMin+360;// เช็คด้วยว่า อย่าเกิน 0 --- 360   gammaN-anglemin  = องศาที่แผงต้องหมุนจริง เอาค่านี้ไป *1.11 จะได้ค่าเซ็นเซอร์ เอาไปบวกกะ ค่าต่ำสุดเซ็นเซอร์
   }
   AngleMax = WindowsN+90;
   if(AngleMax > 360 ){
    AngleMax=AngleMax-360;
   }
   RotateAngle = 90;
   RotateSensor = RotateAngle*1.11;
   azimuthMax = azimuthStart+100;
   if(azimuthMax > 360){
    azimuthMax = azimuthMax-360;
   }
   azimuthMin = azimuthStart-100;
   if(azimuthMin < 0){
    azimuthMin = azimuthMin+360;
   }
   Final=RotateSensor+azimuthMin;
    if(azimuth<Final){
      Serial.println("azimuth J: ");
      Serial.print(azimuth);
      motor.forward();
      Serial.println("Foward");
       if((Final+5>azimuth)&&(Final-5<azimuth)){
      motor.stop();
      Serial.println("stop");
    }
    }
    if(azimuth>Final){
      motor.backward();
      Serial.println("azimuth J: ");
      Serial.print(azimuth);
      Serial.println("back");
    //  Serial.println(Final+10);
      Serial.println(azimuth);   
        if((Final+5>azimuth)&&(Final-5<azimuth)){
      motor.stop();
      Serial.print("Stop");
      }
  }
}
if (j==3){
   AngleMin = WindowsN-90;
   if(AngleMin < 0 ){
    AngleMin = AngleMin+360;// เช็คด้วยว่า อย่าเกิน 0 --- 360   gammaN-anglemin  = องศาที่แผงต้องหมุนจริง เอาค่านี้ไป *1.11 จะได้ค่าเซ็นเซอร์ เอาไปบวกกะ ค่าต่ำสุดเซ็นเซอร์
   }
   AngleMax = WindowsN+90;
   if(AngleMax > 360 ){
    AngleMax=AngleMax-360;
   }
   RotateAngle = 135;
   RotateSensor = RotateAngle*1.11;
   azimuthMax = azimuthStart+100;
   if(azimuthMax > 360){
    azimuthMax = azimuthMax-360;
   }
   azimuthMin = azimuthStart-100;
   if(azimuthMin < 0){
    azimuthMin = azimuthMin+360;
   }
   Final=RotateSensor+azimuthMin;
    if(azimuth<Final){
      Serial.println("azimuth J: ");
      Serial.print(azimuth);
      motor.forward();
      Serial.println("Foward");
       if((Final+5>azimuth)&&(Final-5<azimuth)){
      motor.stop();
      Serial.println("stop");
    }
    }
    if(azimuth>Final){
      motor.backward();
      Serial.println("azimuth J: ");
      Serial.print(azimuth);
      Serial.println("back");
   //   Serial.println(Final+10);
      Serial.println(azimuth);   
        if((Final+5>azimuth)&&(Final-5<azimuth)){
      motor.stop();
      Serial.print("Stop");
      }
  }
}
}
//================================================================================================================================//
void GPSLoop(){
   while (gpsSerial.available()) {
      char data = gpsSerial.read();
      if (gps.encode(data)) {
        if (gps.location.isUpdated()) {
          Serial.print("Latitude: ");
          Serial.println(gps.location.lat(), 6);
          Lat = gps.location.lat();
          Serial.print("Longitude: ");
          Lon = gps.location.lng();
          Serial.println(gps.location.lng(), 6);
          lcd.clear();
          lcd.print("Lat: ");
          lcd.print(gps.location.lat(), 6);
          lcd.setCursor(0, 1);
          lcd.print("Lon: ");
          lcd.print(gps.location.lng(), 6);
        }
      delay(500);
      if (gps.date.isUpdated() && gps.time.isUpdated()) {
        lcd.clear();
        Serial.print("Date: ");
        Serial.print(gps.date.day());
        Serial.print("/");
        Serial.print(gps.date.month());
        Serial.print("/");
        Serial.println(gps.date.year());
        day = gps.date.day();
        month = gps.date.month();
        year = gps.date.year();
        Serial.print("Time: ");
        Serial.print(gps.time.hour());
        Serial.print(":");
        Serial.print(gps.time.minute());
        Serial.print(":");
        Serial.println(gps.time.second());
        hour = gps.time.hour();
        minute = gps.time.minute();
        second = gps.time.second();
        lcd.print("Date : ");
        lcd.print(gps.date.day());
        lcd.print("/");
        lcd.print(gps.date.month());
        lcd.print("/");
        lcd.print(gps.date.year());
        lcd.setCursor(0,1);
        lcd.print("Time");
        lcd.print(gps.time.hour()+7); 
        lcd.print(":");
        lcd.print(gps.time.minute());
      }
      delay(500);
    }
  }
}
//================================================================================================================================//
void SunPos(){
  float hr=hour+7;
     if(year%4>0) //เคสที่ไม่ใช่ปีอธิกสุรทิน
  {
   if(month==1)
   {
   Jd=day;
   }
   else if(month==2)
   {
   Jd=day+31;
   }
   else if(month==3)
   {
   Jd=day+31+28;
   }
   else if(month==4)
   {
   Jd=day+31+28+31;
   }
   else if(month==5)
   {
   Jd=day+31+28+31+30;
   }
   else if(month==6)
   {
   Jd=day+31+28+31+30+31;
   }
   else if(month==7)
   {
   Jd=day+31+28+31+30+31+30;
   }
   else if(month==8)
   {
   Jd=day+31+28+31+30+31+30+31;
   }
   else if(month==9)
   {
   Jd=day+31+28+31+30+31+30+31+31;
   }
   else if(month==10)
   {
   Jd=day+31+28+31+30+31+30+31+31+30;
   }
   else if(month==11)
   {
   Jd=day+31+28+31+30+31+30+31+31+30+31;
   }
   else if(month==12)
   {
   Jd=day+31+28+31+30+31+30+31+31+30+31+30;
   }
  }
   if(year%4==0) //ในเตสที่เป็นปีอธิกสุรทิน
  {
   if(month==1)
   {
   Jd=day;
   }
   if(month==2)
   {
   Jd=day+31;
   }
   if(month==3)
   {
   Jd=day+31+29;
   }
   else if(month==4)
   {
   Jd=day+31+29+31;
   }
   else if(month==5)
   {
   Jd=day+31+29+31+30;
   }
   else if(month==6)
   {
   Jd=day+31+29+31+30+31;
   }
   else if(month==7)
   {
   Jd=day+31+29+31+30+31+30;
   }
   else if(month==8)
   {
   Jd=day+31+29+31+30+31+30+31;
   }
   else if(month==9)
   {
   Jd=day+31+29+31+30+31+30+31+31;
   }
   else if(month==10)
   {
   Jd=day+31+29+31+30+31+30+31+31+30;
   }
   else if(month==11)
   {
   Jd=day+31+29+31+30+31+30+31+31+30+31;
   }
   else if(month==12)
   {
   Jd=day+31+29+31+30+31+30+31+31+30+31+30;
   }
  }
  b=360*(Jd-81)/365;
  Eot=9.87*sin(radians(2*b))-7.5*cos(radians(b))-1.5*sin(radians(b)); //หา EOT
  double Hr=static_cast<int>(hour + 7);
  float mn=static_cast<int>(minute);
  float sc=static_cast<int>(second);
  t=(Hr)+(mn+(sc/60))/60;
  Serial.print(t,4); /*Check Clock*/
  ts=t-(4*(Lon-Lat)+Eot)/60; //หาค่า Solar Time ให้ t เป็นเวลลาตามนาฬิกา
  Delta=23.45*sin(radians(360*(Jd+284)/365)); //หา Declination angle
  Omega=(PI*(ts-12)/12)*(180/PI);
  Alpha=degrees(asin(sin(radians(Lat))*sin(radians(Delta))+cos(radians(Lat))*cos(radians(Delta))*cos(radians(Omega)))); //หามุมเงย
  Gamma=degrees(asin(cos(radians(Delta))*sin(radians(Omega))/cos(radians(Alpha))));//หามุมกวาด Azimuth
  if(Delta<Lat)//อ้อมใต้
   {
   Gamma=Gamma;
   Serial.print("Solar Azimuth=");
   Serial.println(Gamma,4);
   }
  if(Delta>Lat)//อ้อมเหนือ
   {
   if(Gamma<0)
   {
    Gamma=-(180-abs(Gamma));
    Serial.print("Solar Azimuth=");
    Serial.println(Gamma,4);
   }
   if(Gamma>=0)
   {
    Gamma=180-abs(Gamma);
    Serial.print("Solar Azimuth=");
    Serial.println(Gamma,4);
   }
   }
   if (Gamma<0){
   Gamma=360+Gamma;
}
}
//===================================================================================================================================//
void Motor(){
   if (l=0){
   if (Gamma < 180){
    GammaN = Gamma+180;
   }
   else if (Gamma >=180){
    GammaN = Gamma-180;
   }
/*   if (Windows <180){
    WindowsN = Windows+180;
   }
   else if (Windows >=180){
    WindowsN = Windows-180;
   }  */
   AngleMin = WindowsN-90;
   if(AngleMin < 0 ){
    AngleMin = AngleMin+360; // เช็คด้วยว่า อย่าเกิน 0 --- 360   gammaN-anglemin  = องศาที่แผงต้องหมุนจริง เอาค่านี้ไป *1.11 จะได้ค่าเซ็นเซอร์ เอาไปบวกกะ ค่าต่ำสุดเซ็นเซอร์
   }
   AngleMax = WindowsN+90;
   if(AngleMax > 360 ){
    AngleMax=AngleMax-360;
   }
   RotateAngle = GammaN-AngleMin;
   RotateSensor = RotateAngle*1.11;
   azimuthMax = azimuthStart+100;
   if(azimuthMax > 360){
    azimuthMax = azimuthMax-360;
   }
   azimuthMin = azimuthStart-100;
   if(azimuthMin < 0){
    azimuthMin = azimuthMin+360;
   }
   Final=RotateSensor+azimuthMin;
   if(AngleMin<=GammaN<=AngleMax){
    Serial.println("Sun In Range");
    if(azimuth<Final){
      motor.forward();
      Serial.println("Foward");
      if((Final+5>azimuth)&&(Final-5<azimuth)){
      motor.stop();
      Serial.println("stop");
    }
    }
    if(azimuth>Final){
      motor.backward();
      Serial.println("back");
    }
    if((Final+5>azimuth)&&(Final-5<azimuth)){
      motor.stop();
     Serial.println("stop");
    }
   }
    else{
      Serial.println("Sun Out Range");
      if(azimuth < azimuthMax){
        motor.forward();
      }
      if(azimuth > azimuthMax){
      motor.stop();
   }
    }
    l=1;
   }
   if (l=1){
    if((millis() >= time_old + 10000)){
   if (Gamma < 180){
    GammaN = Gamma+180;
   }
   else if (Gamma >=180){
    GammaN = Gamma-180;
   }
/*   if (Windows <180){
    WindowsN = Windows+180;
   }
   else if (Windows >=180){
    WindowsN = Windows-180;
   }  */
   AngleMin = WindowsN-90;
   if(AngleMin < 0 ){
    AngleMin = AngleMin+360; // เช็คด้วยว่า อย่าเกิน 0 --- 360   gammaN-anglemin  = องศาที่แผงต้องหมุนจริง เอาค่านี้ไป *1.11 จะได้ค่าเซ็นเซอร์ เอาไปบวกกะ ค่าต่ำสุดเซ็นเซอร์
   }
   AngleMax = WindowsN+90;
   if(AngleMax > 360 ){
    AngleMax=AngleMax-360;
   }
   RotateAngle = GammaN-AngleMin;
   RotateSensor = RotateAngle*1.11;
   azimuthMax = azimuthStart+100;
   if(azimuthMax > 360){
    azimuthMax = azimuthMax-360;
   }
   azimuthMin = azimuthStart-100;
   if(azimuthMin < 0){
    azimuthMin = azimuthMin+360;
   }
   Final=RotateSensor+azimuthMin;
   if(AngleMin<=GammaN<=AngleMax){
    Serial.println("Sun In Range");
    if(azimuth<Final){
      motor.forward();
      Serial.println("Foward");
      if((Final+10>azimuth)&&(Final-10<azimuth)){
      motor.stop();
      Serial.println("stop");
    }
    }
    if(azimuth>Final){
      motor.backward();
      Serial.println("back");
    }
    if((Final+5>azimuth)&&(Final-5<azimuth)){
      motor.stop();
     Serial.println("stop");
    }
   }
    else{
      Serial.println("Sun Out Range");
      if(azimuth < azimuthMax){
        motor.forward();
      }
      if(azimuth > azimuthMax){
      motor.stop();
   }
    }
    l=1;
    time_old=millis();
   }
   }
}
//===================================================================================================================================//
void SDCARD(){
  String dataString = "";
      dataString += "Date";
      dataString += String(gps.date.day());
      dataString += "/";
      dataString += String(gps.date.month());
      dataString += "/";
      dataString += String(gps.date.year());
      dataString += "\t";
      dataString += "Time";
      dataString += String(gps.time.hour() +7 );
      dataString += ":";
      dataString += String(gps.time.minute());
      dataString += "\t";
      dataString += "PV Angle";
      dataString += String(azimuth);
      dataString += "\t"; 
      dataString +="Voltage: ";
      dataString += String(PZEMVoltage, 1);
      dataString += " V ";
      dataString +="Current: ";
      dataString +=String(PZEMCurrent, 3);
      dataString +=" A ";
      dataString +="Power: ";
      dataString +=String(PZEMEnergy, 0);
      dataString +=" W ";

      if((millis() >= time_old + 10000)){
      File dataFile = SD.open("VPVLog.txt", FILE_WRITE);
      if (dataFile) 
      {
            dataFile.println(dataString);
            dataFile.close();
            Serial.println(dataString);
      }
      else 
      {
        Serial.println("error opening VPVLog.txt");
        lcd.clear();
        lcd.print("error opening VPVLog.txt");
      }
      time_old=millis();
      Serial.println();           
}
}
//===================================================================================================================================//

//==================================================================  DO NOT TOUCH =============================================================================//
void preTransmission()                                                                                    /* transmission program when triggered*/
{
        /* 1- PZEM-017 DC Energy Meter */
        
        digitalWrite(MAX485_RE, 1);                                                                       /* put RE Pin to high*/
        digitalWrite(MAX485_DE, 1);                                                                       /* put DE Pin to high*/
        delay(1);                                                                                         // When both RE and DE Pin are high, converter is allow to transmit communication
}

void postTransmission()                                                                                   /* Reception program when triggered*/
{       
        /* 1- PZEM-017 DC Energy Meter */
        
        delay(3);                                                                                         // When both RE and DE Pin are low, converter is allow to receive communication
        digitalWrite(MAX485_RE, 0);                                                                       /* put RE Pin to low*/
        digitalWrite(MAX485_DE, 0);                                                                       /* put DE Pin to low*/
}

void changeAddress(uint8_t OldslaveAddr, uint8_t NewslaveAddr)                                            //Change the slave address of a node
{
        /* 1- PZEM-017 DC Energy Meter */
        
        static uint8_t SlaveParameter = 0x06;                                                             /* Write command code to PZEM */
        static uint16_t registerAddress = 0x0002;                                                         /* Modbus RTU device address command code */
        uint16_t u16CRC = 0xFFFF;                                                                         /* declare CRC check 16 bits*/
        u16CRC = crc16_update(u16CRC, OldslaveAddr);                                                      // Calculate the crc16 over the 6bytes to be send
        u16CRC = crc16_update(u16CRC, SlaveParameter);
        u16CRC = crc16_update(u16CRC, highByte(registerAddress));
        u16CRC = crc16_update(u16CRC, lowByte(registerAddress));
        u16CRC = crc16_update(u16CRC, highByte(NewslaveAddr));
        u16CRC = crc16_update(u16CRC, lowByte(NewslaveAddr));
      
        Serial.println("Change Slave Address");
        preTransmission();                                                                                 /* trigger transmission mode*/
      
        Serial3.write(OldslaveAddr);                                                                       /* these whole process code sequence refer to manual*/
        Serial3.write(SlaveParameter);
        Serial3.write(highByte(registerAddress));
        Serial3.write(lowByte(registerAddress));
        Serial3.write(highByte(NewslaveAddr));
        Serial3.write(lowByte(NewslaveAddr));
        Serial3.write(lowByte(u16CRC));
        Serial3.write(highByte(u16CRC));
        delay(10);
        postTransmission();                                                                                /* trigger reception mode*/
        delay(100);
        while (Serial3.available())                                                                        /* while receiving signal from Serial3 from meter and converter */
          {   
            Serial.print(char(Serial3.read()), HEX);                                                       /* Prints the response and display on Serial Monitor (Serial)*/
            Serial.print(" ");
          }
}
