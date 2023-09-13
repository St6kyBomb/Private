// ================================================== ( 1 - PZEM-017 DC Energy Meter ) ====================================================================//
#include <ModbusMaster.h>  //เรียก Library ของ PZEM (อุปกรณ์สำหรับวัดไฟและบันทึก หาข้อมูลเพิ่มเติมLibrary ได้ใน Example)
#define MAX485_DE      3   // pin 14 (Tx) and pin 15 (Rx)Serial 3 คือคู่ 14-15//
#define MAX485_RE      2  //กำหนดขา DE RE
static uint8_t pzemSlaveAddr = 0x01;  //อธิบายไม่ถูก มันมากะเครื่อง ต้องไปอ่าน Manual เครื่อง มันเป็นที่อยู่ของข้อมูลที่บันทึก
static uint16_t NewshuntAddr = 0x0003;
ModbusMaster node;  //สร้าง Object
unsigned long time_old;  //สร้างตัวแปรมาเพื่อ Delay การวัดไฟโดยไม่ใช้คำสั่ง Delay 
float PZEMVoltage =0;  //สร้างตัวแปรเก็บ Volt
float PZEMCurrent =0;  //สร้างตัวแปรเก็บ Amp
float PZEMPower =0;  //สร้างตัวแปรเก็บ Watt
float PZEMEnergy=0;  //สร้างตัวแปรเก็บ WattHour
unsigned long startMillisPZEM; //สร้างตัวแปรมาเพื่อ Delay การวัดไฟโดยไม่ใช้คำสั่ง Delay (เก็บเวลาเริ่มต้น)
unsigned long currentMillisPZEM; //สร้างตัวแปรมาเพื่อ Delay การวัดไฟโดยไม่ใช้คำสั่ง Delay  (เก็บเวลาจำนวนวิที่ทำงานไปแล้ว)
const unsigned long periodPZEM = 1000; //สร้างตัวแปรมาเพื่อ Delay การวัดไฟโดยไม่ใช้คำสั่ง Delay (Delay 1 วิ)
// ================================================== (2 - LCD Screen ) ==================================================================== //
#include <LiquidCrystal_I2C.h>  //เรียก Library ของ จอ LCD I2C (I2C มีคู่เดียว ถ้ามีอุปกรณ์หลายตัว สามารถเสียบทับกันได้เลย)
const int lcdRows = 2;  //กำหนดจำนวนบรรทัด (ขึ้นกับขนาดจอ จอนี้เป็น 16x2)
const int lcdColumns = 16; //กำหนดจำนวนตัวอักษรจอใน1บรรทัด (ตามความยาวจอ)
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);  //สร้าง Object 
// ================================================== (3 - SD Card ) ==================================================================== //
#include <SD.h>  //เรียก Library ของ SD Card
#include <SPI.h> //เรียก Library ของ SD Card
#include <Wire.h> //เรียก Library ของ ขาประเภท rx,tx,sck ถ้าจำไม่ผิด
const int chipSelect = 53;  // MOSI - pin 51, MISO - pin 50, sck - pin 52, CS - pin 53 กำหนดขา cs 
// ================================================== (4 - Bluetooth ) ==================================================================== // 
#include <SoftwareSerial.h>  //เรียก Library ของการสร้าง Serial (พวก RX,TX นั้นแหละ Mega มี 3 Serial ถ้าจำไม่ผิด
SoftwareSerial bluetooth(10,11); /* RX, TX */ //สร้าง Object และ กำหนดขา RX TX ว่าให้เสียบช่องไหน 
int inbyte; //สร้างตัวแปรเพื่อรับค่ามาจากในโทรศํพท์
int Window; //สร้างตัวแปรเพื่อรับค่ามุมของหน้าต่าง
int i=0; //สร้างตัวแปรเพื่อกำหนดเข้า Loop Auto 
int j=0; //ตัวแปรที่เอาไว้กำหนดมุมตามโทรศัทพ์ Bluetooth รับได้แค่ค่า Byte มาแปรเป็นคำสั่งต่างๆ (นึกถึงเลขบนรีโมททีวี)
int l=0; //ตัวแปรทำ Loop นับเวลา
int p=0; //ตัวแปร ไม่ให้เครื่องทำงานโดยที่ไม่ได้กำหนดมุมของหน้าต่าง
int k;// สร้างตัวแปร Loop //
// ================================================== (5 - RTC ) ==================================================================== // 
#include <RTClib.h> //เรียก Library ของ นาฬิกา 
RTC_DS1307 rtc; //สร้าง Object
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"}; //สร้างตัวแปรเพื่อแสดงวัน
float Lat = 13.80; //สร้างตัวแปรเพื่อกำหนด Latitude
float Longl = 100.03; //สร้างตัวแปรเพื่อกำหนด Longitude Actual
float Longs = 105; ////สร้างตัวแปรเพื่อกำหนด Longitude Local
float b,Eot,Delta,Omega,Alpha,t,ts,Gamma,GammaOld ; // (เช็คสูตรการหาตำแหน่ง) b=ค่าB ในสมการ,EOT=Equation of time,Delta=Declination angle,Omega=Solar hour angle,Alpha=มุมเงย,tเวลา,ts=solartime,Gammaมุมกวาด
double Jd ; //Julian Date
int YEAR; //ปี
float DAY,MONTH,HOUR,MINUTE,SECOND; //วันเวลา
// ================================================== (6 - Stepper Motor ) ==================================================================== // 
#include <AccelStepper.h> //เรียก Library Stepper
float WindowMax,WindowMin,RotateAngle,Final; //ขอบเขตหน้าต่าง Rotate มุมหมุนที่Manual , Final มุมหมุนของ Auto (จริงๆ Rotate ไม่ต้องใช้แล้วก็ได้มาจากโค้ดเก่า)
#define dirPin 7
#define stepPin 8
#define motorInterfaceType 1
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin); //สร้าง Object ของ Motor
// ||||||||||||||||||||||||||||||||||||||||||||||||||| (Setup Phase ) ||||||||||||||||||||||||||||||||||||||||||||||||||| //
void setup(){
  // ================================================== ( 1 - PZEM-017 DC Energy Meter ) ====================================================================//
  Serial.begin(9600); //กำหนด Buadrate ของSerial Monitor
  startMillisPZEM = millis(); //เอาค่าเวลาที่เครื่องเริ่มทำงานมาบันทึก
  Serial3.begin(9600,SERIAL_8N2);  //กำหนดขา RX TX (อ่านเพิ่มเติมที่ บรรทัดที่ 3) 
  node.begin(pzemSlaveAddr, Serial3);  /* By default communicate via Serial3 port: pin 14 (Tx) and pin 15 (Rx)*/
  pinMode(MAX485_RE, OUTPUT); //กำหนดประเภท PIn
  pinMode(MAX485_DE, OUTPUT); //กำหนดประเภท PIn
  digitalWrite(MAX485_RE, 0); //ปิดขา RE
  digitalWrite(MAX485_DE, 0); //ปิดขา DE
  node.preTransmission(preTransmission); //กระโดดลงไปสื่อสารกับ PZEM (โค้ดอยู่ด้านล่าง)
  node.postTransmission(postTransmission); //กระโดดลงไปสื่อสารกับ PZEM (โค้ดอยู่ด้านล่าง)
  changeAddress(0XF8, 0x01); //กระโดดลงไปเปลี่ยนที่อยู่ PZEM (โค้ดอยู่ด้านล่าง)
  // ================================================== (2 - LCD Screen ) ==================================================================== //
  lcd.begin(); //เริ่มการทำงานหน้าจอ
  lcd.setCursor(0,0); //ตั้ง Cursor
  Wire.begin(); //เริ่มการทำงานการสื่อสารด้วย I2C
  lcd.backlight();  //เปิดไฟพื้นหลังจอ
  // ================================================== (3 - SD Card ) ==================================================================== //
  if (!SD.begin(chipSelect)) { //เช็ค SDcard
    Serial.println("SD card initialization failed!"); //ถ้าใช้ไม่ได้ให้ Print ____
    lcd.print("SD not found"); //ใช้ไม่ได้ให้เขียนบนจอ LCD
  }
  else { //เหมืนอด้านบนแค่ใช้ได้
    Serial.println("SD card initialized successfully!");
    lcd.print("SD Card OK");
  }
  lcd.clear(); //ลบข้อความบนจอ
  // ================================================== (4 - Bluetooth ) ==================================================================== //
  bluetooth.begin(9600); //เริ่มการทำงาน Bluetooth
  // ================================================== (5 - RTC ) ==================================================================== //
  if (! rtc.begin()) { //เช็คการทำงานนาฬิกา
    Serial.println("Couldn't find RTC"); //ใช้ไม่ได้ให้แจ้งบอก
    lcd.print("RTC : ERROR)"); //แจ้งขึ้นจอ
    lcd.clear(); //ลบข้อความบนจอ
    while (1) delay(10); //Loop 
  }
  if (! rtc.isrunning()) { //นาฬิกาทำงานแต่ไม่มีค่าเวลาอยู่
    Serial.println("RTC is NOT running, let's set the time!");
    lcd.print("RTC : OK)");
    lcd.clear();
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); //ปรับเวลาให้ตรง
  }
 // ================================================== (6 - Motor ) ==================================================================== // 
  stepper.setMaxSpeed(50); //กำหนดความเร็วสูงสุด Motor
  stepper.setAcceleration(50); //กำหนดความเร่ง Motor
}


//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| ( เข้า Main Program Loop ) ||||||||||||||||||||||||||||||||||||||||//
void loop(){ 
  PZEMMod();
  BT();
  RTCCheck();
  SunPos();
  if(i==0 && p==1){
    Motor();
  }
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
  delay(100);
  lcd.clear();
  lcd.print("Power: ");
  lcd.print(PZEMPower, 0);
  lcd.print("W");
  delay(100);
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
   myStepper.step(stepsPerRevolution);
       i=1 ; 
       j=0;
  }
if (inbyte==5) // คำสั่งหมุนขวา
  {
   myStepper.step(-stepsPerRevolution);
       i=1;
       j=0; 
  }
if (inbyte==6)   //stop
  {
   myStepper.step(0);
       i=1 ;
       j=0; 
  }
  if (inbyte==7)//South
  {
       i=0 ;
       p=1;
       Window=360;
  }
  if (inbyte==8)//West
 {
       i=0 ;
       p=1;
       Window=90;
  }
  if (inbyte==9)//North
 {
       i=0 ;
       p=1;
       Window=180;
  }
  if (inbyte==10)//East
 {
       i=0 ;
       p=1;
       Window=270;
  }
if (inbyte==0)//Auto
  {
       i=0 ;
       j=0;
  }
  
 }
if (j==1){
   RotateAngle=45;
   
   //หัน 45
}
if (j==2){
   RotateAngle=90;
   //หัน90
}
if (j==3){
   RotateAngle=135;
   //หัน135
  }
  lcd.print("Direction?");
}
//================================================================================================================================//
void RTCCheck(){
  DateTime now = rtc.now();
    
    DAY = (now.day());
    MONTH = (now.month());
    YEAR = (now.year());
    HOUR = (now.hour());
    MINUTE = (now.minute());
    SECOND = (now.second());
    Serial.print(DAY);
    Serial.print('/');
    Serial.print(MONTH);
    Serial.print('/');
    Serial.print(YEAR);
    Serial.print(" (");
    Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
    Serial.print(") ");
    Serial.print(HOUR);
    Serial.print(':');
    Serial.print(MINUTE);
    Serial.print(':');
    Serial.print(SECOND);
    Serial.println();
    lcd.clear();
    lcd.print("Date : ");
    lcd.print(now.day(), DEC);
    lcd.print("/");
    lcd.print(now.month(), DEC);
    lcd.print("/");
    lcd.print(now.year(), DEC);
    lcd.setCursor(0,1);
    lcd.print("Time");
    lcd.print(now.hour(), DEC); 
    lcd.print(":");
    lcd.print(now.minute(), DEC);
    lcd.print(":");
    lcd.print(now.second(), DEC);
    
}
//================================================================================================================================//
void SunPos(){
     if(YEAR%4>0) //เคสที่ไม่ใช่ปีอธิกสุรทิน
  {
   if(MONTH==1)
   {
   Jd=DAY;
   }
   else if(MONTH==2)
   {
   Jd=DAY+31;
   }
   else if(MONTH==3)
   {
   Jd=DAY+31+28;
   }
   else if(MONTH==4)
   {
   Jd=DAY+31+28+31;
   }
   else if(MONTH==5)
   {
   Jd=DAY+31+28+31+30;
   }
   else if(MONTH==6)
   {
   Jd=DAY+31+28+31+30+31;
   }
   else if(MONTH==7)
   {
   Jd=DAY+31+28+31+30+31+30;
   }
   else if(MONTH==8)
   {
   Jd=DAY+31+28+31+30+31+30+31;
   }
   else if(MONTH==9)
   {
   Jd=DAY+31+28+31+30+31+30+31+31;
   }
   else if(MONTH==10)
   {
   Jd=DAY+31+28+31+30+31+30+31+31+30;
   }
   else if(MONTH==11)
   {
   Jd=DAY+31+28+31+30+31+30+31+31+30+31;
   }
   else if(MONTH==12)
   {
   Jd=DAY+31+28+31+30+31+30+31+31+30+31+30;
   }
  }
   if(YEAR%4==0) //ในเตสที่เป็นปีอธิกสุรทิน
  {
   if(MONTH==1)
   {
   Jd=DAY;
   }
   if(MONTH==2)
   {
   Jd=DAY+31;
   }
   if(MONTH==3)
   {
   Jd=DAY+31+29;
   }
   else if(MONTH==4)
   {
   Jd=DAY+31+29+31;
   }
   else if(MONTH==5)
   {
   Jd=DAY+31+29+31+30;
   }
   else if(MONTH==6)
   {
   Jd=DAY+31+29+31+30+31;
   }
   else if(MONTH==7)
   {
   Jd=DAY+31+29+31+30+31+30;
   }
   else if(MONTH==8)
   {
   Jd=DAY+31+29+31+30+31+30+31;
   }
   else if(MONTH==9)
   {
   Jd=DAY+31+29+31+30+31+30+31+31;
   }
   else if(MONTH==10)
   {
   Jd=DAY+31+29+31+30+31+30+31+31+30;
   }
   else if(MONTH==11)
   {
   Jd=DAY+31+29+31+30+31+30+31+31+30+31;
   }
   else if(MONTH==12)
   {
   Jd=DAY+31+29+31+30+31+30+31+31+30+31+30;
   }
  }
  Serial.print("Jd = ");
  Serial.println(Jd);
  b=360*(Jd-81)/365;
  Serial.print("B = ");
  Serial.println(b);
  Eot=9.87*sin(radians(2*b))-7.5*cos(radians(b))-1.5*sin(radians(b)); //หา EOT
  Serial.print("EOT = ");
  Serial.println(Eot);
  t=((HOUR)+(MINUTE+(SECOND/60))/60);
  Serial.print("T = ");
  Serial.println(t,4); /*Check Clock*/
  ts = t-(4*(Longs-Longl)+Eot)/60; //หาค่า Solar Time ให้ t เป็นเวลลาตามนาฬิกา
  Serial.print("TS = ");
  Serial.println(ts);
  Delta=23.45*sin(radians(360*(Jd+284)/365)); //หา Declination angle
  Serial.print("Delta = ");
  Serial.println(Delta);
  Omega=(PI*(ts-12)/12)*(180/PI);
  Serial.print("Omega = ");
  Serial.println(Omega);
  Alpha=degrees(asin(sin(radians(Lat))*sin(radians(Delta))+cos(radians(Lat))*cos(radians(Delta))*cos(radians(Omega)))); //หามุมเงย
  Serial.print("Alpha = ");
  Serial.println(Alpha);
  Gamma=degrees(asin(cos(radians(Delta))*sin(radians(Omega))/cos(radians(Alpha))));//หามุมกวาด Azimuth
  if(Delta<Lat)//อ้อมใต้
   {
    Serial.print("อ้อมใต้ ");
   Gamma=Gamma;
   Serial.print("Solar Azimuth=");
   Serial.println(Gamma,4);
   }
  if(Delta>Lat)//อ้อมเหนือ
   {
   if(Gamma<0)
   {
    Serial.print("อ้อมเหนือ <0");
    Gamma=-(180-abs(Gamma));
    Serial.print("Solar Azimuth=");
    Serial.println(Gamma,4);
   }
   if(Gamma>=0)
   {
     Serial.print("อ้อมเหนือ >0");
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
   if (l==0){   //loop delay
    if (k==0){
   //map(MotorAngle, 0ของ motor , 180ของ motor , 0,179);
   //เคสทิศ S
   if (Window == 360){
    WindowMin=270;
    WindowMax=450;
      if (Gamma<270){
      Gamma=Gamma+360;
    }
      if (Gamma<WindowMax && Gamma>WindowMin){
        if (Gamma<Window){
        Final = Window-Gamma;
        stepsPerRevolution = Final*0.7; //0.7 คือค่าองศาต่อrev
        myStepper.step(stepsPerRevolution);
        GammaOld = Gamma;
          }
        else if(Gamma>=Window){
        Final = Gamma-Window;
        stepsPerRevolution = Final*0.7; //0.7 คือค่าองศาต่อrev
        myStepper.step(stepsPerRevolution);
        GammaOld = Gamma;
          }
        k=1;
        }
      else{
        Final = 90;
        stepsPerRevolution = Final*0.7; //0.7 คือค่าองศาต่อrev
        myStepper.step(stepsPerRevolution);
        GammaOld=Final;
        k=1;
        }
   } 
    //เคสทิศ W
    if (Window == 90){
    WindowMin=0;
    WindowMax=180;
      if (Gamma<WindowMax && Gamma>WindowMin){
        if (Gamma <Window){
    Final = Window-Gamma;
    stepsPerRevolution = Final*0.7; //0.7 คือค่าองศาต่อrev
    myStepper.step(stepsPerRevolution);
    GammaOld = Gamma;
    }
        else if (Gamma>Window){
    Final = Gamma-Window;
    stepsPerRevolution = Final*0.7; //0.7 คือค่าองศาต่อrev
    myStepper.step(stepsPerRevolution);
    GammaOld = Gamma;
        }
        k=1;
      }
      else{
      Final = 90;
      stepsPerRevolution = Final*0.7; //0.7 คือค่าองศาต่อrev
      myStepper.step(stepsPerRevolution);
      GammaOld=Window;
      k=1;
    }
    }
   //เคสทิศ N
    if (Window == 180){
    WindowMin=90;
    WindowMax=270;
      if (Gamma<WindowMax && Gamma>WindowMin){
        if (Gamma<Window){
    Final = Window-Gamma;
    stepsPerRevolution = Final*0.7; //0.7 คือค่าองศาต่อrev
    myStepper.step(stepsPerRevolution);
    GammaOld = Gamma;
    }
      }
        else if (Gamma>Window){
    Final = Gamma-Window;
    stepsPerRevolution = Final*0.7; //0.7 คือค่าองศาต่อrev
    myStepper.step(stepsPerRevolution);
    GammaOld = Gamma;
        }
        k=1;
      
    }
    //เคสทิศ E
    if (Window == 270){
    WindowMin=180;
    WindowMax=360;
      if (Gamma<WindowMax && Gamma>WindowMin){
        if (Gamma <Window){
    Final = Window-Gamma;
    stepsPerRevolution = Final*0.7; //0.7 คือค่าองศาต่อrev
    myStepper.step(stepsPerRevolution);
    GammaOld = Gamma;
    }
        else if (Gamma>Window){
    Final = Gamma-Window;
    stepsPerRevolution = Final*0.7; //0.7 คือค่าองศาต่อrev
    myStepper.step(stepsPerRevolution);
    GammaOld = Gamma;
        }
        k=1;
      }
          else{
      Final = 90;
      stepsPerRevolution = Final*0.7; //0.7 คือค่าองศาต่อrev
      myStepper.step(stepsPerRevolution);
      GammaOld=Window;
      k=1;
            }
          }
        }
      }
   else if (k==1){
    if (Gamma<WindowMax && Gamma>WindowMin){
      if (Gamma<GammaOld){
    Final = GammaOld-Gamma;
    stepsPerRevolution = Final*0.7; //0.7 คือค่าองศาต่อrev
    myStepper.step(stepsPerRevolution);
    GammaOld = Gamma;
   }
      else if (Gamma>GammaOld){
    Final = Gamma-GammaOld;
    stepsPerRevolution = Final*0.7; //0.7 คือค่าองศาต่อrev
    myStepper.step(stepsPerRevolution);
    GammaOld = Gamma;
   }
    }
    l=1;
    time_old=millis();
   
}
}
//===================================================================================================================================//
void SDCARD(){
  String dataString = "";
      dataString += "Date";
      dataString += String(DAY);
      dataString += "/";
      dataString += String(MONTH);
      dataString += "/";
      dataString += String(YEAR);
      dataString += "\t";
      dataString += "Time";
      dataString += String(HOUR);
      dataString += ":";
      dataString += String(MINUTE);
      dataString += "\t";
      dataString += "PV Angle";
      dataString += String(RotateAngle);
      dataString += "\t"; 
      dataString +="Voltage: ";
      dataString += String(PZEMVoltage, 2);
      dataString += " V ";
      dataString +="Current: ";
      dataString +=String(PZEMCurrent, 2);
      dataString +=" A ";
      dataString +="Power: ";
      dataString +=String(PZEMEnergy, 2);
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
        Serial.println("error VPVLog.txt");
        lcd.clear();
        lcd.print("error VPVLog.txt");
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
