#include<Wire.h>
#include <Servo.h>
const int MPU_addr=0x68;
double AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; 
uint32_t timer; 
double compAngleX, compAngleY; 
#define degconvert 57.2957786
long X, Y;
Servo servo;


void setup() {

  Wire.begin();
  #if ARDUINO >= 157
  Wire.setClock(400000UL); 
  #else
    TWBR = ((F_CPU / 400000UL) - 16) / 2; 
  #endif

  
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  
  Wire.write(0);     
  Wire.endTransmission(true);
  Serial.begin(115200);
  delay(100);

 

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  
  AcX=Wire.read()<<8|Wire.read();      
  AcY=Wire.read()<<8|Wire.read();  
  AcZ=Wire.read()<<8|Wire.read(); 
  Tmp=Wire.read()<<8|Wire.read(); 
  GyX=Wire.read()<<8|Wire.read();  
  GyY=Wire.read()<<8|Wire.read(); 
  GyZ=Wire.read()<<8|Wire.read();  

 
  double roll = atan2(AcY, AcZ)*degconvert;
  double pitch = atan2(-AcX, AcZ)*degconvert;

  double gyroXangle = roll;
  double gyroYangle = pitch;
  double compAngleX = roll;
  double compAngleY = pitch;

  
  timer = micros();
  servo.attach(5);
}

void loop() {
  
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true); 
  AcX=Wire.read()<<8|Wire.read();  
  AcY=Wire.read()<<8|Wire.read();  
  AcZ=Wire.read()<<8|Wire.read();  
  Tmp=Wire.read()<<8|Wire.read();  
  GyX=Wire.read()<<8|Wire.read();  
  GyY=Wire.read()<<8|Wire.read(); 
  GyZ=Wire.read()<<8|Wire.read();  
  
  double dt = (double)(micros() - timer) / 1000000; 
  timer = micros(); 
  double roll = atan2(AcY, AcZ)*degconvert;
  double pitch = atan2(-AcX, AcZ)*degconvert;

  
  double gyroXrate = GyX/131.0;
  double gyroYrate = GyY/131.0;

  compAngleX = 0.99 * (compAngleX + gyroXrate * dt) + 0.01 * roll; 
  compAngleY = 0.99 * (compAngleY + gyroYrate * dt) + 0.01 * pitch; 
  double compAngleX_val = compAngleX - 1.06;
  double compAngleY_val = compAngleY - 10.34;
  
  Serial.print(compAngleX_val);Serial.print("\t");
  Serial.print(compAngleY_val);Serial.print("\n");
  compAngleY_val = map(compAngleY_val, -180, 180, 0, g180);
  servo.write(compAngleY_val);

}



