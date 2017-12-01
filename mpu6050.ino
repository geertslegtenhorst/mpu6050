#include<Wire.h>                        //de Wire.h library bijvoegen
#include <Servo.h>                      //de Servo.h library bijvoegen
const int MPU_addr=0x68;                // het c2l adres van de mpu6050 sensor(de gyro/acc. sensor)
double AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;     //variabelen voor de mpu6050 sensor
uint32_t timer;                         //een timer met 32 bits, zodat de timer altijd genoeg data kan opslaan
double compAngleX, compAngleY;          //complementary filters voor hoek X en hoek Y
#define degconvert 57.2957786           //radialen naar graden
long X, Y;                              //variablen voor hoek X en hoek Y
Servo servo_links, servo_rechts;        //servo_links en rechts
int chA = 9;                            //kanaal A, voor de manuel / autopilot mode switch
int chB = 11;                           //kanaal B voor hoogte
int chB = 12;                           // kanaal C voor de richting
int chA_data, chB_data, chC_data;       //variablen die de waarden van de kanalen opslaan


void setup() {
  
  pinMode(chA, INPUT);                  //chA moet een input zijn om waarden te kunnen lezen
  pinMode(chB, INPUT);                  //chB moet een input zijn om waarden te kunnen lezen
  pinMode(chC, INPUT);                  //chC moet een input zijn om waarden te kunnen lezen
  Wire.beginTransmission(MPU_addr);     //begin de communicatie met de mpu6050 sensor
  Wire.write(0x6B);                     //ga naar register 0x6B(powermanagement)
  Wire.write(0);                        //schrijf 0 bits zodat de sensor uit sleeping mode gaat
  Wire.endTransmission(true);           //stop de communicatie met de mpu6050 sensor
  Serial.begin(115200);                 //start een serële comicatie met de mpu6050 sensor
  calibration();                        //voer de callibratie uit
  timer = micros();                     //start de timer in microseconden
 
}

void loop() {
  
  chA_data = pulseIn(chA, HIGH);                  //lees ChA en sla de waarden op in chA_data
  chB_data = pulseIn(chB, HIGH);                  //lees ChB en sla de waarden op in chB_data
  chC_data = pulseIn(chC, HIGH);                  //lees ChC en sla de waarden op in chC_data
  if(chA_data >= 1500){                           //als de waarden van chA groter zijn dan 1500(switch activated), dan ;
    chB_data = map(chB_data, 1200, 1800, 0, 180); //zet de waarden van chB om van 1200-1800 naar 0-180
    chC_data = map(chC_data, 1200, 1800, 180, 0); //zet de waarden van chC om van 1200-1800 naar 180-0 want deze servo is omgekeerd 
    chB_data = 90 - chB_data;                     //chB moet beginngen met 90 graden door de positie van de servo. 
    chC_data = 90 - chC_data;                     //chC moet ook beginnen met 90 graden
    servo_links.write(chB_data);                  //schrijf de waarden wan ChB en ChC naar de juiste servo's
    servo_rechts.write(chC_data);
}
  if(chA_data <= 1500){                           //als chA kleiner is dan 1500(autopilot activated) dan;
   read_data();                                   //voer de functier read_data uit(zie onder voor de functie met uitleg)
   compAngleX = 90 - compAngleX;                  //compAngleX moet beginnen met 90 graden
   servo_rechts.write(compAngleX);                //schrijf de waarden naar de servo's
   servo_links.write(compAngleX);
}

void calibration(){                                // functie voor het callibreren
  for(int i = 0; i < 2000; i++){                   //door deze functie herhaalt de arduino het onderstaande deel 2000 keer
  Wire.beginTransmission(MPU_addr);                //begin de communicatie met de mpu6050 sensor
  Wire.write(0x3B);                                //ga naar het register van de acceleratometer om daar de metingen te beginnen
  Wire.endTransmission(false);                     //stop de communicatie weer
  Wire.requestFrom(MPU_addr,14,true);              //vraag aan de sensor de waarden van 14 registers vanaf het accel. regisrer
  AcX=Wire.read()<<8|Wire.read();                  //sla de waarden op in acx en zet het om van 2 8 bits waarden naar 1 16 bit waarde
  AcY=Wire.read()<<8|Wire.read();                  //sla de waarden op in acY en zet het om van 2 8 bits waarden naar 1 16 bit waarde
  AcZ=Wire.read()<<8|Wire.read();                  //sla de waarden op in acZ en zet het om van 2 8 bits waarden naar 1 16 bit waarde
  Tmp=Wire.read()<<8|Wire.read();                  //sla de waarden op in tmp em zet het om van 2 8 bits waarden naar 1 16 bit waarde
  GyX=Wire.read()<<8|Wire.read();                  //sla de waarden op in gYX en zet het om van 2 8 biys waarden naar 1 16 bit waarde
  GyY=Wire.read()<<8|Wire.read();                  //sla de waarden op in gYZ en zet het om van 2 8 bits waarden naar 1 16 bit waarde
  GyZ=Wire.read()<<8|Wire.read();                  //sla de waarden op in gyZ en zet het om van 2 8 bits waarden naar 1 16 bit waarde
  double dt = (double)(micros() - timer) / 1000000; // de tijd voor deze metingen = de tijd tot nu toe - de tijd voor de berekingen
                                                    // en maak er seconden van (/ 1000000)
  timer = micros();                                 //hersart de timer


  double roll = atan2(AcY, AcZ)*degconvert;         //functie om de vectoren van de acc. sensor op te tellen en er graden van te maken
  double pitch = atan2(-AcX, AcZ)*degconvert;   

 
  double gyroXrate = GyX/131.0;                     //de gyro sensor geeft een waarde van 131 graden/s
  double gyroYrate = GyY/131.0;

  compAngleX = 0.99 * (compAngleX + gyroXrate * dt) + 0.01 * roll;   /*compAngleX neemt een deel van elke sensor voor nauwkeurige 
  metingen. 0.99 van (de compAngleX(in het begin 0 om zo elke keer te zien hoeveel de vorige waarde was) + de waarde van de gyro 
  sensor * de tijd) + 0.01 van de acc. sensor */
  
  compAngleY = 0.99 * (compAngleY + gyroYrate * dt) + 0.01 * pitch;
  X += compAngleX;       //compAngleX wordt steeds bij X opgeteld
  Y += compAngleY;       //compAngleY wordt steeds bij Y opgeteld
  }
  X /= 2000;             //X heeft nu een hele grote waarde van 2000 metingen. Delen door 2000 voor het gemiddelde van 2000 metingen
  Y /= 2000;             //Y heeft nu een hele grote waarde van 2000 metingen. Delen door 2000 voor het gemiddelde van 2000 metingen
  
  }

void read_data(){                                        //functie om de waarden te lezen en uit te rekenen
  Wire.beginTransmission(MPU_addr);                      //begin de communicatie met de mpu6050 sensor
  Wire.write(0x3B);                                      //ga naar het register van de acceleratometer om daar de metingen te beginnen
  Wire.endTransmission(false);                           //stop de communicatie weer
  Wire.requestFrom(MPU_addr,14,true);                    //vraag aan de sensor de waarden van 14 registers vanaf het accel. regisrer
  AcX=Wire.read()<<8|Wire.read();                        //sla de waarden op in acx en zet het om van 2 8 bits waarden naar 1 16 bit waarde
  AcY=Wire.read()<<8|Wire.read();                        //sla de waarden op in acY en zet het om van 2 8 bits waarden naar 1 16 bit waarde
  AcZ=Wire.read()<<8|Wire.read();                        //sla de waarden op in acZ en zet het om van 2 8 bits waarden naar 1 16 bit waarde
  Tmp=Wire.read()<<8|Wire.read();                        //sla de waarden op in tmp em zet het om van 2 8 bits waarden naar 1 16 bit waarde
  GyX=Wire.read()<<8|Wire.read();                        //sla de waarden op in gYX en zet het om van 2 8 biys waarden naar 1 16 bit waarde
  GyY=Wire.read()<<8|Wire.read();                        //sla de waarden op in gYZ en zet het om van 2 8 bits waarden naar 1 16 bit waarde
  GyZ=Wire.read()<<8|Wire.read();                        //sla de waarden op in gyZ en zet het om van 2 8 bits waarden naar 1 16 bit waarde
  
  double dt = (double)(micros() - timer) / 1000000; 
  timer = micros(); 
  double roll = atan2(AcY, AcZ)*degconvert;
  double pitch = atan2(-AcX, AcZ)*degconvert;

  
  double gyroXrate = GyX/131.0;
  double gyroYrate = GyY/131.0;

  compAngleX = 0.99 * (compAngleX + gyroXrate * dt) + 0.01 * roll; 
  compAngleY = 0.99 * (compAngleY + gyroYrate * dt) + 0.01 * pitch; 
  double compAngleX_val = compAngleX - X;
  double compAngleY_val = compAngleY - Y;
  
  
