// MPU-6050 Short Example Sketch By Arduino User JohnChi (only raw values)
// modification by Said Karagoz nog te doen: auto-offset, significantie range, negatieve g
#include<Wire.h>
const int MPU_addr=0x68;  // I2C adres van de MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

float vX ;  float gXYZ; // versnelling X-as , gecombineerde zwaartekracht
float vY ;  float gYZ_interval;
float vZ ;  float gYZ_delta;
float f =16384/9.81653; // berekening voor m/s^2 voor op tot 2g nauwkeurigheid

float v_delta;
float v_interval;
float v[3] = {0,0};
float offset = 9.81-0.6;
byte i;

const int max_metingen = 50; // metingen per berekening 
int tijd_delay = 2; 
float metingen[max_metingen];      
int LeesIndex = 0;              
float tot_som = 0;
int berekening_periode = tijd_delay*max_metingen; // periodetijd van één berekening

void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
  Serial.println("Snelheid meten v1.3 met Smoothing en interval");
  Serial.print(" | metingenen per seconde = "); Serial.print(1000 / berekening_periode*max_metingen);  
  Serial.print(" | berekeningen per seconde = "); Serial.print(1000 / berekening_periode);  
  Serial.print(" | metingen per berekening = "); Serial.print(max_metingen);  
  Serial.print(" | tijd delay = "); Serial.println(tijd_delay);  
  Metingen_schoonmaken();
  }

void Metingen_schoonmaken(){
      for (int dezeMeting = 0; dezeMeting < max_metingen; dezeMeting++) {
    metingen[dezeMeting] = 0; }
} 

void Smooth_gYZ_offset(){
 
  metingen[LeesIndex] = gYZ_delta;
  tot_som = tot_som + metingen[LeesIndex]; // deSom heeft x waardes met ieder een tijdverschil van y ms
  LeesIndex = LeesIndex + 1;

}

void loop(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

// Eenheden (bits) omzetten naar m/s^2
vX = (AcX/f);
vY = (AcY/f);
vZ = (AcZ/f);

// zwaartekracht berekening (tussen Y- en Z as)
gYZ_delta = sqrt( sq(vY)+sq(vZ) ) ; 
//gXYZ = sqrt( sq(gYZ)+sq(vX) ) ; 
     
// Smoothing  van gYZ_delta naar gYZ_interval
  metingen[LeesIndex] = gYZ_delta;
  tot_som = tot_som + metingen[LeesIndex]; // deSom heeft x waardes met ieder een tijdverschil van y ms
  LeesIndex = LeesIndex + 1;

  if (LeesIndex >= max_metingen) {   
    gYZ_interval= tot_som / max_metingen;    
    LeesIndex = 0;  
    tot_som = 0;  
    SnelheidPrinten();
    Metingen_schoonmaken();
}

delay(tijd_delay);
}

void SnelheidPrinten(){   
  //  Snelheid berekening in km/u in de YZ-as (interval)
  if(i == 2){ v[0] = v[2]; i=0; }   i++; 
  v_interval = (gYZ_interval-9.8)*3.6;   // gYZ is gelijk aan 2x de cosinus van beide assen 
  v[i] = v[i-1] + v_interval;
  v[i] = constrain(v[i],0,120);
  
// Serial.print(" | AcX = "); Serial.print(vX,1);  
  Serial.print(" | AcY = "); Serial.print(vY,1);  
  Serial.print(" | AcZ = "); Serial.print(vZ,1);  
  Serial.print(" | g_Dt = "); Serial.print(gYZ_delta,2); // delta g vóór het moment van gYZ_interval 
  Serial.print(" | g = "); Serial.print(gYZ_interval,2);  // g met een interval
  Serial.print(" | v = "); Serial.print(v[i],1); // snelheid met interval
  Serial.print(" | a_HOR = "); Serial.println(gYZ_interval-offset,1); // Horizontale acceleratie    
// Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53); 
}

