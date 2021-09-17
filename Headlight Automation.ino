#include<Wire.h>
#include<DHT.h>

const int ldrPin = 7;  //LDR Pin

//Accel init
long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;
long gyroXCalli = 0, gyroYCalli = 0, gyroZCalli = 0;
long gyroXPresent = 0, gyroYPresent = 0, gyroZPresent = 0;
long gyroXPast = 0, gyroYPast = 0, gyroZPast = 0;
float rotX, rotY, rotZ;
float AngleX = 0, AngleY = 0, AngleZ = 0;
long timePast = 0;
long timePresent = 0;

//Constants DHT
#define DHTPIN 13     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino

//Variables DHT
int chk;
float hum;  //Stores humidity value
float tem; //Stores temperature value
float dp;



void setup() {
  Serial.begin(9600);
  pinMode(ldrPin, INPUT);
  pinMode(10,OUTPUT);
  pinMode(9,OUTPUT);
  digitalWrite(9,LOW);
  digitalWrite(10,LOW);
  Wire.begin();
  setUpMPU();
  delay(250);
  callibrateGyroValues();
  timePresent = millis();
  dht.begin();
  Serial.print("SetupComplete");
}

void loop() {

int ldrStatus = digitalRead(ldrPin);
  readAndProcessAccelData();
  readAndProcessGyroData();
  delay(2000);
    hum = dht.readHumidity();       //Read data and store it to variables humidity
    tem= dht.readTemperature();     //Read data and store it to variables temperature
    dp=calcDewpoint(hum,tem);
if (ldrStatus==0){
  if(30>rotZ>0){
    digitalWrite(9,LOW);
    digitalWrite(10,HIGH);
  }
  else if(rotZ>30 || dp>tem){
    digitalWrite(9,LOW);
    digitalWrite(10,LOW);
  }
}
else if (ldrStatus==1){
  if (dp>tem){
    digitalWrite(9,LOW);
    digitalWrite(10,LOW);
  }
}
  else{
    digitalWrite(9,HIGH);
    digitalWrite(10,HIGH);
  }
}

float calcDewpoint(float humi, float temp) 
{
  float k;
  k = log(humi/100) + (17.62 * temp) / (243.12 + temp);
  return 243.12 * k / (17.62 - k);
  delay(1000);
}

void setUpMPU() {
  // power management
  Wire.beginTransmission(0b1101000);          // Start the communication by using address of MPU
  Wire.write(0x6B);                           // Access the power management register
  Wire.write(0b00000000);                     // Set sleep = 0
  Wire.endTransmission();                     // End the communication

  // configure gyro
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1B);                           // Access the gyro configuration register
  Wire.write(0b00000000);
  Wire.endTransmission();

  // configure accelerometer
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1C);                           // Access the accelerometer configuration register
  Wire.write(0b00000000);
  Wire.endTransmission();  
}

void callibrateGyroValues() {
    for (int i=0; i<5000; i++) {
      getGyroValues();
      gyroXCalli = gyroXCalli + gyroXPresent;
      gyroYCalli = gyroYCalli + gyroYPresent;
      gyroZCalli = gyroZCalli + gyroZPresent;
    }
    gyroXCalli = gyroXCalli/5000;
    gyroYCalli = gyroYCalli/5000;
    gyroZCalli = gyroZCalli/5000;
}

void readAndProcessAccelData() {
  Wire.beginTransmission(0b1101000); 
  Wire.write(0x3B); 
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); 
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read(); 
  accelY = Wire.read()<<8|Wire.read(); 
  accelZ = Wire.read()<<8|Wire.read(); 
  processAccelData();
}

void processAccelData() {
  gForceX = accelX/16384.0;
  gForceY = accelY/16384.0; 
  gForceZ = accelZ/16384.0;
}

void readAndProcessGyroData() {
  gyroXPast = gyroXPresent;                                   // Assign Present gyro reaging to past gyro reading
  gyroYPast = gyroYPresent;                                   // Assign Present gyro reaging to past gyro reading
  gyroZPast = gyroZPresent;                                   // Assign Present gyro reaging to past gyro reading
  timePast = timePresent;                                     // Assign Present time to past time
  timePresent = millis();                                     // get the current time in milli seconds, it is the present time
  
  getGyroValues();                                            // get gyro readings
  getAngularVelocity();                                       // get angular velocity
  calculateAngle();                                           // calculate the angle  
}

void getGyroValues() {
  Wire.beginTransmission(0b1101000);                          // Start the communication by using address of MPU 
  Wire.write(0x43);                                           // Access the starting register of gyro readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6);                              // Request for 6 bytes from gyro registers (43 - 48)
  while(Wire.available() < 6);                                // Wait untill all 6 bytes are available
  gyroXPresent = Wire.read()<<8|Wire.read();                  // Store first two bytes into gyroXPresent
  gyroYPresent = Wire.read()<<8|Wire.read();                  // Store next two bytes into gyroYPresent
  gyroZPresent = Wire.read()<<8|Wire.read();                  //Store last two bytes into gyroZPresent
}

void getAngularVelocity() {
  rotX = gyroXPresent / 131.0;                                
  rotY = gyroYPresent / 131.0; 
  rotZ = (gyroZPresent / 131.0)*3.6;
}

void calculateAngle() {  
  AngleX = AngleX + ((timePresent - timePast)*(gyroXPresent + gyroXPast - 2*gyroXCalli)) * 0.00000382;
  AngleY = AngleY + ((timePresent - timePast)*(gyroYPresent + gyroYPast - 2*gyroYCalli)) * 0.00000382;
  AngleZ = AngleZ + ((timePresent - timePast)*(gyroZPresent + gyroZPast - 2*gyroZCalli)) * 0.00000382;
}