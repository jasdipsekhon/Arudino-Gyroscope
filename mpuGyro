#include <Wire.h>
#include <Servo.h>

Servo servoObj;          

long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

long angleX, angleY, angleZ;
float gyroX, gyroY, gyroZ;



void setup() {
  servoObj.attach (9); // Connect servo to this Digital PWM pin on Arduino
  Serial.begin(9600);
  Wire.begin();
  setupMPU();
}



void loop()
{
  recordAccelRegisters();
  recordGyroRegisters();
  Serial.print(accelY);
  // map(value, fromLow, fromHigh, toLow, toHigh); map(accel, accelLower, accelUpper, angleLower, angleUpper)
  accelY = map (accelY, -16000, 16000, 0, 180) ;
  servoObj.write(accelY);
  printData();
  delay(10);
}



void setupMPU()
{
  Wire.beginTransmission(0b1101000); // begin communication w/ I2C address of the MPU-6050 (0x68H, SIGNAL_PATH_RESET); no need to reset to 0
  Wire.write(0x6B); // access PWR_MGMT_1 register
  Wire.write(0b00000000); // Set register 0x6B to 0 to turn off Sleep mode; MPU begins in Sleep Mode
  Wire.endTransmission();
  
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1B); // access 0x1B to access Gyroscope register
  Wire.write(0x00000000); // set the gyro to +/-250deg full scale range
  Wire.endTransmission(); 
  
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1C); // access 0x1C to access Accelerometer register full scale range
  Wire.write(0b00000000); //Set the accel to +/-2g
  Wire.endTransmission();
}



// read from Accelerometer
void recordAccelRegisters() 
{
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); // Starting register for Accel Readings
  Wire.endTransmission();
  
  Wire.requestFrom(0b1101000,6); // Request all 6 Accelerometer registers (3B-40)
  
  while (Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read(); // shift left 8 bits; 1st 2 bytes stored in accelX; // 16 bit
  accelY = Wire.read()<<8|Wire.read(); // middle 2 bytes stored in accelY
  accelZ = Wire.read()<<8|Wire.read(); // last 2 bytes stored in accelZ
  processAccelData();
}



// Convert Accelerometer to gForce
void processAccelData()
{
  gForceX = accelX / 16384.0; // convert reading from accelerometer to gForce (Accel full-scale range); accel / least significant bit per g (9.81m/s^2)
  gForceY = accelY / 16384.0; 
  gForceZ = accelZ / 16384.0;
}



// read from Gyroscope
void recordGyroRegisters()
{
  Wire.beginTransmission(0b1101000);
  Wire.write(0x43); // // Starting register for Gyroscope Readings
  Wire.endTransmission();

  Wire.requestFrom(0b1101000,6); // Request all 6 Gyroscope registers (43-48)

  while (Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read(); // shift left 8 bits; 1st 2 bytes stored in gyroX
  gyroY = Wire.read()<<8|Wire.read(); // Store middle two bytes into gyroY
  gyroZ = Wire.read()<<8|Wire.read(); // Store last two bytes into gyroZ
  processGyroData();
}



// Convert Gyroscope to angle
void processGyroData()
{
  angleX = gyroX / 131.0; // convert reading from gyroscope to rotational angle in degrees; gyro(degrees per sec) / least significant bit per degree per second
  angleY = gyroY / 131.0;
  angleZ = gyroZ / 131.0; 
}


void printData()
{
  Serial.print(" Y=");
  Serial.print(accelY);
  Serial.print("\n");
}















//i2c bus 
// MPU6050
//Kalman Filter 


/*
void angleCalc()
{
  finalAngle = finalAngle + (((angleY - finalAngle) * 0.8 + gyroX) * 0.01);
}
*/




/*
void setup ( )
{ 
servoObj.attach ( servo_pin );
servoObj.write(0);// init 0 deg in the beginning 

Wire.begin ( );

Serial.begin  (9600); // initialize serial port to 9600 bps

Serial.println  ( "Initializing the sensor..." ); 
//replace with setupmpu();
//sensor.initialize ( ); 

Serial.println (sensor.testConnection ( ) ? "Successfully Connected" : "Connection failed"); 

delay (1000); 

Serial.println ( "Taking Values from the sensor..." );

delay (1000);

}


void loop ( ) 
{ 
  recordGyroRegister();
  recordAccelRegister();
  angleCalc();
  
  Serial.println (angleY);
  servoObj.write (angleY);
  

sensor.getMotion6 (&accelX, &accelY, &accelZ, &gyroX, &gyroY, &gyroZ);

accelX = map (accelX, -17000, 17000, 0, 180) ;

Serial.println (accelX);

servoObj.write (accelX); 

delay (200);
*/
