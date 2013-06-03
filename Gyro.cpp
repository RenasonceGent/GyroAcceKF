// Code was modified from original by Homer Creutz 
// obtained from: 
// http://ifroboclub.com/index.php/arduino/6-arduino-gyroscope?showall=1&limitstart=

#include <Gyro.h>
#include <Wire.h>
#include <math.h>
#include <MathUtil.h>

// Public Methods //////////////////////////////////////////////////////////////

bool Gyro::init()
{

	//Save the scale value that will be used when the program starts
	scaleFactor = 0.0f;

	//Digital zero-rate values (calculated from calibration instead of data sheet)
	DVoff_X = 0.0f;
	DVoff_Y = 0.0f;
	DVoff_Z = 0.0f;

	L3G4200D_Address = 105; //I2C address of the L3G4200D

	//actual sensor readings
	g.dX = 0.0f;
	g.dY = 0.0f;
	g.dZ= 0.0f;
	g.t = 0.0f;

	temperature = 0.0f;


	//last time in ms
	lastTimeMS = 0;
	currTimeMS = 0;

	spamTimer = -1.0f;
	spamInterval = 0.125f; //seconds

	timeElapsed = 0.0f;

	isCalibrating = true;
	calibrationTimer = 3.0f;
	GTimer = 0;
	GyroTimer = 0;
	return true;
}


void Gyro::getGyroValues()
{

  if(millis() >= GyroTimer ){
	  GyroTimer += 40;
	  //Convert temp reading into a 0 to 1 float value
	  float tempT = 1.0f - ((float)g.t+128.0f)/255.0f;
	  
	  //Interpolate from min to max temperature based on tempT
	  //temperature = Lerp(temperature_Min,temperature_Max,tempT);
	  temperature = tempT;
	  //Get DPS values from registers
	  byte xMSB = readRegister(L3G4200D_Address, 0x29);
	  byte xLSB = readRegister(L3G4200D_Address, 0x28);

	  byte yMSB = readRegister(L3G4200D_Address, 0x2B);
	  byte yLSB = readRegister(L3G4200D_Address, 0x2A);

	  byte zMSB = readRegister(L3G4200D_Address, 0x2D);
	  byte zLSB = readRegister(L3G4200D_Address, 0x2C);

	  g.dX = scaleFactor * ((xMSB << 8) | xLSB);
	  g.dY = scaleFactor * ((yMSB << 8) | yLSB);
	  g.dZ = scaleFactor * ((zMSB << 8) | zLSB);
	  currTimeMS = millis();
	  timeElapsed = (currTimeMS-lastTimeMS)/1000.0f;
	  lastTimeMS = currTimeMS;
	  g.t = readRegister(L3G4200D_Address, 0x26);  
	  if(isCalibrating == false){
	  
	  	g.dX -= DVoff_X;
	  	g.dY -= DVoff_Y;
	  	g.dZ -= DVoff_Z;

	  }
//    if(isCalibrating == false)
// 	  {
// 		  //If the DPS values are less than or equal to the Digital zero-rate levels from
// 		  //the datasheet, set the values to 0
// 		  if(abs(g.dX) <= DVoff_X)
// 		  {
// 			g.dX = 0.0f;
// 		  }
// 	      
// 		  if(abs(g.dY) <= DVoff_Y)
// 		  {
// 			g.dY = 0.0f;
// 		  }
// 	      
// 		  if(abs(g.dZ) <= DVoff_Z)
// 		  {
// 			g.dZ = 0.0f;
// 		  }
// 
// 
// 	  }  
	 
  }
}
void Gyro::Calibration(){

float valX = 0.0;
float valY = 0.0;
float valZ = 0.0;

int cnt = 0;    
while (cnt < 100)
  {
	getGyroValues();
    valX += g.dX;
    valY += g.dY;
    valZ += g.dZ;
    
//     Serial.print(g.dX);
//     Serial.print(", ");
//     Serial.print(g.dY);
//     Serial.print(",");
//     Serial.print(g.dZ);
//     Serial.println(";");
    
    delay(50); 
    cnt++;
    }
    
DVoff_X = valX / 100.0;
DVoff_Y = valY / 100.0;
DVoff_Z = valZ / 100.0;

// Serial.print(DVoff_X);
// Serial.print(", ");
// Serial.print(DVoff_Y);
// Serial.print(",");
// Serial.print(DVoff_Z);
// Serial.println(";");
isCalibrating = false;

// if(isCalibrating == true)
//   {
// 	getGyroValues();
//     const float absValX = abs(g.dX);
//     const float absValY = abs(g.dY);
//     const float absValZ = abs(g.dZ);
//     
//     if(absValX > DVoff_X)
//     {
//       DVoff_X = absValX;
//     }
//     
//     if(absValY > DVoff_Y)
//     {
//       DVoff_Y = absValY;
//     }
//     
//     if(absValZ > DVoff_Z)
//     {
//       DVoff_Z = absValZ;
//     }
//     
//     calibrationTimer -= timeElapsed;
//     
//     if(calibrationTimer < 0.0f)
//     { 
//       isCalibrating = false;
//       
//       //Some fudging to account for the slight
//       //amount of movement that slips through
//       DVoff_X *= DVOffScale;
//       DVoff_Y *= DVOffScale;
//       DVoff_Z *= DVOffScale;
//     }
    
    return;
//}
}

void Gyro::setupL3G4200D(){
 
  //int scale = 250;

  //From  Jim Lindblom of Sparkfun's code

  // Enable x, y, z and turn off power down:
  writeRegister(L3G4200D_Address, CTRL_REG1, 0b00001111);

  // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
  writeRegister(L3G4200D_Address, CTRL_REG2, 0b00000000);

  // Configure CTRL_REG3 to generate data ready interrupt on INT2
  // No interrupts used on INT1, if you'd like to configure INT1
  // or INT2 otherwise, consult the datasheet:
  writeRegister(L3G4200D_Address, CTRL_REG3, 0b00001000);


  writeRegister(L3G4200D_Address, CTRL_REG4, 0b00000000);
  scaleFactor = 8.75f/1000.0f;
  //DVoff = DVoff_250;
 
  // CTRL_REG5 controls high-pass filtering of outputs, use it
  // if you'd like:
  writeRegister(L3G4200D_Address, CTRL_REG5, 0b00000000);
  GTimer = millis() +1500; //wait for the sensor to be ready
  delay(1500);
  while(isCalibrating){
	Calibration();
	if(isCalibrating == false) return;
  }
}

void Gyro::writeRegister(int deviceAddress, byte address, byte val) {
    Wire.beginTransmission(deviceAddress); // start transmission to device 
    Wire.write(address);       // send register address
    Wire.write(val);         // send value to write
    Wire.endTransmission();     // end transmission
}

int Gyro::readRegister(int deviceAddress, byte address){

    int v;
    Wire.beginTransmission(deviceAddress);
    Wire.write(address); // register to read
    Wire.endTransmission();

    Wire.requestFrom(deviceAddress, 1); // read a byte

    while(!Wire.available()) {
        // waiting
    }

    v = Wire.read();
    return v;
}
