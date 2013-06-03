// Code was modified from original by Homer Creutz 
// obtained from: 
// http://ifroboclub.com/index.php/arduino/6-arduino-gyroscope?showall=1&limitstart=

#ifndef Gyro_h

#define Gyro_h

#include <Arduino.h> // for byte data type



// Defines ////////////////////////////////////////////////////////////////



// The Arduino two-wire interface uses a 7-bit number for the address,

// and sets the last bit correctly based on reads and writes



#define CTRL_REG1 0x20

#define CTRL_REG2 0x21

#define CTRL_REG3 0x22

#define CTRL_REG4 0x23

#define CTRL_REG5 0x24



//Sensitivity from data sheet in mdps/digit (millidegrees per second) with DPS conversion



//const float SoDR_250 = 8.75f/1000.0f;

// const float SoDR_500 = 17.50f/1000.0f;
// 
// const float SoDR_2000 = 70.0f/1000.0f;



//Digital zero-rate level from data sheet in dps (degrees per second)

/*const float DVoff_250 = 10.0f;

const float DVoff_500 = 15.0f;

const float DVoff_2000 = 75.0f;*/

//Temporarily made up numbers

//TODO: do some measurements to figure these numbers out

const float temperature_Min = 35.0f;

const float temperature_Max = 100.0f;



//const float DVOffScale = 1.2f;

const float DVOffScale = 1.9f;

class Gyro

{

  public:

    typedef struct vector
    {

      float dX, dY, dZ, t;

    } vector;



    vector g; // gyro angular velocity readings





	//Digital zero-rate level from data sheet in dps (degrees per second)

	/*const float DVoff_250 = 10.0f;

	const float DVoff_500 = 15.0f;

	const float DVoff_2000 = 75.0f;*/



	//Save the scale value that will be used when the program starts

	float scaleFactor;



	//Digital zero-rate values (calculated from calibration instead of data sheet)

	float DVoff_X;

	float DVoff_Y;

	float DVoff_Z;



	float DVOffScale;



	int L3G4200D_Address; //I2C address of the L3G4200D



	//actual sensor readings



	float temperature;


	//last time in ms

	float lastTimeMS;

	float currTimeMS;



	float spamTimer;

	float spamInterval; //seconds



	float timeElapsed;



	bool isCalibrating;

	float calibrationTimer;

	unsigned long GTimer;

	unsigned long GyroTimer;

	bool init();

	void getGyroValues();

	void Calibration();

	float WrapAngle(float angle);

	void setupL3G4200D();





	void writeRegister(int deviceAddress, byte address, byte val);

	int readRegister(int deviceAddress, byte address);

	  private:



};



#endif


