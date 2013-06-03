#include <Gyro.h>
#include <Wire.h>
#include <stdio.h>

const int xPin = 2;
const int yPin = 4;
int xTime, yTime, xCorrection, yCorrection, xAcc, yAcc;

Gyro G;

char strA[41];
char strX[10];
char strY[20];
char strZ[29];

int dX, dY, dZ, mX, mY, mZ;


void calibrate(){

  float xSum = 0;
  float ySum = 0;

  int numSamps = 100;

  for (int ii = 0; ii < numSamps; ii++){

    xTime = pulseIn(xPin, HIGH);
    yTime = pulseIn(yPin, HIGH);

    xSum += xTime;
    ySum += yTime;

    delay(100);

  }

  xCorrection = xSum / numSamps;
  yCorrection = ySum / numSamps;

}

void setup(){

  Serial.begin(9600);
  Wire.begin(); 
  G.init(); 
  G.setupL3G4200D(); 

  pinMode(xPin, INPUT);
  pinMode(yPin, INPUT);

  calibrate();

}


void loop(){

  delay(100);

  xTime = pulseIn(xPin, HIGH);
  yTime = pulseIn(yPin, HIGH);

  G.getGyroValues();

  // Formatting values read from accelerometer
  xAcc = xTime - xCorrection;
  yAcc = yTime - yCorrection; 
  
  sprintf(strA, "%i, %i", xAcc, yAcc);

  // Formatting values read from gyro (necessary for printing floats to serial)  
  dX = int(abs(G.g.dX));
  dY = int(abs(G.g.dY));
  dZ = int(abs(G.g.dZ)); 

  mX = 1000 * (abs(G.g.dX) - float(dX));
  mY = 1000 * (abs(G.g.dY) - float(dY));
  mZ = 1000 * (abs(G.g.dZ) - float(dZ));

  if (G.g.dX < 0){
    sprintf(strX, "-%d.%03i, ", dX, int(mX));
  }
  else{
    sprintf(strX, "%d.%03i, ", dX, int(mX));
  }
  if (G.g.dY < 0){
    sprintf(strY, "%s-%d.%03i, ", strX, dY, int(mY));
  }
  else{
    sprintf(strY, "%s%d.%03i, ", strX, dY, int(mY));
  }
  if (G.g.dZ < 0){
    sprintf(strZ, "%s-%d.%03i, ", strY, dZ, int(mZ));
  }
  else{ 
    sprintf(strZ, "%s%d.%03i, ", strY, dZ, int(mZ));
  }

  sprintf(strA, "%s%i, %i", strZ, xAcc, yAcc);
  
  Serial.println(strA);

}


