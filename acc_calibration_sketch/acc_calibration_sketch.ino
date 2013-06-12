/*
* Accelerometer calibration sketch
* Author: Bill Earl
* Modified by: Mikhail Mansion, June 12, 2013
* Ref: http://learn.adafruit.com/adafruit-analog-accelerometer-breakouts/programming
*
*/

/* ACCELEROMETER PINS
  ----------------------------------------------------------*/
const int powerpin  = 14; // analog input pin 5 -- voltage
const int groundpin = 16; // analog input pin 4 -- ground
const int xPin = A5;      // x-axis of the accelerometer
const int yPin = A4;      // y-axis
const int zPin = A3;      // z-axis (only on 3-axis models)

const int buttonPin = 10;

// Raw Ranges:
int xRawMin, xRawMax, xRawCen, yRawMin, yRawMax, yRawCen, zRawMin, zRawMax, zRawCen;


boolean startOver = false;

// Take multiple samples to reduce noise
const int sampleSize = 10;

void setup() {
  //analogReference(EXTERNAL); if using aref
  Serial.begin(115200);

  //calibration pin
  digitalWrite(buttonPin, HIGH);
  pinMode(buttonPin, INPUT);

   //setup pins for accelerometer
  pinMode(groundpin, OUTPUT);
  pinMode(powerpin, OUTPUT);
  digitalWrite(groundpin, LOW); 
  digitalWrite(powerpin, HIGH);
}

void loop() {

  if (digitalRead(buttonPin) == LOW) { //ENTER CALIBRATION MODE

    startOver = false;
    Serial.println("");
    Serial.print(F("ENTERING CALIBRATION MODE"));
    delay(1000);
    Serial.print(F(" ."));
    delay(200);
    Serial.print(F(" ."));
    delay(200);
    Serial.print(F(" ."));
    delay(200);
    Serial.println("");

    /* CALIBRATE THE X AXIS
      ----------------------------------------------------------*/

    if(!startOver) {
      Serial.print(F("PLACE UNIT AT CENTER X, ENTER 'c' TO CONTINUE, ENTER 'r' TO RESET"));

      while(Serial.available() < 1) { /*hold until user sends a byte or starts over*/ }
      if(Serial.read() == 'r' && !startOver) startOver = true;
      xRawCen = ReadAxis(xPin); 
    }
    
    if(!startOver) {
      Serial.print(F("PLACE UNIT AT MIN X, ENTER 'c' TO CONTINUE, ENTER 'r' TO RESET"));
      while(Serial.available() < 1 && !startOver) { /*hold until user sends a byte*/ }
      if(Serial.read() == 'r') startOver = true;
      xRawMin = ReadAxis(xPin);
    }
    if(!startOver) {
      Serial.print(F("PLACE UNIT AT MAX X, ENTER 'c' TO CONTINUE, ENTER 'r' TO RESET"));
      while(Serial.available() < 1 && !startOver) { /*hold until user sends a byte*/ }
      if(Serial.read() == 'r') startOver = true;
      xRawMax = ReadAxis(xPin);
    }

    /* CALIBRATE THE X AXIS
      ----------------------------------------------------------*/

    if(!startOver) {
      Serial.print(F("PLACE UNIT AT CENTER Y, ENTER 'c' TO CONTINUE, ENTER 'r' TO RESET"));
      while(Serial.available() < 1 && !startOver) { /*hold until user sends a byte or starts over*/ }
      if(Serial.read() == 'r') startOver = true;
      yRawCen = ReadAxis(yPin); 
    }
    
    if(!startOver) {
      Serial.print(F("PLACE UNIT AT MIN Y, ENTER 'c' TO CONTINUE, ENTER 'r' TO RESET"));
      while(Serial.available() < 1 && !startOver) { /*hold until user sends a byte*/ }
      if(Serial.read() == 'r') startOver = true;
      yRawMin = ReadAxis(yPin);
    }
    if(!startOver) {
      Serial.print(F("PLACE UNIT AT MAX Y, ENTER 'c' TO CONTINUE, ENTER 'r' TO RESET"));
      while(Serial.available() < 1 && !startOver) { /*hold until user sends a byte*/ }
      if(Serial.read() == 'r') startOver = true;
      yRawMax = ReadAxis(yPin);
    }

    /* CALIBRATE THE Z AXIS
      ----------------------------------------------------------*/

    if(!startOver) {
      Serial.print(F("PLACE UNIT AT CENTER Z, ENTER 'c' TO CONTINUE, ENTER 'r' TO RESET"));
      while(Serial.available() < 1 && !startOver) { /*hold until user sends a byte or starts over*/ }
      if(Serial.read() == 'r') startOver = true;
      zRawCen = ReadAxis(zPin); 
    }
    
    if(!startOver) {
      Serial.print(F("PLACE UNIT AT MIN Z, ENTER 'c' TO CONTINUE, ENTER 'r' TO RESET"));
      while(Serial.available() < 1 && !startOver) { /*hold until user sends a byte*/ }
      if(Serial.read() == 'r') startOver = true;
      zRawMin = ReadAxis(zPin);
    }
    if(!startOver) {
      Serial.print(F("PLACE UNIT AT MAX Z, ENTER 'c' TO CONTINUE, ENTER 'r' TO RESET"));
      while(Serial.available() < 1 && !startOver) { /*hold until user sends a byte*/ }
      if(Serial.read() == 'r') startOver = true;
      zRawMax = ReadAxis(zPin);
    }

     /* PRINT RESULTS
      ----------------------------------------------------------*/
  
    Serial.print("X CEN: ");
    Serial.print(xRawCen);
    Serial.print("X MIN: ");
    Serial.print(xRawMin);
    Serial.print("X MAX: ");
    Serial.print(xRawMax);
    Serial.println("");

    Serial.print("Y CEN: ");
    Serial.print(yRawCen);
    Serial.print("Y MIN: ");
    Serial.print(yRawMin);
    Serial.print("Y MAX: ");
    Serial.print(yRawMax);
    Serial.println("");

    Serial.print("Z CEN: ");
    Serial.print(zRawCen);
    Serial.print("Z MIN: ");
    Serial.print(zRawMin);
    Serial.print("Z MAX: ");
    Serial.print(zRawMax);
    Serial.println("");

  } else {

    Serial.print("X => ");
    Serial.print(analogRead(xPin));
    Serial.print("\t Y => ");
    Serial.print(analogRead(yPin));
    Serial.print("\t Z => ");
    Serial.println(analogRead(zPin));

  }

  delay(100);
}

//
// Read "sampleSize" samples and report the average
//
int ReadAxis(int axisPin)
{
  long reading = 0;
  analogRead(axisPin);
  delay(1);
  for (int i = 0; i < sampleSize; i++)
  {
    reading += analogRead(axisPin);
  }
  return reading/sampleSize;
}

//
// Find the extreme raw readings from each axis
//
void AutoCalibrate(int xRaw, int yRaw, int zRaw)
{
  Serial.println("Calibrate");
  if (xRaw < xRawMin)
  {
    xRawMin = xRaw;
  }
  if (xRaw > xRawMax)
  {
    xRawMax = xRaw;
  }
  
  if (yRaw < yRawMin)
  {
    yRawMin = yRaw;
  }
  if (yRaw > yRawMax)
  {
    yRawMax = yRaw;
  }

  if (zRaw < zRawMin)
  {
    zRawMin = zRaw;
  }
  if (zRaw > zRawMax)
  {
    zRawMax = zRaw;
  }
}
