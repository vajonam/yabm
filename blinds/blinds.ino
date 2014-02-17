
/*
  Yet another Blind Minder - Arudino blind controller
  Copyright (C) 2018 vajonam

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.


  Arduino Code that opens and closes blinds similar to
  http://homeawesomation.wordpress.com/2013/02/26/automated-window-blinds-with-arduino/

  The difference is that input methods are different and the blinds are
  open during reasonable light conditions during the day but closes
  when direct sunlight hits the blinds and also closes at night
  time. This is inteded to be completely automatic and require no
  user intervention to open close the blinds. The blinds close
  gradually as light increase upto max threshold. Also there is code
  for smoothing the LDR inputs to produce a flutter/jitter
  free opreation of the blinds.

*/



#include <Servo.h>
#include <Timer.h>



// create servo object to control a servo
Servo myservo;

// create a timer to use
Timer timer;



//arduino pins in use

const byte ldrAnalogPin = 0;    // LDR connected to A0
const byte servoPin = 2;        // Servo connected to D2
const byte autoSwitch = 4;      // Auto Switch connected to D4
const byte openCloseSwitch = 7; // Manual open/close switch connected to D7
const byte closeLED = 6;        // Closed LED connected to D6.  PWM Function used for pusling LED
const byte openLED  = 5;        // Open LED connected to D5. PWM Function used for pusling LED


// other constants

const  byte  maxServo = 148;       // fully closed blinds
const  byte  minServo = 55;        // fully open blinds
const  byte  deadBand = 2;         // dead band, if the change to servos is less than this dont move the blinds, prevents fluttering back and forth when light is fluctutating (partly cloudy day)
const  byte  deadBandLimit = 75;   // if LDR value is less than deadBand for more than this many loops, move the blinds anyway, means its stabilized


const unsigned int brightnessStartThresh = 350;   //start closing blinds when LDR value reaches brightness
const unsigned int brightnessEndThresh = 620;     //close blinds fully at when LDR value reaches brightness
const unsigned int darknessThreshold = 10;        //close blinds fully at when LDR value reaches darkness

const unsigned int moveDelay = 750;               //delay to allow servo to move a few degrees based on sunlight
const unsigned int openCloseDelay = 1000;         //delay to allow servo to open or close fully
const unsigned int loopDelay = 150;               ///delay between loop reads can increase for faster response
const unsigned long antiFlutterDelay = 120000UL;  //stop blinds from flutter at edge points.
const unsigned long printDelay = 10000UL;         // delay to print status every 10 seconds.


// global variables
boolean firstRun = true;            // boolean to store state
boolean manualMode = false;         // boolean to store auto/manual mode switch's state
boolean openClose = false;          // boolean to store open/close mode switch's state
boolean antiFlutter = false;
boolean isClosed;                   // boolean to store state of blinds
unsigned int ldrValue = 0;          // current LDR value
byte oldServPos  = minServo;        // previous servo position
byte servoPos = minServo;            // current servo position
byte deadBandCounter = 0;           // counter to count deadband hits

// global variables for input smoothing

const byte numReadings = 20;          // number of readings to average over
const byte alpha = 4;                 // alpha value to smooth readings over
unsigned int readings[numReadings];   // the readings from the analog input
unsigned int index = 0;               // the index of the current reading
unsigned long total = 0UL;            // the running total using long incase of larger number of readings
unsigned int average = 0;             // smoothing moving average
unsigned int smoothed = 0;            // weighted expoenential smoothing

// Timers and events
int antiFlutterEvent;
int checkInputEvent;
int moveBlindsEvent;
int printStatusEvent; 
int pulseLEDevent; 



// Testing Framework
//#define DEBUG

#ifdef TESTING_MODE
int t_run_number = 0;
int t_myldrValue = 0;
#endif


void setup()
{

  Serial.begin(9600);
  Serial.println("Yet Another Blind Minder Copyright (C) 2014 vajonam");
  Serial.println("");
  Serial.println("This program comes with ABSOLUTELY NO WARRANTY;.");
  Serial.println("This is free software, and you are welcome to redistribute it");
  Serial.println("under certain conditions;");

  pinMode(openLED, OUTPUT);
  pinMode(closeLED, OUTPUT);
  pinMode(autoSwitch, INPUT_PULLUP);
  pinMode(openCloseSwitch, INPUT_PULLUP);

  for (int thisReading = 0; thisReading < numReadings; thisReading++)
    readings[thisReading] = 0;

  checkInputEvent = timer.every(loopDelay / 6, checkInput, (void*)0);
  moveBlindsEvent = timer.every(loopDelay, moveBlinds, (void*)0);
  printStatusEvent = timer.every(printDelay, printStatus, (void*)0);
  pulseLEDevent = timer.every(loopDelay / 4, updateLED, (void*)0);

}

void loop()
{
  timer.update();

}

void checkInput(void *context) {

  manualMode = (boolean) digitalRead(autoSwitch);
  openClose = (boolean) !digitalRead(openCloseSwitch);

#ifdef DEBUG
  Serial.print("Manual-" + (String) manualMode);
  Serial.print(" Open-" + (String) openClose);
  Serial.print(" Anti Flutter " + (String) antiFlutter);
  Serial.println(" isClosed-" + (String) isClosed);
#endif


  ldrValue = analogRead(ldrAnalogPin);
#ifdef TESTING_MODE
  ldrValue = testGenerator();
#endif


  ldrValue = expSmoothing(ldrValue); // weighted smoothing to get rid of noise
  ldrValue = avgSmoothing(ldrValue);    // avg smoothing to get rid of fluctiations


}

void moveBlinds (void *context) {


  if (manualMode) {
    // if the auto mode switch is manual and the open/close switch is closed
    if (!openClose && !isClosed  )   {
      openCloseBlinds(false);
    }
   // if the auto mode switch is manual and the open/close switch is open
    if (openClose && isClosed) {
        openCloseBlinds(true);
    }
  } else {
    //if outside is bright greater than start thresh but still not bright enough to close
    if (ldrValue >= brightnessStartThresh && ldrValue <= brightnessEndThresh )  {
      adjustBlinds();
    }
    //if outside is very dark or very bright, close the blinds fully
    if ( (ldrValue < darknessThreshold || ldrValue > brightnessEndThresh ) && !isClosed && !antiFlutter)  {
      openCloseBlinds(false);
    }

    //if outside is resonably bright open the blinds.
    //Serial.println("Is Closed" + (String) isClosed);
    if ((ldrValue >= darknessThreshold && ldrValue < brightnessStartThresh) && isClosed && !antiFlutter) {
      openCloseBlinds(true);
    }
  }
}





void adjustBlinds() {

  servoPos = map(ldrValue, brightnessStartThresh, brightnessEndThresh, minServo, maxServo);
  servoPos = constrain(servoPos, minServo, maxServo);

  // when are adjusting we are open
  isClosed = false;
  
  int difference = oldServPos - servoPos;
  difference = abs(difference); // doesn't matter if less or more by 10000the deadband

  if (difference > deadBand) {  // opening blinds because the increment is > deadBand
    Serial.println ((String) "Moving  form : " + (String) oldServPos  + (String) " to " + (String)   servoPos );
    moveServo(servoPos,moveDelay);
  } else {
    if (difference > 0) {
      if (deadBandCounter >= deadBandLimit) {
        Serial.println ((String) "Dead Band Limit Reached moving " + (String) oldServPos  + (String) " to " + (String)   servoPos );
        moveServo(servoPos,moveDelay);
        deadBandCounter = 0;
      }
      deadBandCounter++; // increment the deadband counter because we have been through one cycle
    }
  }
}



void moveServo(int position, int moveDelay) {

  myservo.attach(servoPin);
  myservo.write(position);
  delay(moveDelay);
  oldServPos  =  position;
  servoPos = position;
  myservo.detach();

}

void openCloseBlinds(boolean open) {
  if (open) {
    moveServo(minServo,openCloseDelay);
    isClosed = false;
    Serial.println((String) "Blinds are fully open");
  }
  else  {
    moveServo(maxServo,openCloseDelay);
    isClosed = true;
    Serial.println ((String) "Blinds are fully closed");
  }
  if (!manualMode && !firstRun) {
    antiFlutter = true;
    antiFlutterEvent = timer.after(antiFlutterDelay, clearFlutterFlag, (void*)0);
    Serial.println("Enabling Anti Flutter");
  }
}

void clearFlutterFlag(void *context) {
  antiFlutter = false;
  Serial.println("Clearing Flutter flag to " + (String) antiFlutter);
}



// taken from credit goes to http://www.tigoe.com/pcomp/code/arduinowiring/41/

int expSmoothing (int ldrValue) {


  if (ldrValue > smoothed)
    smoothed = smoothed + (ldrValue - smoothed) / alpha ;
  else
    smoothed = smoothed - (smoothed - ldrValue) / alpha ;

  return smoothed;
}




// taken from http://arduino.cc/en/Tutorial/Smoothing

int avgSmoothing(int ldrValue) {
  
  // subtract the last reading:
  total = total - readings[index];

  // store value in array
  readings[index] = ldrValue;

  // add the reading to the total:
  total = total + (unsigned long) readings[index];

  // advance to the next position in the array:
  index = index + 1;

  // if we're at the end of the array...
  if (index >= numReadings)
    // ...wrap around to the beginning:
    index = 0;

  // calculate the average:
  average = total / numReadings;


  return average;
}

void printStatus (void *context) {

#ifdef TESTING_MODE
  Serial.print(" Free Ram: " + freeRam());
#endif
  Serial.print( (String)"LDR: " + (String) ldrValue + (String)" Avg: " +  (String)  average );
  Serial.println ((String) " Prev Servo Pos : " + (String) oldServPos  + (String) " Cur Servo Pos: " + (String)   servoPos);
}


void updateLED (void *context) {

  if (manualMode) {
    float ledVal = (exp(sin(millis() / 500.0 * PI)) - 0.36787944) * 108.0;
    if (isClosed) {
      digitalWrite(openLED, LOW);
      analogWrite(closeLED, ledVal);
    }
    else {
      digitalWrite(closeLED, LOW);
      analogWrite(openLED, ledVal);

    }
  }
  else if (antiFlutter) {
    if (isClosed) {
      digitalWrite(openLED, LOW);
      digitalWrite(closeLED, HIGH);
    }
    else {
      digitalWrite(openLED, HIGH);
      digitalWrite(closeLED, LOW);

    }
  }
  else {
    float ledVal = (exp(sin(millis() / 2000.0 * PI)) - 0.36787944) * 108.0;
    if (isClosed) {
      digitalWrite(openLED, LOW);
      analogWrite(closeLED, ledVal);
    }
    else {
      digitalWrite(closeLED, LOW);
      analogWrite(openLED, ledVal);
    }
  }
}


#ifdef TESTING_MODE

// test LDR value generator, starts from 0 to brightnessEndThresh+100, so it will create a set of pseudo-randmon values in clusters of
// t_cluster, and then move the window for the randmon numbers. This simulators the LDR reistor jumping around a bit and also sunrise to sunset.
// can be improved on.

int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

int testGenerator() {


  if ( (t_run_number % 2) == 0 ) {

    t_myldrValue++;
    if (t_myldrValue > brightnessEndThresh) {
      t_run_number++;
      delay(1000);
    }

  } else {
    t_myldrValue--;
    if (t_myldrValue < -200) {
      delay(1000);
      t_run_number++;
    }

  }


  Serial.println((String)" Test ldrValue : " +  (String)  t_myldrValue );

  return t_myldrValue;

}

#endif








