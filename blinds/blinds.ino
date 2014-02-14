

/*
Yet another Blind Minder - Arudino blind controller
 Copyright (C) 2014 vajonam
 
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
 devoted to smoothing the LDR inputs to produce a flutter/jitter
 free opreation of the blinds.
 
 */



#include <Servo.h> 
#include <Timer.h>


Servo myservo;
Timer timer;


// create servo object to control a servo 

//arduino pins in use

const int ldrAnalogPin = 0;    // LDR connected to A0
const int servoPin = 2;        // Servo connected to D2
const int autoSwitch = 4;      // Auto Switch connected to D4
const int openCloseSwitch = 7; // Manual open/close switch connected to D7
const int closeLED = 6;          // Closed LED connected to D6 PWM Function used for pusling LED
const int openLED  = 5;        // Open LED connected to D5 - PWM Function used for pusling LED

const int maxServo = 155;       // flly closed blinds 
const int minServo = 55;        // fully open blinds
const int deadBand = 2;         // dead band, if the change to servos is less than this dont move the blinds, prevents fluttering back and forth when light is fluctutating (partly cloudy day)
const int deadBandLimit = 75;   // if LDR value is less than deadBand for more than this many loops, move the blinds anyway.


const int startCloseThreshold = 480; //start closing blinds when LDR value reaches brightness
const int endCloseThreshold = 640;   //close blinds fully at when LDR value reaches brightness
const int minThreshold = 10;         //close blinds fully at when LDR value reaches darkness

const int moveDelay = 500;           //delay to allow servo to move a few degrees based on sunlight
const int openCloseDelay = 1000;     //delay to allow servo to open or close fully
const int loopDelay = 150;           //delay between loop reads can increase for faster response 
const long antiFlutterDelay = 60000*2;  //stop blinds from flutter at edge points.

// global variables
boolean firstRun = true;            // boolean to store state
boolean manualMode = false;           // boolean to store auto/manual mode switch's state
boolean openClose = false;          // boolean to store open/close mode switch's state
boolean antiFlutter = false;
boolean isClosed;                   // boolean to store state of blinds
int ldrValue = 0;                   // current LDR value
int blindsChangeServoPos = 0;    // LDR value when blinds were last moved
int deadBandCounter = 0;            // counter to count deadband hits 

// global variables for input smoothing

const int numReadings = 50;     // number of readings to average over
const int alpha = 4;            // alpha value to smooth readings over 
int readings[numReadings];      // the readings from the analog input
int index = 0;                  // the index of the current reading
unsigned long total = 0L;       // the running total using long incase of larger number of readings 
int average = 0;                // smoothing moving average
int smoothed = 0;               // weighted expoenential smoothing
long servoPos = minServo;
// Timers and events

int checkInputEvent ;
int antiFlutterEvent; 
int moveBlindsEvent;

// Testing Framework
//#define TESTING_MODE
//#define DEBUG

#ifdef TESTING_MODE
int t_run_number = 1; 
int t_counter = 0;
#endif


void setup() 
{ 

  Serial.begin(9600); 
  Serial.println("Yet Another Blind Minder Copyright (C) 2014 vajonam");
  Serial.println("");
  Serial.println("This program comes with ABSOLUTELY NO WARRANTY;.");
  Serial.println("This is free software, and you are welcome to redistribute it");
  Serial.println("under certain conditions;");

  pinMode(openLED,OUTPUT);
  pinMode(closeLED,OUTPUT);
  pinMode(autoSwitch,INPUT_PULLUP);
  pinMode(openCloseSwitch,INPUT_PULLUP);

  for (int thisReading = 0; thisReading < numReadings; thisReading++)
    readings[thisReading] = 0;   

  checkInputEvent = timer.every(loopDelay/2, checkInput, (void*)0); 
  moveBlindsEvent = timer.every(loopDelay, moveBlinds, (void*)0); 
  int printStatusEvent = timer.every(30000, printStatus, (void*)0);
  int pulseLEDevent = timer.every(loopDelay, updateLED,(void*)0);

}

void loop() 
{ 
  timer.update();

}


void clearFlutterFlag(void *context) {

  antiFlutter = false;
  Serial.println("Clearing Flutter flag to " + (String) manualMode);

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
#else
#endif
  ldrValue = expSmoothing(ldrValue); // weighted smoothing to get rid of noise
  ldrValue = avgSmoothing(ldrValue);    // avg smoothing to get rid of fluctiations 


}

void moveBlinds (void *context) {



  // set the old_val and current val as the same if its the first run
  if (firstRun) {
    blindsChangeServoPos = minServo;
  }

  //if outside is bright greater than start thresh but still not bright enough to close
  if ( (ldrValue >= startCloseThreshold && ldrValue <= endCloseThreshold ) && !manualMode)  {
    adjustBlinds();
  }
  //if outside is very dark or very bright, close the blinds fully
  if ( (ldrValue < minThreshold || ldrValue > endCloseThreshold ) && (!isClosed || firstRun) && !manualMode && !antiFlutter)  {
    openCloseBlinds(false);
  }

  //if outside is resonably bright open the blinds.

  if ((ldrValue >= (minThreshold) && ldrValue < startCloseThreshold) && (isClosed || firstRun) && !manualMode && !antiFlutter) {
    openCloseBlinds(true);
  }


  // if the auto mode switch is manual and the open/close switch is closed
  if (((!openClose && !isClosed )|| firstRun ) && manualMode )   {
    openCloseBlinds(false);
  } 

  // if the auto mode switch is manual and the open/close switch is open
  if (((openClose && isClosed) || firstRun )  && manualMode ) {
    openCloseBlinds(true); 
  }


  // the loop has run atleast once.
  firstRun = false;
#ifdef TESTING_MODE
  //Serial.println(freeRam());
#endif

}



#ifdef TESTING_MODE

int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
#endif


void adjustBlinds() {

  servoPos = map(ldrValue,startCloseThreshold,endCloseThreshold,minServo,maxServo);
  servoPos = constrain(servoPos,minServo,maxServo);

  isClosed = false;

  if (blindsChangeServoPos > (servoPos + deadBand)) {  // opening blinds because the increment is > deadBand
    Serial.println ((String) "Opening form : " + (String) blindsChangeServoPos + (String) " to " + (String)   servoPos );
    myservo.attach(servoPin);
    myservo.write(servoPos);
    delay(moveDelay);
    blindsChangeServoPos =  servoPos;
  } 
  else if (blindsChangeServoPos < (servoPos - deadBand)) { // close blinds because the decrement is > deadBand
    Serial.println ((String) "Closing form : " + (String) blindsChangeServoPos + (String) " to " + (String)   servoPos );        
    myservo.attach(servoPin);
    myservo.write(servoPos);
    delay(moveDelay);
    blindsChangeServoPos =  servoPos;
  } 
  else {
    Serial.println ((String) "Ignoring Change too small delta " + (String) blindsChangeServoPos + (String) " to " + (String)   servoPos );
    if (deadBandCounter > deadBandLimit && blindsChangeServoPos != servoPos)  {
      deadBandCounter = 0;
      myservo.attach(servoPin);
      myservo.write(servoPos);
      delay(moveDelay);
      Serial.println ((String) "Dead Band Limit Reached moving " + (String) blindsChangeServoPos + (String) " to " + (String)   servoPos );
      blindsChangeServoPos =  servoPos;
    }

    // hit dead band, increment counter so if we are just off by less than deadBand and its stable we can move the servo by less than deadBand
    deadBandCounter++;

  }
  myservo.detach();
}

void openCloseBlinds(boolean open) {
  if (open) {
    isClosed = false;
    myservo.attach(servoPin);
    myservo.write(minServo);
    delay(openCloseDelay);
    myservo.detach();
    Serial.println("Blinds are fully open");
    blindsChangeServoPos = minServo;
  } 
  else  {
    myservo.attach(servoPin);
    myservo.write(maxServo);
    delay(openCloseDelay);
    isClosed = true;
    Serial.println ((String) "Closing form : " + (String) blindsChangeServoPos + (String) " to " + (String)   servoPos );        
    myservo.detach();
    blindsChangeServoPos = maxServo;
    Serial.println ((String) "Blinds are fully closed");
  }

  if (!manualMode && !firstRun){
     antiFlutter = true;
     antiFlutterEvent = timer.after(antiFlutterDelay, clearFlutterFlag, (void*)0);
  }


}



// taken from credit goes to http://www.tigoe.com/pcomp/code/arduinowiring/41/

int expSmoothing (int ldrValue) {



  if (ldrValue > smoothed)
    smoothed = smoothed + (ldrValue - smoothed)/alpha ;
  else
    smoothed = smoothed - (smoothed - ldrValue)/alpha ;


  return smoothed;
}




// taken from http://arduino.cc/en/Tutorial/Smoothing

int avgSmoothing(int ldrValue) {
  // subtract the last reading:
  total= total - readings[index];         
  // read from the sensor:  
  //ldrValue = analogRead(inputPin);

  readings[index] = ldrValue; 
  // add the reading to the total:
  total= total + readings[index];       
  // advance to the next position in the array:  
  index = index + 1;                    

  // if we're at the end of the array...
  if (index >= numReadings)              
    // ...wrap around to the beginning: 
    index = 0;                           

  // calculate the average:
  average = total / numReadings;         
  // send it to the computer as ASCII digits


    return average;    
}

void printStatus (void *context) {
  Serial.print( (String)"LDR: " + (String) ldrValue + (String)" Avg: " +  (String)  average );
  Serial.println ((String) " Prev Servo Pos : " + (String) blindsChangeServoPos + (String) " Cur Servo Pos: " + (String)   servoPos );        
}


void updateLED (void *context) {

  if (antiFlutter) {
    if(isClosed) {
      digitalWrite(closeLED, HIGH);
      digitalWrite(openLED, LOW);
    } 
    else
    {
      digitalWrite(closeLED, LOW);
      digitalWrite(openLED, HIGH);
    }
  } 
  else {
    float ledVal = (exp(sin(millis()/2000.0*PI)) - 0.36787944)*108.0;
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

// test LDR value generator, starts from 0 to endCloseThreshold+100, so it will create a set of pseudo-randmon values in clusters of 
// t_cluster, and then move the window for the randmon numbers. This simulators the LDR reistor jumping around a bit and also sunrise to sunset.
// can be improved on. 

int testGenerator() {
  int myldrValue = 0;
  if (t_counter < numReadings*2) {

    if (t_run_number == 1)  
      myldrValue = 0 ;
    if (t_run_number == 2)
      myldrValue = minThreshold;
    if (t_run_number == 3)
      myldrValue= startCloseThreshold;
    if (t_run_number == 4)
      myldrValue= endCloseThreshold;
    if (t_run_number == 5)
      myldrValue= 0;



  } 
  else {
    t_counter = 0;
    t_run_number++;

  }

  t_counter++;

  if (t_run_number > 5)
    t_run_number = 0;


  Serial.print((String)" Test ldrValue : " +  (String)  myldrValue );



  return myldrValue;

}

#endif







