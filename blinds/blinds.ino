/*
Yet another Blind Minder - Arudino blind controller
Copyright (C) 2014 Manojav Sridhar

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

Servo myservo;  

// create servo object to control a servo 

//arduino pins in use

const int ldrAnalogPin = 0;    // LDR connected to A0
const int servoPin = 2;        // Servo connected to D2
const int autoSwitch = 4;      // Auto Switch connected to D4
const int openCloseSwitch = 5; // Manual open/close switch connected to D5
const int closeLED = 6;          // Closed LED connected to D6
const int openLED  = 7;        // Open LED connected to D7

const int maxServo = 155;       // flly closed blinds 
const int minServo = 55;        // fully open blinds
const int deadBand = 2;         // dead band, if the change to servos is less than this dont move the blinds, prevents fluttering back and forth when light is fluctutating (partly cloudy day)
const int deadBandLimit = 25;   // if LDR value is less than deadBand for more than this many loops, move the blinds anyway.


const int startCloseThreshold = 480; //start closing blinds when LDR value reaches brightness
const int endCloseThreshold = 640;   //close blinds fully at when LDR value reaches brightness
const int minThreshold = 20;         //close blinds fully at when LDR value reaches darkness

const int moveDelay = 500;           //delay to allow servo to move a few degrees based on sunlight
const int openCloseDelay = 1000;     //delay to allow servo to open or close fully
const int loopDelay = 150;           //delay between loop reads can increase for faster response 

// global variables
boolean firstRun = true;            // boolean to store state
boolean autoMode = false;           // boolean to store auto/manual mode switch's state
boolean openClose = false;          // boolean to store open/close mode switch's state
boolean isClosed;                   // boolean to store state of blinds
int ldrValue = 0;                   // current LDR value
int prevBlindChangeLdrValue = 0;    // LDR value when blinds were last moved
int deadBandCounter = 0;            // counter to count deadband hits 

// global variables for input smoothing

const int numReadings = 50;     // number of readings to average over
const int alpha = 4;            // alpha value to smooth readings over 
int readings[numReadings];      // the readings from the analog input
int index = 0;                  // the index of the current reading
unsigned long total = 0L;       // the running total using long incase of larger number of readings 
int average = 0;                // smoothing moving average
int smoothed = 0;               // weighted expoenential smoothing


// Testing Framework
// #define TESTING_MODE

#ifdef TESTING_MODE
int t_counter = 0;
int t_range = 5; 
int t_cluster = 10;
int t_clusterCounter = 0;
#endif


void setup() 
{ 

  Serial.begin(9600); 
  Serial.println("Yet Another Blind Minder Copyright (C) 2014 Manojav Sridhar");
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

}

void loop() 
{ 


  autoMode = (bool) digitalRead(autoSwitch);
  openClose = (bool) digitalRead(openCloseSwitch);


#ifdef TESTING_MODE  
  ldrValue = testGenerator();
#else
  ldrValue = expSmoothing(ldrAnalogPin); // weighted smoothing to get rid of noise
#endif

  ldrValue = avgSmoothing(ldrValue);    // avg smoothing to get rid of fluctiations 


  // set the old_val and current val as the same if its the first run
  if (firstRun) {
    prevBlindChangeLdrValue = -5;
  }

  //if outside is bright greater than start thresh 
  if ( (ldrValue >= startCloseThreshold && ldrValue <= endCloseThreshold ) && !autoMode)  {
    close(false);
  }
  //if outside is very dark, close the blinds fully
  if ( (ldrValue < minThreshold || ldrValue > endCloseThreshold ) && (!isClosed || firstRun) && !autoMode)  {
    close(true);
  }

  //if outside is resonably bright open the blinds.

  if ((ldrValue >= (minThreshold) && ldrValue < startCloseThreshold) && (isClosed || firstRun) && !autoMode) {
    open();
  }


  // if the auto mode switch is manual and the open/close switch is closed
  if (((openClose && !isClosed )|| firstRun ) && autoMode )   {
    close(true);
  } 

  // if the auto mode switch is manual and the open/close switch is open
  if (((!openClose && isClosed) || firstRun )  && autoMode) {
    open(); 
  }

  // the loop has run atleast once.
  firstRun = false;
#ifdef TESTING_MODE
  Serial.println(freeRam());
#endif
  delay(loopDelay);

}

#ifdef TESTING_MODE

int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
#endif


void close(boolean full) {


  int servoPos = map(ldrValue,startCloseThreshold,endCloseThreshold,minServo,maxServo);
  int new_servoPos = constrain(servoPos, minServo,maxServo);
  int mydelta = ldrValue-prevBlindChangeLdrValue;
  mydelta = abs(mydelta);


  if (full ) {
    if (mydelta > deadBand ) {
      myservo.attach(servoPin);
      myservo.write(maxServo);
      delay(openCloseDelay);
      digitalWrite(closeLED, HIGH);
      digitalWrite(openLED, LOW);
      isClosed = true;
      Serial.println ((String) "Blinds are fully closed");
      prevBlindChangeLdrValue = ldrValue;
    }
  }
  else {      

    if (prevBlindChangeLdrValue > (new_servoPos + deadBand)) {  // opening blinds because the increment is > deadBand
      Serial.println ((String) "Opening form : " + (String) prevBlindChangeLdrValue + (String) " to " + (String)   new_servoPos );
      myservo.attach(servoPin);
      myservo.write(new_servoPos);
      delay(moveDelay);
      prevBlindChangeLdrValue =  new_servoPos;
    } 
    else if (prevBlindChangeLdrValue < (new_servoPos-deadBand)) { // close blinds because the decrement is > deadBand
      Serial.println ((String) "Closing form : " + (String) prevBlindChangeLdrValue + (String) " to " + (String)   new_servoPos );        
      myservo.attach(servoPin);
      myservo.write(new_servoPos);
      delay(moveDelay);
      prevBlindChangeLdrValue =  new_servoPos;
    } 
    else {

      if (deadBandCounter > deadBandLimit && prevBlindChangeLdrValue != new_servoPos)  {
        deadBandCounter = 0;
        myservo.attach(servoPin);
        myservo.write(new_servoPos);
        delay(moveDelay);
        prevBlindChangeLdrValue =  new_servoPos;
        Serial.println ((String) "Dead Band Limit Reached moving " + (String) prevBlindChangeLdrValue + (String) " to " + (String)   new_servoPos );
      }

      // hit dead band, increment counter so if we are just off by less than deadBand and its stable we can move the servo by less than deadBand
      deadBandCounter++;

    }

  }
  myservo.detach();

}

void open() {
  int mydelta = ldrValue-prevBlindChangeLdrValue;
  mydelta = abs(mydelta);
  if (mydelta > deadBand ) {
    isClosed = false;
    myservo.attach(servoPin);
    myservo.write(minServo);
    digitalWrite(closeLED, LOW);
    digitalWrite(openLED, HIGH);
    delay(openCloseDelay);
    myservo.detach();
    Serial.println("Blinds are fully open");
    prevBlindChangeLdrValue = ldrValue;
  }

}


// taken from credit goes to http://www.tigoe.com/pcomp/code/arduinowiring/41/

int expSmoothing (int inputPin) {

  ldrValue = analogRead(inputPin);

  if (ldrValue > smoothed)
    smoothed = smoothed + (ldrValue - smoothed)/alpha ;
  else
    smoothed = smoothed - (smoothed - ldrValue)/alpha ;

  Serial.print((String)"LDR: " + (String) ldrValue);
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

    Serial.println((String)" Avg: " +  (String)  average );

  return average;    
}



#ifdef TESTING_MODE

// test LDR value generator, starts from 0 to endCloseThreshold+100, so it will create a set of pseudo-randmon values in clusters of 
// t_cluster, and then move the window for the randmon numbers. This simulators the LDR reistor jumping around a bit and also sunrise to sunset.
// can be improved on. 

int testGenerator() {

  int myldrValue = random(t_counter,t_counter+t_range);

  if (t_clusterCounter > t_cluster) {
    t_counter++;
    t_clusterCounter = 0;     
  }
  t_clusterCounter++;

  if ( t_counter > endCloseThreshold+100)
    t_counter=0;

  Serial.println((String)" Test ldrValue : " +  (String)  myldrValue );

  return myldrValue;

}

#endif


