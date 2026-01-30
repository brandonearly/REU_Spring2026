/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

#include <avr/sleep.h> //allows for 'sleep' mode power saving
#include <avr/wdt.h> //for interval wakes
#include <avr/interrupt.h> 

//RBG PINS and initalized color values
#define redPin 9
#define greenPin 10
#define bluePin 11
#define baseR 0
#define baseG 255
#define baseB 0

//ULTRASONIC PINS/VARIABLES
#define trigPin 12
#define echoPin 13
#define MINDIST 0 //centimeters, hardcoded test values for prototype's sake
#define MAXDIST 5.0 //centimeters, hardcoded test values for prototype's sake
boolean ultrasonicHigh; 

//PHOTORESISTOR PINS/VARIABLES
#define lightPin A0 //implicitly INPUT pin
int lightLevelADC = 0;
#define MINLIGHT .3 //hardcoded test values for prototype's sake
#define MAXLIGHT 4.5 //hardcoded test values for prototype's sake

//THERMISTOR PINS/VARIABLES
#define tempPin A1 //implicitly INPUT pin
int temperatureADC = 0;
#define MINTEMP 2.6 //hardcoded test values for prototype's sake
#define MAXTEMP 3.0 //hardcoded test values for prototype's sake

//sensor reading and normalization variables
float distance;
float lightLevel;
float temperature;
float distNorm;
float lightNorm;
float tempNorm;
#define distWeight .4 //adjust based on surrounding environment
#define lightWeight .4 //adjust based on surrounding environment
#define tempWeight .2 //adjust based on surrounding environment
float normalizedAvg;

//COMMUNICATION PINS/VARIABLES
float stateVector[2]; //meant to have 3 elements, 'current state' , 'current brightness' and 'color'
int stateBit = 0; //default to inactive
int brightVector[2]; //vector to contain each bit of the normalizedAvg variable
int RBGVector[2]; //vector to contain each 8 bit word for R G and B
boolean commWake = false; 

//Sleep Mode VARIABLES
#define WAKEAVG_THRESHOLD .35 
unsigned long belowThresholdStart = 0;
boolean SLEEP = true; //current state flag, initialized to true so the nodes default to an inactive state
boolean wdtWake = false; //flag to indicate the WDT woke the MCU
#define SLEEP_DELAY 300

/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/



/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

void setup() {
  
  //communication intializations
  //Serial1.begin(9600);
  //Serial2.begin(9600);

  //LED/Sensor pins
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  //interval wakes WDT
  MCUSR &= ~(1 << WDRF); // Clear watchdog reset flag
  WDTCSR |= (1 << WDCE) | (1 << WDE); // Enable configuration
  WDTCSR = (1 << WDIE) | (1 << WDP3) | (1 << WDP0); // ~8s interrupt

  //sleep mode 
  set_sleep_mode(SLEEP_MODE_IDLE);
  sei(); //enable interrupts
  sleep_enable();
  sleep_cpu();
  sleep_disable();

}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/



/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

/*Reads the ultrasonic sensor and converts the time of the pulse to distance of the object*/
float getDistanceCM() {
  // Insurance for clean trigger
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Trigger ultrasonic reading
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  //Read echo pulse
  long duration = pulseIn(echoPin, HIGH, 30000); // 30 ms timeout

  if (duration == 0) {
    return -1; // no echo detected, no object seen in the sensors range
  }

  //math converting speed of sound in air (at room temperature, for more advanced stuff could have this value adjust based on temperature reading), to CM
  //CHATGPT sourced
  return (duration * 0.0343) / 2.0;
}

/*Reads photoresistor's voltage*/
float getLightLevel(){
  lightLevelADC = analogRead(A0);
  float lightLevelReading = lightLevelADC * (5.0/1023.0);
  return lightLevelReading;
}

/*Reads thermistor's voltage*/
float getTemperature(){
  temperatureADC = analogRead(A1);
  float temperatureReading = temperatureADC * (5.0/1023.0);
  return temperatureReading;
}

/*Read current time in millis*/
unsigned long getTime(){
  return millis();
}

float readSensors(){

  //read in measurements
    distance = getDistanceCM();
    lightLevel = getLightLevel();
    temperature = getTemperature();
    
    //normallize readings into one number scale
    distNorm = constrain((MAXDIST - distance) / (MAXDIST - MINDIST), 0.0, 1.0);
    lightNorm = constrain((lightLevel - MINLIGHT) / (MAXLIGHT - MINLIGHT), 0.0, 1.0);
    tempNorm = constrain((temperature - MINTEMP) / (MAXTEMP - MINTEMP), 0.0, 1.0);

    /*Once we have the normalized values, we can combine and adjust LED brightness accordingly, using  a 
    weighted average of the readings, weights are adjustable based on conditions (maybe dynamically adjustable down the line?)*/
    normalizedAvg = ( (distWeight * distNorm) + (lightWeight * lightNorm) + (tempWeight * tempNorm) );
    return normalizedAvg;
}

ISR(WDT_vect  ){
  wdtWake = true;
}

/*
void checkComms(){
  if(Serial1.available()){

  }
}
*/

/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/



/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

void loop() {
  
  if(SLEEP){

    //wake on 8 second interval
    if(wdtWake){

      //reset flag
      wdtWake = false;

      //get sensor readings
      normalizedAvg = readSensors();

      //check if readings require active node
      if(normalizedAvg > WAKEAVG_THRESHOLD){

        //update flag if node awakes  
        SLEEP = false;
      
      }
      else{
          
        //put back to sleep
        sleep_enable();
        sleep_cpu();
        sleep_disable();
      
      }
    }
  }
  //if the node is active
  else{

    normalizedAvg = readSensors(); 

    if(normalizedAvg <= WAKEAVG_THRESHOLD){
      
      //this clocks the first time normalized average goes low
      if(belowThresholdStart == 0){
        
        belowThresholdStart = getTime();
      
      }
      //this clocks the avg low at a close time to the first low, insurance against debounce style issues
      else if(getTime() - belowThresholdStart >= SLEEP_DELAY){
        //reset sleep timer
        belowThresholdStart = getTime();

        //update flag
        SLEEP = true;


        //explicitly turn off LED
        analogWrite(redPin, 0);
        analogWrite(greenPin, 0);
        analogWrite(bluePin, 0);

        //go to sleep
        sleep_enable();
        sleep_cpu();
        sleep_disable();
      }
    }
    else{

      //reset sleep timer
      belowThresholdStart = 0;
      
      //configure the LED
      analogWrite(redPin, baseR * normalizedAvg);
      analogWrite(greenPin, baseG * normalizedAvg);
      analogWrite(bluePin, baseB * normalizedAvg);

      //communicate local state
      //Serial1.write();
      //Serial2.write();

    }

  }
  delay(50); //super small delay for sensor smoothing
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/