/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

#include <ArduinoLowPower.h>

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

//COMMUNICATION PINS/VARIABLES
float stateVector[2]; //meant to have 3 elements, 'current state' , 'current brightness' and 'color'
int stateBit = 0; //default to inactive
int brightVector[2]; //vector to contain each bit of the normalizedAvg variable
int RBGVector[2]; //vector to contain each 8 bit word for R G and B 

//Sleep Mode VARIABLES
#define WAKEAVG_THRESHOLD .35 
#define SLEEP_INTERVAL 5000
#define SLEEP_DELAY 300

//State variables
unsigned long belowThresholdStart = 0;
boolean SLEEP = true; //current state flag, initialized to true so the nodes default to an inactive state
float normalizedAvg;
boolean commWake = false; 

//creating the packet for peer to peer communication
struct Packet{

  //this represents the on/off state of the node
  int currentState;

  //this represents the brightness of the node
  float LEDbrightness; 

  //this represents the color of the node (work in progress)
  int RLEDcolor;
  int GLEDcolor;
  int BLEDcolor;
};

Packet packet = {0, 0.00f, 0, 0, 0}; //this is the general format of the packet


/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/



/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

void setup() {
  
  //communication intializations
  Serial1.begin(9600);
  Serial2.begin(9600);

  //LED/Sensor pins
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
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
    
    //update the communicated brightness
    packet.LEDbrightness = normalizedAvg; 
    
    return normalizedAvg;
}

void checkComms(){

  //if there is a packet available for reading
  if(Serial1.available() >= sizeof(packet)){

    //read and store the packet
    Serial1.readBytes((uint8_t*)&packet, sizeof(packet));

    //if the communicated packet says this node should be 'on' 
    if(packet.currentState == 1){
      
      //update the communication flag
      commWake = true; 
      return;
    }
    else{
      
      //ensure the flag is false
      commWake = false;
      return; 
    }
  }
  else{

    //there is no packet to read
    return; 
  }
}

void writeComms(){

  //write the packet to neighbors
  Serial1.write((uint8_t*) &packet, sizeof(packet));
  Serial2.write((uint8_t*) &packet, sizeof(packet));

}


/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/



/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

void loop() {

  if(SLEEP){

    //store the current communicated state
    packet.currentState = 0;

    //reset the communication flag
    commWake = false;

    //Sleep for a set, adjustable period
    LowPower.sleep(SLEEP_INTERVAL);

    //the board wakes early if it gets UART packet, so read it
    checkComms();

    //if the board nees to stay awake
    if(commWake){

      //update the sleep mode flag
      SLEEP = false; 
      return;

    }

    //wake after interval and read sensors
    normalizedAvg = readSensors();

    if(normalizedAvg > WAKEAVG_THRESHOLD){
      
      //update the communicated state
      packet.currentState = 1;
      packet.LEDbrightness = normalizedAvg; 
      writeComms();

      //update the state
      SLEEP = false; 
    
    }

    //this ends this iteration of the loop, the node stays inactive if normalizedAvg doesn't meet the threshold
    return;

  }

  //if the node is not sleeping already
  normalizedAvg = readSensors();

  //if the avg doesn't meet the threshold
  if(normalizedAvg <= WAKEAVG_THRESHOLD){

    //check to see if there has been another recent avg low
    if(belowThresholdStart == 0){

      belowThresholdStart = getTime();
    
    }
    else if(getTime() - belowThresholdStart >= SLEEP_DELAY){

      //update the communicated state
      packet.currentState = 0;
      
      //go to sleep, multiple low avg readings
      SLEEP = true; 
      belowThresholdStart = 0;

      //explictly disable LED
      analogWrite(redPin, 0);
      analogWrite(greenPin, 0);
      analogWrite(bluePin, 0);

      //end this iteration
      return;

    }
  }
  else{

    belowThresholdStart = 0; 

    //update and communicate
    packet.currentState = 1;
    packet.LEDbrightness = normalizedAvg; 

    //this could be a more dynamic thing- but for now this will do
    packet.RLEDcolor = baseR;
    packet.GLEDcolor = baseG;
    packet.BLEDcolor = baseB; 
    
    //send it
    writeComms();

    //configure LED
    analogWrite(redPin, baseR * normalizedAvg);
    analogWrite(greenPin, baseG * normalizedAvg);
    analogWrite(bluePin, baseB * normalizedAvg);

    return;
  }

  //for sensor smoothness
  delay(50);
}
/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/