/*-------------------------------------------------------------Libraries----------------------------------------------------------------------------------------------------*/

#include <ArduinoLowPower.h>

/*------------------------------------------LED/SENSOR Definitions & Variables----------------------------------------------------------------------------------------------*/
//RBG PINS and initalized color values
#define redPin 9
#define greenPin 10
#define bluePin 11
#define baseR 0
#define baseG 4095
#define baseB 0

//ULTRASONIC PINS/VARIABLES
#define trigPin 12
#define echoPin 8
#define MINDIST 0 //centimeters, hardcoded test values for prototype's sake
#define MAXDIST 10.0 //centimeters, hardcoded test values for prototype's sake 

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
struct sensorData{
  float distance;
  float lightLevel;
  float temperature;
};

sensorData currentData; 

float distNorm;
float lightNorm;
float tempNorm;
#define distWeight .4 //adjust based on surrounding environment
#define lightWeight .4 //adjust based on surrounding environment
#define tempWeight .2 //adjust based on surrounding environment

/*-----------------------------------Sleep/State & Communication Definitions & Variables------------------------------------------------------------------------------------*/
//Sleep Mode VARIABLES
#define WAKEAVG_THRESHOLD .35 
#define SLEEP_INTERVAL 5000
#define SLEEP_DELAY 300
#define WAKE_IN_PIN 2 //external interupt pin
#define WAKE_OUT_PIN 3 //send interrupts to neighbors
volatile boolean wakeFlag = false; //flag

//State variables
unsigned long belowThresholdStart = 0; //first low reading time
boolean SLEEP = true; //current state flag, initialized to true so the nodes default to an inactive state
float normalizedAvg; //final brightness variable
float localAvg; //local belief
float peerAvg; //peer belief
#define HIGH_COMMS_THRESHOLD .8 //value where very high readings from peers are weighed heavier in final state calculation
#define LOW_COMMS_THRESHOLD .2 //value where very low readings from peers are weighed lighter in final state calculation
#define EXTREME_COMMS_WEIGHT .75 //weight for peer belief when communicated value is extreme 
#define GENERAL_COMMS_WEIGHT .5 //weight for peer belief for majority cases


//COMMUNICATION PINS/VARIABLES
#define START_BYTE 0xAA //byte to confirm communication is starting
#define AUTH_ID1 1 //the id of the prior node, must be changed per board
#define AUTH_ID2 3 //the id of the after node, must be changed per board
#define NODE_ID 2
volatile boolean newPacket = false;
boolean peerActive = false; 

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

Packet rxPacket = {0, 0.00f, 0, 0, 0}; //this is the general format of the packet
Packet txPacket = {0, 0.00f, 0, 0, 0}; //this is the general format of the packet

/*---------------------------------------------------------Setup------------------------------------------------------------------------------------------------------------*/
void setup() {
  
  //communication intializations
  Serial1.begin(9600); //going to use a UART bus, since NANO 33 boards have just 1 set of RX/TX pins

  //LED/Sensor pins
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  //PWM initialization for LED brightness
  analogWriteResolution(12); //shouldn't really need but can be extra safe here
  analogReadResolution(12);

  //External Interrupt
  pinMode(WAKE_IN_PIN, INPUT_PULLUP);
  pinMode(WAKE_OUT_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(WAKE_IN_PIN), wakeISR, FALLING);

}

/*------------------------------------------------------Sensor Readings & Local Belief--------------------------------------------------------------------------------------*/
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
  long duration = pulseIn(echoPin, HIGH, 580); // 580 microseconds timeout for ~10 CM range, this IS blocking, but for prototyping purposes I don't think it matters that much

  if (duration == 0) {

    return MAXDIST + 1; // no echo detected, no object seen in the sensors range

  }

  //math converting speed of sound in air (at room temperature, for more advanced stuff could have this value adjust based on temperature reading), to CM
  //CHATGPT sourced
  return (duration * 0.0343) / 2.0;

}

/*Reads photoresistor's voltage*/
float getLightLevel(){

  lightLevelADC = analogRead(lightPin);
  float lightLevelReading = lightLevelADC * (3.3/4095.0);
  return lightLevelReading;

}

/*Reads thermistor's voltage*/
float getTemperature(){

  temperatureADC = analogRead(tempPin);
  float temperatureReading = temperatureADC * (3.3/4095.0);
  return temperatureReading;

}

/*Read current time in millis*/
unsigned long getTime(){

  return millis();

}

float readSensors(){

  //read in measurements
  currentData.distance = getDistanceCM();
  currentData.lightLevel = getLightLevel();
  currentData.temperature = getTemperature();
    
  //normallize readings into one number scale
  distNorm = constrain((MAXDIST - currentData.distance) / (MAXDIST - MINDIST), 0.0, 1.0);
  lightNorm = constrain((currentData.lightLevel - MINLIGHT) / (MAXLIGHT - MINLIGHT), 0.0, 1.0);
  tempNorm = constrain((currentData.temperature - MINTEMP) / (MAXTEMP - MINTEMP), 0.0, 1.0);

  /*Once we have the normalized values, we can combine and adjust LED brightness accordingly, using  a 
  weighted average of the readings, weights are adjustable based on conditions (maybe dynamically adjustable down the line?)*/
  localAvg = ( (distWeight * distNorm) + (lightWeight * lightNorm) + (tempWeight * tempNorm) );
 
  return localAvg;

}

/*---------------------------------------------Peer Belief & State Change---------------------------------------------------------------------------------------------------*/


/*use the packet information to generate a normalized value for the peer belief as well*/
float getBelief(){

  //check the value of the communicating node's state
  if(rxPacket.LEDbrightness > HIGH_COMMS_THRESHOLD || rxPacket.LEDbrightness < LOW_COMMS_THRESHOLD){

    peerAvg = (rxPacket.LEDbrightness * EXTREME_COMMS_WEIGHT);
    return peerAvg; 

  }
  else{

    peerAvg = (rxPacket.LEDbrightness * GENERAL_COMMS_WEIGHT);
    return peerAvg; 

  }

}

//create final state
void updateState(){

  //all weighing should be done prior to this...
  if(peerActive){

    normalizedAvg = (peerAvg + localAvg) / 2;

  }
  else{

    normalizedAvg = localAvg; 

  }

  //shouldn't be needed but just to be sure
  normalizedAvg = constrain(normalizedAvg, 0.0, 1.0);  

}

/*-------------------------------------------------------------Communication------------------------------------------------------------------------------------------------*/
void sendPacket(Packet p){

  //give neighbor a second to wake after interrupt sent
  digitalWrite(WAKE_OUT_PIN, LOW);
  delayMicroseconds(300);
  digitalWrite(WAKE_OUT_PIN, HIGH);

  //indicate communication needs to start
  Serial1.write(START_BYTE);

  //communicate the expected packet size
  Serial1.write(sizeof(Packet)); 

  //for identifying who sent the packet --> NODE_ID def up top has to be updated when uploading to each board
  Serial1.write(NODE_ID);

  //the actual packet data
  Serial1.write((uint8_t*)&p, sizeof(Packet));

  //checksum for verification/security
  uint8_t checksum = 0;
  uint8_t* bytes = (uint8_t*)&p;
  for(int i = 0; i < sizeof(Packet); i++){
    
    //using a XOR
    checksum ^= bytes[i];

  }

  //send the checksum
  Serial1.write(checksum);

}

//parses through a recieved packet
void checkPacket(){

  //variables for what the packet should include
  static uint8_t state = 0;
  static uint8_t length;
  static uint8_t senderID;
  static uint8_t buf[sizeof(Packet)];
  static int index = 0;
  static uint8_t checksum = 0;

  //if there is a packet in the bus
  while(Serial1.available()){

    //read in the next byte of the packet
    uint8_t byte = Serial1.read();

    switch (state){

      //starting byte
      case 0:
        if(byte == START_BYTE){

          state = 1; 

        }
        break;

      //length section
      case 1:
        length = byte;
        if(length != sizeof(Packet)){
          
          //packet is the wrong size
          state = 0;

        }
        else{

          index = 0;
          checksum = 0;
          state = 2;

        }
        break;
      
      //id section
      case 2:
        senderID = byte;
        state = 3;
        break;

      //data itself
      case 3:
        buf[index++] = byte;
        checksum ^= byte;

        if(index >= length){

          //whole dataset recieved
          state = 4;

        }
        break;

      //checksum section
      case 4:
        if(byte == checksum && (senderID == AUTH_ID1 || senderID == AUTH_ID2)){
          
          //successfully read
          memcpy(&rxPacket, buf, sizeof(Packet));
          newPacket = true;

        }

        //reset
        state = 0;
        index = 0;
        checksum = 0;
        break;
    }
  }
}

/*-------------------------------------------------------------------Housekeeping-------------------------------------------------------------------------------------------*/
void sleepLED(){

  //explictily disable LED
  analogWrite(redPin, 0);
  analogWrite(greenPin, 0);
  analogWrite(bluePin, 0);

}

void wakeISR(){
  
  //update the wake flag, the board has been woken from UART
  wakeFlag = true;

}

/*----------------------------------------------------------------------Loop------------------------------------------------------------------------------------------------*/
void loop() {

  //insurance for peerAvg to not mess up local belief too bad
  peerAvg = 0;

  //check for neighbor communication
  checkPacket();

  //check if there is a new packet to recieve
  if(newPacket){
    //generate a belief of peer state
    if(rxPacket.currentState == 0){

      peerActive = false; 
      peerAvg = 0;

    }
    else{

      peerActive = true;
      peerAvg = getBelief();

    }

    //reset the flag
    newPacket = false; 

  }

  //read the sensors, generate a local belief of state
  localAvg = readSensors();

  //update the state
  updateState();

  //FSM
  if(SLEEP){

    //store the current communicated state
    txPacket.currentState = 0;

    //disable the LED explicitly
    sleepLED();

    //if the local sensors meet the threshold
    if(normalizedAvg > WAKEAVG_THRESHOLD){

      //update the current state
      txPacket.currentState = 1;

      //configure the current brightness
      txPacket.LEDbrightness = localAvg;

      //communicate to neighbors
      sendPacket(txPacket);

      //update sleep flag
      SLEEP = false;

    }
    else{

      //send a 'going to sleep' packet to avoid old states from playing a large impact on local belief of other nodes
      txPacket.currentState = 0;
      txPacket.LEDbrightness = 0.0f;
      sendPacket(txPacket);

      //go back to sleep
      LowPower.sleep(SLEEP_INTERVAL);

      if(wakeFlag){
        
        wakeFlag = false;
        checkPacket();

      }

      return;

    }
  }

  //node is active, check to see if it should be inactive
  if(localAvg <= WAKEAVG_THRESHOLD){

    //check for multiple lows, don't want super sensitive node
    if(belowThresholdStart == 0){

      //store time of first low
      belowThresholdStart = getTime();

    }
    //if its been long enough and the reading is still low, go back to sleep
    else if(getTime() - belowThresholdStart >= SLEEP_DELAY){

      //update the sleep flag
      SLEEP = true;

      //reset the threshold check
      belowThresholdStart = 0;

      //update current state
      txPacket.currentState = 0;

      //turn off the LED
      sleepLED();
      return;

    }
  }
  //reading was high
  else{

    //reset the low timer
    belowThresholdStart = 0;

  }

  //update stored packet state
  txPacket.currentState = 1;

  //update the stored brightness
  txPacket.LEDbrightness = normalizedAvg;

  //update the stored color, kind of a placeholder as well for now
  txPacket.RLEDcolor = baseR;
  txPacket.GLEDcolor = baseG;
  txPacket.BLEDcolor = baseB;

  //communicate with neighbors
  sendPacket(txPacket);

  //finally configure the LED
  analogWrite(redPin, baseR * normalizedAvg);
  analogWrite(greenPin, baseG * normalizedAvg);
  analogWrite(bluePin, baseB * normalizedAvg);
}
/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/