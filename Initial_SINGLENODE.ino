//RBG PINS and initalized color values
int redPin = 9;
int greenPin = 10;
int bluePin = 11;
int baseR = 0;
int baseG = 255; 
int baseB = 0;
//ULTRASONIC PINS/VARIABLES
int trigPin = 12;
int echoPin = 13;
//float distThreshold = 5; //centimeters
float MINDIST = 0; //centimeters, hardcoded test values for prototype's sake
float MAXDIST = 5.0; //centimeters, hardcoded test values for prototype's sake
boolean ultrasonicHigh; 
//PHOTORESISTOR PINS/VARIABLES
int lightPin = A0; //implicitly INPUT pin
int lightLevelADC = 0;
float MINLIGHT = .3; //hardcoded test values for prototype's sake
float MAXLIGHT = 4.5; //hardcoded test values for prototype's sake
//THERMISTOR PINS/VARIABLES
int tempPin = A1; //implicitly INPUT pin
int temperatureADC = 0;
float MINTEMP = 2.6; //hardcoded test values for prototype's sake
float MAXTEMP = 3.0; //hardcoded test values for prototype's sake
//COMMUNICATION PINS/VARIABLES
String state;

void setup() {
  Serial.begin(9600);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

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

void loop() {
  float distance = getDistanceCM();
  float lightLevel = getLightLevel();
  float temperature = getTemperature();

  /*Normalizing sensor inputs onto one numerical scale, constrain() syntax ChatGPT sourced. These will give us 3 values between 0.0 and 1.0*/
  float lightNorm = constrain((lightLevel - MINLIGHT) / (MAXLIGHT - MINLIGHT), 0.0,1.0);
  float tempNorm = constrain((temperature - MINTEMP) / (MAXTEMP - MINTEMP), 0.0, 1.0);
  float distNorm = constrain((MAXDIST - distance) / (MAXDIST - MINDIST), 0.0, 1.0);

  /*Once we have the normalized values, we can combine and adjust LED brightness accordingly, weights/coeffcients should be adjusted as needed, must add to 1.0*/
  float normalizedAvg = (.4*lightNorm + .2*tempNorm + .4*distNorm);

  /*Placehold state/communication updates*/
  if(normalizedAvg >= .5){
    state = "ON";
  }
  else{
    state ="OFF";
  }

  /*LED writing, brightness adjusted based on the normalized, weighted average of the sensors*/
  analogWrite(redPin, baseR * normalizedAvg);
  analogWrite(greenPin, baseG * normalizedAvg);
  analogWrite(bluePin, baseB * normalizedAvg);
  
  /*Debugging/placeholder for potential data saving*/
  Serial.println("Normalized Light Level: " + String(lightNorm));
  Serial.println("Normalized Temperature: " + String(tempNorm));
  Serial.println("Normalized Ultrasonic: " + String(distNorm));
  Serial.println("Normalized Average: " + String(normalizedAvg));

  //initial placeholder for wired communication output of somesort 
  Serial.println("NODE State: " + state);

  delay(50); //50 ms delay ensure sensor readings are accurate and as smooth/close to real time as possible
}