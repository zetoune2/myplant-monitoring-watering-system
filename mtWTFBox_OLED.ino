
/* Sketch to control a full monitoring and watering stations that generate cvs log files and control
/ a programmable 4 x multiplug box. The logs are wirelessly sent to a central control and command system
/ using a RF2.4Ghz extension. Build for compatibility with MyFunnyWardrobe Schema version 1.0
/ Features: humidity and temperature sensor, soil moisture sensor, leak sensor, ultrasound sensor to control water
/ in the reserve water tank + 4 x motor shield to control the WTFBox+ transmitter 2.4Ghz
/ Copyright (c) 2020, Zetoune Corp
/ All rights reserved.
*/

#include <stdio.h>
#include <EEPROM.h>

#include <DS1302.h> // Clock librairy

#include "HX711.h"

#include<RF24.h> // NRFs24 library
#define PIN_CE  9
#define PIN_CSN 10

#include "DHT.h" // DHT sensor Librairy
#define DHTTYPE DHT11     // DHT 11
#define DHTPIN A3    // Arduino pin on which DHT is connected to

#include <Wire.h>
#include <Adafruit_SSD1306.h> //OLED screen librairies


#define OLED_RESET 4
Adafruit_SSD1306 display( OLED_RESET );


namespace{

  /*************************/
  /*CONSTANTS INITILISATION*/
  /*************************/

  //ARDUINO CARD NAME
  const int arduinoName=3;


  //TIMERS INITIALISATION
  const int DELAY_SENSOR_MEASURE = 250;

  const int TIMER = 1000;

  //Watering Time Variables
  const int WATERING_TIMESLOT_SEC = 120;
  const int WATERING_WAITING_TIME = 30000;

  //Waiting time between 2 loops
  const int WAITING_TIME = 3000;


  //NUMBER OF MEASURE TO AVERAGE ON
  const int NBR_MEASURE = 5;


  //CONSTANT TO MONITOR PLANT HEALTH - REQUIRED TO BE TUNED
  const int MIN_HUMIDITY = 40;
  const int MAX_HUMIDITY = 55;

  const int MIN_TEMPERATURE = 17;
  const int MAX_TEMPERATURE = 27;

  const int MIN_MOISTURE = 350;
  const int MAX_MOISTURE = 567;
  const int MOISTURE_WATERING_TRIGGER = 450;

  const float MIN_WEIGHT = 7;
  const float MAX_WEIGHT = 12;
  const float WEIGHT_WATERING_TRIGGER = 7;

  const int TANK_HEIGHT = 20;
  const int FULL_TANK_HEIGHT = 4;
  const int EMPTY_TANK_TRIGGER = 17;

  const int LEAK_DETECTION_TRIGGER = 100;

  /*********************/
  /*CLOCK INITILISATION*/
  /*********************/
  //SET DIGITAL I/O PIN CONNECTIONS FOR DS1302 RTC CLOCK
  const int kCePin   = 5;  // Chip Enable
  const int kIoPin   = 6;  // Input/Output
  const int kSclkPin = 7;  // Serial Clock

  // Create a DS1302 clock object.
  DS1302 rtc(kCePin, kIoPin, kSclkPin);

  Time last_watering_time = rtc.time();

  /*********************/
  /*SCALE INITILISATION*/
  /*********************/
  HX711 scale;
  const int SCALE_DOUT = 22;
  const int SCALE_CLK = 23;
  const float SCALE_CALIBRATION_FACTOR = -22500;

  /*************************************/
  /*NR24F GHZ COMM MODULE INITILISATION*/
  /*************************************/
  RF24 radio(PIN_CE, PIN_CSN) ;  // ce, csn pins


  /***********************/
  /*SENSORS INITILISATION*/
  /***********************/
  //HUMIDITY & TEMPERATURE SENSOR
  DHT dht(DHTPIN, DHTTYPE);

  //LEAK SENSOR
  int leakSensorPin = A1;
  int leakSensorPower = 42;

  //SOIL MOISTURE SENSOR
  int soilMoistureSensorPin = A7;
  int soilMoistureSensorPower = 50;

  //INITIALISATION OF THE UTRASOUND SENSOR
  int ultraSoundTrigPin = 39;
  int ultraSoundEchoPin = 36;


  /**********************/
  /*LEDS INITILISATION*/
  /**********************/
  int redLedPin = 3;
  int greenLedPin = 4;
  int blueLedPin = 2;


  /**********************/
  /*BUTTON INITILISATION*/
  /**********************/
  int buttonForceWaterTrigger = 14;


  /************************/
  /*WTF PLUG INITILISATION*/
  /************************/
  int WTFPlug1Pin = 41; //WaterPump1
  int WTFPlug2Pin = 35; //Extractor
  int WTFPlug3Pin = 34; //Humidificator
  int WTFPlug4Pin = 40; //Fan

}


/***************************/
/* COMMUNICATION FUNCTIONS */
/***************************/

//Function to return as cvs formatted line the measurement output4void sendAsCvs(Time t, Time t2, float temp, float hum, int leakValue, int soilMoisture,int dummy1,int dummy2,float weight, char* situation)
void sendAsCvs(Time t, Time t2, float temp, float hum, int leakValue, int soilMoisture,int waterReserveFullness,int dummy2,float weight, char* situation)
{
  char buf[160];
  char humstr[6];
  char tempstr[6];
  char weightstr[6];

  dtostrf(hum,2,2,humstr);
  dtostrf(temp,2,2,tempstr);
  dtostrf(weight,2,2,weightstr);

  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d#%04d-%02d-%02d %02d:%02d:%02d#%s#%s#%d#%d#%d#%d#%s#%s#%01d", t.yr, t.mon, t.date, t.hr, t.min, t.sec, t2.yr, t2.mon, t2.date, t2.hr, t2.min, t2.sec, tempstr, humstr, leakValue, soilMoisture, waterReserveFullness, dummy2, weightstr, situation, arduinoName);
  Serial.println("");
  Serial.println(buf);
  Serial.println("");


  //DISPLAY ON THE OLED SCREEN
  display.setTextSize( 1 );
  display.setTextColor( WHITE );
  display.clearDisplay();

  display.setCursor( 0, 0 );
  display.println(situation);
  display.print( "Moi:" );display.print( soilMoisture );display.print( " / " );display.print( "Lea:" );display.println( leakValue );
  display.print( "Tem:" );display.print( temp );display.print( " / " );display.print( "Hum:" );display.println( hum );
  display.print( "WRs:" );display.print( waterReserveFullness );display.print( "cm" );display.print( " / " );display.print( "Wei:" );display.println( weight);display.println( "kg" );
  display.display();
}


/***********************/
/* OLED SCREEN DISPLAY */
/***********************/

void afficheOledScreen(float temp, float hum, int leakValue, int soilMoisture, int waterReserveFullness, float weight, char* situation)
{
  display.setTextSize( 1 );
  display.setTextColor( WHITE );
  display.clearDisplay();

  display.setCursor( 0, 0 );
  display.println(situation);
  display.print( "Moi:" );display.print( soilMoisture );display.print("%");display.print( " / " );display.print( "Lea:" );display.println( leakValue );
  display.print( "Tem:" );display.print( temp );display.print( " / " );display.print( "Hum:" );display.println( hum );
  display.print( "WRs:" );display.print( waterReserveFullness );display.print( "%" );display.print( " / " );display.print( "Wei:" );display.println( weight);display.println( "kg" );

  display.display();
}


/*************************/
/* READ SENSOR FUNCTIONS */
/*************************/

//FUNCTION TO RETRIEVE THE SOIL MOISTURE VALUE (RETURN %)
int readSoilMoistureValue()
{
  int i = 0;
  int val = 0;

  while (i < NBR_MEASURE)
  {
    digitalWrite(soilMoistureSensorPower, HIGH);//turn Sensor "On"
    delay(DELAY_SENSOR_MEASURE);
    val = val + analogRead(soilMoistureSensorPin);//Read the SIG value form sensor
    digitalWrite(soilMoistureSensorPower, LOW);//turn Sensor "Off"
    i = i + 1;
  }

  return (val/NBR_MEASURE);

}


//FUNCTION TO CHECK IF LEAK IN THE SAFETY RECIPIENT
int readLeakWaterLevel()
{
  int i = 0;
  int val = 0;

  while (i < NBR_MEASURE)
  {
    digitalWrite(leakSensorPower, HIGH);//turn Sensor "On"
    delay(DELAY_SENSOR_MEASURE);
    val = val + analogRead(leakSensorPin);//Read the SIG value form sensor
    digitalWrite(leakSensorPower, LOW);//turn Sensor "Off"
    i = i + 1;
  }
  return val/NBR_MEASURE;
}


//FUNCTION TO READ THE WEIGHT APPLIED ON THE SCALE (IN KG)
float readScaleValue()
{
  scale.set_scale(SCALE_CALIBRATION_FACTOR);
  float weight = scale.get_units();
  return -1 * weight;
}



//FUNCTION TO MEASURE THE HEIGHT OF THE WATER IN THE RESERVE TANK (RETURN %)
int readWaterReserveLevel() {

  long duration, distance;

   //Send the sound wave
   digitalWrite(ultraSoundTrigPin, LOW);
   delayMicroseconds(2);
   digitalWrite(ultraSoundTrigPin, HIGH);
   delayMicroseconds(10);
   digitalWrite(ultraSoundTrigPin, LOW);

   // Echo reception
  duration = pulseIn(ultraSoundEchoPin, HIGH); // Calcul de la distance
  distance = (duration/2) / 29.1;

  if (distance >= 400 || distance <= 0){
    Serial.println("Out of Scope");
    return -10000;
  }else{
   float n = (distance*100)/TANK_HEIGHT;
   return (100-n);
  }

}


/************************/
/* SETUP TIME FUNCTIONS */
/***********************/

//Function to set initially the time of the RTC clock (external arduino module)
void set_time() {

  // Initialize a new chip by turning off write protection and clearing the
  // clock halt flag. These methods needn't always be called. See the DS1302
  // datasheet for details.
  rtc.writeProtect(false);
  rtc.halt(true);

  // Make a new time object to set the date and time.
  //   Sunday, September 22, 2013 at 01:38:50.
  Time t(2020, 6, 17, 19, 20, 0, Time::kWednesday);

  // Set the time and date on the chip.
  //rtc.time(t);
  //EEPROM.put(0,rtc.time());
  rtc.halt(false);

  }


/*********************/
/* ACTIONS FUNCTIONS */
/*********************/

void leakDetectionAction()
{
   //DISPLAY LEDS INFOS
    digitalWrite(redLedPin, HIGH);
    digitalWrite(greenLedPin, LOW);
    digitalWrite(blueLedPin, LOW);

    //TAKE ACTIONS
    digitalWrite(WTFPlug1Pin,HIGH); //Stop WaterPump1
    digitalWrite(WTFPlug3Pin,HIGH); //Stop Humidificator
    digitalWrite(WTFPlug2Pin,LOW); //Start Extractor
    digitalWrite(WTFPlug4Pin,LOW); //Start Fan

}

void tankEmptyDetectionAction()
{

   //DISPLAY LEDS INFOS
    digitalWrite(redLedPin, HIGH);
    digitalWrite(greenLedPin, HIGH);
    digitalWrite(blueLedPin, HIGH);

    //TAKE ACTIONS
    digitalWrite(WTFPlug1Pin,HIGH); //Stop WaterPump1
    digitalWrite(WTFPlug3Pin,HIGH); //Stop Humidificator
    digitalWrite(WTFPlug2Pin,HIGH); //Stop Extractor
    digitalWrite(WTFPlug4Pin,HIGH); //Stop Fan

}

void weightIssueDetectionAction()
{
   //DISPLAY LEDS INFOS
    digitalWrite(redLedPin, HIGH);
    digitalWrite(greenLedPin, LOW);
    digitalWrite(blueLedPin, LOW);

    //TAKE ACTIONS
    digitalWrite(WTFPlug1Pin,HIGH); //Stop WaterPump1
    digitalWrite(WTFPlug3Pin,HIGH); //Stop Humidificator
    digitalWrite(WTFPlug2Pin,LOW); //Start Extractor
    digitalWrite(WTFPlug4Pin,LOW); //Start Fan

}


void displayTimer(int timer)
{
  int i = 1;

  //DISPLAY ON THE OLED SCREEN
  display.setTextSize( 1 );
  display.setTextColor( WHITE );
  display.clearDisplay();
  display.setCursor( 0, 0 );
  display.println("WATERING STARTING");
  display.display();
  Serial.println("WATERING STARTING");

  while (i < timer)
  {
    i = i + 1;
    Serial.print("Watering Timer: ");Serial.print(i);Serial.println(" sec");
    delay(1000);
    display.setTextSize( 1 );
    display.setTextColor( WHITE );
    display.clearDisplay();
    display.setCursor( 0, 0 );
    display.print( "Watering Timer: " );display.print( i );display.print( " sec" );
    display.display();
  }

}

void plantWateringAction()
{
  //DISPLAY LEDS INFOS
  digitalWrite(blueLedPin, HIGH);
  digitalWrite(redLedPin, LOW);
  digitalWrite(greenLedPin, LOW);

  //UPDATE LAST WATERING TIME
  last_watering_time = rtc.time();
  EEPROM.put(0,last_watering_time);

  //TAKE ACTIONS
  digitalWrite(WTFPlug1Pin,LOW); //Start WaterPump1
  displayTimer(WATERING_TIMESLOT_SEC);
  digitalWrite(WTFPlug1Pin,HIGH); //Stop WaterPump1
  delay(WATERING_WAITING_TIME);

}


void plantNormalSituationAction(float temperature, float humidity)
{

  //DISPLAY LEDS INFOS
  digitalWrite(blueLedPin, LOW);
  digitalWrite(redLedPin, LOW);
  digitalWrite(greenLedPin, HIGH);

  //TAKE ACTIONS
  digitalWrite(WTFPlug1Pin,HIGH); //Stop WaterPump1

  Time check_current_time = rtc.time();

  /*****************************/
  /* CLIMATE ADJUSTMENTS SETUP */
  /*****************************/
  if ( (temperature > MAX_TEMPERATURE) || (humidity > MAX_HUMIDITY) || (checkTimeExtractor(check_current_time.hr) == true ) )
  {
    Serial.println("IF (  (temperature > MAX_TEMPERATURE) || (humidity > MAX_HUMIDITY) || (checkTimeExtractor(check_current_time.hr) == true ) )");
    digitalWrite(WTFPlug2Pin, LOW);
  }else{
      Serial.println("ELSE (  (temperature > MAX_TEMPERATURE) || (humidity > MAX_HUMIDITY) || (checkTimeExtractor(check_current_time.hr) == true ) )");
      digitalWrite(WTFPlug2Pin,HIGH); //Stop Extractor
  }

  if (humidity < MIN_HUMIDITY)
  {
    Serial.println("humidity < MIN_HUMIDITY");
    digitalWrite(WTFPlug3Pin, LOW);
  }else{
    Serial.println("humidity >= MIN_HUMIDITY");
    digitalWrite(WTFPlug3Pin, HIGH);
  }

  if ( checkTimeFan(check_current_time.hr) == true )
  {
    Serial.println("IF ( checkTimeFan(check_current_time.hr) == true )");
    digitalWrite(WTFPlug4Pin,LOW); //Stop Fan
  }else{
    Serial.println("ELSE ( checkTimeFan(check_current_time.hr) == false)");
    digitalWrite(WTFPlug4Pin,HIGH); //Start Fan
  }


}


boolean checkTimeExtractor(int theHour)
{
  int MORNING = 7;
  int EVENING = 23;

  Serial.print("checkTimeExtractor(int theHour)");Serial.println(theHour);

  if (MORNING < theHour < EVENING)
  {
    if (theHour%2 == 0){
      Serial.println("checkTimeExtractor: RETURN TRUE");
      return true;
    }
  }
  Serial.println("checkTimeExtractor: RETURN FALSE");
  return false;

}

boolean checkTimeFan(int theHour)
{
  int MORNING = 7;
  int EVENING = 23;

  Serial.print("checkTimeFan(int theHour)");Serial.println(theHour);

  if (MORNING < theHour < EVENING)
  {
    if (theHour%3 == 0){
      Serial.println("checkTimeFan: RETURN TRUE");
      return true;
    }
  }
  Serial.println("checkTimeFan: RETURN FALSE");
  return false;

}


/*****************/
/* TESTS SYSTEMS */
/*****************/

void blinkLights() {

  digitalWrite(redLedPin, HIGH);
  delay(TIMER);
  digitalWrite(redLedPin, LOW);
  digitalWrite(greenLedPin, HIGH);
  delay(TIMER);
  digitalWrite(greenLedPin, LOW);
  digitalWrite(blueLedPin, HIGH);
  delay(TIMER);
  digitalWrite(blueLedPin, LOW);
  delay(TIMER);

}


void testOutputPlugs()
{
  digitalWrite(WTFPlug1Pin, LOW);
  delay(TIMER);
  digitalWrite(WTFPlug1Pin, HIGH);
  digitalWrite(WTFPlug2Pin, LOW);
  delay(TIMER);
  digitalWrite(WTFPlug2Pin, HIGH);
  digitalWrite(WTFPlug3Pin, LOW);
  delay(TIMER);
  digitalWrite(WTFPlug3Pin, HIGH);
  digitalWrite(WTFPlug4Pin, LOW);
  delay(TIMER);
  digitalWrite(WTFPlug4Pin, HIGH);
}


/******************/
/* SETUP FUNCTION */
/******************/

void setup() {

  /**********************/
  /*SERIAL CONSOLE SETUP*/
  /**********************/
  Serial.begin(9600);


  /***************/
  /*TIME SETUP*/
  /***************/
  set_time();
  last_watering_time = rtc.time();
  EEPROM.get(0,last_watering_time);


  /*************/
  /*SCALE SETUP*/
  /*************/
  scale.begin(SCALE_DOUT, SCALE_CLK);
  scale.set_scale();
  scale.set_offset(-507780);
  scale.set_scale(SCALE_CALIBRATION_FACTOR);


  /***************/
  /*SENSORS SETUP*/
  /***************/
  //Hum&Temp Sensor Setup
  dht.begin();

  //Water tank fullness Sensor Setup
  pinMode(ultraSoundTrigPin, OUTPUT);
  pinMode(ultraSoundEchoPin, INPUT);

  digitalWrite(ultraSoundTrigPin, LOW);


  /************/
  /*LEDS SETUP*/
  /************/
  pinMode(redLedPin, OUTPUT); //Set output to power on sensor when needed
  pinMode(greenLedPin, OUTPUT); //Set output to power on sensor when needed
  pinMode(blueLedPin, OUTPUT); //Set output to power on sensor when needed

  digitalWrite(redLedPin, LOW);
  digitalWrite(greenLedPin, LOW);
  digitalWrite(blueLedPin, LOW);


  /**************/
  /*SENSOR SETUP*/
  /**************/
  pinMode(leakSensorPin, INPUT);
  pinMode(soilMoistureSensorPin, INPUT);
  pinMode(leakSensorPower, OUTPUT);
  pinMode(soilMoistureSensorPower, OUTPUT);

  digitalWrite(leakSensorPower, LOW);
  digitalWrite(soilMoistureSensorPower, LOW);

  /**************/
  /*OUTPUT SETUP*/
  /**************/
  //WTFBox plug Pins Setup
  pinMode(WTFPlug1Pin, OUTPUT);
  pinMode(WTFPlug2Pin, OUTPUT);
  pinMode(WTFPlug3Pin, OUTPUT);
  pinMode(WTFPlug4Pin, OUTPUT);

  digitalWrite(WTFPlug1Pin, HIGH);
  digitalWrite(WTFPlug2Pin, HIGH);
  digitalWrite(WTFPlug3Pin, HIGH);
  digitalWrite(WTFPlug4Pin, HIGH);


  /**************/
  /*BUTTON SETUP*/
  /**************/
  //Initialisation of the Button to Force Water Pump Trigger
  pinMode(buttonForceWaterTrigger, INPUT_PULLUP);


  /*******************/
  /*OLED SCREEN SETUP*/
  /*******************/
  display.begin( SSD1306_SWITCHCAPVCC, 0x3C );
  display.clearDisplay();


  /******************************/
  /* TEST OUTPUT - LEDS + PLUGS */
  /******************************/
  display.setTextSize( 1 );
  display.setTextColor( WHITE );
  display.clearDisplay();
  display.setCursor( 1, 0 );

  Serial.print("SYSTEM INITIALISATION: 0% ");
  display.print( "SYSTEM INITIALISATION 0%");
  display.display();

  blinkLights();
  Serial.print(".....50%");
  display.print( ".....50%");
  display.display();

  testOutputPlugs();
  Serial.print(".....100%");
  display.print( ".....100%");
  display.display();
  delay(TIMER);

  Serial.println("\nSYSTEM STARTING\n\n");
  display.println( "\nSYSTEM STARTING\n");
  display.display();
}


/*****************/
/* LOOP FUNCTION */
/*****************/

void loop() {

  Time current_time = rtc.time();


  char situation[10];

  /*****************/
  /*SENSORS READING*/
  /*****************/
  int leakValue = readLeakWaterLevel();

  int moistureValue = readSoilMoistureValue();

  float weight = readScaleValue();

  int waterReserveLevel = readWaterReserveLevel();

  /*TEMPERATURE AND HUMIDITY MESURE THROUGH DHT11*/
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  // Stop program and return error message if sesor doesn't return any value
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("DHT11 reading error !");
    return;
  }


  /**************************/
  /*CHECK SYSTEM OPERATIONAL*/
  /**************************/
  //LEAK IS DETECTED IN THE SAFETY PLATE - //Stop WaterPump1 + Stop Humidificator + Start Extractor + Start Fan
  if (leakValue > LEAK_DETECTION_TRIGGER)
  {

    //SEND MESSAGE TO COMMAND CENTER
    snprintf(situation, sizeof(situation),"%s","LEAK");
    sendAsCvs(current_time,last_watering_time, temperature, humidity, leakValue, moistureValue, waterReserveLevel, 0, weight, situation);

    leakDetectionAction();

    return;
  }

  //WATER RESERVE LEVEL IS TOO LOW - //Stop WaterPump1 + Stop Humidificator + Stop Extractor + Stop Fan
  if ( (waterReserveLevel > (EMPTY_TANK_TRIGGER*100)/20*100) || (waterReserveLevel < (FULL_TANK_HEIGHT*100)/20 ) )
  {
    //SEND MESSAGE TO COMMAND CENTER
    snprintf(situation, sizeof(situation),"%s","TANK_ISSUE");
    sendAsCvs(current_time,last_watering_time, temperature, humidity, leakValue, moistureValue, waterReserveLevel,0,weight, situation);

    tankEmptyDetectionAction();

    return;
  }

  //WEIGHT MEASURE IS OUT OF THE VALID RANGE - //Stop WaterPump1 + Stop Humidificator + Start Extractor + Stop Fan
  if ( (weight > MAX_WEIGHT) || (weight < MIN_WEIGHT) )
  {

    //SEND MESSAGE TO COMMAND CENTER
    snprintf(situation, sizeof(situation),"%s","WEIGHT_ISSUE");
    sendAsCvs(current_time,last_watering_time, temperature, humidity, leakValue, moistureValue, waterReserveLevel,0,weight, situation);

    weightIssueDetectionAction();

    return;
  }


  /*****************************/
  /* NORMAL SITUATION DETECTED */
  /*****************************/
  if ( (MIN_HUMIDITY < humidity < MAX_HUMIDITY) && (MIN_TEMPERATURE < temperature < MAX_TEMPERATURE) && ((moistureValue < MOISTURE_WATERING_TRIGGER) || (WEIGHT_WATERING_TRIGGER < weight < MIN_WEIGHT)) )
  {
    Serial.println("( (MIN_HUMIDITY < humidity < MAX_HUMIDITY) && (MIN_TEMPERATURE < temperature < MAX_TEMPERATURE) && (moistureValue < MOISTURE_WATERING_TRIGGER) && ( < weight < ) )");

    //SEND MESSAGE TO COMMAND CENTER
    snprintf(situation, sizeof(situation),"%s","NORMAL");
    sendAsCvs(current_time,last_watering_time, temperature, humidity, leakValue, moistureValue, waterReserveLevel, 0, weight, situation);

    plantNormalSituationAction(temperature, humidity);

  }


  /**********************************/
  /* WATERING REQUIREMENT SITUATION */
  /**********************************/
  //BUTTON PRESSED -> FORCE WATERING && WATERING SITUATION DETECTED
  if ( (digitalRead(buttonForceWaterTrigger) == LOW) || ( ( moistureValue > MOISTURE_WATERING_TRIGGER) && ( weight < WEIGHT_WATERING_TRIGGER) ) )
  {
     Serial.println("(  (digitalRead(buttonForceWaterTrigger) == LOW) || ( moistureValue > MOISTURE_WATERING_TRIGGER) )");

     //SEND MESSAGE TO COMMAND CENTER
     snprintf(situation, sizeof(situation),"%s","WATERING");
     sendAsCvs(current_time,last_watering_time, temperature, humidity, leakValue, moistureValue, waterReserveLevel,0,weight, situation);

     plantWateringAction();

     return;

  }

}
