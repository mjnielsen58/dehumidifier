/************************************************************
 * Remotely Supervised Dehumidifyer System for the Basement *
 * ======================================================== *
 *                                                          *
 * Control timely Flushing of the Water-Sink                *
 * - connected to the Dehumidifier                          *
 * Meassure Humidity to Keep track of DEW-Point             *
 * External-, Ambient & Cabinet Temperature                 *
 * Cool Cabinet when overheated (> 40oC)                    *
 * Audiitive Alert, when malfunction                        *
 * Verify Pump Functionality using a Flow Meter             *
 * Control Dehumifier and Pump via 2-channel Opto Relay     *
 * Send Timestamped log data to remote unit                 *
 * **********************************************************
 * Author: Michael J. Nielsen 11/2019   rev.: 0.2           *
 * **********************************************************
 *  
 * About Wf1 Water Flow Meter
 * --------------------------
 * Water Canister can sink up to 35 liter
 * It - ideally - pump 10 liter/min to 5.5 meter hight @ 0.55 bar   
 * Estimated Drainage Duration: (35 liter / 10 liter/min = approx.: 3.5 Minutes)
 *  
 * Transition STATE Machine:
 * -------------------------
 * Valid States:      FULL > DRAINING > EMPTY
 * Exception States:  ALERT, SHUT_DOWN
 * Illegal States:    All other
 * 
 * Sw2 0 > 1  =>  Sink Filled => FULL
 *                ? Sw1 == 1, Sw3 == 0, Wf1 <= 0 ´=> Start Pump => DRAINING else ALERT
 * Wait 60 sec    Wf1 still <= 0 => ALERT
 * Wait for       Sw1 1 == 0 ==> still HIGH after 300 sec => ALERT
 *                ? Sw2 == 0, Sw3 == 0, Wf1 <= 0 ´=> EMPTY else ALERT 
 * Sw3 0 > 1      Trigger SHUT_DOWN Routine via IRQ 0 (Shut DeHumidfier and Pump)
 *  
 *******************************************************/
/*
 * INCLUDE POOL
 */
// Include Adafruit's Generic Sensor library
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <OneWire.h> 
#include <DallasTemperature.h>
#include <DHT.h>
#include <DHT_U.h>

/*
 * DEFINE STATEMENTS for PIN Assignment
 */
#define DHTPIN          A0 // Ts2 Temperature, Ambient (DTH22)
#define DHTTYPE      DHT22 // Ts2 Temperature, Ambient (DTH22) 
#define SDAPin          A4 // Rt1 DS3231 RTC Presition sdaPin
#define SCLPin          A5 // Rt2 DS3231 RTC Presition SclPin 

#define RxPin            0 // Ms1 HC-12 Serial Communication Channel 
#define TxPin            1 // Ms1 HC-12 Serial Communication Channel 
#define alertPin         2 // Sw3 Humidity Alert (trigger IRQ.0)
#define waterFlowPin     3 // Wf Water Flow Meter(YF-S201) (trigger IRQ.1)
#define oneWireBusPin    4 // Ts1 Temperature, Internal (DS18B20)
#define lowLevelPin      5 // Sw1 Low Level Float Switch
#define highLevelPin     6 // Sw2 High Leve Float Switchl
#define fan              7 // Fn1 5V-7VDC 280mA Cooling Fan
//#define xxx              8 // available
#define buzzer           9 // Bz1 Piezzo Buzzer (Low Level Value)
#define pumpRelayPin    10 // Re1 12Vdc Submersible Pump Comet Elegant
#define dehumRelayPin   11 // Re1 230Vac DeHumidifier
#define HC12SetPin      12 // Ms1 HC-12 Serial Communication Channel 

/*
 * VARIABLE POOL
 */
volatile int flowPulse; // Count the number of pulses from the Flow Meter (YF-S201)
unsigned long cTime;    // Hold Current Time in mS

bool sinkEmpty = true;  // Hold Sensor Data from EMPTY Level Sensor
bool sinkFull = false;  // Hold Sensor Data from FULL Level Sensor
int sinkState = 1;      // Hold current state of the Sink   // 0 = Empty, 1 = Filling, 2 = Full, 3 = Draining
                        //
int maxRetry = 125;     // Max number of retries to Start/Stop an Actuator (e.g. a relaycontrolled Pump
int maxHum = 60.0;      //
int maxTemp = 25.0;     // 40
int minflow = 5;        // Lowest value of Rpm for a flow to be present ()
int frqAvake = 5000;    // poll sensors once per 5 seconds 
int frqDormant = 60000; // poll sensors once per 60 seconds 
                        //
float t1 = 0;           // Ambient Temperature  Hold Sensor Data from DHT-22 Sensor
float t2 = 0;           // Ambient Temperature  Hold Sensor Data from DS18B20 Sensor
float h1 = 0;           // Ambient Humidity     Hold Sensor Data from DHT-22 Sensor
bool fanActive;         // Hold current state of the Fan
bool pumpActive;        // Hold current state of the Pump
bool flowActive;        // Hold current state of the Waterflow from Draining the Sink
bool dehumActive;       // Hold current state of the DeHumiditifier

/*
 * <INSTANTIATE OBJECTS>
 */
  OneWire oneWire(oneWireBusPin); 
  DallasTemperature sensors(&oneWire);  // Passes the Address of the oneWire instance
  DHT dht(DHTPIN, DHTTYPE);

/*
 * PROGRAMMING LOGIC
 */

void  playATune(int tune){
  if (tune == 1){
      tone (buzzer, 2000);
      noTone(buzzer);
      delay(100);
      tone (buzzer, 3000);
      noTone(buzzer);
      delay(100);
      tone (buzzer, 4000);
      noTone(buzzer);
      delay(100);
   }
}

void shutDown(){  //ToDo
  // Entering Emergency SHUT_DOWN mode
  // Cut the Power @ relay 3
}

void waterFlow(){
  // Count the Pulses from the Water Flow Meter(YF-S201)
  flowPulse++;
}

void  alert(int alarmCode){
  switch (alarmCode) {
    case 1:
      // Failed to start the DeHumidifyer :(
      Serial.println("ERROR! - - - Failed to start the DeHumidifyer");
      shutDown();
      break;
    case 2:
      // Failed to stop the DeHumidifyer :(
      Serial.println("ERROR! - - - Failed to stop the DeHumidifyer");
      shutDown();
      break;
    case 3:
      // Failed to start the pump :(
      Serial.println("ERROR! - - - Failed to start the pump");
      shutDown();
      break;
    case 4:
      // Failed to detect Water Flow after Starting the Pump"
      Serial.println("ERROR! - - - Failed to detect water flow");
      shutDown();
      break;
    case 5:
      // Failed to stop the pump :(
      Serial.println("ERROR! - - - Failed to stop the pump");
      shutDown();
      break;
    case 6:
      // Detected Water Flow after Stopping the Pump"
      Serial.println("ERROR! - - - Detected water flow");
      shutDown();
      break;    
    case 7:
      // Illegal STATE of the SINK, not EMPTY, NOT FILLING and NOT FULL (?)
      Serial.println("ERROR! - - - Level Indicators are NOT reliable!");

      if (sinkFull){
        Serial.print("----------------- SINK is FULL -----------------");  
      }else{
        Serial.print("----------------- SINK is NOT FULL -----------------");  
      }
      
      if (sinkEmpty){
        Serial.print("----------------- SINK is EMPTY -----------------");  
      }else{
        Serial.print("----------------- SINK is NOT EMPTY -----------------");  
      }

      shutDown();
      break;
    default:
      Serial.println("ERROR! - - - UnExpected Incident");
      shutDown();
  }
}

/*
 * HANDLE ACTUATORS
 */

bool  detectFlow(){
  /************************************************************************** 
   *  Detect if Water is Flowing, using interrupt IRQ.1
   *  Sensor the Water Hall Effect Flow Meter(YF-S201)
   *  Pulses per Liter: 450
   *  Liter per Pulse: 2.2 mL
   *  Max Flow: 7.5 L/min = 125 mL/Sec
   *  Max Frq:  = 56.25 Hz(+/- 3%)
   **************************************************************************/
  flowPulse = 0;    // initialize the Pulse Counter

  // Activate IRQ.1 and Start Counting
  attachInterrupt(digitalPinToInterrupt(waterFlowPin), waterFlow, RISING);

  // Wait for 5 Sec - does not use any counter, so it will work as normal
  delayMicroseconds(5000);

  // DeActivate IRQ.1 and Stop Counting
  detachInterrupt(digitalPinToInterrupt(waterFlowPin));
  
  if (flowPulse > 4){
    return true;  
  }else{
    return false;
  }  
}

void  startPump(){
  int cnt = maxRetry;
  if (pumpRelayPin == HIGH){
    Serial.println("WARNING: Trying to START an ACTIVE Pump?");
  }else{
    // Turn ON Pump by Activating Relay1
    while (digitalRead(dehumRelayPin) == LOW){
      digitalWrite(pumpRelayPin, HIGH);    
      delay(1000);  
        
      // Has PumpRelay been switched ON?
      if (digitalRead(pumpRelayPin) == LOW){
        Serial.println("<--- PUMP Relay1 has NOT been Switched ON --->");
        alert(3);
      }else{
        // Do we detect flow from the Sink?
        while (!detectFlow()) {
          if (cnt-- < 1){
            alert(4);
          }
        }
      }
    }
  } 
}

void  stopPump(){
  int cnt = maxRetry;
  if (pumpRelayPin == LOW){
    Serial.println("WARNING: Trying to STOP a TURNED OFF Pump?");
  }else{
    // Turn OFF Pump by DeActivating Relay1
    while (digitalRead(dehumRelayPin) == HIGH){
      digitalWrite(pumpRelayPin, LOW);    
      delay(1000);  

      // Has PumpRelay been switched OFF?
      if (digitalRead(pumpRelayPin) == HIGH){
        Serial.println("<--- PUMP Relay1 has NOT been Switched OFF --->");
        alert(5);
      }else{
        // Can we detect flow from the Sink?
        while (detectFlow()) {
          if (cnt-- < 1){
            alert(6);  
          }
        }
      }
    }
  } 
}

void  startDehumid(){
  int cnt = maxRetry;
  if (dehumRelayPin == HIGH){
    Serial.println("WARNING: Trying to START an ACTIVE DeHumidifyer?");
  }else{
    // Turn ON DeHumidifyer by Activating Relay2
    while (digitalRead(dehumRelayPin) == LOW){
      digitalWrite(dehumRelayPin, HIGH);    
      delay(1000);  
        
      if (digitalRead(dehumRelayPin) == HIGH){
        dehumActive = true;
        Serial.println("<--- DeHumidifyer Relay2 has been Switched ON --->");
      } else if (cnt-- < 1){
          alert(1);
      }
    }
  }
}

void  stopDehumid(){
  int cnt = maxRetry;
  if (dehumRelayPin == LOW){
    Serial.println("WARNING: Trying to STOP a TURNED OFF DeHumidifyer?");
  }else{
    // Turn OFF DeHumidifyer by DeActivating Relay2
    while (digitalRead(dehumRelayPin) == HIGH){
      digitalWrite(dehumRelayPin, LOW);    
      delay(1000);  
        
      if (digitalRead(dehumRelayPin) == LOW){
        dehumActive = false;
        Serial.println("<--- DeHumidifyer Relay2 has been Switched OFF --->");
      } else if (cnt-- < 1){
          alert(2);
      }
    }
  }
}

/*
 * READ SENSORS
 */

void  getWaterLevelData(){
  if (digitalRead(lowLevelPin) > 0){
    sinkEmpty = false;
    Serial.println("<--- SINK is ABOVE LOW LEVEL --->");
  }else{
    sinkEmpty = true;
    Serial.println("<--- SINK is BELOW LOW LEVEL --->");
}
  
  if (digitalRead(highLevelPin) > 0){
    sinkFull = true;
    Serial.println("<--- SINK is ABOVE HIGH LEVEL --->");
  }else{
    sinkFull = false;
    Serial.println("<--- SINK is BELOW HIGH LEVEL --->");
  }
}

void  getAmbientClimateData(){
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  h1 = dht.readHumidity();     // Read temperature as Celsius
  t1 = dht.readTemperature();  // Check if any reads failed and exit early (to try again).
  t2 = sensors.getTempCByIndex(0);
  sensors.requestTemperatures(); // Send the command to get temperature readings 
}

void  validateAmbientClimateData(){
  if (isnan(h1) || isnan(t1)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  if(h1 > maxHum || t1 > maxTemp || t2 > maxTemp) {
    if (!fanActive){
      digitalWrite(fan, HIGH);
      Serial.println("----------------- Switching Fan ON -----------------");
      fanActive = true;
    }
  } else {
    if (fanActive){
      digitalWrite(fan, LOW); 
      Serial.println("----------------- Switching Fan OFF -----------------");
      fanActive = false;
    }
  }
}

void  validateWaterLevelData(){
  if (sinkEmpty && !sinkFull){
    sinkState = 0;  // SINK is Empty
  } else if (!sinkEmpty && !sinkFull && !pumpActive){
    sinkState = 1;  // SINK is Filling
  } else if (!sinkEmpty && sinkFull){
    sinkState = 2;  // SINK is Full
  } else if (!sinkEmpty && !sinkFull && pumpActive){     
    sinkState = 3;  // SINK is Draining
  } else{
    sinkState = 99;  // Illegal State
  }
}

void  putWaterLevelData(){
  switch (sinkState) {
    case 0:
      Serial.println("----------------- SINK is EMPTY -----------------");
      break;
    case 1:
      Serial.println("----------------- SINK is FILLING ---------------");
      break;
    case 2:
      Serial.println("----------------- SINK is FULL ------------------");
      break;
    case 3:
      Serial.println("----------------- SINK is DRAINING --------------");
      break;
    default:
      Serial.println("WARNING! - - - Unknown ERROR");
      shutDown();
      break;
  }  
}

void  putAmbientClimateData(){
  Serial.print("Humidity: "); 
  Serial.print(h1);
  Serial.print(" %\t");
  Serial.print("DHT-22 Temperature: "); 
  Serial.print(t1);
  Serial.print(" *C ");
  Serial.print("DS18B20 Temperature is: "); 
  Serial.print(t2);
  Serial.println(" *C ");
}

void  drain(){
  
}

/*  
 * * Sw2 0 > 1  =>  Sink Filled => FULL
 *                ? Sw1 == 1, Sw3 == 0, Wf1 <= 0 ´=> Start Pump => DRAINING else ALERT
 * Wait 60 sec    Wf1 still <= 0 => ALERT
 * Wait for       Sw1 1 == 0 ==> still HIGH after 300 sec => ALERT
 *                ? Sw2 == 0, Sw3 == 0, Wf1 <= 0 ´=> EMPTY else ALERT 
 */

void setup() {
  pinMode(lowLevelPin, INPUT);       // Sw1 Low Level Float Switch
  pinMode(highLevelPin, INPUT);      // Sw2 High Leve Float Switchl
  pinMode(DHTPIN, INPUT);            // Ts2 Temperature, Ambient (DTH22)
  pinMode(oneWireBusPin, INPUT);     // Ts1 Temperature, Internal (DS18B20)
  pinMode(RxPin, INPUT);             // Ms1 HC-12 Serial Communication Channel 
  pinMode(SCLPin, INPUT);            // Rt2 DS3231 RTC Presition SclPin 
  pinMode(alertPin, INPUT);          // Sw1 Soil Humidity Break Switch (trigger INT.0)
  pinMode(waterFlowPin, INPUT);      // Wf1 Water Flow Meter
 
  pinMode(fan, OUTPUT);              // Fn1 5V-7VDC 280mA Cooling Fan
  pinMode(buzzer, OUTPUT);           // Bz1 Piezzo Buzzer
  pinMode(pumpRelayPin, OUTPUT);     // Re1 12Vdc Submersible Pump Comet Elegant
  pinMode(dehumRelayPin, OUTPUT);    // Re1 230Vac DeHumidifier
  pinMode(TxPin, OUTPUT);            // Ms1 HC-12 Serial Communication Channel 
  pinMode(HC12SetPin, OUTPUT);       // Ms1 HC-12 Serial Communication Channel 
  pinMode(SDAPin, OUTPUT);           // Rt1 DS3231 RTC Presition sdaPin

  // Initialization
  Serial.begin(9600); 
  dht.begin();

  attachInterrupt(digitalPinToInterrupt(alertPin), shutDown, FALLING);
  // sei();    // Enable interrupts  
  sensors.begin(); 

  // Activate 230Vac DeHumidifier   
  digitalWrite(highLevelPin, HIGH);
  Serial.println("----------------- Switching DeHumidifier ON -----------------");
  
  // DeActivate Dykpumpe Comet Elegant
  digitalWrite(lowLevelPin, LOW);  
  Serial.println("----------------- Switching Pump OFF -----------------");
  
  // DeActivate the Cooling Fan 
  fanActive = false;
  Serial.println("----------------- Switching Fan OFF -----------------");  

  playATune(1);
}

void loop() {

  getWaterLevelData();
  validateWaterLevelData();
  putWaterLevelData();
  
  getAmbientClimateData();
  validateAmbientClimateData();
  putAmbientClimateData();

  // Wait for approx. 5 minutes.
  delay(1000);                    // delay(frqDormant);
}