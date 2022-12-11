#include <Arduino.h>
#include "Keypad.h"
//#include "avr/sleep.h"

//#define DEBUG

#define MAXTIMERUN 2000
#define CURRENTCOUNTER 20

#define currentDelta 185
#define offset       2500

#define ATO_SENSOR   2
#define CH1L_pin     6
#define CH1R_pin     5

#define REDLED       3
#define GREENLED     4

#define CURRENTPIN  A0

#define MAXCURRENT 0.8
#define VOLTAGE 5000.0



uint8_t run = 1;
uint8_t oldrun = 1;
uint8_t emergencyStop = 0;
uint32_t analog= 0;
double voltage = 0;
double current = 0;
double average = 0;
uint8_t i = 0;
unsigned long motorStartTime = 0;

//keypad
#define ROWS 1
#define COLS 4

uint8_t keys[ROWS][COLS] = {
  {'2','1','4','3'}
};

uint8_t rowPins[ROWS] = {A1};
uint8_t colPins[COLS] = {A2, A3, A4, A5};
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS); 

void startM() {
  if (emergencyStop != 1 ) {
      motorStartTime = millis();
      digitalWrite(GREENLED, HIGH);    
      digitalWrite(CH1L_pin, HIGH);
  }
}

void stopM() {
      digitalWrite(GREENLED,LOW);
      digitalWrite(CH1L_pin, LOW);      
}

void interuptFnc() {
  //sleep_disable();
  run = digitalRead(ATO_SENSOR);
}


void keypadEvent(KeypadEvent key){
    switch (keypad.getState()){
    case PRESSED:
        if (key == '1') {
            digitalWrite(GREENLED,HIGH);
            startM();
        }
        if (key == '3') {
            digitalWrite(REDLED,HIGH);
            stopM();
            emergencyStop = 1;
        }        
        break;

    case RELEASED:
        if (key == '1') {
            digitalWrite(GREENLED,LOW);
            stopM();
        }    
        break;

    case HOLD: 
        if (key == '4') {
          if (emergencyStop == 1) {
            digitalWrite(REDLED,LOW);
            emergencyStop = 0;
          }            
        }
        break;
    }
}

void setup() {
#ifdef DEBUG
  Serial.begin(9600);
#endif

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(CURRENTPIN, INPUT);
  pinMode(CH1L_pin, OUTPUT);
  pinMode(CH1R_pin , OUTPUT);
  pinMode(GREENLED , OUTPUT);
  pinMode(REDLED, OUTPUT);
  pinMode(ATO_SENSOR,INPUT_PULLUP);
  digitalWrite(CH1R_pin , LOW);
  digitalWrite(CH1L_pin , LOW);
  digitalWrite(GREENLED,HIGH);
  digitalWrite(REDLED,HIGH);

  attachInterrupt(digitalPinToInterrupt(ATO_SENSOR), interuptFnc, CHANGE);
  delay(500);
  digitalWrite(GREENLED,LOW);
  digitalWrite(REDLED,LOW);
  keypad.addEventListener(keypadEvent);
}
/*
void going_to_sleep() {
  sleep_enable();
  set_sleep_mode(SLEEP_MODE_PWR_SAVE);
  delay(100);
  sleep_cpu();
}
*/

void loop() {
  char customKey = keypad.getKey();
  
  #ifdef DEBUG
  if (customKey){
    Serial.println(customKey);
  }
  #endif

  //motor
  if (oldrun != run) {
    if ( run == 0 ) {
        delay(250);
        startM();
    } else {
        stopM();
    }
    oldrun = run;
  }
  
  //motor timeout
  if ((run == 0) && (millis() - motorStartTime) > MAXTIMERUN) {
    #ifdef DEBUG
      Serial.println("Time STOP");
    #endif
    run = 1;
    oldrun = 1;
    stopM();
  }

  //measure currrent
  if (run == 0) {
    i++;
    analog += analogRead(CURRENTPIN);
    delay(10);
  }
  if ((i >= CURRENTCOUNTER )) {      
    average = (double)analog/i;
    voltage = (average * VOLTAGE) / 1023.0;
    current = (voltage - offset) / currentDelta;
    #ifdef DEBUG
    Serial.print("Proud: ");
    Serial.print(current,3);
    Serial.println(" A");
    #endif
    if ( abs(current) > MAXCURRENT ) {
      stopM();
      emergencyStop = 1;
      digitalWrite(REDLED,HIGH);
    }
    analog = 0;
    voltage = 0;
    current = 0;
    average = 0;
    i = 0;
  }
/*
  if (run == 1) {
    going_to_sleep();
  }
*/
}
