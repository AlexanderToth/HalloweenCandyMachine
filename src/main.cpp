#include <millisDelay.h>
#include <Arduino.h>
#include <stdlib.h>
/*
  IR Breakbeam sensor demo!
*/

#define LEDPIN 13
// Pin 13: Arduino has an LED connected on pin 13
// Pin 11: Teensy 2.0 has the LED on pin 11
// Pin  6: Teensy++ 2.0 has the LED on pin 6
// Pin 13: Teensy 3.0 has the LED on pin 13

#define SENSORPIN 5

#define FORWARDPINA 8
#define FORWARDPINB 9
#define BACKWARDPINA 10
#define BACKWARDPINB 11
//#define FLICKERINGLIGHTPIN 3
#define SOUNDEFFECTPIN 3

#define DISPENSEPIN 12
#define REWINDPIN 13

enum MotorDirection
{
  Forward,
  Backward,
  Off
};

millisDelay flickeringLightWaiting;
millisDelay flickeringLightOn;
millisDelay soundEffectDelay;
millisDelay soundMachineRunning;

// variables will change:
int sensorState = 0, lastState = 0, motorDirection = MotorDirection::Off; // variable for reading the pushbutton status

/// @brief Enables the relay pair for the desired direction
/// @param direction Direction to go
void enableRelay(MotorDirection direction)
{
  if (direction == Forward)
  {
    Serial.println("Forward");
    digitalWrite(BACKWARDPINA, LOW);
    digitalWrite(BACKWARDPINB, LOW);
    digitalWrite(FORWARDPINA, HIGH);
    digitalWrite(FORWARDPINB, HIGH);
  }
  else if (direction == Backward)
  {
    Serial.println("Backward");
    if (motorDirection != direction) {
      digitalWrite(FORWARDPINA, LOW);
      digitalWrite(FORWARDPINB, LOW);
      digitalWrite(BACKWARDPINA, HIGH);
      digitalWrite(BACKWARDPINB, HIGH);
    }
  }
  else if (direction == Off)
  {
    Serial.println("Off");
    digitalWrite(FORWARDPINA, LOW);
    digitalWrite(FORWARDPINB, LOW);
    digitalWrite(BACKWARDPINA, LOW);
    digitalWrite(BACKWARDPINB, LOW);
  }
  else
  {
    Serial.println("Invalid option " + direction);
  }
  motorDirection = direction;
}

void setup()
{
  randomSeed(analogRead(0));

  // initialize the LED pin as an output:
  pinMode(LEDPIN, OUTPUT);

  // initialize the sensor pin as an input:
  pinMode(SENSORPIN, INPUT);
  pinMode(FORWARDPINA, OUTPUT);
  pinMode(FORWARDPINB, OUTPUT);
  pinMode(BACKWARDPINA, OUTPUT);
  pinMode(BACKWARDPINB, OUTPUT);
  //pinMode(FLICKERINGLIGHTPIN, OUTPUT);
  pinMode(SOUNDEFFECTPIN, OUTPUT);
  pinMode(DISPENSEPIN, INPUT_PULLUP);
  pinMode(REWINDPIN, INPUT_PULLUP);

  digitalWrite(SENSORPIN, HIGH); // turn on the pullup

  Serial.begin(9600);
}

void loop()
{
  //Serial.print(flo); Serial.print(" = On Finished; Light wait "); Serial.println(flw);
  // if (flickeringLightOn.justFinished() || (!flickeringLightOn.isRunning() && !flickeringLightWaiting.isRunning())) { 
  //   Serial.println("Start waiting for flicker ");
  //   //flickeringLightWaiting.start(random(1, 3) * 500);
  //   digitalWrite(FLICKERINGLIGHTPIN, LOW);
  // }
  // if (flickeringLightWaiting.justFinished()) {
  //   Serial.println("Flicker on");
  //   //Serial.println(flickeringLightWaiting.isRunning());
  //   digitalWrite(FLICKERINGLIGHTPIN, HIGH);
  //   flickeringLightOn.start(random(1,5) * 500);//5 * random(0, 1) * 0.5 * 1000);
  // }

  // read the state of the pushbutton value:
  sensorState = digitalRead(SENSORPIN);

  if (soundMachineRunning.justFinished()) {
     Serial.println("Sound machine done");
  }

  if (digitalRead(REWINDPIN) == LOW)
  {
    // Rewind requested
    Serial.println("Rewind down");
    enableRelay(MotorDirection::Backward);
  }
  else if (digitalRead(DISPENSEPIN) == LOW)
  {
    // Begin dispensing
    Serial.println("Begin dispensing");
    Serial.println(soundMachineRunning.isRunning());
    if (!soundMachineRunning.isRunning()) {
      // Wait a 1.5 seconds for the sound machine then start dispensing
      digitalWrite(SOUNDEFFECTPIN, HIGH);
      soundMachineRunning.start(18 * 1000);
      soundEffectDelay.start(2500);
    }
    else if (!soundEffectDelay.isRunning()) {
      // Sound is playing already and we're not delaying, just dispense
      enableRelay(MotorDirection::Forward);
    }
  }
  else if (motorDirection == Backward)
  {
    // Dispense pin not actively depressed, disable motor
    Serial.println("Dispense pin off, motor off");
    enableRelay(MotorDirection::Off);
    soundEffectDelay.stop();
  }

  if (soundEffectDelay.justFinished()) {
    digitalWrite(SOUNDEFFECTPIN, LOW);
    enableRelay(MotorDirection::Forward);
  }

  // check if the sensor beam is broken
  // if it is, the sensorState is LOW:
  if (sensorState == LOW)
  {
    // turn LED on:
    digitalWrite(LEDPIN, HIGH);
    if (motorDirection == Forward)
    {
      Serial.println("Motor off");
      enableRelay(MotorDirection::Off);
    }
  }
  else
  {
    // turn LED off:
    digitalWrite(LEDPIN, LOW);
  }

  if (sensorState && !lastState)
  {
    Serial.println("Unbroken");
  }
  if (!sensorState && lastState)
  {
    Serial.println("Broken");
  }
  lastState = sensorState;
}
