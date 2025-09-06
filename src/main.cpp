#include <millisDelay.h>
#include <Arduino.h>

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

#define DISPENSEPIN 12
#define REWINDPIN 13

enum MotorDirection
{
  Forward,
  Backward,
  Off
};

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
  // initialize the LED pin as an output:
  pinMode(LEDPIN, OUTPUT);

  // initialize the sensor pin as an input:
  pinMode(SENSORPIN, INPUT);
  pinMode(FORWARDPINA, OUTPUT);
  pinMode(FORWARDPINB, OUTPUT);
  pinMode(BACKWARDPINA, OUTPUT);
  pinMode(BACKWARDPINB, OUTPUT);
  pinMode(DISPENSEPIN, INPUT_PULLUP);
  pinMode(REWINDPIN, INPUT_PULLUP);

  digitalWrite(SENSORPIN, HIGH); // turn on the pullup

  Serial.begin(9600);
}

void loop()
{
  // read the state of the pushbutton value:
  sensorState = digitalRead(SENSORPIN);

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
    enableRelay(MotorDirection::Forward);
  }
  else if (motorDirection == Backward)
  {
    // Dispense pin not actively depressed, disable motor
    Serial.println("Dispense pin off, motor off");
    enableRelay(MotorDirection::Off);
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
