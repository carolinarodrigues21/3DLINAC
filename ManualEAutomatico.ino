// ----------------------------------------------------------------------------
// Sample Code for operating a 3D printed model of a linear accelerator.
// This sketch is part of the CERN - S'Cool LAB 3D printed LINAC project.
// More information to be found on the S'Cool LAB website: https://scoollab.web.cern.ch/linac3D
//
// This project has been tested on an Arduino UNO microcontroller board.
//
// Copyright 2021, Fabian Bernstein CERN - S'Cool LAB
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights to
// use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
// of the Software, and to permit persons to whom the Software is furnished to do
// so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// ----------------------------------------------------------------------------

// If you are building the simple model without the light barriers, simply comment the LDRAuslesen()
// function in void loop()

//Load servo library
#include <Servo.h> //Initialization of the servo library
#include <ezButton.h>
Servo relais; //Create servo instance „relais“

//Eingänge und Ausgänge
const int pot_PIN = A0; //The potentiometer is connected to analog input A0.
const int servo_PIN = 10; //The control line of the servo is connected to digital output 10.
const int taster_PIN = 12; //The pushbutton is connected to input 12.
const int yellowLED =  11; 
const int redLED =  13;
ezButton button(6);  // create ezButton object that attach to pin 7;

/* The outputs from the photoresistor module rows are connected consecutively from PIN 2,
   i.e. the first row of photoresistor modules is at PIN 2, the second at PIN 3, and so on.
   The input pin initialization takes place in the setup function in a for loop.*/


//Variable declarations
//1) Potentiometer
int potentiometer_zustand_raw = 0; //State of the potentiometer as read with analogRead(), i.e. values are between 0 and 1023.
int potentiometer_zustand = 0; /*State of the potentiometer, mapped into the value range 0 to 255.
  This is required because the value is written to the servo using the servo.write() function. Servo.Write() only takes values
  between 0 and 255 as arguments.*/

//2) Servo
boolean relais_zustand = 0; //State of the relay, where "0" indicates the starting position and "1" indicates the stop position.
int pos_start = 0; //Starting position of the servo in the value range 0 to 255.
int pos_stop = 0; // Stop position of the servo in the value range 0 to 255.

//3) Taster
const int taster_debouncing = 3; /* Specifies how often the pushbutton input is successively read out.
  All readout values must be the same in order for the value to be accepted. The multiple readout ("debouncing") is part of the electromagnetic inference suppression.*/
int taster_zeit_zwischen_auslesen = 20; //Specifies the waiting time between the readouts of the pushbutton input in milliseconds.
unsigned long taster_data[taster_debouncing + 5]; /*Array containing all data for the pusbutton.
  Data stored in one row contains [Pushbutton current state, Pushbutton last state, Pushbutton last state timestamp, Pushbutton counter,
  Button readout value 1, 2, 3, ...]. The pushbutton is read as often as defined in the constant taster_debouncing*/

//4) Fotowiderstand (LDR)
const int LDR_anzahl = 2; //Number of LDR sensor rows, each occupying its own input on the Arduino.
const int LDR_debouncing = 3; //Specifies how often the LDR is read out. All readouts must have the same result for the value to be accepted.
int LDR_zeit_zwischen_auslesen = 5; //Specifies the waiting time between the readout processes of the photoresistor modules in milliseconds.
unsigned long LDR_data[LDR_anzahl][LDR_debouncing + 5]; /*Array containing all data for the LDR sensor rows.
  The number of rows corresponds to the number of LDR sensor rows, i.e. all data for one LDR sensor row is stored in one row of the array.
  The array contains [LDR connection pin, LDR current value, LDR last value, LDR timestamp, LDR counter, LDR readout value 1, 2, 3, ...].
  The photoresistors are read out as often as defined in the constant LDR_debouncing*/


//5) Sonstiges
unsigned long zeitstempel = 0; //Stores the current value of the internal timer.


void setup()
{
  delay(500); //Security buffer to prevent problems when uploading new sketches to the Arduino.
  Serial.begin(9600);
  pinMode(yellowLED, OUTPUT);
  pinMode(redLED, OUTPUT);
  //Initialize inputs and outputs
  //Pushbutton
  taster_data[0] = taster_PIN; //Stores the pushbutton input pin to the first entry of the array.
  pinMode(taster_data[0], INPUT); //Initialize pushbutton pin as input.
  for (int taster_init = 1; taster_init < taster_debouncing + 5; taster_init++) { //Initialize the values of the pushbutton to 1 / HIGH
    taster_data[taster_init] = 1;
  }

  //Photoresistor
  for (int LDR_data_zeile = 0; LDR_data_zeile < LDR_anzahl; LDR_data_zeile++) { //Initialize array for LDRs
    LDR_data[LDR_data_zeile][0] = 2 + LDR_data_zeile; /*Defines the input pins for the LDR rows. The first sensor row is connected to pin 2,
    all others to the following pins (i.e. 3, 4, ...)*/
    for (int LDR_data_spalte = 1; LDR_data_spalte < (5 + LDR_debouncing); LDR_data_spalte++) {
      LDR_data[LDR_data_zeile][LDR_data_spalte] = 0; //All values of the array except for the pin values are pre-initialized with 0 / LOW.
    }
  }
  for (int LDR_inputs = 0; LDR_inputs < LDR_anzahl; LDR_inputs++) { //Declare the input pins for all LDR_rows as input
    pinMode(LDR_data[LDR_inputs][0], INPUT);
  }

  //Initialize the servo
  relais.attach(servo_PIN); //The control line of the servo is connected to pin servo_PIN (default: 10).
  relais.write(pos_start); //Set the initial position of the servo for calibration.

  //Calibrate start and stop position of the servo
  pos_start = KalibriereServo(); //Set start position
  pos_stop = KalibriereServo(); //Set stop position
}


void loop() {
  button.loop(); // MUST call the loop() function first

  int btnState = button.getState();
  if (btnState == 1) {
    // turn LED on:
    digitalWrite(yellowLED, HIGH);
    digitalWrite(redLED, LOW);
    Manual();
  } else {
    digitalWrite(redLED, HIGH);
    digitalWrite(yellowLED, LOW);
    Automatic();
    
  }
}

//Functions

void Manual(){
  zeitstempel = millis();
  TasterAuslesen();
}

void Automatic(){
  zeitstempel = millis();
  TasterAuslesen();
  LDRAuslesen();
}
//The function controls the readout of the pushbutton
void TasterAuslesen() {
  taster_data[1] = digitalRead(taster_PIN);
  if ((taster_data[1] != taster_data[2]) && taster_data[4] == 0) { //If the currently read value does not match the last stored value and the counter is at 0 ...
    taster_data[3] = millis(); // ... set the timestamp for the start of the readout process.
    taster_data[4] = taster_data[4] + 1; //Increment the counter by one; shows that a value has been read and stored.
    taster_data[5] = taster_data[1]; //Save the read value as the first value.
  }
  else if (taster_data[4] > 0 ) { //When the readout process has been started and ...
    if ((zeitstempel - taster_data[3]) > (taster_data[4] * taster_zeit_zwischen_auslesen)) { //... if between the start of the readout and the present time at least Taster_Counter * the time between readouts has elapsed.
      taster_data[taster_data[4] + 5] = taster_data[1]; //... write the currently read value as the nth value of the pushbutton array.
      taster_data[4] = taster_data[4] + 1; //Increment the counter by one; shows that a value has been read and stored.

      if (taster_data[4] == (taster_debouncing)) { //When as many values have been read out as specified in the variable taster-debouncing
        taster_data[4] = 0; //Reset the counter
        if (UeberpruefeTaster()) { //Check if the read data is consistent
          if (taster_data[2] == HIGH && taster_data[5] == LOW) {
            SchalteServo();
          }
          taster_data[2] = taster_data[5]; //If the data is consistent, set the current value as the last value.
        }
      }
    }
  }
}



//The function checks whether all probe values read out are identical
int UeberpruefeTaster()
{
  for (unsigned int taster_ausleseversuch = 1; taster_ausleseversuch < taster_debouncing; taster_ausleseversuch++) {
    if (taster_data[5 + taster_ausleseversuch] != taster_data[5]) {//If not all read out values match the first read out value, the data is not consistent
      return false;
    }
  }
  return true;
}

//The function controls the readout of the photoresistor modules (light barriers)
void LDRAuslesen() {
  for (int LDR_nummer = 0; LDR_nummer < LDR_anzahl; LDR_nummer++) {
    LDR_data[LDR_nummer][1] = digitalRead(LDR_data[LDR_nummer][0]); //Read the current value for each LDR and store it in the array
  }

  for (int LDR_nummer = 0; LDR_nummer < LDR_anzahl; LDR_nummer++) {
    if ((LDR_data[LDR_nummer][1] != LDR_data[LDR_nummer][2]) && LDR_data[LDR_nummer][4] == 0) { //If the currently read value does not match the last stored value and the counter is at 0
      LDR_data[LDR_nummer][3] = millis(); //Set the timestamp for the start of the readout process
      LDR_data[LDR_nummer][4] = LDR_data[LDR_nummer][4] + 1; //Increment the counter by one; shows that a value has been read and stored
      LDR_data[LDR_nummer][5] = LDR_data[LDR_nummer][1]; //Save the read value as the first value
    }
    else if (LDR_data[LDR_nummer][4] > 0 ) { //When the readout process has been started
      if ((zeitstempel - LDR_data[LDR_nummer][3]) > (LDR_data[LDR_nummer][4] * taster_zeit_zwischen_auslesen)) { //If at least n * the time has elapsed between the start of the readout operation
        LDR_data[LDR_nummer][LDR_data[LDR_nummer][4] + 5] = LDR_data[LDR_nummer][1]; //Set the nth value of the array
        LDR_data[LDR_nummer][4] = LDR_data[LDR_nummer][4] + 1; //Increment the counter by one; shows that a value has been read and stored

        if (LDR_data[LDR_nummer][4] == (LDR_debouncing)) { //When as many values have been read out as specified in the variable LDR_Debouncing
          LDR_data[LDR_nummer][4] = 0; //Reset the counter
          if (UeberpruefeLDR(LDR_nummer)) { //Check whether all readouts have produced the same result
            if (LDR_data[LDR_nummer][2] == LOW && LDR_data[LDR_nummer][5] == HIGH) { //If all readouts have produced the same result and the LDR was previously on LOW and is now on HIGH
              SchalteServo();
            }
            LDR_data[LDR_nummer][2] = LDR_data[LDR_nummer][5]; //If the data is consistent, set the current value as the last value
          }
        }
      }
    }
  }
}

//The function checks whether all read LDR values are identical
int UeberpruefeLDR(int LDR_nummer2)
{
  for (unsigned int LDR_ausleseversuch = 1; LDR_ausleseversuch < LDR_debouncing; LDR_ausleseversuch++) {
    if (LDR_data[LDR_nummer2][5 + LDR_ausleseversuch] != LDR_data[LDR_nummer2][5]) {//If not all read out values match the first read out value, the data is not consistent
      return false;
    }
  }
  return true;
}

void SchalteServo() {
  if (relais_zustand == 0)
  {
    relais.write(pos_stop); //Move servo to end position
    relais_zustand = 1; //Set state of servo to 1
    delay(150);
  }
  else
  {
    relais.write(pos_start); //Move servo to starting position
    relais_zustand = 0; //Set state of servo to 0
    delay(150);
  }
}

int KalibriereServo() {
  while (taster_data[1] == HIGH) {
    delay(50);
    potentiometer_zustand_raw = analogRead(pot_PIN); //Read out the potentiometer pin
    potentiometer_zustand = map(potentiometer_zustand_raw, 0, 1023, 0, 150); //Map the read value to the value range 0-255
    relais.write(potentiometer_zustand); //Move servo to the set position
    taster_data[1] = digitalRead(taster_PIN); //Read the pushbutton pin
  }
  int temp_pos = potentiometer_zustand; //Store the value of the potentiometer in a temporary variable.
  relais.write(potentiometer_zustand + 20); //Indicate by a small movement of the servo that the value has been saved
  delay(300);
  relais.write(temp_pos);
  taster_data[1] = HIGH; //Set the pushbutton state back to HIGH
  return temp_pos;
}
