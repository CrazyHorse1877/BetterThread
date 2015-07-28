#include <EEPROM.h>
#include <Servo.h>
#include <Time.h>
#include <TimeAlarms.h>
#include <Thread.h>

// Baud rate for bluetooth module
// (Default 9600 for most modules)
#define BAUD_RATE 9600

// Special commands
#define CMD_SPECIAL '<'
#define CMD_ALIVE   '['

// Number of relays
#define MAX_RELAYS 5
#define MAX_INPUTS 2

#define LDR_ON 1
#define SERVO_ON 1
#define BT_ON 1

//Set output for on board LED to flash
int ledPin = 13;

const int numReadings = 10;
int Direction = 1;
int setMotorSpeed = 0;
boolean speedReached = true;
int doorState = 0;
int Daylight = 0;
int drivetimer = 0;
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int LDR_Average = 0;                // the average

typedef unsigned char  u08;

volatile u08 ldr_flag = 0x01;          //LDR flags
volatile u16 us_range = 0;          //range measured by ultrasonic sensor
volatile u16 ldr_value = 0;         //value measured by ldr sensor
volatile u08 servopos = 0;          //Servo Position
volatile u08 servoflg = 0;          //Servo Flag
volatile u08 door_timer = 0;        //Door time to open/close
volatile u08 door_state = 0x01;          //Door State flag
volatile u08 door_open = 0x00;          //Door open sensor
volatile u08 door_closed = 0x00;        //Door closed sensor
volatile u08 daylight_flag = 0x01;        //Daylight detected?
volatile u08 motor_control_flag = 0x00;    //Motor control state

// Relay 1 is at pin 2, relay 2 is at pin 3 and so on.
int RelayPins[MAX_RELAYS]  = {2, 3, 4, 5};//, 6};//, 7, 10, 13};
// Relay 1 will report status to toggle button and image 1, relay 2 to button 2 and so on.
String RelayAppId[] = {"04", "05", "06", "07"};//, "08"};//, "09", "10", "11"};
// Command list (turn on - off for eachr relay)
const char CMD_ON[] = {'E', 'F', 'G', 'H'};//, 'I'};//, 'J', 'K', 'L'};
const char CMD_OFF[] = {'e', 'f', 'g', 'h'};//, 'i'};//, 'j', 'k', 'l'};

// Used to keep track of the relay status
int RelayStatus = 0;
int STATUS_EEADR = 20;

// Used to ramp up PWM
int clicks = 0;

boolean buttonLatch[] = {false, false};
int inputPin = A0;  //ANALOG INPUT PIN
int PWM_Pin = 11;
int ButtonPins[] = {A4, A5};  //Manual Override Buttons

// Data and variables received from especial command
int Accel[3] = {0, 0, 0};
int SeekBarValue[8] = {0, 0, 0, 0};//, 0, 0, 0, 0};
Servo mServo[2];

Thread pinThread = Thread();
Thread writeThread = Thread();
Thread setdriveMotor = Thread();
Thread readFromLDR =  Thread();
Thread keepCount =  Thread();
Thread pwmThread =  Thread();

void setup(){
  
  // initialize BT Serial port
  Serial.begin(BAUD_RATE);

  // Set each input as digital input + pull up resistor
  for (int i = 0; i < 2; i++) {
    pinMode(ButtonPins[i], INPUT);
    digitalWrite(ButtonPins[i], HIGH);
  }
  
  setTime(00,00,00,1,1,15); // set time to Saturday 8:29:00am Jan 1 2011
  Alarm.timerRepeat(1, Repeats);            // timer for every 1 seconds
  //Alarm.timerRepeat(5, showClock);            // timer for every 2 seconds
    
  // initialize all the readings to 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++)
    readings[thisReading] = 0;
    
	pinMode(ledPin, OUTPUT);
  //pinMode(PWM_Pin, OUTPUT);
  
  // Initialize Output PORTS
  for (int i = 0; i < MAX_RELAYS; i++) {
    pinMode(RelayPins[i], OUTPUT);
  }

if(SERVO_ON){
// Initialize servos
  mServo[0].attach(8);
  mServo[0].write(90);
  mServo[1].attach(9);
  mServo[1].write(90);
}

// Load last known status from eeprom
  RelayStatus = EEPROM.read(STATUS_EEADR);
  for (int i = 0; i < MAX_RELAYS; i++) {
    // Turn on and off according to relay status
    String stringNumber = String(i);
    
    if ((RelayStatus & (1 << i)) == 0) {
      digitalWrite(RelayPins[i], LOW);
      Serial.println("<Butn" + RelayAppId[i] + ":0");
    }
    else {
      digitalWrite(RelayPins[i], HIGH);
      Serial.println("<Butn" + RelayAppId[i] + ":1");
    }
  }

   
//SETUP THREADS
    
//  pinThread.onRun(togglePin);
//  pinThread.setInterval(500);
  
  writeThread.onRun(writeRunning);
  writeThread.setInterval(30);

  pwmThread.onRun(rampPWM);
  pwmThread.setInterval(30);

//  setdriveMotor.onRun(writeLed);
//  setdriveMotor.setInterval(1000);

  // Greet arduino total control on top of the app
  Serial.println("Chicken Time!");

  // Make the app talk in english (lang number 00, use 01 to talk in your default language)
  Serial.println("<TtoS00: Welcome to automatic chicken coop controller");
  Alarm.delay(5000);
}

void loop(){
  String sSample;
  String sSampleNo;
  int iSample;
  int appData;
  
	// checks if thread should run
	if(pinThread.shouldRun()) pinThread.run();
  if(writeThread.shouldRun()) writeThread.run();
  if(pwmThread.shouldRun()) pwmThread.run();

  if (speedReached){
    if(setMotorSpeed == 0)setMotorSpeed = 255;
    else if(setMotorSpeed == 255)setMotorSpeed = 0;
  }
}

// Sets the relay state for this example
// relay: 0 to 7 relay number
// state: 0 is off, 1 is on
void setRelayState(int relay, int state) {
  if (state == 1) {
    digitalWrite(RelayPins[relay], HIGH);           // Write ouput port
    Serial.println("<Butn" + RelayAppId[relay] + ":1"); // Feedback button state to app
    Serial.println("<Imgs" + RelayAppId[relay] + ":1"); // Set image to pressed state

    RelayStatus |= (0x01 << relay);                 // Set relay status
    EEPROM.write(STATUS_EEADR, RelayStatus);        // Save new relay status
  }
  else {
    digitalWrite(RelayPins[relay], LOW);            // Write ouput port
    Serial.println("<Butn" + RelayAppId[relay] + ":0"); // Feedback button state to app
    Serial.println("<Imgs" + RelayAppId[relay] + ":0"); // Set image to default state

    RelayStatus &= ~(0x01 << relay);                // Clear relay status
    EEPROM.write(STATUS_EEADR, RelayStatus);        // Save new relay status
  }
}

// DecodeSpecialCommand
//
// A '<' flags a special command comming from App. Use this function
// to get Accelerometer data (and other sensors in the future)
// Input:
//   None
// Output:
//   None
void DecodeSpecialCommand() {
  // Read the hole command
  String thisCommand = Readln();

  // First 5 characters will tell us the command type
  String commandType = thisCommand.substring(0, 5);

  // Next 6 characters will tell us the command data
  String commandData = thisCommand.substring(5, 11);

  if (commandType.equals("AccX:")) {
    if (commandData.charAt(0) == '-') // Negative acceleration
      Accel[0] = -commandData.substring(1, 6).toInt();
    else
      Accel[0] = commandData.substring(1, 6).toInt();

    mServo[0].write(((Accel[0] + 1000) * 9) / 100);
  }

  if (commandType.equals("AccY:")) {
    if (commandData.charAt(0) == '-') // Negative acceleration
      Accel[1] = -commandData.substring(1, 6).toInt();
    else
      Accel[1] = commandData.substring(1, 6).toInt();

    mServo[1].write(((Accel[1] + 1000) * 9) / 100);
  }

  if (commandType.equals("AccZ:")) {
    if (commandData.charAt(0) == '-') // Negative acceleration
      Accel[2] = -commandData.substring(1, 6).toInt();
    else
      Accel[2] = commandData.substring(1, 6).toInt();
  }

  if (commandType.substring(0, 3).equals("Skb")) {
    int sbNumber = commandType.charAt(3) & ~0x30;
    SeekBarValue[sbNumber] = commandData.substring(1, 6).toInt();
  }
}

// Readln
// Use this function to read a String line from Bluetooth
// returns: String message, note that this function will pause the program
//          until a hole line has been read.
String Readln() {
  char inByte = -1;
  String message = "";

  while (inByte != '\n') {
    inByte = -1;

    if (Serial.available() > 0)
      inByte = Serial.read();

    if (inByte != -1)
      message.concat(String(inByte));
  }

  return message;
}

void digitalClockDisplay(){
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.println(); 
}

void printDigits(int digits){
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

// functions to be called when an alarm triggers:
void Repeats(){
  String sTimer;
  if((motor_control_flag & 0x02) || (motor_control_flag & 0x20)) drivetimer++;
  sTimer = String(drivetimer);
  Serial.println("<Text02:Timer: " + sTimer);
  //setRelayState(2, !digitalRead(RelayPins[2]));
//  ldr_flag &= 0x01;
}

void showClock(){
  digitalClockDisplay();
  Alarm.delay(1000); // wait one second between clock display
}

void rampPWM(){
  if (!speedReached){
    if (abs(clicks - setMotorSpeed) < 5 ) speedReached = true ;
    if (clicks+5 < setMotorSpeed) analogWrite(PWM_Pin, clicks+=5);
    else if (clicks-5 > setMotorSpeed) analogWrite(PWM_Pin, clicks-=5);
  }
}

/*void fadeInPWM(){
  // fade out from max to min in increments of 5 points:
  for (int fadeValue = 255 ; fadeValue >= 0; fadeValue -= 5) {
    // sets the value (range from 0 to 255):
    analogWrite(ledPin, fadeValue);
    // wait for 30 milliseconds to see the dimming effect
//    delay(30);
  else analogWrite(PWM_Pin, (510-(clicks+=5)));
  }
}
*/
void every1sec(){
 digitalClockDisplay();
}

void readInputs(){
    // Manual buttons
  for (int i = 0; i < MAX_INPUTS; i++) {
    if (!digitalRead(ButtonPins[i])) { // If button pressed
      // don't change relay status until button has been released and pressed again
      if (buttonLatch[i]) {
        setRelayState(2, !digitalRead(RelayPins[2])); // toggle relay 0 state
        buttonLatch[i] = false;
      }
    }
    else {
      // button released, enable next push
      buttonLatch[i] = true;
    }
  }
}

void readFromApp(){
    // ===========================================================
  // This is the point were you get data from the App
  int appData;
    
  appData = Serial.read();   // Get a byte from app, if available
  switch (appData) {
    case CMD_SPECIAL:
      // Special command received
      DecodeSpecialCommand();
      analogWrite(12, SeekBarValue[0]);
      break;

    case CMD_ALIVE:
      // Character '[' is received every 2.5s, use
      // this event to tell the android all relay states
      for (int i = 0; i < MAX_RELAYS; i++) {
        // Refresh button states to app (<BtnXX:Y\n)
        if (digitalRead(RelayPins[i])) {
          Serial.println("<Butn" + RelayAppId[i] + ":1");
          Serial.println("<Imgs" + RelayAppId[i] + ":1");
        }
        else {
          Serial.println("<Butn" + RelayAppId[i] + ":0");
          Serial.println("<Imgs" + RelayAppId[i] + ":0");
        }
      }
      break;

    default:
      // If not '<' or '[' then appData may be for relays
      for (int i = 0; i < MAX_RELAYS; i++) {
        if (appData == CMD_ON[i]) {
          // Example of how to make beep alarm sound
          Serial.println("<Alrm00");
        }
        else if (appData == CMD_OFF[i]) setRelayState(i, 0);
      }
  }
  
}

void sendAnalog2BT(){
    String sSample;
    
    sSample = String(LDR_Average);
    Serial.println("<Text00:Light1: " + sSample);
    if (LDR_Average > 383) Serial.println("<Imgs00:1"); // Day state
    else if (LDR_Average > 241) Serial.println("<Imgs00:2"); // rise/set state
    else Serial.println("<Imgs00:0"); // Default state
/*
    iSample = analogRead(A2);
    sSample = String(iSample);
    Serial.println("<Text01:Speed2: " + sSample);
    if (iSample > 683)
      Serial.println("<Imgs01:1"); // Pressed state
    else if (iSample > 341)
      Serial.println("<Imgs01:2"); // Extra state
    else
      Serial.println("<Imgs01:0"); // Default state

    iSample = analogRead(A3);
    sSample = String(iSample);
    Serial.println("<Text02:Photo3: " + sSample);
    if (iSample > 683)
      Serial.println("<Imgs02:1"); // Pressed state
    else if (iSample > 341)
      Serial.println("<Imgs02:2"); // Extra state
    else
      Serial.println("<Imgs02:0"); // Default state

    iSample = analogRead(A4);
    sSample = String(iSample);
    Serial.println("<Text03:Flux4: " + sSample);
    if (iSample > 683)
      Serial.println("<Imgs03:1"); // Pressed state
    else if (iSample > 341)
      Serial.println("<Imgs03:2"); // Extra state
    else
      Serial.println("<Imgs03:0"); // Default state
 */ 
}

void updateAnalog(){

    if ((ldr_flag & 0x01) && LDR_ON){   //read from LDR
        // subtract the last reading:
    total = total - readings[readIndex];
    // read from the sensor:
     //Serial.println("<TtoS00: Read");
     //Alarm.delay(800);
     readings[readIndex] = analogRead(inputPin);
     // add the reading to the total:
     total = total + readings[readIndex];
    // advance to the next position in the array:
    readIndex = readIndex + 1;

    // if we're at the end of the array...
   if (readIndex >= numReadings)
    // ...wrap around to the beginning:
    readIndex = 0;

  // calculate the average:
  LDR_Average = total / numReadings;
  // send it to the computer as ASCII digits
  
      ldr_flag &= ~0x01;
    }
}

void onLDRUpdate(){
//if LDR > X
      if ((LDR_Average > 400) && (daylight_flag & 0x01)){
        //PORTD = (1<<4);   //Light indicator on
          setRelayState(3, 1);
          Serial.println("<TtoS00: Sun has come up");
          Alarm.delay(3000);
        // then set Daylight_flag
        daylight_flag = 0x02;
      }
      if((LDR_Average < 250) && (daylight_flag & 0x02)){
        Serial.println("<TtoS00: Sun has gone down");
        Alarm.delay(3000);
        //PORTD &= ~(1<<4);   //Light indicator off
            setRelayState(3, 0);
        // else set Daylight_flag = ~0x02
        daylight_flag = 0x01;
        Alarm.delay(3000);
      }
}

void setDriveMode(){
      //If Daylight and door closed
      if((daylight_flag & 0x02) && (door_state & 0x01) && (motor_control_flag == 0x00)){
        //Then motor_control_flag = ~0x01 // Open Seasame
        motor_control_flag = 0x01;
        Serial.println("<TtoS00: Time to open the door");
        Alarm.delay(2000);
      }
        
      //If Night and door open
      if((daylight_flag & 0x01) && (door_state & 0x02) && (motor_control_flag == 0x00)){
        //Then motor_control_flag = 0x10 // Close me
        motor_control_flag = 0x10;
        Serial.println("<TtoS00: Time to close the door");
        Alarm.delay(2000);
      }
      //ldr_flag &= ~0x02;
}

void onReachedEnd(){
    // Do every 5 sec
    if ((drivetimer >= 10) && (motor_control_flag & 0x02)){ //Driven UP 
//      PORTC &= ~(1<<4);   //Motor output = OFF
          setRelayState(1, 1);
          Serial.println("<TtoS00: stopped");
        Alarm.delay(2000);
//      PORTC &= ~(1<<5);   //Direction Relay set to DOWN
          setRelayState(0, 1);
      motor_control_flag = 0x00; //Off
      door_state = 0x02;
      Serial.println("<TtoS00: Door is now open. goodmorning chicks");
        Alarm.delay(3000);
      //door_open = 0x01;
      drivetimer = 0;
    }
    else if((drivetimer >= 10) && motor_control_flag & 0x20){ //Driven down
//      PORTC &= ~(1<<4);   //Motor output = OFF
          setRelayState(1, 1);
          Serial.println("<TtoS00: stopped");
        Alarm.delay(2000);
//      PORTC &= ~(1<<5);   //Direction Relay set to DOWN
          setRelayState(0, 1);
      motor_control_flag = 0x00;
      door_state = 0x01;
      Serial.println("<TtoS00: door is now closed. goodnight chickens");
        Alarm.delay(4000);
      //door_closed = 0x01;
      drivetimer = 0;
    }
}

void driveMotor(){
    if (motor_control_flag & 0x01){ //Drive UP
      drivetimer = 0;
      setRelayState(0, 0);
      setRelayState(1, 0);
      Serial.println("<TtoS00: driving Up");
      Alarm.delay(3000);
      motor_control_flag = 0x02; //Going up
    }
    else if(motor_control_flag & 0x10){ //Drive down
      drivetimer = 1;
      setRelayState(0, 1); //Direction Relay set to DOWN
      setRelayState(1, 0); //Motor Relay set to ON
      Serial.println("<TtoS00: driving down");
      Alarm.delay(2000);
//    setRelayState(2, !digitalRead(RelayPins[2]));  manual 
      motor_control_flag = 0x20;//Going Down
      
//      timer2_flag |= 0x01;
    }
}

void togglePin(){
  static bool ledStatus = false;
  ledStatus = !ledStatus;

  digitalWrite(ledPin, ledStatus);
}


void writeRunning(){
//  Serial.print("Online for: ");
//  Serial.println(millis() / 1000);
  Serial.print("PWM: ");
  Serial.println(clicks);
}

