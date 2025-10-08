// =======================================================================================
// /////////////////////////Padawan360 Body Code - Mega I2C v2.0 ////////////////////////////////////
// =======================================================================================
/*
by Dan Kraus
dskraus@gmail.com
Astromech: danomite4047
Project Site: https://github.com/dankraus/padawan360/

Heavily influenced by DanF's Padwan code which was built for Arduino+Wireless PS2
controller leveraging Bill Porter's PS2X Library. I was running into frequent disconnect
issues with 4 different controllers working in various capacities or not at all. I decided
that PS2 Controllers were going to be more difficult to come by every day, so I explored
some existing libraries out there to leverage and came across the USB Host Shield and it's
support for PS3 and Xbox 360 controllers. Bluetooth dongles were inconsistent as well
so I wanted to be able to have something with parts that other builder's could easily track
down and buy parts even at your local big box store.

V3.0 Add PWM Foot drive for use with Q85 style hub drives with individual speed controllers
- Based on KnightShade's SHADOW code with contributions for PWM Motor Controllers by JoyMonkey/Paul Murphy
- With credit to Brad/BHD

v2.0 Changes:
- Makes left analog stick default drive control stick. Configurable between left or right stick via isLeftStickDrive 

Hardware:
***Arduino Mega 2560***
USB Host Shield from circuits@home
Microsoft Xbox 360 Controller
Xbox 360 USB Wireless Reciver
Sabertooth Motor Controller
Syren Motor Controller
Sparkfun MP3 Trigger

This sketch supports I2C and calls events on many sound effect actions to control lights and sounds.
It is NOT set up for Dan's method of using the serial packet to transfer data up to the dome
to trigger some light effects.It uses Hardware Serial pins on the Mega to control Sabertooth and Syren

Set Sabertooth 2x25/2x12 Dip Switches 1 and 2 Down, All Others Up
For SyRen Simple Serial Set Switches 1 and 2 Down, All Others Up
For SyRen Simple Serial Set Switchs 2 & 4 Down, All Others Up
Placed a 10K ohm resistor between S1 & GND on the SyRen 10 itself

*/

// PWM Hub Motor Mode settings...
#define FOOT_CONTROLLER 1  //0 for Sabertooth Serial or 1 for individual R/C output (for Q85/NEO motors with 1 controller for each foot, or Sabertooth Mode 2 Independant Mixing)
int footDriveSpeed = 0;    //This was moved to be global to support better ramping of NPC Motors
int SteeringFactor = 100;  //The larger SteeringFactor is the less senstitive steering is...//Smaller values give more accuracy in making fine steering corrections
                           //XDist*sqrt(XDist+SteeringFactor)
#define leftFootPin 44     //connect this pin to motor controller for left foot (R/C mode)
#define rightFootPin 45    //connect this pin to motor controller for right foot (R/C mode)
#define leftDirection 1    //change this if your motor is spinning the wrong way
#define rightDirection 0   //change this if your motor is spinning the wrong way
int leftFoot = 90;
int rightFoot = 90;

// ************************** Options, Configurations, and Settings ***********************************


// SPEED AND TURN SPEEDS
//set these 3 to whatever speeds work for you. 0-stop, 127-full speed.
const byte DRIVESPEED1 = 50;
// Recommend beginner: 50 to 75, experienced: 100 to 127, I like 100.
// These may vary based on your drive system and power system
const byte DRIVESPEED2 = 100;
//Set to 0 if you only want 2 speeds.
const byte DRIVESPEED3 = 127;

// Default drive speed at startup
byte drivespeed = DRIVESPEED1;

// the higher this number the faster the droid will spin in place, lower - easier to control.
// Recommend beginner: 40 to 50, experienced: 50 $ up, I like 70
// This may vary based on your drive system and power system
const byte TURNSPEED = 70;

// Set isLeftStickDrive to true for driving  with the left stick
// Set isLeftStickDrive to false for driving with the right stick (legacy and original configuration)
boolean isLeftStickDrive = true;

// If using a speed controller for the dome, sets the top speed. You'll want to vary it potenitally
// depending on your motor. My Pittman is really fast so I dial this down a ways from top speed.
// Use a number up to 127 for serial
const byte DOMESPEED = 110;

// Ramping- the lower this number the longer R2 will take to speedup or slow down,
// change this by incriments of 1
const byte RAMPING = 5;

// Compensation is for deadband/deadzone checking. There's a little play in the neutral zone
// which gets a reading of a value of something other than 0 when you're not moving the stick.
// It may vary a bit across controllers and how broken in they are, sometimex 360 controllers
// develop a little bit of play in the stick at the center position. You can do this with the
// direct method calls against the Syren/Sabertooth library itself but it's not supported in all
// serial modes so just manage and check it in software here
// use the lowest number with no drift
// DOMEDEADZONERANGE for the left stick, DRIVEDEADZONERANGE for the right stick
const byte DOMEDEADZONERANGE = 20;
const byte DRIVEDEADZONERANGE = 20;

// Set the baude rate for the Sabertooth motor controller (feet)
// 9600 is the default baud rate for Sabertooth packet serial.
// for packetized options are: 2400, 9600, 19200 and 38400. I think you need to pick one that works
// and I think it varies across different firmware versions.
const int SABERTOOTHBAUDRATE = 9600;

// Set the baude rate for the Syren motor controller (dome)
// for packetized options are: 2400, 9600, 19200 and 38400. I think you need to pick one that works
// and I think it varies across different firmware versions.
const int DOMEBAUDRATE = 9600;


// Automation Delays
// set automateDelay to min and max seconds between sounds
byte automateDelay = random(5, 20);
//How much the dome may turn during automation.
int turnDirection = 20;

// Pin number to pull a relay high/low to trigger my upside down compressed air like R2's extinguisher
#define EXTINGUISHERPIN 3

#include <Sabertooth.h>
#include <Wire.h>
#include <XBOXRECV.h>
#include <hcr.h>
#include <Servo.h>


/////////////////////////////////////////////////////////////////
// Setup the HCR Vocalizer for serial communication
// Default sound volume at startup
// 100 = full volume, 0 off
int vol = 80;

const int HCRBAUDERATE = 9600;
// Set the baude rate for the Teensy 4.1 running Human Cyborg Relations
// 9600 is the default

// Initialise the HCR Vocalizer API
HCRVocalizer HCR(&Serial3,9600); // Serial (Stream Port, baud rate)
//HCRVocalizer HCR(16,17,9600); // Serial (RX Pin, TX Pin, baud rate)
// HCRVocalizer HCR(14,15,HCRBAUDERATE); // Serial (RX Pin, TX Pin, baud rate)

/////////////////////////////////////////////////////////////////
#if FOOT_CONTROLLER == 0
Sabertooth Sabertooth2x(128, Serial1);
#endif
Sabertooth Syren10(128, Serial2);

// Satisfy IDE, which only needs to see the include statment in the ino.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif

// Set some defaults for start up
// false = drive motors off ( right stick disabled ) at start
boolean isDriveEnabled = false;

// Automated functionality
// Used as a boolean to turn on/off automated functions like periodic random sounds and periodic dome turns
boolean isInAutomationMode = false;
unsigned long automateMillis = 0;
// Action number used to randomly choose a sound effect or a dome turn
byte automateAction = 0;


int driveThrottle = 0;
int throttleStickValue = 0;
int throttleStickValueraw = 0;
int domeThrottle = 0;
int turnThrottle = 0;
int turnThrottleraw = 0;

boolean firstLoadOnConnect = false;

AnalogHatEnum throttleAxis;
AnalogHatEnum turnAxis;
AnalogHatEnum domeAxis;
ButtonEnum speedSelectButton;
ButtonEnum hpLightToggleButton;


// this is legacy right now. The rest of the sketch isn't set to send any of this
// data to another arduino like the original Padawan sketch does
// right now just using it to track whether or not the HP light is on so we can
// fire the correct I2C event to turn on/off the HP light.
//struct SEND_DATA_STRUCTURE{
//  //put your variable definitions here for the data you want to send
//  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
//  int hpl; // hp light
//  int dsp; // 0 = random, 1 = alarm, 5 = leia, 11 = alarm2, 100 = no change
//};
//SEND_DATA_STRUCTURE domeData;//give a name to the group of data

boolean isHPOn = false;

USB Usb;
XBOXRECV Xbox(&Usb);

// PWM Hub Motor Mode settings...
#if FOOT_CONTROLLER == 1
Servo leftFootSignal;
Servo rightFootSignal;
#endif

void setup() {
  Serial1.begin(SABERTOOTHBAUDRATE);
  Serial2.begin(DOMEBAUDRATE);
  Serial3.begin(HCRBAUDERATE);
  Serial.begin(38400); // Debug console
  delay(100);
  
  // Initiate and set some default values for the HCR Vocalizer
  HCR.begin(125); // Refresh Speed (Default:125 ms)
  HCR.OverrideEmotions(1); // Override emotions and prevent emotions normalising
  HCR.SetEmotion(HAPPY,89); // Set Happy to 89%
  HCR.SetEmotion(SAD,22); // Set Sadness to 22%
  HCR.SetEmotion(MAD,0); // Set Anger to 0%
  HCR.SetEmotion(SCARED,0); // Set Anxiety to 0% 

  // Set volume for all channels
  HCR.SetVolume(1,vol); // Set volume (0-255)
  HCR.SetVolume(2,vol); // Set volume (0-255)
  HCR.SetVolume(3,vol); // Set volume (0-255)
  
  Serial.println("Hello World");


#if defined(SYRENSIMPLE)
  Syren10.motor(0);
#else
  Syren10.autobaud();
#endif

#if FOOT_CONTROLLER == 0
  // Send the autobaud command to the Sabertooth controller(s).
  /* NOTE: *Not all* Sabertooth controllers need this command.
  It doesn't hurt anything, but V2 controllers use an
  EEPROM setting (changeable with the function setBaudRate) to set
  the baud rate instead of detecting with autobaud.
  If you have a 2x12, 2x25 V2, 2x60 or SyRen 50, you can remove
  the autobaud line and save yourself two seconds of startup delay.
  */
  Sabertooth2x.autobaud();
  // The Sabertooth won't act on mixed mode packet serial commands until
  // it has received power levels for BOTH throttle and turning, since it
  // mixes the two together to get diff-drive power levels for both motors.
  Sabertooth2x.drive(0);
  Sabertooth2x.turn(0);


  Sabertooth2x.setTimeout(950);
#elif FOOT_CONTROLLER == 1
  leftFootSignal.attach(leftFootPin);
  rightFootSignal.attach(rightFootPin);
  stopFeet();
#endif

  Syren10.setTimeout(950);

  pinMode(EXTINGUISHERPIN, OUTPUT);
  digitalWrite(EXTINGUISHERPIN, HIGH);

  if (isLeftStickDrive) {
    throttleAxis = LeftHatY;
    turnAxis = LeftHatX;
    domeAxis = RightHatX;
    speedSelectButton = L3;
    hpLightToggleButton = R3;

  } else {
    throttleAxis = RightHatY;
    turnAxis = RightHatX;
    domeAxis = LeftHatX;
    speedSelectButton = R3;
    hpLightToggleButton = L3;
  }


  // Start I2C Bus. The body is the master.
  Wire.begin();

  //Serial.begin(115200);
  // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  while (!Serial)
    ;
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1)
      ;  //halt
  }
  Serial.print(F("\r\nXbox Wireless Receiver Library Started"));
}


void loop() {

  Usb.Task();
  // if we're not connected, return so we don't bother doing anything else.
  // set all movement to 0 so if we lose connection we don't have a runaway droid!
  // a restraining bolt and jawa droid caller won't save us here!
  if (!Xbox.XboxReceiverConnected || !Xbox.Xbox360Connected[0]) {
#if FOOT_CONTROLLER == 0
    Sabertooth2x.drive(0);
    Sabertooth2x.turn(0);
#elif FOOT_CONTROLLER == 1
    stopFeet();
#endif

    Syren10.motor(1, 0);
    firstLoadOnConnect = false;
    // Serial.println("Xbox Controller Disconnected");
    return;
  }

  // After the controller connects, Blink all the LEDs so we know drives are disengaged at start
  if (!firstLoadOnConnect) {
    firstLoadOnConnect = true;

    HCR.PlayWAV(1,"0021");
    Xbox.setLedMode(ROTATING, 0);
  }

  if (Xbox.getButtonClick(XBOX, 0)) {
    if (Xbox.getButtonPress(L1, 0) && Xbox.getButtonPress(R1, 0)) {
      Xbox.disconnect(0);
    }
  }

  // enable / disable right stick (droid movement) & play a sound to signal motor state
  if (Xbox.getButtonClick(START, 0)) {
    if (isDriveEnabled) {
      isDriveEnabled = false;
      Xbox.setLedMode(ROTATING, 0);

      HCR.PlayWAV(1,"0053");
    } else {
      isDriveEnabled = true;
        HCR.PlayWAV(1,"0052");
      // //When the drive is enabled, set our LED accordingly to indicate speed
      if (drivespeed == DRIVESPEED1) {
        Xbox.setLedOn(LED1, 0);
      } else if (drivespeed == DRIVESPEED2 && (DRIVESPEED3 != 0)) {
        Xbox.setLedOn(LED2, 0);
      } else {
        Xbox.setLedOn(LED3, 0);
      }
    }
  }

  //Toggle automation mode with the BACK button
  if (Xbox.getButtonClick(BACK, 0)) {
    if (isInAutomationMode) {
      isInAutomationMode = false;
      automateAction = 0;

      HCR.PlayWAV(1,"0053");

    } else {
      isInAutomationMode = true;
      HCR.PlayWAV(1,"0052");
    }
  }

  // Plays random sounds or dome movements for automations when in automation mode
  if (isInAutomationMode) {
    unsigned long currentMillis = millis();

    if (currentMillis - automateMillis > (automateDelay * 1000)) {
      automateMillis = millis();
      automateAction = random(1, 5);

      if (automateAction > 1) {
        HCR.PlayRandomWAV(1,"0009","0052");
      }
      if (automateAction < 4) {
#if defined(SYRENSIMPLE)
        Syren10.motor(turnDirection);
#else
        Syren10.motor(1, turnDirection);
#endif

        delay(750);

#if defined(SYRENSIMPLE)
        Syren10.motor(0);
#else
        Syren10.motor(1, 0);
#endif

        if (turnDirection > 0) {
          turnDirection = -45;
        } else {
          turnDirection = 45;
        }
      }

      // sets the mix, max seconds between automation actions - sounds and dome movement
      automateDelay = random(3, 10);
    }
  }

  // Volume Control of HCR
  // Hold R1 and Press Up/down on D-pad to increase/decrease volume
  if (Xbox.getButtonClick(UP, 0)) {
    // volume up
    if (Xbox.getButtonPress(R1, 0)) {
      Serial.println("Volume Up");
      if (vol < 100) {
        vol++;
          // Set volume for all channels
        HCR.SetVolume(1,vol); // Set volume (0-255)
        HCR.SetVolume(2,vol); // Set volume (0-255)
        HCR.SetVolume(3,vol); // Set volume (0-255)
      }
    }
  }
  if (Xbox.getButtonClick(DOWN, 0)) {
    //volume down
    if (Xbox.getButtonPress(R1, 0)) {
      if (vol > 0) {
        vol--;
        // Set volume for all channels
        HCR.SetVolume(1,vol); // Set volume (0-255)
        HCR.SetVolume(2,vol); // Set volume (0-255)
        HCR.SetVolume(3,vol); // Set volume (0-255)
      }
    }
  }

  // Logic display brightness.
  // Hold L1 and press up/down on dpad to increase/decrease brightness
  if (Xbox.getButtonClick(UP, 0)) {
    if (Xbox.getButtonPress(L1, 0)) {
      triggerI2C(10, 24);
    }
  }
  if (Xbox.getButtonClick(DOWN, 0)) {
    if (Xbox.getButtonPress(L1, 0)) {
      triggerI2C(10, 25);
    }
  }


  //FIRE EXTINGUISHER
  // When holding L2-UP, extinguisher is spraying. WHen released, stop spraying

  // TODO: ADD SERVO DOOR OPEN FIRST. ONLY ALLOW EXTINGUISHER ONCE IT'S SET TO 'OPENED'
  // THEN CLOSE THE SERVO DOOR
  if (Xbox.getButtonPress(L1, 0)) {
    if (Xbox.getButtonPress(UP, 0)) {
      digitalWrite(EXTINGUISHERPIN, LOW);
    } else {
      digitalWrite(EXTINGUISHERPIN, HIGH);
    }
  }


  // GENERAL SOUND PLAYBACK AND DISPLAY CHANGING

  // Y Button and Y combo buttons
  if (Xbox.getButtonClick(Y, 0)) {
    if (Xbox.getButtonPress(L1, 0)) {
      HCR.PlayWAV(1,"0008");
      //logic lights, random
      triggerI2C(10, 0);
    } else if (Xbox.getButtonPress(L2, 0)) {
      HCR.PlayWAV(1,"0002");
      //logic lights, random
      triggerI2C(10, 0);
    } else if (Xbox.getButtonPress(R1, 0)) {
      HCR.PlayWAV(1,"0009");
      //logic lights, random
      triggerI2C(10, 0);
    } else {
      // mp3Trigger.play(random(13, 17));
      //logic lights, random
      triggerI2C(10, 0);
    }
  }

  // A Button and A combo Buttons
  if (Xbox.getButtonClick(A, 0)) {
    if (Xbox.getButtonPress(L1, 0)) {
      HCR.PlayWAV(1,"0006");
      //logic lights
      triggerI2C(10, 6);
      // HPEvent 11 - SystemFailure - I2C
      triggerI2C(25, 11);
      triggerI2C(26, 11);
      triggerI2C(27, 11);
    } else if (Xbox.getButtonPress(L2, 0)) {
      HCR.PlayWAV(1,"0001");
      //logic lights, alarm
      triggerI2C(10, 1);
      //  HPEvent 3 - alarm - I2C
      triggerI2C(25, 3);
      triggerI2C(26, 3);
      triggerI2C(27, 3);
    } else if (Xbox.getButtonPress(R1, 0)) {
      HCR.PlayWAV(1,"0011");
      //logic lights, alarm2Display
      triggerI2C(10, 11);
    } else {
      HCR.PlayRandomWAV(1,"0017","0025");
      //logic lights, random
      triggerI2C(10, 0);
    }
  }

  // B Button and B combo Buttons
  if (Xbox.getButtonClick(B, 0)) {
    if (Xbox.getButtonPress(L1, 0)) {
      HCR.PlayWAV(1,"0007");
      //logic lights, random
      triggerI2C(10, 0);
    } else if (Xbox.getButtonPress(L2, 0)) {
      HCR.PlayWAV(1,"0003");
      //logic lights, random
      triggerI2C(10, 0);
    } else if (Xbox.getButtonPress(R1, 0)) {
      HCR.PlayWAV(1,"0010");
      //logic lights bargrap
      triggerI2C(10, 10);
      // HPEvent 1 - Disco - I2C
      triggerI2C(25, 10);
      triggerI2C(26, 10);
      triggerI2C(27, 10);
    } else {
      HCR.PlayRandomWAV(1,"0032","0052");
      //logic lights, random
      triggerI2C(10, 0);
    }
  }

  // X Button and X combo Buttons
  if (Xbox.getButtonClick(X, 0)) {
    // leia message L1+X
    if (Xbox.getButtonPress(L1, 0)) {
      HCR.PlayWAV(1,"0005");
      //logic lights, leia message
      triggerI2C(10, 5);
      // Front HPEvent 1 - HoloMessage - I2C -leia message
      triggerI2C(25, 9);
    } else if (Xbox.getButtonPress(L2, 0)) {
      HCR.PlayWAV(1,"0004");
      //logic lights
      triggerI2C(10, 4);
    } else if (Xbox.getButtonPress(R1, 0)) {
      HCR.PlayWAV(1,"0012");
      //logic lights, random
      triggerI2C(10, 0);
    } else {
      HCR.PlayRandomWAV(1,"0025","0032");
      //logic lights, random
      triggerI2C(10, 0);
    }
  }

  // turn hp light on & off with Right Analog Stick Press (R3) for left stick drive mode
  // turn hp light on & off with Left Analog Stick Press (L3) for right stick drive mode
  if (Xbox.getButtonClick(hpLightToggleButton, 0)) {
    // if hp light is on, turn it off
    if (isHPOn) {
      isHPOn = false;
      // turn hp light off
      // Front HPEvent 2 - ledOFF - I2C
      triggerI2C(25, 2);
    } else {
      isHPOn = true;
      // turn hp light on
      // Front HPEvent 4 - whiteOn - I2C
      triggerI2C(25, 1);
    }
  }


  // Change drivespeed if drive is enabled
  // Press Left Analog Stick (L3) for left stick drive mode
  // Press Right Analog Stick (R3) for right stick drive mode
  // Set LEDs for speed - 1 LED, Low. 2 LED - Med. 3 LED High
  if (Xbox.getButtonClick(speedSelectButton, 0) && isDriveEnabled) {
    //if in lowest speed
    if (drivespeed == DRIVESPEED1) {
      //change to medium speed and play sound 3-tone
      drivespeed = DRIVESPEED2;
      Xbox.setLedOn(LED2, 0);
      HCR.PlayWAV(1,"0053");
      triggerI2C(10, 22);
    } else if (drivespeed == DRIVESPEED2 && (DRIVESPEED3 != 0)) {
      //change to high speed and play sound scream
      drivespeed = DRIVESPEED3;
      Xbox.setLedOn(LED3, 0);
      HCR.PlayWAV(1,"0001");
      triggerI2C(10, 23);
    } else {
      //we must be in high speed
      //change to low speed and play sound 2-tone
      drivespeed = DRIVESPEED1;
      Xbox.setLedOn(LED1, 0);
      HCR.PlayWAV(1,"0052");
      triggerI2C(10, 21);
    }
  }



  // FOOT DRIVES
  // Xbox 360 analog stick values are signed 16 bit integer value
// PWM Hub Motor Mode settings...
#if FOOT_CONTROLLER == 0
  // Sabertooth runs at 8 bit signed. -127 to 127 for speed (full speed reverse and  full speed forward)
  // Map the 360 stick values to our min/max current drive speed
  throttleStickValue = (map(Xbox.getAnalogHat(throttleAxis, 0), -32768, 32767, -drivespeed, drivespeed));
  if (throttleStickValue > -DRIVEDEADZONERANGE && throttleStickValue < DRIVEDEADZONERANGE) {
    // stick is in dead zone - don't drive
    driveThrottle = 0;
    stopFeet();
  } else {
    if (isInAutomationMode) {  // Turn of automation if using the drive motors
      isInAutomationMode = false;
      automateAction = 0;
    }
    if (driveThrottle < throttleStickValue) {
      if (throttleStickValue - driveThrottle < (RAMPING + 1)) {
        driveThrottle += RAMPING;
      } else {
        driveThrottle = throttleStickValue;
      }
    } else if (driveThrottle > throttleStickValue) {
      if (driveThrottle - throttleStickValue < (RAMPING + 1)) {
        driveThrottle -= RAMPING;
      } else {
        driveThrottle = throttleStickValue;
      }
    }
  }

  turnThrottle = map(Xbox.getAnalogHat(turnAxis, 0), -32768, 32767, -TURNSPEED, TURNSPEED);

  // DRIVE!
  // right stick (drive)
  if (isDriveEnabled) {
    // Only do deadzone check for turning here. Our Drive throttle speed has some math applied
    // for RAMPING and stuff, so just keep it separate here
    if (turnThrottle > -DRIVEDEADZONERANGE && turnThrottle < DRIVEDEADZONERANGE) {
      // stick is in dead zone - don't turn
      turnThrottle = 0;
      stopFeet();
    }
    Sabertooth2x.turn(-turnThrottle);
    Sabertooth2x.drive(driveThrottle);
  }

#elif FOOT_CONTROLLER == 1
  //Experimental Q85. Untested Madness!!! Use at your own risk and expect your droid to run away in flames.
  //use BigHappyDude's mixing algorythm to get values for each foot...
  throttleStickValueraw = Xbox.getAnalogHat(throttleAxis, 0);
  turnThrottleraw = Xbox.getAnalogHat(turnAxis, 0);

  if (isDriveEnabled) {
    // Check if EITHER the throttle (Y-axis) OR the turning (X-axis) is outside the deadzone
    if ( (throttleStickValueraw > (-DRIVEDEADZONERANGE * 258) && throttleStickValueraw < (DRIVEDEADZONERANGE * 258)) &&
         (turnThrottleraw > (-DRIVEDEADZONERANGE * 258) && turnThrottleraw < (DRIVEDEADZONERANGE * 258)) ) {
      
      // Both axes are in the dead zone - stop
      stopFeet();
    } else {
      if (isInAutomationMode) {  // Turn of automation if using the drive motors
        isInAutomationMode = false;
        automateAction = 0;
      }

      mixBHD(throttleStickValueraw, turnThrottleraw, drivespeed);  // Call function to gett values for leftFoot and rightFoot

      leftFootSignal.write(leftFoot);
      rightFootSignal.write(rightFoot);
    }
  } else {
    stopFeet();
  }

#endif

  // DOME DRIVE!
  domeThrottle = (map(Xbox.getAnalogHat(domeAxis, 0), -32768, 32767, DOMESPEED, -DOMESPEED));
  if (domeThrottle > -DOMEDEADZONERANGE && domeThrottle < DOMEDEADZONERANGE) {
    //stick in dead zone - don't spin dome
    domeThrottle = 0;
  }

  Syren10.motor(1, domeThrottle);
}  // END loop()

void triggerI2C(byte deviceID, byte eventID) {
  Wire.beginTransmission(deviceID);
  Wire.write(eventID);
  Wire.endTransmission();
}

void triggerAutomation() {
  // Plays random sounds or dome movements for automations when in automation mode
  unsigned long currentMillis = millis();

  if (currentMillis - automateMillis > (automateDelay * 1000)) {
    automateMillis = millis();
    automateAction = random(1, 5);

    if (automateAction > 1) {
      // mp3Trigger.play(random(32, 52));
    }
    if (automateAction < 4) {

      //************* Move the dome for 750 msecs  **************

#if defined(SYRENSIMPLE)
      Syren10.motor(turnDirection);
#else
      Syren10.motor(1, turnDirection);
#endif

      delay(750);

//************* Stop the dome motor **************
#if defined(SYRENSIMPLE)
      Syren10.motor(0);
#else
      Syren10.motor(1, 0);
#endif

      //************* Change direction for next time **************
      if (turnDirection > 0) {
        turnDirection = -45;
      } else {
        turnDirection = 45;
      }
    }

    // sets the mix, max seconds between automation actions - sounds and dome movement
    automateDelay = random(5, 15);
  }
}

// =======================================================================================
// //////////////////////////Mixing Function for R/C Mode////////////////////////////////
// =======================================================================================
// Based on KnightShade's SHADOW code with contributions for PWM Motor Controllers by JoyMonkey/Paul Murphy
// With credit to Brad/BHD
#if FOOT_CONTROLLER == 1
void mixBHD(int stickX, int stickY, byte maxDriveSpeed) {
  // This is BigHappyDude's mixing function, for differential (tank) style drive using two motor controllers.
  // Takes a joysticks X and Y values, mixes using the diamind mix, and output a value 0-180 for left and right motors.
  // 180,180 = both feet full speed forward.
  // 000,000 = both feet full speed reverse.
  // 180,000 = left foot full forward, right foot full reverse (spin droid clockwise)
  // 000,180 = left foot full reverse, right foot full forward (spin droid counter-clockwise)
  // 090,090 = no movement
  // for simplicity, we think of this diamond matrix as a range from -100 to +100 , then map the final values to servo range (0-180) at the end
  //  Ramping and Speed mode applied on the droid.

  if (stickX < (-DRIVEDEADZONERANGE * 258) || stickX > (DRIVEDEADZONERANGE * 258)) {
    //  if movement outside deadzone
    //  Map to easy grid -100 to 100 in both axis, including deadzones.
    int YDist = 0;  // set to 0 as a default value if no if used.
    int XDist = 0;

    YDist = (map(stickY, -32768, 32767, -100, 100));
    XDist = (map(stickX, -32768, 32767, -100, 100));

    /* Debugging by KnightShade 
      //Driving is TOO sensitive.   Need to dial down the turning to a different scale factor.
      This code will map the linear values to a flatter value range.

      //The larger SteeringFactor is the less senstitive steering is...  
      //Smaller values give more accuracy in making fine steering corrections
        XDist*sqrt(XDist+SteeringFactor)
      */
    //Convert from Linear to a scaled/exponential Steering system
    //int SteeringFactor = 100;  //TODO - move a constant at top of script
    int TempScaledXDist = (int)(abs(XDist) * sqrt(abs(XDist) + SteeringFactor));
    int MaxScale = (100 * sqrt(100 + SteeringFactor));
    XDist = (map(stickX, 0, MaxScale, 1, 100));  //  Map the left direction stick value to Turn speed

    if (stickX <= (-DRIVEDEADZONERANGE * 258)) {                 //if not in deadzone
      XDist = -1 * (map(TempScaledXDist, 0, MaxScale, 1, 100));  //  Map the left direction stick value to Turn speed
    } else if (stickX >= (DRIVEDEADZONERANGE * 258)) {
      XDist = (map(TempScaledXDist, 0, MaxScale, 1, 100));  //  Map the right direction stick value to Turn speed
    }
    //END Convert from Linear to a scaled/exponential Steering system

    //  Constrain to Diamond values.  using 2 line equations and find the intersect, boiled down to the minimum
    //  This was the inspiration; https://github.com/declanshanaghy/JabberBot/raw/master/Docs/Using%20Diamond%20Coordinates%20to%20Power%20a%20Differential%20Drive.pdf
    float TempYDist = YDist;
    float TempXDist = XDist;
    if (YDist > (XDist + 100)) {  //  if outside top left.  equation of line is y=x+Max, so if y > x+Max then it is above line
      // OK, the first fun bit. :)  so for the 2 lines this is always true y = m1*x + b1 and y = m2*x - b2
      // y - y = m1*x + b1  - m2*x - b2  or 0 = (m1 - m2)*x + b1 - b2
      // We have y = x+100 and y = ((change in y)/Change in x))x
      // So:   x = -100/(1-(change in y)/Change in x)) and using y = x+100 we can find y with the new x
      // Not too bad when simplified. :P
      TempXDist = -100 / (1 - (TempYDist / TempXDist));
      TempYDist = TempXDist + 100;
    } else if (YDist > (100 - XDist)) {  //  if outside top right
      // repeat intesection for y = 100 - x
      TempXDist = -100 / (-1 - (TempYDist / TempXDist));
      TempYDist = -TempXDist + 100;
    } else if (YDist < (-XDist - 100)) {  //  if outside bottom left
      // repeat intesection for y = -x - 100
      TempXDist = 100 / (-1 - (TempYDist / TempXDist));
      TempYDist = -TempXDist - 100;
    } else if (YDist < (XDist - 100)) {  //  if outside bottom right
      // repeat intesection for y = x - 100
      TempXDist = 100 / (1 - (TempYDist / TempXDist));
      TempYDist = TempXDist - 100;
    }
    //  all coordinates now in diamond. next translate to the diamond coordinates.
    //  for the left.  send ray to y = x + Max from coordinates along y = -x + b
    //  find for b, solve for coordinates and resut in y then scale using y = (y - max/2)*2
    float LeftSpeed = ((TempXDist + TempYDist - 100) / 2) + 100;
    LeftSpeed = (LeftSpeed - 50) * 2;
    //  for right send ray to y = -x + Max from coordinates along y = x + b find intersction coordinates and then use the Y vaule and scale.
    float RightSpeed = ((TempYDist - TempXDist - 100) / 2) + 100;
    RightSpeed = (RightSpeed - 50) * 2;
    //  this results in a -100 to 100 range of speeds, so shift to servo range.

    /* Debugging by KnightShade - this didn't do the speed like we expected.  Notice that they are constant values.....
      // map(maxDriveSpeed, 0, 127, 90, 180); //drivespeed was defined as 0 to 127 for Sabertooth serial, now we want something in an upper servo range (90 to 180)
      #if leftDirection == 0
      leftFoot=map(LeftSpeed, -100, 100, 180, 0);
      #else
      leftFoot=map(LeftSpeed, -100, 100, 0, 180);
      #endif
      #if rightDirection == 0
      rightFoot=map(RightSpeed, -100, 100, 180, 0);
      #else
      rightFoot=map(RightSpeed, -100, 100, 0, 180);
      #endif
      
      First pass, treat the throttle as ON/OFF - not an Analog shift (as Sabertooth code does)
      Based on that Paul passed in Drive Speed 1 or 2.
      */
    int maxServoForward = map(maxDriveSpeed, 0, 127, 90, 180);  // 90 is stop, 180 is full forward
    int maxServoReverse = map(maxDriveSpeed, 0, 127, 90, 0);    // 90 is stop, 0 is full reverse

    // The mapping from -100 to 100 is based on the motor's *desired direction*.

    // *** LEFT FOOT MAPPING FIX ***
#if leftDirection == 0 // Assuming leftDirection 0 means 0 is FORWARD, 180 is REVERSE
    // LeftSpeed -100 (REV) maps to maxServoForward (0)
    // LeftSpeed 100 (FWD) maps to maxServoReverse (180)
    // Map (-100, 100) to (maxServoForward, maxServoReverse)
    leftFoot = map(LeftSpeed, -100, 100, maxServoReverse, maxServoForward);
#else  // leftDirection 1 means 180 is FORWARD, 0 is REVERSE (Standard Servo)
    // LeftSpeed -100 (REV) maps to maxServoReverse (0)
    // LeftSpeed 100 (FWD) maps to maxServoForward (180)
    // Map (-100, 100) to (maxServoReverse, maxServoForward)
    leftFoot = map(LeftSpeed, -100, 100, maxServoReverse, maxServoForward);
#endif

    // *** RIGHT FOOT MAPPING FIX ***
#if rightDirection == 0 // Assuming rightDirection 0 means 0 is FORWARD, 180 is REVERSE
    // RightSpeed -100 (REV) maps to maxServoForward (0)
    // RightSpeed 100 (FWD) maps to maxServoReverse (180)
    // Map (-100, 100) to (maxServoReverse, maxServoForward)
    rightFoot = map(RightSpeed, -100, 100, maxServoReverse, maxServoForward);
#else  // rightDirection 1 means 180 is FORWARD, 0 is REVERSE (Standard Servo)
    // RightSpeed -100 (REV) maps to maxServoReverse (0)
    // RightSpeed 100 (FWD) maps to maxServoForward (180)
    // Map (-100, 100) to (maxServoReverse, maxServoForward)
    rightFoot = map(RightSpeed, -100, 100, maxServoReverse, maxServoForward);
#endif
    /*  END Knightshade Debug */
  } else {
    leftFoot = 90;
    rightFoot = 90;
  }
}
#endif

// =======================================================================================
// ////////////////////////END:  Mixing Function for R/C Mode/////////////////////////////
// =======================================================================================

void stopFeet() {
#if FOOT_CONTROLLER == 0
  Sabertooth2x.drive(0);
  Sabertooth2x.turn(0);
#elif FOOT_CONTROLLER == 1
  leftFootSignal.write(90);
  rightFootSignal.write(90);
#endif
}
