/*
 * Camera Actuation Control Software V 1.0 
 * Authored by Stefan Bichlmaier
 * V00852866
 * 
 * Description:
 * This software provides an easy way to program camera movements on a motorized camera slider.
 * It creates an interface between a set of stepper motors and their standard, 2 wire drivers through an LCD screen and rotary encoder
 * 
 * Features:
 * -Interrupt-driven rotary encoder and push button input for menu navigation
 * -Navigatable menu using the ArduinoMenu library
 * -Stepper motor enable/disable
 * -Stepper motor homing (finding the desired zero position)
 * -Simplified timelapse mode
 *  -Contains parameters which are used to calculate the timing and quantity of photos to be taken, as well as the distance travelled
 *  -Triggers the release of a DSLR camera by pulsing an infra-red LED
 *  
 * Future plans:
 * -Second axis motor and control
 * -Code for calculating camera rotation during timelapse
 * -Before adding more movement modes, generalize the movement struct to minimize code repeats
*/
#include <Arduino.h>
#include <Wire.h>
#include <menu.h> //https://github.com/neu-rah/ArduinoMenu used for menu creation and navigation
#include <menuIO/liquidCrystalOut.h>
#include <menuIO/serialOut.h>
#include <menuIO/serialIn.h>
#include <menuIO/encoderIn.h>
#include <menuIO/keyIn.h>
#include <menuIO/chainStream.h>
#include <AccelStepper.h> //https://www.airspayce.com/mikem/arduino/AccelStepper/ used as a non blocking, open loop stepper motor controller
#include "PinMap.h"

using namespace Menu;

AccelStepper stepperX(AccelStepper::DRIVER, X_STEP, X_DIR); //Initialize X axis stepper object

//--------------------------------------------------------LCD--------------------------------------------------------
#define LCD_RS 12
#define LCD_EN 11
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
//--------------------------------------------------------LCD--------------------------------------------------------

//--------------------------------------------------------Encoder----------------------------------------------------
//Create serial streams from encoder quadrature outputs and push button
encoderIn<ENC_A, ENC_B> encoder; //simple quad encoder driver
encoderInStream<ENC_A, ENC_B> encStream(encoder, 4); // simple quad encoder Stream

//a keyboard with only one key as the encoder button
keyMap encBtn_map[] = {{ -ENC_PB, defaultNavCodes[enterCmd].ch}}; //negative pin numbers use internal pull-up, this is on when low
keyIn<1> encButton(encBtn_map);//1 is the number of keys

//link input streams into one for menu input
serialIn serial(Serial);
menuIn* inputsList[] = {&encStream, &encButton, &serial};
chainStream<3> in(inputsList);//3 is the number of inputs
//--------------------------------------------------------Encoder-----------------------------------------------------

//--------------------------------------------------------Movement Modes--------------------------------------------------------

enum ModeStates {//keep track of which mode the system is in to determine which update functions to call in the event loop
  MENUSTATE = 0,
  TIMELAPSE = 1
};

ModeStates cameraMode = MENUSTATE;

//timelapse struct contains the variables and functions necessary for deriving camera movements for a timelapse
//TODO add rotation calculations and second motor control
struct timelapse{
  //variables set from the menu
  int duration; //duration of the timelapse in real time
  int clipLength; //duration of the resulting clip (ie. a 4 hour duration resulting in a 10 second clipLength)
  int framesPerSecond; //frame rate of the resulting clip
  int distanceX; //distance in X travelled in mm 

  //helper variables
  int currentShot;
  unsigned long timeAtLastShot;
  
  int getNumFrames(void){//calculate the number of photos which will be taken through the timelapse
    return clipLength * framesPerSecond;
  }
  int getTimeToShot(){//calculate the time between each shot
    return duration/getNumFrames();
  }
  long getDistanceToShot(){//calculate the distance in X between each shot
    return (long)((float)distanceX/(float)(0.009*getNumFrames())); 
  }
  
  //takePhoto() function from https://www.christidis.info/index.php/personal-projects/arduino-nikon-infrared-command-code
  //Creates the sequence of timed LED pulses to trigger a Nikon camera's shutter release
  //this is a blocking function, but it only takes ~50ms to execute. In most cases, the carriage will be stopped for much longer than this
  void takePhoto(void) {
    int i;
    for (i = 0; i < 76; i++) {
      digitalWrite(IR_LED, HIGH);
      delayMicroseconds(7);
      digitalWrite(IR_LED, LOW);
      delayMicroseconds(7);
    }
    delay(27);
    delayMicroseconds(810);
    for (i = 0; i < 16; i++) {
      digitalWrite(IR_LED, HIGH);
      delayMicroseconds(7);
      digitalWrite(IR_LED, LOW);
      delayMicroseconds(7);
    }
    delayMicroseconds(1540);
    for (i = 0; i < 16; i++) {
      digitalWrite(IR_LED, HIGH);
      delayMicroseconds(7);
      digitalWrite(IR_LED, LOW);
      delayMicroseconds(7);
    }
    delayMicroseconds(3545);
    for (i = 0; i < 16; i++) {
      digitalWrite(IR_LED, HIGH);
      delayMicroseconds(7);
      digitalWrite(IR_LED, LOW);
      delayMicroseconds(7);
    }
  }

  //function called on inital execution of timelapse
  void updateTimelapse(){
    Serial.println("Initial release!");
    this->takePhoto();
    this->currentShot = 1;
    cameraMode = TIMELAPSE;
    stepperX.move(this->getDistanceToShot()); //move to next shot relative to current position
    this->timeAtLastShot = millis();
  }
  
  //overloaded function for initial movement, determines whether an X movement or shutter release is due
  //function called in main event loop, takes the current time in ms and compares to the time at last shot
  void updateTimelapse(unsigned long currentTime){
    if(this->currentShot == this->getNumFrames()) cameraMode = MENUSTATE; //reached end of sequence, go back to menu state
    //
    if((currentTime - this->timeAtLastShot) >= this->getTimeToShot()*1000){//if we have reached or passed the time between shots, command next movement
      this->takePhoto();
      this->currentShot++;
      stepperX.move(this->getDistanceToShot()); //move to next shot relative to current position
      this->timeAtLastShot = millis();
    }
  }
};

timelapse timelapseMode;
//--------------------------------------------------------Movement Modes--------------------------------------------------------

//--------------------------------------------------------Menu Callbacks--------------------------------------------------------

result initTimelapse(){ //needed for callback function format
//Callback for execute menu option
  timelapseMode.updateTimelapse();
  return proceed;
}

int stepperEnable = HIGH;
int stepperDistance = 0;

result turnOnStepper() {
  digitalWrite(X_EN, stepperEnable);
  Serial.println("Toggled Stepper");
  Serial.println(stepperEnable);
  return proceed;
}

result homeCarriage(){
  while(digitalRead(X_LIM)){
    stepperX.setSpeed(-3000);
    stepperX.runSpeed();
  }
  stepperX.setCurrentPosition(0);
  stepperX.moveTo(200);
  return proceed;
}


result translateStepper(){
  Serial.println("Translating Stepper");
  stepperX.move(stepperDistance*100);
  return proceed;
}

result alert(menuOut& o, idleEvent e) {
  if (e == idling) {
    o.setCursor(0, 0);
    o.print("alert test");
    o.setCursor(0, 1);
    o.print("[select] to continue...");
  }
  return proceed;
}

result doAlert(eventMask e, prompt &item) {
  nav.idleOn(alert);
  return proceed;
}

result idle(menuOut& o, idleEvent e) {
  switch (e) {
    case idleStart: o.print("suspending menu!"); break;
    case idling: o.print("suspended..."); break;
    case idleEnd: o.print("resuming menu."); break;
  }
  return proceed;
}
//--------------------------------------------------------Menu Callbacks--------------------------------------------------------

//------------------------------------------------------Menu Initialization-----------------------------------------------------
//The code within this section is a collection of macros used by the arduinomenu library to minimize the amount of repeated code in the editor

//Initialize a toggle option to enable the stepper
TOGGLE(stepperEnable,enableStepper,"Stepper: ",turnOnStepper,enterEvent,noStyle//,doExit,enterEvent,noStyle
  ,VALUE("On",LOW,doNothing,noEvent)
  ,VALUE("Off",HIGH,doNothing,noEvent)
);

//Initialize the timelapse submenu
MENU(timelapseMenu, "Timelapse Mode", doNothing, noEvent, wrapStyle
  ,FIELD(timelapseMode.distanceX,"Distance","mm",0,1000,100,10,doNothing,noEvent,wrapStyle)
  ,FIELD(timelapseMode.duration,"Duration","s",0,2000,100,10,doNothing,noEvent,wrapStyle)
  ,FIELD(timelapseMode.clipLength,"Cliplength","s",0,100,10,1,doNothing,noEvent,wrapStyle)
  ,FIELD(timelapseMode.framesPerSecond,"FPS","fps",0,100,10,1,doNothing,noEvent,wrapStyle)
  ,OP("Go!",initTimelapse,enterEvent)
  ,EXIT("<Back")
  );

//Initialize the main menu
MENU(mainMenu,"Main menu",doNothing,noEvent,wrapStyle
  ,SUBMENU(enableStepper)
  ,SUBMENU(timelapseMenu)
  ,OP("Home Carriage",homeCarriage,enterEvent)
);

#define MAX_DEPTH 2

MENU_OUTPUTS(out, MAX_DEPTH
             , LIQUIDCRYSTAL_OUT(lcd, {0, 0, 16, 2})
             , NONE
            );
NAVROOT(nav, mainMenu, MAX_DEPTH, in, out); //the navigation root object
//------------------------------------------------------Menu Initialization-----------------------------------------------------

void setup() {
  pinMode(ENC_PB, INPUT_PULLUP);
  pinMode(X_LIM, INPUT_PULLUP);
  pinMode(IR_LED, OUTPUT);
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Arduino Menu Library"); Serial.flush();
  encoder.begin();
  lcd.begin(16, 2);
  nav.idleTask = idle; //point a function to be used when menu is suspended
  nav.showTitle = false;
  lcd.setCursor(0, 0);
  lcd.print("Ardu-Slider");
  lcd.setCursor(0, 1);
  lcd.print("Stefan B.");

  stepperX.setMaxSpeed(10000);
  stepperX.setSpeed(8000);
  stepperX.setAcceleration(800);
  delay(2000);
}

void loop() {
  nav.poll(); //poll the menu navigation object
  switch(cameraMode){ //poll the appropriate function depending on the current mode
    case MENUSTATE:
    break;
    case TIMELAPSE:
      timelapseMode.updateTimelapse(millis());
    break;
    default:
    break;
  }
  stepperX.run(); //poll the stepper motor object to see if a step is due
}
