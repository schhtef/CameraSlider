#include <avr/interrupt.h>
#include <LiquidCrystal.h>
#include <LiquidMenu.h>
#include "PinMap.h"

volatile byte aFlag = 0;
volatile byte bFlag = 0;
volatile byte pbFlag = 0;

enum EncoderEvent : uint8_t {
  ENC_NC = 0, //No rotation, default value
  ENC_CW = 1, //Clockwise rotation
  ENC_CCW = 2, //Counter-clockwise rotation
  ENC_PRESS = 3 //Button event
};

enum EncoderState : uint8_t {
  ENC_FOC = 0, //Rotation changes the focused line
  ENC_VAL = 1, //Rotation changes a specific variable
};

struct Encoder {
  EncoderEvent encoderEvent;
  EncoderState encoderState;
};

Encoder encoder;


// -------------------------------------------------------------------MENU OBJECTS--------------------------------------------------------------------
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

LiquidLine MainMenuTitle(0, 0, "Main Menu");
LiquidLine Timelapse(0, 1, "Timelapse Mode");
LiquidLine Oneshot(0, 1, "Oneshot Mode");
LiquidLine Parallax(0, 1, "Parallax Mode");

LiquidLine Framerate(6, 0, "Framerate");
LiquidLine FramerateVar(6, 1, 123);
LiquidLine Speed(6, 0, "Speed");
LiquidLine SpeedVar(6, 1, 456);
LiquidLine Distance(6, 0, "Distance");
LiquidLine DistanceVar(6, 1, 789);
LiquidLine Rotation(6, 0, "Rotation");
LiquidLine RotationVar(6, 1, 246);
LiquidLine Period(6, 0, "Loop Period");
LiquidLine PeriodVar(6, 1, 135);
LiquidLine Execute(0, 1, "Execute");
LiquidLine Back(0, 1, "Back");

LiquidScreen MainMenuScreen;
LiquidScreen TimelapseScreen;
LiquidScreen OneshotScreen;
LiquidScreen ParallaxScreen;

LiquidMenu MainMenu(lcd, MainMenuScreen);
LiquidMenu TimelapseMenu(lcd, TimelapseScreen);
LiquidMenu OneshotMenu(lcd, OneshotScreen);
LiquidMenu ParallaxMenu(lcd, ParallaxScreen);

LiquidSystem menuSystem(MainMenu, TimelapseMenu, OneshotMenu, ParallaxMenu, 1);
// -------------------------------------------------------------------MENU OBJECTS--------------------------------------------------------------------

void setup() {
  //This is code to enable pin change interrupts, will need this for push button and limit switch interrupts
  PCICR |= 0x4; //enable port D interrupts
  PCMSK2 |= 0x10; //mask PCINT20

  //Init Encoder Struct
  encoder.encoderEvent = ENC_NC;

  //Configure interrupts for rotary encoder
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_PB, INPUT_PULLUP);
  attachInterrupt(0, ENC_A_ISR, RISING);
  attachInterrupt(1, ENC_B_ISR, RISING);
  Serial.begin(115200); // start the serial monitor link

  // Add more "lines" than the display has. The extra will be scrolled.
  lcd.begin(16, 2);
  initMenu();
  menuSystem.update();
}

void initMenu(){
  Timelapse.attach_function(ENC_PRESS, changeMenu);
  Oneshot.attach_function(ENC_PRESS, changeMenu);
  Parallax.attach_function(ENC_PRESS, changeMenu);
  Back.attach_function(ENC_PRESS, changeMenu);

  Framerate.attach_function(ENC_PRESS, blankFunction);
  Speed.attach_function(ENC_PRESS, blankFunction);
  Distance.attach_function(ENC_PRESS, blankFunction);
  Rotation.attach_function(ENC_PRESS, blankFunction);
  Period.attach_function(ENC_PRESS, blankFunction);
  Execute.attach_function(ENC_PRESS, blankFunction);
  Back.attach_function(ENC_PRESS, changeMenu);

  //MainMenuScreen.add_line(MainMenuTitle);
  MainMenuScreen.add_line(Timelapse);
  MainMenuScreen.add_line(Oneshot);
  MainMenuScreen.add_line(Parallax);

  TimelapseScreen.add_line(Framerate);
  TimelapseScreen.add_line(Distance);
  TimelapseScreen.add_line(Rotation);
  TimelapseScreen.add_line(Execute);
  TimelapseScreen.add_line(Back);

  OneshotScreen.add_line(Speed);
  OneshotScreen.add_line(Distance);
  OneshotScreen.add_line(Rotation);
  OneshotScreen.add_line(Execute);
  OneshotScreen.add_line(Back);

  ParallaxScreen.add_line(Speed);
  ParallaxScreen.add_line(Rotation);
  ParallaxScreen.add_line(Period);
  ParallaxScreen.add_line(Execute);
  ParallaxScreen.add_line(Back);

  MainMenuScreen.set_displayLineCount(2);
  TimelapseScreen.set_displayLineCount(2);
  OneshotScreen.set_displayLineCount(2);
  ParallaxScreen.set_displayLineCount(2);
}
// -------------------------------------------------------------------INTERRUPTS-------------------------------------------------------------------
void ENC_A_ISR() {
  noInterrupts();
  byte reading = 0;
  reading = PIND & 0xC; //read all eight pin values then strip away all but ENC_A and ENC_B's values
  if (reading == 0xC && aFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoder.encoderEvent = ENC_CCW;
    //menu.switch_focus(0);
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading = 0x4) bFlag = 1;
  interrupts();
}

void ENC_B_ISR() {
  noInterrupts();
  byte reading = 0;
  reading = PIND & 0xC; //read all eight pin values then strip away all but ENC_A and ENC_B's values
  if (reading == 0xC && bFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoder.encoderEvent = ENC_CW;
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading = 0x8) aFlag = 1;
  interrupts();
}

ISR(PCINT2_vect) {
  if (pbFlag) {
    encoder.encoderEvent = ENC_PRESS;
    pbFlag = 0;
  }
  else if (!pbFlag) {
    pbFlag = 1;
  }
}
// -------------------------------------------------------------------INTERRUPTS-------------------------------------------------------------------

//switch focus when encoder event is detected
void onEncoderEvent() {
  //changes line focus or calls attached function when encoder is in the ENC_FOC state
  if (encoder.encoderState == ENC_FOC) {
    switch (encoder.encoderEvent) {
      case ENC_CW: //next line
        menuSystem.switch_focus(1);
        //Serial.println("CW");
        break;
      case ENC_CCW: //previous line
        menuSystem.switch_focus(0);
        //Serial.println("CCW");
        break;
      case ENC_PRESS: //select
        menuSystem.call_function(ENC_PRESS);
        //Serial.println("PRESS");
        break;
      case ENC_NC: //invalid, shouldn't be true in this function
        //Serial.println("INVALID-NO CHANGE");
        break;
      default: //invalid
        //Serial.println("INVALID");
        break;
    }
  }
  else if (encoder.encoderState == ENC_VAL) {

  }
  encoder.encoderEvent = ENC_NC;
}

// -------------------------------------------------------------------CALLBACKS-------------------------------------------------------------------

// Blank function, it is attached to the lines so that they become focusable.
void blankFunction() {
  return;
}

void changeMenu() {
  switch (menuSystem.get_focusedLine()) {
    case 0: //timelapse
      menuSystem.change_menu(TimelapseMenu);
      break;
    case 1: //oneshot
      menuSystem.change_menu(OneshotMenu);
      break;
    case 2:
      menuSystem.change_menu(ParallaxMenu);
      break;
    case 4:
      menuSystem.change_menu(MainMenu);
      break;
    default:
      Serial.println(menuSystem.get_focusedLine());
      Serial.println("Unexpected Line Index");
      break;
  }
  //menuSystem.change_menu(subMenu);
}

int modifyVariable() {

  return 125;
}
// -------------------------------------------------------------------CALLBACKS-------------------------------------------------------------------

void loop() {
  if (encoder.encoderEvent != ENC_NC) onEncoderEvent(); //Update menu based on encoder value
}
