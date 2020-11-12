#include <Arduino.h>

/********************
  Sept. 2014 Rui Azevedo - ruihfazevedo(@rrob@)gmail.com
  menu output to standard arduino LCD (LiquidCrystal)
  output: LCD
  input: encoder and Serial
  www.r-site.net
***/
#include <Wire.h>
#include <menu.h>
#include <menuIO/liquidCrystalOut.h>
#include <menuIO/serialOut.h>
#include <menuIO/serialIn.h>
#include <menuIO/encoderIn.h>
#include <menuIO/keyIn.h>
#include <menuIO/chainStream.h>
#include <AccelStepper.h>
#include "PinMap.h"

using namespace Menu;

AccelStepper stepperX(AccelStepper::DRIVER, X_STEP, X_DIR);

//--------------------------------------------------------LCD--------------------------------------------------------
#define LCD_RS 12
#define LCD_EN 11
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
//--------------------------------------------------------LCD--------------------------------------------------------

//--------------------------------------------------------Encoder--------------------------------------------------------
#define ENC_A 3
#define ENC_B 2
//this encoder has a button here
#define ENC_PB 4
#define LEDPIN 13


encoderIn<ENC_A, ENC_B> encoder; //simple quad encoder driver
encoderInStream<ENC_A, ENC_B> encStream(encoder, 4); // simple quad encoder fake Stream

//a keyboard with only one key as the encoder button
keyMap encBtn_map[] = {{ -ENC_PB, defaultNavCodes[enterCmd].ch}}; //negative pin numbers use internal pull-up, this is on when low
keyIn<1> encButton(encBtn_map);//1 is the number of keys

//input from the encoder + encoder button + serial
serialIn serial(Serial);
menuIn* inputsList[] = {&encStream, &encButton, &serial};
chainStream<3> in(inputsList);//3 is the number of inputs
//--------------------------------------------------------Encoder--------------------------------------------------------
int stepperEnable = HIGH;
int stepperDistance = 0;
result turnOnStepper() {
  digitalWrite(X_EN, stepperEnable);
  Serial.println("Toggled Stepper");
  Serial.println(stepperEnable);
  return proceed;
}

result translateStepper(){
  Serial.println("Translating Stepper");
  stepperX.moveTo(stepperDistance);
  return proceed;
}

TOGGLE(stepperEnable,enableStepper,"Stepper: ",turnOnStepper,enterEvent,noStyle//,doExit,enterEvent,noStyle
  ,VALUE("On",LOW,doNothing,noEvent)
  ,VALUE("Off",HIGH,doNothing,noEvent)
);

MENU(timelapseMenu, "Timelapse Mode", doNothing, noEvent, wrapStyle
  ,FIELD(stepperDistance,"Distance","cm",0,2000,100,10,translateStepper,exitEvent,wrapStyle)
  ,EXIT("<Back")
  );

MENU(mainMenu,"Main menu",doNothing,noEvent,wrapStyle
  ,SUBMENU(enableStepper)
  ,SUBMENU(timelapseMenu)
);

//const panel panels[] MEMMODE={{0,0,16,2}};
//navNode* nodes[sizeof(panels)/sizeof(panel)];
//panelsList pList(panels,nodes,1);

#define MAX_DEPTH 2
/*idx_t tops[MAX_DEPTH];
  liquidCrystalOut outLCD(lcd,tops,pList);//output device for LCD
  menuOut* constMEM outputs[] MEMMODE={&outLCD};//list of output devices
  outputsList out(outputs,1);//outputs list with 2 outputs*/

MENU_OUTPUTS(out, MAX_DEPTH
             , LIQUIDCRYSTAL_OUT(lcd, {0, 0, 16, 2})
             , NONE
            );
NAVROOT(nav, mainMenu, MAX_DEPTH, in, out); //the navigation root object

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

void setup() {
  pinMode(ENC_PB, INPUT_PULLUP);
  pinMode(LEDPIN, OUTPUT);
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Arduino Menu Library"); Serial.flush();
  encoder.begin();
  lcd.begin(16, 2);
  nav.idleTask = idle; //point a function to be used when menu is suspended
  nav.showTitle = false;
  lcd.setCursor(0, 0);
  lcd.print("Camera Ctrl");
  lcd.setCursor(0, 1);
  lcd.print("Stefan B.");

  stepperX.setMaxSpeed(5000);
  stepperX.setSpeed(5000);
  stepperX.setAcceleration(100);
  delay(2000);
}

void loop() {
  nav.poll();
  stepperX.run();
  //delay(100);//simulate a delay as if other tasks are running
}
