/*------03/13/2021------*/
#include <Adafruit_I2CDevice.h>
#include <Arduino.h>
#include <Wire.h>
#include <RotaryEncoder.h>
#include <menu.h>
#include <menuIO/serialOut.h>
#include <menuIO/serialIn.h>
#include <menuIO/chainStream.h>
#include <menuIO/rotaryEventIn.h>
#include <menuIO/adafruitGfxOut.h>
#include <Adafruit_SSD1306.h>
#include <Plotter.h>

#include <AceButton.h> // https://github.com/bxparks/AceButton

using namespace Menu;
using namespace ::ace_button;




/*--------READ BAT VOLTAGE---------*/
#define VBATPIN A7
#define VBAT_MV_PER_LSB   (0.73242188F)   // for 12bit
#define VBAT_DIVIDER      (0.71275837F)   // 2M + 0.806M voltage divider on VBAT = (2M / (0.806M + 2M))
#define VBAT_DIVIDER_COMP (1.403F)        // Compensation factor for the VBAT divider
#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)
float battVOLT;
/*--------READ BAT VOLTAGE---------*/

/*--------COLOR SENSOR---------*/
#include <Adafruit_AS7341.h>
Adafruit_AS7341 as7341;
/*--------COLOR SENSOR---------*/

/*--------ENCODER---------*/
#define ROTARY_PIN_BUT 27
AceButton button(ROTARY_PIN_BUT, /*default is pull-up high*/HIGH);             

RotaryEventIn reIn(
  RotaryEventIn::EventType::BUTTON_CLICKED | // select
  RotaryEventIn::EventType::BUTTON_DOUBLE_CLICKED | // back
  RotaryEventIn::EventType::BUTTON_LONG_PRESSED | // also back
  RotaryEventIn::EventType::ROTARY_CCW | // up
  RotaryEventIn::EventType::ROTARY_CW // down
); // register capabilities, see AndroidMenu MenuIO/RotaryEventIn.h file

void encoder( int8_t step)                                                                      //Encoder rotation 
{
    if( step)
    {
        if( step > 0 /* 1 == CW, -1 == CCW, 0 == idle*/ )
        {
            reIn.registerEvent(RotaryEventIn::EventType::ROTARY_CW);
        } else
        {
            reIn.registerEvent(RotaryEventIn::EventType::ROTARY_CCW);
        }
    }
}

void handleButtonEvent(AceButton* /* button */, uint8_t eventType, uint8_t buttonState) {       //Handles button (click, double click, click&hold)

  switch (eventType) {
    case AceButton::kEventClicked:
      reIn.registerEvent(RotaryEventIn::EventType::BUTTON_CLICKED);
      break;
    case AceButton::kEventDoubleClicked:
      reIn.registerEvent(RotaryEventIn::EventType::BUTTON_DOUBLE_CLICKED);
      break;
    case AceButton::kEventLongPressed:
      reIn.registerEvent(RotaryEventIn::EventType::BUTTON_LONG_PRESSED);
      break;
  }
}

/*--------ENCODER---------*/

/*--------THERMISTOR---------*/
//https://learn.adafruit.com/thermistor/using-a-thermistor

// which analog pin to connect
#define THERMISTORPIN A2
// resistance at 25 degrees C
#define THERMISTORNOMINAL 10000
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 10
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
// the value of the 'other' resistor
#define SERIESRESISTOR 10000

int samples[NUMSAMPLES];
/*--------THERMISTOR---------*/

/*--------PID---------*/
// http://brettbeauregard.com/blog/2017/06/introducing-proportional-on-measurement/

#define HeaterPIN 16

unsigned long lastTime;
double Input, Output, Setpoint;
double ITerm, lastInput;
double kp = 0, ki = 0, kd = 0;
int SampleTime = 100; //1 sec
double outMin, outMax;
bool inAuto = false;

#define MANUAL 0
#define AUTOMATIC 1
/*--------PID---------*/

/*--------DISPLAY---------*/
#define fontX 6
#define fontY 9
#define WIDTH 128
#define HEIGHT 64

Adafruit_SSD1306 display(WIDTH,HEIGHT);
const colorDef<uint16_t> colors[6] MEMMODE={
  {{BLACK,WHITE},{BLACK,WHITE,WHITE}},//bgColor
  {{WHITE,BLACK},{WHITE,BLACK,BLACK}},//fgColor
  {{WHITE,BLACK},{WHITE,BLACK,BLACK}},//valColor
  {{WHITE,BLACK},{WHITE,BLACK,BLACK}},//unitColor
  {{WHITE,BLACK},{BLACK,BLACK,BLACK}},//cursorColor
  {{WHITE,BLACK},{BLACK,WHITE,WHITE}},//titleColor
};
/*--------DISPLAY---------*/

/*--------Plotter---------*/
double w, x, z, F8_680;
int y;
Plotter p;
/*--------Plotter---------*/

/*--------Millis Delay------*/
const long eventTime_1 = 200;  //in ms
const long eventTime_2 = 200;  //in ms
const long eventTime_3 = 200;  //in ms
const long eventTime_4 = 200;  //in ms

unsigned long previousTime_1 = 0;
unsigned long previousTime_2 = 0;
unsigned long previousTime_3 = 0;
unsigned long previousTime_4 = 0;
/*--------Millis Delay------*/

/*--------MISC---------*/
uint16_t    denatureTemp = 25,
            annealTemp = 25,
            extendTemp = 25;

uint16_t    cycle = 5;

uint16_t    timeOn = 100;
uint16_t    timeOff = 100;

int ledCtrl=LOW;
bool blink(int timeOn,int timeOff) 
{
    return millis()%(unsigned long)(timeOn+timeOff)<(unsigned long)timeOn;
}
/*--------MISC---------*/

void Compute()
{
   if(!inAuto) return;
   unsigned long now = millis();
   int timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      /*Compute all the working error variables*/
      double error = Setpoint - Input;
      ITerm+= (ki * error);
      if(ITerm> outMax) ITerm= outMax;
      else if(ITerm< outMin) ITerm= outMin;
      double dInput = (Input - lastInput);

      /*Compute PID Output*/
      Output = kp * error + ITerm- kd * dInput;
      if(Output> outMax) Output = outMax;
      else if(Output < outMin) Output = outMin;

      /*Remember some variables for next time*/
      lastInput = Input;
      lastTime = now;
   }
}


void SetTunings(double Kp, double Ki, double Kd)
{
  double SampleTimeInSec = ((double)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
}

void SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}

void SetOutputLimits(double Min, double Max)
{
   if(Min > Max) return;
   outMin = Min;
   outMax = Max;

   if(Output > outMax) Output = outMax;
   else if(Output < outMin) Output = outMin;

   if(ITerm> outMax) ITerm= outMax;
   else if(ITerm< outMin) ITerm= outMin;
}

void Initialize()
{
   lastInput = Input;
   ITerm = Output;
   if(ITerm> outMax) ITerm= outMax;
   else if(ITerm< outMin) ITerm= outMin;
}

void SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto && !inAuto)
    {  /*we just went from manual to auto*/
        Initialize();
    }
    inAuto = newAuto;
}

float getTemp(){
    uint8_t i;
    float average;
    
    // take N samples in a row, with a slight delay
    for (i=0; i< NUMSAMPLES; i++) {
        samples[i] = analogRead(THERMISTORPIN);
    }
    
    // average all the samples out
    average = 0;
    for (i=0; i< NUMSAMPLES; i++) {
        average += samples[i];
    }
    average /= NUMSAMPLES;
    
    // convert the value to resistance
    average = 1023 / average - 1;
    average = SERIESRESISTOR / average;
    
    float steinhart;
    steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
    steinhart = log(steinhart);                  // ln(R/Ro)
    steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
    steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart;                 // Invert
    steinhart -= 273.15;                         // convert absolute temp to C

    return steinhart;
}

float readBAT(int pin){
    float avg = 0;
    analogReference(AR_INTERNAL_3_0);
    analogReadResolution(12);

    for( uint8_t i = 0; i< 20; i++){
        avg += analogRead(pin);
    }
    analogReference(AR_DEFAULT);
    analogReadResolution(10);

    float measuredvbat = avg / 20;

    return measuredvbat * REAL_VBAT_MV_PER_LSB / 1000;
}




/*--------GUI---------*/
result myLedOn() {
    ledCtrl=HIGH;
    return proceed;
}
result myLedOff() {
    ledCtrl=LOW;
    return proceed;
}
result ledToggle(){
    ledCtrl = !ledCtrl;
    return proceed;
}

MENU(checkSensors, "SENSORS READ",doNothing,noEvent,wrapStyle                  //Working
    ,FIELD(y,"Current Temp: ","C",25,150,0,0,doNothing, noEvent, noStyle) 
    ,FIELD(battVOLT,"Batt Voltage: ","V",2,5,0,0,doNothing, noEvent, noStyle) 
    ,FIELD(w,"PID Output: ","%",0,100,0,0,doNothing, noEvent, noStyle) 
    ,EXIT("<Back")
);

MENU(setConfig, "SETUP",doNothing,noEvent,wrapStyle                  //Working
    ,FIELD(denatureTemp,"Denature: ","c",25,100,1,0,Menu::doNothing,Menu::noEvent,Menu::noStyle)
    ,FIELD(annealTemp,"Anneal: ","c",25,100,1,0,Menu::doNothing,Menu::noEvent,Menu::noStyle)
    ,FIELD(extendTemp,"Extension: ","c",25,100,1,0,Menu::doNothing,Menu::noEvent,Menu::noStyle)
    ,FIELD(cycle,"Cycle: ","",0,100,1,0,Menu::doNothing,Menu::noEvent,Menu::noStyle)
    ,EXIT("<Back")
);

MENU(about, "ABOUT",doNothing,noEvent,wrapStyle                         //Works
    ,OP("CCNYSenior II Spr'21", doNothing, noEvent)                             //Works
    ,OP("      AUTHOR        ", doNothing, noEvent)                                  //Works  
    ,OP("Quang T, Yossel N,", doNothing, noEvent)                                   //Works
    ,OP("Gnimdou T, Yousra T", doNothing, noEvent)                                  //Works    
    ,OP("      MENTOR        ", doNothing, noEvent)                                  //Works 
    ,OP("Prof. Sang-woo Seo", doNothing, noEvent)                           //Works
    ,EXIT("<Back")
);

MENU(mainMenu,"MAIN",doNothing,noEvent,wrapStyle
    ,OP("Run", ledToggle, enterEvent)                                                //Works
    ,FIELD(timeOn,"OnTime: ","ms",1,1000,10,1, doNothing, noEvent, noStyle)         //Works
    ,FIELD(timeOff,"OffTime: ","ms",1,1000,10,1,doNothing, noEvent, noStyle)        //Works
    ,SUBMENU(setConfig)
    ,SUBMENU(about)
    ,SUBMENU(checkSensors)
    // ,OP("LED On",myLedOn,enterEvent)
    // ,OP("LED Off",myLedOff,enterEvent)
    ,EXIT("<Back")
);

#define MAX_DEPTH 2

MENU_OUTPUTS(out,MAX_DEPTH
    ,ADAGFX_OUT(display,colors,fontX,fontY,{0,0,WIDTH/fontX,HEIGHT/fontY})
    ,SERIAL_OUT(Serial)
    ,NONE
);

serialIn serial(Serial);        //Serial input
MENU_INPUTS(in,&reIn);          //Physical input

NAVROOT(nav,mainMenu,MAX_DEPTH,in,out);

/*--------GUI---------*/




void setup(void)
{

    Wire.begin();
    //analogReference(3.43);
    SetOutputLimits(0, 255);

    SetTunings(9, 0.8, 6);
    SetMode(AUTOMATIC);

    Setpoint = 30.0;
    Wire.beginTransmission(0x39);
    as7341.begin();
    as7341.setATIME(65534);
    as7341.setASTEP(0);
    as7341.setGain(AS7341_GAIN_128X);
    as7341.startReading();
    as7341.enableLED(false);
    pinMode(LED_BUILTIN, OUTPUT);

    pinMode(ROTARY_PIN_BUT, INPUT_PULLUP);
    ButtonConfig* buttonConfig = button.getButtonConfig();
    buttonConfig->setEventHandler(handleButtonEvent);
    buttonConfig->setFeature(ButtonConfig::kFeatureClick);
    buttonConfig->setFeature(ButtonConfig::kFeatureDoubleClick);
    buttonConfig->setFeature(ButtonConfig::kFeatureLongPress);
    buttonConfig->setFeature(ButtonConfig::kFeatureSuppressClickBeforeDoubleClick);
    buttonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterClick);
    buttonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterDoubleClick);
    
    RotaryEncoder.begin(7, 30);
    RotaryEncoder.setDebounce(true);
    //RotaryEncoder.setSampler(5120);
    RotaryEncoder.start();

    display.begin(SSD1306_EXTERNALVCC, 0x3c);
    display.clearDisplay();
    delay(100);
    display.display();
    // u8x8.begin();
    // u8x8.setFont(fontName);
    // u8x8.drawString(0,0,"Hello");
    // u8x8.print("Menu 4.x");

    //Serial.begin(115200);
    // Start plotter
    p.Begin();

    // Add 5 variable time graph
    p.AddTimeGraph( "PID", 800, "PID Output %", w, "Setpoint", x, "Temp", y, "Error", z, "F8_680 RED", F8_680 );
}

void loop(void)
{
    unsigned long currentTime = millis();
    battVOLT = readBAT(VBATPIN);
    if ( currentTime - previousTime_1 >= eventTime_1) {
        as7341.readAllChannels( );
        F8_680 = as7341.getChannel(AS7341_CHANNEL_680nm_F8);

        previousTime_1 = currentTime;
    }
    Setpoint = map(analogRead(A4), 0, 1023, 25, 80);
    Input = getTemp();
    w = map(Output, 0, 255, 0, 100);
    x = Setpoint;
    y = getTemp();
    z = Setpoint - Input;
    Compute();
    p.Plot(); // usually called within loop()
    analogWrite(HeaterPIN, Output);

    encoder( RotaryEncoder.read() );
    button.check();
    digitalWrite(LED_BUILTIN, blink(timeOn, timeOff));

    //nav.poll();              //Polling based, laggy inputs but better for time sensitive application
    //display.display();    

    nav.doInput();              //Event Based. This display method is more responsive, but lags background process
    if (nav.changed(0)) {       //only draw if changed
        nav.doOutput();
        display.display();
    }
}
