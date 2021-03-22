/*------03/15/2021------*/
#include <Adafruit_I2CDevice.h>
#include <Arduino.h>
#include <Wire.h>
#include <RotaryEncoder.h>
#include <InternalFileSystem.h>

#include <menu.h>
#include <menuIO/serialOut.h>
#include <menuIO/serialIn.h>
#include <menuIO/chainStream.h>
#include <menuIO/rotaryEventIn.h>
#include <menuIO/adafruitGfxOut.h>
#include <plugin/barField.h> // numeric field edit with a graph bar
#include <AceButton.h>

#include <Adafruit_SSD1306.h>
#include <Plotter.h>

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
double F3_480;
/*--------COLOR SENSOR---------*/

/*--------ENCODER---------*/
#define ENC_A 30
#define ENC_B 11
#define ROTARY_PIN_BUT 27
AceButton button(ROTARY_PIN_BUT, /*default is pull-up high*/HIGH);             

RotaryEventIn reIn(
  RotaryEventIn::EventType::BUTTON_CLICKED | // select
  RotaryEventIn::EventType::BUTTON_DOUBLE_CLICKED | // back
  RotaryEventIn::EventType::BUTTON_LONG_PRESSED | // also back
  RotaryEventIn::EventType::ROTARY_CCW | // up
  RotaryEventIn::EventType::ROTARY_CW // down
);

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

#define THERMISTORPIN A2        // which analog pin to connect
#define THERMISTORNOMINAL 10000 // resistance at 25 degrees C
#define TEMPERATURENOMINAL 25   // temp. for nominal resistance (almost always 25 C)
#define NUMSAMPLES 10           // how many samples to take and average, more takes longer but smoother
#define BCOEFFICIENT 3950       // The beta coefficient of the thermistor (usually 3000-4000)
#define SERIESRESISTOR 10000    // the value of the 'other' resistor    

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

double setTemp = 25;
/*--------PID---------*/

/*--------DISPLAY---------*/
#define fontX 5
#define fontY 9
#define WIDTH 128
#define HEIGHT 64

//U8G2_SSD1306_128X64_ALT0_F_2ND_HW_I2C u8g2(U8G2_R0);
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
double w, x, z;
int y;
Plotter p;
/*--------Plotter---------*/

/*--------Millis Delay------*/
const long eventTime_1 = 2000;  //in ms
const long eventTime_2 = 75;  //in ms
const long eventTime_3 = 100;  //in ms
const long eventTime_4 = 100;  //in ms

unsigned long previousTime_1 = 0;
unsigned long previousTime_2 = 0;
unsigned long previousTime_3 = 0;
unsigned long previousTime_4 = 0;
/*--------Millis Delay------*/

/*--------PCR LOGIC---------*/
uint16_t    denatureTemp = 25,
            annealTemp = 25,
            extendTemp = 25,
            cycle = 5;

uint16_t    currDenatureTemp = 25,
            currAnnealTemp = 25,
            currExtendTemp = 25,
            currCycle = 0;

uint16_t    timeOn = 100;                                                   //Indicator LED to make sure MCU has no stutter/hiccup
uint16_t    timeOff = 100;
int ledCtrl = LOW;
bool blink(int timeOn,int timeOff) 
{
    return millis()%(unsigned long)(timeOn+timeOff)<(unsigned long)timeOn;
}

/*--------PID---------*/
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
/*--------PID---------*/

/*--------Temp Reading---------*/
float getTemp(void){
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

float smoothedTemp = 0; 
float tempSmoother = 0.05;
float tempLog[WIDTH];
long tempLogInterval = 500;
long lastTempLog = 0;
long numTempSamples = 0;

float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
     return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float fmapConstrained(float x, float in_min, float in_max, float out_min, float out_max)
{
    float f = fmap( x,  in_min, in_max, out_min, out_max);

    if( f < out_min )
        f = out_min;

    if( f > out_max )
        f = out_max;

    return f;
}
void setGraphDirty();
void loopTempRead(void)
{
    smoothedTemp = (1.0 - tempSmoother) * smoothedTemp + tempSmoother * Input;

    long now = millis();
    if( now - lastTempLog  > tempLogInterval)
    {
        // shift array down
        for( int i = 0; i < WIDTH - 1; i ++ )
            tempLog[i] = tempLog[i+1];

        // add new entry
        tempLog[WIDTH - 1] = smoothedTemp;  
        lastTempLog = now;
        setGraphDirty();
        numTempSamples++;
    }
}
/*--------Temp Reading---------*/

/*--------Batt Reading---------*/
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
/*--------Batt Reading---------*/

/*--------GUI---------*/
class tempTimeline:public prompt {
    public:
        unsigned int t=0;
        unsigned int last=0;

        tempTimeline(constMEM promptShadow& p):prompt(p) {}
        Used printTo(navRoot &root,bool sel,menuOut& out, idx_t idx,idx_t len,idx_t panelNr) override {
            last=t;
            
            gfxOut*g = (gfxOut*)&out;
            adaGfxOut*u = (adaGfxOut*)&out;

            display.setTextColor(u->getColor(fgColor,sel,enabledStatus,false));
            const panel p=out.panels[panelNr];

            int bottom = (p.y+idx+2) * g->resY;             
            int height = g->resY;
            int top = bottom-height;
            
            if( bottom < 0 || top >  display.width())
                {
                    //Serial.println("not drawing");
                    return 0; // not on screen
                }
                //Serial.print(millis()); Serial.println("- drawing");
                for( int x = p.x; x < display.width();x++)
                {
                    int barHeight = 1 + fmap(tempLog[x], 0.0, 250.0, 0.0, (float)g->resY);
                    if( barHeight < 1 )
                        barHeight = 1;
                    
                    int y = bottom-barHeight; // actually the top of the bar, as y=0 is at the top
                    int h = barHeight;

                    // make the axis line crawl
                    if( (numTempSamples+x)%4 != 0)
                    {
                        y+=1;
                        h-=1;
                    }
                    
                    
                    display.drawFastVLine(x,y,h, SSD1306_WHITE);
                }
            //display.drawFastVLine(random(0,127),40,random(0,20), SSD1306_WHITE);
            

            return 1;
        }

        // virtual bool changed(const navNode &nav,const menuOut& out,bool sub=true) {

        //     // is never called!
            
        //     t = millis()/tempLogInterval;
        //     if( last!=t )
        //     {
        //         Serial.println("changed");
        //         return true;
        //     }
        //     else
        //     {
        //         Serial.println("Not changed");
        //         return false;
        //     }
        // }
};

MENU(checkSensors, "SENSORS READ",doNothing,noEvent,wrapStyle                  //Working
    ,FIELD(y,"Curr Temp: ","C",25,150,0,0,doNothing, noEvent, noStyle) 
    ,FIELD(battVOLT,"Batt Voltage: ","V",2,5,0,0,doNothing, noEvent, noStyle) 
    ,FIELD(w,"PID Output: ","%",0,100,0,0,doNothing, noEvent, noStyle) 
    ,FIELD(F3_480,"480nm: ","",0,100000,0,0,doNothing, noEvent, noStyle)
    ,EXIT("<Back")
);

MENU(setConfig, "SETUP",doNothing,noEvent,wrapStyle                  //Working
    //,FIELD(Setpoint,"Set Temp: ","*C",25,100,1,0, doNothing, noEvent, noStyle)         //Works
    ,FIELD(denatureTemp,"Denature: ","c",25,100,1,0,doNothing,noEvent,noStyle)
    ,FIELD(annealTemp,"Anneal: ","c",25,100,1,0,doNothing,noEvent,noStyle)
    ,FIELD(extendTemp,"Extension: ","c",25,100,1,0,doNothing,noEvent,noStyle)
    ,FIELD(cycle,"Cycle: ","",0,100,1,0,doNothing,noEvent,noStyle)
    ,EXIT("<Back")
);

MENU(about, "ABOUT",doNothing,noEvent,wrapStyle                                          //Works
    ,OP("CCNYSenior II Spr'21", doNothing, noEvent)                                    //Works
    ,OP("      AUTHOR        ", doNothing, noEvent)                                      //Works  
    ,OP("Quang T, Yossel N,", doNothing, noEvent)                                       //Works
    ,OP("Gnimdou T, Yousra T", doNothing, noEvent)                                      //Works    
    ,OP("      MENTOR        ", doNothing, noEvent)                                     //Works 
    ,OP("Prof. Sang-woo Seo", doNothing, noEvent)                                        //Works
    ,EXIT("<Back")
);
int test=0;
MENU(pcrRun, "RUN",doNothing,noEvent,wrapStyle                                          //Works
    ,FIELD(Input,"CurrTemp: ","C",25,150,0,0,doNothing, noEvent, noStyle)           //
    ,FIELD(currCycle,"CurrCycle: ","C",0,100,0,0,doNothing, noEvent, noStyle)
    ,altOP(tempTimeline,"",doNothing,noEvent)
    ,EXIT("<Back")
);
MENU(mainMenu,"MAIN",doNothing,noEvent,wrapStyle
    //,OP("Run", ledToggle, enterEvent)                                                //Works
    //,FIELD(timeOff,"OffTime: ","ms",1,1000,10,1,doNothing, noEvent, noStyle)         //Works
    ,SUBMENU(pcrRun)
    ,SUBMENU(setConfig)
    ,SUBMENU(about)
    ,SUBMENU(checkSensors)
    // ,OP("LED On",myLedOn,enterEvent)
    // ,OP("LED Off",myLedOff,enterEvent)
    //,EXIT("<Back")
);

void setGraphDirty()
{
    pcrRun[1].dirty=true;       //2nd Option within pcrRun menu, auto update/scroll graph
}
#define MAX_DEPTH 3

MENU_OUTPUTS(out,MAX_DEPTH
    //,U8G2_OUT(u8g2,colors,fontX,fontY,0,0,{0,0,WIDTH/fontX,HEIGHT/fontY})
    ,ADAGFX_OUT(display,colors,fontX,fontY,{0,0,WIDTH/fontX,HEIGHT/fontY})
    ,SERIAL_OUT(Serial)
    ,NONE
);

serialIn serial(Serial);        //Serial input
MENU_INPUTS(in,&reIn);          //Physical input

NAVROOT(nav,mainMenu,MAX_DEPTH,in,out);


result idle(menuOut& o,idleEvent e) {
    o.clear();
    switch(e) {
        case idleStart:o.println("suspending menu!");break;
        case idling:o.println("suspended...");break;
        case idleEnd:o.println("resuming menu.");break;
    }
    return proceed;
}
/*--------GUI---------*/



void setup(void)
{
    //analogReference(3.43);
    SetOutputLimits(0, 255);

    SetTunings(9, 0.8, 6);
    SetMode(AUTOMATIC);

    Setpoint = 30.0;

    Wire.beginTransmission(0x39);
    as7341.begin();
    as7341.setATIME(100);
    as7341.setASTEP(999);
    as7341.setGain(AS7341_GAIN_256X);
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
    
    RotaryEncoder.begin(ENC_A, ENC_B);
    RotaryEncoder.setDebounce(true);
    //RotaryEncoder.setSampler(5120);
    RotaryEncoder.start();

    display.begin(SSD1306_EXTERNALVCC, 0x3c);
    display.clearDisplay();
    delay(100);
    display.display();

    for( uint16_t i = 0; i < WIDTH; i++)
    {
        tempLog[i] = 100;
    }
    about[0].enabled = disabledStatus;
    about[1].enabled = disabledStatus;
    about[2].enabled = disabledStatus;

    nav.idleTask=idle;//point a function to be used when menu is suspended

    //Serial.begin(115200);
    // Start plotter
    p.Begin();

    // Add 5 variable time graph
    p.AddTimeGraph( "PID", 800, "PID Output %", w, "Setpoint", x, "Temp", y, "Error", z, "F3_480 RED", F3_480 );
    F3_480 = 0.0;
}

void loop(void)
{
    loopTempRead();
    unsigned long currentTime = millis();
    battVOLT = readBAT(VBATPIN);
    
    if ( currentTime - previousTime_1 >= eventTime_1 ){
        // as7341.readAllChannels();
        // F3_480 = as7341.getChannel(AS7341_CHANNEL_480nm_F3);    

        previousTime_1 = currentTime;
    }

    if ( currentTime - previousTime_2 >= eventTime_2) {
        encoder( RotaryEncoder.read() );
        previousTime_2 = currentTime;
    }

    //Setpoint = map(analogRead(A4), 0, 1023, 25, 80);
    Input = getTemp();
    w = map(Output, 0, 255, 0, 100);
    x = Setpoint;
    y = getTemp();
    z = Setpoint - Input;
    Compute();
    
    analogWrite(HeaterPIN, Output);

    button.check();
    digitalWrite(LED_BUILTIN, blink(timeOn, timeOff));
    p.Plot(); // usually called within loop()

    nav.poll();              //Polling based, laggy inputs but better for time sensitive application
    display.display(); 

    // nav.doInput();              //Event Based. This display method is more responsive, but lags background process
    // if (nav.changed(0)) {       //only draw if changed
    //     nav.doOutput();
    //     display.display();
    // }
}
