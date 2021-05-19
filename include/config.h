#ifndef __CONFIG_H__
#define __CONFIG_H__

#pragma once


//Initial Stuff
#define PLOTTER //Turn on plotting through serial connection to computer
//#define DEBUG   //Turn on debugging via serial connection, it is recommended not to turn on both PLOTTER and DEBUG simultaneously
                  //otherwise the message will move too fast to read.

/*--------COLOR SENSOR---------*/
/*--------ENCODER---------*/
/*--------THERMISTOR---------*/
/*--------BAT VOLTAGE READ---------*/
/*--------PID---------*/
/*--------DISPLAY---------*/
/*--------Plotter---------*/
/*--------Timing Stuff------*/
/*--------Temp Reading---------*/
/*--------PCR LOGIC---------*/

/*--------COLOR SENSOR---------*/
Adafruit_AS7341 as7341;
double F4_515, detectAVG;               //Must be double or compiler screams, probably Plotter only accepts double not int
/*--------COLOR SENSOR---------*/

/*--------ENCODER---------*/
#define ENC_A 30
#define ENC_B 11
#define ROTARY_PIN_BUT 27

using namespace ::ace_button;

AceButton button(ROTARY_PIN_BUT, /*default is pull-up high*/HIGH);             

RotaryEventIn reIn(
  RotaryEventIn::EventType::BUTTON_CLICKED | // select
  RotaryEventIn::EventType::BUTTON_DOUBLE_CLICKED | // back
  RotaryEventIn::EventType::BUTTON_LONG_PRESSED | // also back
  RotaryEventIn::EventType::ROTARY_CCW | // up
  RotaryEventIn::EventType::ROTARY_CW // down
);

//Encoder rotation 
void encoder( int8_t step)                                                                     
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

//Handles button (click, double click, click&hold)
void handleButtonEvent(AceButton* /* button */, uint8_t eventType, uint8_t buttonState) {      

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

#define THERMISTORNOMINAL 9980 // resistance at 25 degrees C
#define TEMPERATURENOMINAL 25   // temp. for nominal resistance (almost always 25 C)
#define NUMSAMPLES 10           // how many samples to take and average, more takes longer but smoother
#define BCOEFFICIENT 3950       // The beta coefficient of the thermistor (usually 3000-4000)
#define SERIESRESISTOR 9980    // the value of the 'other' resistor    

int samples[NUMSAMPLES];
/*--------THERMISTOR---------*/

/*--------BAT VOLTAGE READ---------*/
#define VBATPIN A7
float battVOLT;

#define VBAT_MV_PER_LSB   (0.73242188F)   // for 12bit
#define VBAT_DIVIDER      (0.71275837F)   // 2M + 0.806M voltage divider on VBAT = (2M / (0.806M + 2M))
#define VBAT_DIVIDER_COMP (1.403F)        // Compensation factor for the VBAT divider
#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)

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
/*--------BAT VOLTAGE READ---------*/

/*--------PID---------*/
// http://brettbeauregard.com/blog/2017/06/introducing-proportional-on-measurement/

#define HeaterPIN 16

uint8_t Output_Percent = 0;
double Input, Output, Setpoint;
double ITerm, lastInput;
double kp = 9, ki = 0.8, kd = 6;
int SampleTime = 100; //1 sec

PID myPID(&Input, &Output, &Setpoint, kp, ki, kd, DIRECT);
// void Compute()
// {
//    if(!inAuto) return;
//    unsigned long now = millis();
//    int timeChange = (now - lastTime);
//    if(timeChange>=SampleTime)
//    {
//       /*Compute all the working error variables*/
//       double error = Setpoint - Input;
//       ITerm+= (ki * error);
//       if(ITerm> outMax) ITerm= outMax;
//       else if(ITerm< outMin) ITerm= outMin;
//       double dInput = (Input - lastInput);

//       /*Compute PID Output*/
//       Output = kp * error + ITerm- kd * dInput;
//       if(Output> outMax) Output = outMax;
//       else if(Output < outMin) Output = outMin;

//       /*Remember some variables for next time*/
//       lastInput = Input;
//       lastTime = now;
//    }
// }


// void SetTunings(double Kp, double Ki, double Kd)
// {
//     double SampleTimeInSec = ((double)SampleTime)/1000;
//     kp = Kp;
//     ki = Ki * SampleTimeInSec;
//     kd = Kd / SampleTimeInSec;
// }

// void SetSampleTime(int NewSampleTime)
// {
//     if (NewSampleTime > 0)
//     {
//         double ratio  = (double)NewSampleTime
//                         / (double)SampleTime;
//         ki *= ratio;
//         kd /= ratio;
//         SampleTime = (unsigned long)NewSampleTime;
//     }
// }

// void SetOutputLimits(double Min, double Max)
// {
//     if(Min > Max) return;
//     outMin = Min;
//     outMax = Max;

//     if(Output > outMax) Output = outMax;
//     else if(Output < outMin) Output = outMin;

//     if(ITerm> outMax) ITerm= outMax;
//     else if(ITerm< outMin) ITerm= outMin;
// }

// void Initialize()
// {
//     lastInput = Input;
//     ITerm = Output;
//     if(ITerm> outMax) ITerm= outMax;
//     else if(ITerm< outMin) ITerm= outMin;
// }

// void SetMode(int Mode)
// {
//     bool newAuto = (Mode == AUTOMATIC);
//     if(newAuto && !inAuto)
//     {  /*we just went from manual to auto*/
//         Initialize();
//     }
//     inAuto = newAuto;
// }

/*--------PID---------*/

/*--------DISPLAY---------*/
#define fontX 5
#define fontY 9
#define WIDTH 128
#define HEIGHT 64
/*--------DISPLAY---------*/

/*--------Plotter---------*/
//#define PLOTTER
#ifdef PLOTTER
double w, x, z;
int y;
Plotter p;
#endif

/*--------Plotter---------*/


/*--------Timing Stuff------*/
unsigned long currentTime;

const long eventTime_1 = 2000;  //in ms
const long eventTime_2 = 75;  //in ms
const long eventTime_3 = 100;  //in ms

unsigned long previousTime_1 = 0;
unsigned long previousTime_2 = 0;
unsigned long previousTime_3 = 0;

SoftwareTimer swTimer;
/*--------Timing Stuff------*/

/*--------DISPLAY---------*/
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
/*--------Temp Reading---------*/

/*--------PCR LOGIC---------*/
bool        isRunning = false;

#define INITIAL 0
#define DENATURE 1
#define ANNEAL 2
#define EXTENSION 3
#define FINAL 4

uint8_t         currStageCount = 0;
const String    currStage[6] = {"Stage: Initial", "Stage: Denature", "Stage: Anneal", "Stage: Extension", "Stage: Final", "Stage: PCR DONE"};

const String    runStatus[3] = {">>Press to Run<<", ">>Press to Cancel<<"};
uint16_t        tempSetting[5] = {80, 90 ,55, 65, 60};  //Init, Denature, Anneal, Extension, and Final temperature
unsigned long   timeSetting[5] = {5, 10 ,30, 10, 5};  //Init, Denature, Anneal, Extension, and Final time (milli sec) spend in each stage

uint16_t        setCycle = 5,
                setTemp = 25;

unsigned long   currCycleTime = 0;      //in millisecc
unsigned long   currentTime_sec;      //in sec
unsigned long   cycleTime = 0;          //in millisecc
unsigned long   previousTime_Cycle = 0; //in millisecc

uint16_t        currTemp = 0,
                currCycle = 0;

uint16_t        timeOn = 100;                                                   //Indicator LED to make sure MCU has no stutter/hiccup
uint16_t        timeOff = 100;

bool ledTOGGLE = LOW;

void runPCR()
{
    bool blink(int timeOn,int timeOff); 
    if (isRunning == true && currCycle < setCycle)
    {
        as7341.setLEDCurrent(12); // 4mA
        as7341.enableLED(true);
        Setpoint = setTemp;                 //Desired output for PID
        Serial.print("PCR RUNNING, ");

        if(currStageCount == INITIAL)
        {
            cycleTime = timeSetting[INITIAL];
            setTemp = tempSetting[INITIAL];

            if ( currentTime_sec > cycleTime )
            { 
                Serial.println("INITIAL DONE");
                digitalWrite(LED_BLUE, LOW); 
                swTimer.reset();
                currentTime_sec = 0;
                previousTime_Cycle = 0;
                currStageCount++; 
                Serial.println(currStageCount);

                //previousTime_Cycle = currentTime_sec;
            }else{ 
                digitalWrite(LED_BLUE, blink(timeOn,timeOff)); 
                Serial.print(currentTime_sec);
                Serial.println(" INITIAL RUNNING");
            }
        }else if(currStageCount == DENATURE)
        {
            cycleTime = timeSetting[DENATURE];
            setTemp = tempSetting[DENATURE];

            if ( currentTime_sec > cycleTime )
            { 
                Serial.println("DENATURE DONE");
                digitalWrite(LED_BLUE, LOW); 
                swTimer.reset();
                currentTime_sec = 0;
                previousTime_Cycle = 0;
                currStageCount++; 
                Serial.println(currStageCount);

            }else{ 
                digitalWrite(LED_BLUE, blink(timeOn,timeOff)); 
                Serial.print(currentTime_sec);
                Serial.println(" DENATURE RUNNING");
            }
        }else if(currStageCount == ANNEAL)
        {
            cycleTime = timeSetting[ANNEAL];
            setTemp = tempSetting[ANNEAL];

            if ( currentTime_sec > cycleTime )
            { 
                Serial.println("ANNEAL DONE");
                digitalWrite(LED_BLUE, LOW); 
                swTimer.reset();
                currentTime_sec = 0;
                previousTime_Cycle = 0;
                currStageCount++; 
                Serial.println(currStageCount);

            }else{ 
                digitalWrite(LED_BLUE, blink(timeOn,timeOff)); 
                Serial.print(currentTime_sec);
                Serial.println(" ANNEAL RUNNING");
            }
        }else if(currStageCount == EXTENSION)
        {
            cycleTime = timeSetting[EXTENSION];
            setTemp = tempSetting[EXTENSION];

            if ( currentTime_sec > cycleTime )
            { 
                Serial.println("EXTENSION DONE");
                digitalWrite(LED_BLUE, LOW); 
                swTimer.reset();
                currentTime_sec = 0;
                previousTime_Cycle = 0;
                currStageCount++; 
                Serial.println(currStageCount);

            }else{ 
                digitalWrite(LED_BLUE, blink(timeOn,timeOff)); 
                Serial.print(currentTime_sec);
                Serial.println(" EXTENSION RUNNING");
            }
        }else if(currStageCount == FINAL)
        {
            cycleTime = timeSetting[FINAL];
            setTemp = tempSetting[FINAL];

            if ( currentTime_sec > cycleTime )
            { 
                Serial.println("FINAL DONE");
                digitalWrite(LED_BLUE, LOW); 
                swTimer.reset();
                currentTime_sec = 0;
                previousTime_Cycle = 0;

                as7341.readAllChannels();
                F4_515 = as7341.getChannel(AS7341_CHANNEL_515nm_F4);   
                detectAVG += F4_515;
                detectAVG /= (currCycle+1);

                currStageCount++; 
                Serial.println(currStageCount);

            }else{ 
                digitalWrite(LED_BLUE, blink(timeOn,timeOff));   
                Serial.print(currentTime_sec);
                Serial.println(" FINAL RUNNING");
            }
        }
        else{
            currCycle++;
            currStageCount = INITIAL;
        }

    }else{
        as7341.enableLED(false);
        swTimer.reset();
        isRunning = false;
        currStageCount = INITIAL;
        currCycle = 0;
        cycleTime = 0;
        //setTemp = 0;
        Setpoint = 0;
        currentTime_sec = 0;
        previousTime_Cycle = 0;
        detectAVG = 0;
        digitalWrite(LED_BLUE, !LOW); 
        Serial.println("PCR STOPPED");
    }
    Input = getTemp(); 
    myPID.Compute();                    //PID Math stuff in here
    analogWrite(HeaterPIN, Output);     //PID Output to heater
}

bool blink(int timeOn,int timeOff) 
{
    return millis()%(unsigned long)(timeOn+timeOff)<(unsigned long)timeOn;
}
/*--------PCR LOGIC---------*/
#endif