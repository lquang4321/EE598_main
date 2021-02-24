/*-----02/17/2021----*/
/*-----CompactPCR----*/
#include <Arduino.h>            // If you are using Arduino IDE, you can remove or ignore this line.
#include <Adafruit_AS7341.h>    // Color sensor library
#include <Plotter.h>            // Displays 2D Graph of X vs Y
#include <Wire.h>               // I2C library
#include <U8x8lib.h>            // Display Library (Text only)
#include <RotaryEncoder.h>      // Encoder Library
#include <nrf_qdec.h>
#include <Thermistor.h>

/*--------COLOR SENSOR---------*/
Adafruit_AS7341 as7341;
/*--------COLOR SENSOR---------*/

/*--------ENCODER---------*/
#define ENC_A     15
#define ENC_B     11
int counter = 0;
/*--------ENCODER---------*/


/*--------OLED STUFF---------*/
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);  
/*--------OLED STUFF---------*/

/*--------THERMISTOR---------*/
//https://learn.adafruit.com/thermistor/using-a-thermistor

// which analog pin to connect
#define THERMISTORPIN A1
// resistance at 25 degrees C
#define THERMISTORNOMINAL 10000
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 8
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
// the value of the 'other' resistor
#define SERIESRESISTOR 10000

Thermistor tempSensor(THERMISTORPIN, THERMISTORNOMINAL,TEMPERATURENOMINAL,NUMSAMPLES,BCOEFFICIENT,SERIESRESISTOR );
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

/*--------Plotter---------*/
double w, x, z, F8_680;
int y;
Plotter p;
/*--------Plotter---------*/

/*--------Millis Delay------*/
const long eventTime_1 = 1;  //in ms
const long eventTime_2 = 50; //in ms
const long eventTime_3 = 500; //in ms

unsigned long previousTime_1 = 0;
unsigned long previousTime_2 = 0;
unsigned long previousTime_3 = 0;
/*--------Millis Delay------*/

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



void setup(void) {
    Serial.begin(115200);
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

    u8x8.begin();
    u8x8.setFont(u8x8_font_5x7_f);

    // Start plotter
    p.Begin();

    // Add 5 variable time graph
    p.AddTimeGraph( "PID", 800, "PID Output %", w, "Setpoint", x, "Temp", y, "Error", z, "F8_680 RED", F8_680 );

    RotaryEncoder.begin(ENC_A, ENC_B);  // Initialize Encoder
    RotaryEncoder.start();              // Start encoder
    RotaryEncoder.setDebounce(true);
}


void loop(void) {
    unsigned long currentTime = millis();

    int value = RotaryEncoder.read();

    if (value)
    {
        if ( value > 0 )
        {
            counter++;
        }else
        {
            counter--;
        }
    }    

    if ( currentTime - previousTime_1 >= eventTime_1) {
        as7341.readAllChannels( );
        F8_680 = as7341.getChannel(AS7341_CHANNEL_680nm_F8);

        previousTime_1 = currentTime;
    }

    //Serial.print("TargetTemp:");
    //Serial.print(Setpoint);
    //Serial.print(" Temp:");
    //Serial.print(getTemp());
    Setpoint = map(analogRead(A4), 0, 1023, 25, 80);
    Input = tempSensor.temperature;
    //Serial.print(" Error:");
    //Serial.print(Setpoint - Input);
    Compute();

    //Serial.print(" PID_Output:");
    //Serial.println(Output);
    w = map(Output, 0, 255, 0, 100);
    x = Setpoint;
    y = tempSensor.temperature;
    z = Setpoint - Input;


    if ( currentTime - previousTime_2 >= eventTime_2) {
        u8x8.setCursor(0,0);
        u8x8.print("SetTmp: ");
        u8x8.print(x);
        u8x8.print("C");

        u8x8.setCursor(0,17);
        u8x8.print("CurTmp: ");
        u8x8.print(y);
        u8x8.print("C");

        u8x8.setCursor(0,34);
        u8x8.print("680nm: ");
        u8x8.print(F8_680);
        
        u8x8.setCursor(0,51);
        u8x8.print("Encoder: ");
        u8x8.print(counter);

        previousTime_2 = currentTime;
    }
    if ( currentTime - previousTime_3 >= eventTime_3) {
        u8x8.clearDisplay();
        previousTime_3 = currentTime;
    }

    p.Plot(); // usually called within loop()

    analogWrite(HeaterPIN, Output);

}
