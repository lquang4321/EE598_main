/*------04/06/2021------*/
#include <Adafruit_I2CDevice.h>
#include <Arduino.h>
#include <Wire.h>
#include <RotaryEncoder.h>
#include <InternalFileSystem.h>
#include <PID_v1.h>
#include <utility/SoftwareTimer.h>      //For keeping cycle time

#include <menu.h>                       //GUI related stuff
#include <menuIO/serialOut.h>
#include <menuIO/serialIn.h>
#include <menuIO/chainStream.h>
#include <menuIO/rotaryEventIn.h>
#include <menuIO/adafruitGfxOut.h>
#include <AceButton.h>                  //Push Button features

#include <Adafruit_AS7341.h>            //Light sensor
#include <Adafruit_SSD1306.h>           //OLED
#include <Plotter.h>                    //Serial plotter on PC

#include <config.h>                     //This project settings and functions

using namespace Menu;

/*--------COLOR SENSOR---------*/
/*--------ENCODER---------*/
/*--------THERMISTOR---------*/
/*--------READ BAT VOLTAGE---------*/
/*--------PID---------*/
/*--------DISPLAY---------*/
/*--------Plotter---------*/
/*--------Millis Delay------*/
/*--------PCR LOGIC---------*/
/*--------PID---------*/
/*--------Temp Reading---------*/
/*--------Batt Reading---------*/

void RotaryInit(void)
{
    pinMode(ROTARY_PIN_BUT, INPUT_PULLUP);
    RotaryEncoder.begin(ENC_A, ENC_B);
    RotaryEncoder.setDebounce(true);
    RotaryEncoder.start();
}

/*--------GUI---------*/
class Stages:public prompt {
    public:
        Stages(constMEM promptShadow& p):prompt(p) {}
        Used printTo(navRoot &root,bool sel,menuOut& out, idx_t idx,idx_t len,idx_t) override {

            //return out.printRaw((constText*)(currStage[0]),len);
            return out.printRaw(String(currStage[currStageCount]).c_str(),len);
        }
};
class confirmRun:public menu {
    public:
    confirmRun(constMEM menuNodeShadow& shadow):menu(shadow) {}
        Used printTo(navRoot &root,bool sel,menuOut& out, idx_t idx,idx_t len,idx_t p) override {
            //digitalWrite(LED_BUILTIN, HIGH);
            //isRunning = true;
            return idx<0? 
            menu::printTo(root,sel,out,idx,len,p)://when printing title
            out.printRaw(String(runStatus[isRunning]).c_str(),len);//when printing as regular option
        }
};

result systemExit();

altMENU(confirmRun,startPCR,"Confirm Action?",doNothing,noEvent,noStyle,(Menu::_menuData|Menu::_canNav)
    ,OP("Yes",systemExit,enterEvent)
    ,EXIT("Cancel")
);

PADMENU(Temperature,"",doNothing,noEvent,noStyle
    ,FIELD(currTemp,"Temp:","c/",0,150,0,0,doNothing, noEvent, noStyle)           
    ,FIELD(setTemp,"","c/",0,150,0,0,doNothing, noEvent, noStyle)
    ,FIELD(Output_Percent,"","%",0,100,0,0,doNothing, noEvent, noStyle)
);

PADMENU(Cycle,"",doNothing,noEvent,wrapStyle
    ,FIELD(currCycle,"Cycle:"," /",0,150,0,0,doNothing, noEvent, noStyle)           
    ,FIELD(setCycle,"","",0,150,0,0,doNothing, noEvent, noStyle)
);

PADMENU(Cycle_Time,"",doNothing,noEvent,wrapStyle
    ,FIELD( currentTime_sec ,"Time:","s /",0,1000,0,0,doNothing, noEvent, noStyle)           
    ,FIELD(cycleTime,"","s",0,150,0,0,doNothing, noEvent, noStyle)
);

MENU(checkSensors, "SENSORS READ",doNothing,noEvent,wrapStyle                  //Working
    ,FIELD(currTemp,"Curr Temp:"," c",0,150,0,0,doNothing, noEvent, noStyle) 
    ,FIELD(battVOLT,"Batt Voltage:","V",2,5,0,0,doNothing, noEvent, noStyle) 
    ,FIELD(Output_Percent,"PID Output:","%",0,100,0,0,doNothing, noEvent, noStyle) 
    ,FIELD(F3_480,"480nm:","",0,100000,0,0,doNothing, noEvent, noStyle)

    ,EXIT("<Back")
);

MENU(setConfig, "SETUP",doNothing,noEvent,wrapStyle                 
    //,FIELD(Setpoint,"Set Temp: ","*C",25,100,1,0, doNothing, noEvent, noStyle)
    ,FIELD(tempSetting[0],"InitialTemp:","c",0,100,10,1,doNothing,noEvent,noStyle)         //Init, Denature, Anneal, Extension, and Final temperature 
    ,FIELD(tempSetting[1],"DenatureTemp:","c",0,100,10,1,doNothing,noEvent,noStyle)       
    ,FIELD(tempSetting[2],"AnnealTemp:","c",0,100,10,1,doNothing,noEvent,noStyle)
    ,FIELD(tempSetting[3],"ExtensionTemp:","c",0,100,10,1,doNothing,noEvent,noStyle)
    ,FIELD(tempSetting[4],"FinalTemp:","c",0,100,10,1,doNothing,noEvent,noStyle)

    ,FIELD(timeSetting[0],"InitialPeriod:","sec",0,180,10,1,doNothing,noEvent,noStyle)         //Init, Denature, Anneal, Extension, and Final temperature 
    ,FIELD(timeSetting[1],"DenaturePeriod:","sec",0,180,10,1,doNothing,noEvent,noStyle)       
    ,FIELD(timeSetting[2],"AnnealPeriod:","sec",0,180,10,1,doNothing,noEvent,noStyle)
    ,FIELD(timeSetting[3],"ExtendPeriod:","sec",0,180,10,1,doNothing,noEvent,noStyle)
    ,FIELD(timeSetting[4],"FinalPeriod:","sec",0,180,10,1,doNothing,noEvent,noStyle)
    ,FIELD(setCycle,"# of Cycle:","",0,100,10,1,doNothing,noEvent,noStyle)

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

MENU(pcrRun, "RUN",doNothing,noEvent,wrapStyle
    ,SUBMENU(startPCR)
    ,altOP(Stages,"",doNothing,noEvent)
    ,SUBMENU(Temperature) 
    ,SUBMENU(Cycle)
    ,SUBMENU(Cycle_Time)                                    
    ,EXIT("<Back")
);

MENU(mainMenu,"MAIN",doNothing,noEvent,wrapStyle
    ,SUBMENU(pcrRun)
    ,SUBMENU(setConfig)
    ,SUBMENU(about)
    ,SUBMENU(checkSensors)
);

#define MAX_DEPTH 4

MENU_OUTPUTS(out,MAX_DEPTH
    ,ADAGFX_OUT(display,colors,fontX,fontY,{0,0,WIDTH/fontX,HEIGHT/fontY})
    //,SERIAL_OUT(Serial)
    ,NONE
);

//serialIn serial(Serial);        //Serial input
MENU_INPUTS(in,&reIn);          //Physical input

NAVROOT(nav,mainMenu,MAX_DEPTH,in,out);


result systemExit() {
    isRunning = !isRunning;
    digitalWrite(LED_BUILTIN, isRunning);

    return quit;
}
/*--------GUI---------*/

void vCallbackFunction(TimerHandle_t pxTimer)
{
    currentTime_sec++;
}
void setup(void)
{
    Serial.begin(115200);
    myPID.SetOutputLimits(0,255);
    myPID.SetMode(AUTOMATIC);

    Setpoint = 25.0;

    Wire.beginTransmission(0x39);
    as7341.begin();
    as7341.setATIME(100);
    as7341.setASTEP(999);
    as7341.setGain(AS7341_GAIN_256X);
    as7341.startReading();
    as7341.enableLED(false);

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);

    ButtonConfig* buttonConfig = button.getButtonConfig();
    buttonConfig->setEventHandler(handleButtonEvent);
    buttonConfig->setFeature(ButtonConfig::kFeatureClick);
    buttonConfig->setFeature(ButtonConfig::kFeatureDoubleClick);
    buttonConfig->setFeature(ButtonConfig::kFeatureLongPress);
    buttonConfig->setFeature(ButtonConfig::kFeatureSuppressClickBeforeDoubleClick);
    buttonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterClick);
    buttonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterDoubleClick);
    
    RotaryInit();

    display.begin(SSD1306_EXTERNALVCC, 0x3c);
    display.clearDisplay();
    delay(100);
    display.display();

    //Serial.begin(115200);
    //Start plotter
    #ifdef PLOTTER
    p.Begin();
    p.AddTimeGraph( "PID", 800, "PID Output %", w, "Setpoint", x, "Temp", y, "Error", z, "F3_480 RED", F3_480 ); // Add 5 variable time graph
    #endif
    #ifndef PLOTTER
    Serial.print("Plotter is disabled");
    #endif
    F3_480 = 0.0;
    swTimer.begin(1000,vCallbackFunction, 0,true);
    swTimer.start();
}

void loop(void)
{
    Output_Percent = map(Output, 0, 255, 0, 100);
    runPCR();

    pcrRun[0].dirty = enabledStatus;        //Update all submenu in pcrRun
    pcrRun[1].dirty = enabledStatus;        //Update all submenu in pcrRun
    pcrRun[2].dirty = enabledStatus;        //Update all submenu in pcrRun
    pcrRun[3].dirty = enabledStatus;        //Update all submenu in pcrRun
    pcrRun[4].dirty = enabledStatus;        //Update all submenu in pcrRun

    currTemp = getTemp();                                 
    currentTime = millis();
    battVOLT = readBAT(VBATPIN);

    if ( currentTime - previousTime_1 >= eventTime_1 ){
        // as7341.readAllChannels();
        // F3_480 = as7341.getChannel(AS7341_CHANNEL_480nm_F3);   
        previousTime_1 = currentTime;
    }    

    if ( currentTime - previousTime_2 >= eventTime_2) {             //Slows down Encoder reading to prevent double-read
        encoder( RotaryEncoder.read() );
        previousTime_2 = currentTime;
    }

    if( isRunning == true)
    {
        Input = getTemp();                  //Input to PID
        Setpoint = setTemp;                 //Desired output for PID
        myPID.Compute();                    //PID Math stuff in here
        analogWrite(HeaterPIN, Output);     //PID Output to heater
    }else{
        Input = 0;                          //Input to PID
        Setpoint = 0;                       //Desired output for PID
    }

    #ifdef PLOTTER
    w = map(Output, 0, 255, 0, 100);    //Plotter Stuff to see on the computer
    x = Setpoint;   
    y = getTemp();
    z = Setpoint - Input;
    p.Plot();                           // usually called within loop()
    #endif

    button.check();                     //Read Encoder Button Presses
    //digitalWrite(LED_BLUE, blink(timeOn, timeOff));      //Not required, but shows any hiccups in the MCU if there's inconsistent blinking
    

    /* Choose one below*/

    // nav.poll();              //Polling based, laggy inputs but better for time sensitive application
    // display.display(); 

    nav.doInput();              //Event Based. This display method is more responsive, but lags background process
    if (nav.changed(0)) {       //only draw if changed
        nav.doOutput();
        display.display();
    }
}
