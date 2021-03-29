/*------03/15/2021------*/
#include <Adafruit_I2CDevice.h>
#include <Arduino.h>
#include <Wire.h>
#include <RotaryEncoder.h>
#include <InternalFileSystem.h>
#include <PID_v1.h>

#include <menu.h>
#include <menuIO/serialOut.h>
#include <menuIO/serialIn.h>
#include <menuIO/chainStream.h>
#include <menuIO/rotaryEventIn.h>
#include <menuIO/adafruitGfxOut.h>
#include <AceButton.h>

#include <Adafruit_AS7341.h>
#include <Adafruit_SSD1306.h>
#include <Plotter.h>

#include <config.h>

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
class altPrompt:public prompt {
    public:
        altPrompt(constMEM promptShadow& p):prompt(p) {}
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
            out.printRaw((constText*)F(">>CANCEL<<"),len);//when printing as regular option
        }
};

result systemExit();

altMENU(confirmRun,startPCR,"Cancel PCR Progress?",doNothing,noEvent,wrapStyle,(Menu::_menuData|Menu::_canNav)
    ,OP("Yes",systemExit,enterEvent)
    ,EXIT("Cancel")
);


MENU(checkSensors, "SENSORS READ",doNothing,noEvent,wrapStyle                  //Working
    ,FIELD(currTemp,"Curr Temp: "," c",0,150,0,0,doNothing, noEvent, noStyle) 
    ,FIELD(battVOLT,"Batt Voltage: ","V",2,5,0,0,doNothing, noEvent, noStyle) 
    ,FIELD(Output,"PID Output: ","%",0,100,0,0,doNothing, noEvent, noStyle) 
    ,FIELD(F3_480,"480nm: ","",0,100000,0,0,doNothing, noEvent, noStyle)
    ,EXIT("<Back")
);

MENU(setConfig, "SETUP",doNothing,noEvent,wrapStyle                 
    //,FIELD(Setpoint,"Set Temp: ","*C",25,100,1,0, doNothing, noEvent, noStyle)
    ,FIELD(tempSetting[0],"InitialTemp: "," c",25,100,1,0,doNothing,noEvent,noStyle)         //Init, Denature, Anneal, Extension, and Final temperature 
    ,FIELD(tempSetting[1],"DenatureTemp: "," c",25,100,1,0,doNothing,noEvent,noStyle)       
    ,FIELD(tempSetting[2],"AnnealTemp: "," c",25,100,1,0,doNothing,noEvent,noStyle)
    ,FIELD(tempSetting[3],"ExtensionTemp: "," c",25,100,1,0,doNothing,noEvent,noStyle)
    ,FIELD(tempSetting[4],"FinalTemp: "," c",25,100,1,0,doNothing,noEvent,noStyle)
    ,FIELD(cycle,"# of Cycle: ","",0,100,1,0,doNothing,noEvent,noStyle)
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
    ,FIELD(currTemp,"Temperature: ","C",25,150,0,0,doNothing, noEvent, noStyle)           
    ,FIELD(currCycle,"Cycle: ","",0,100,0,0,doNothing, noEvent, noStyle)
    ,altOP(altPrompt,"",doNothing,noEvent)
    //,SUBMENU(exitMenu)
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

#define MAX_DEPTH 4

MENU_OUTPUTS(out,MAX_DEPTH
    //,U8G2_OUT(u8g2,colors,fontX,fontY,0,0,{0,0,WIDTH/fontX,HEIGHT/fontY})
    ,ADAGFX_OUT(display,colors,fontX,fontY,{0,0,WIDTH/fontX,HEIGHT/fontY})
    ,SERIAL_OUT(Serial)
    ,NONE
);

serialIn serial(Serial);        //Serial input
MENU_INPUTS(in,&reIn);          //Physical input

NAVROOT(nav,mainMenu,MAX_DEPTH,in,out);


result systemExit() {
    isRunning = false;
    ledTOGGLE = !ledTOGGLE;
    digitalWrite(LED_BUILTIN, ledTOGGLE);
    return quit;
}
/*--------GUI---------*/

void runPID(uint16_t inputTemp )
{
    Input = inputTemp;
}

void setup(void)
{
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
    
    F3_480 = 0.0;
}

void loop(void)
{
    pcrRun[3].dirty = enabledStatus;
    currTemp = getTemp();                                 
    unsigned long currentTime = millis();
    battVOLT = readBAT(VBATPIN);
    (currStageCount > 4) ? currStageCount = 0 : NULL;

    if ( currentTime - previousTime_1 >= eventTime_1 ){
        // as7341.readAllChannels();
        // F3_480 = as7341.getChannel(AS7341_CHANNEL_480nm_F3);   
        currStageCount++;
        previousTime_1 = currentTime;
    }


    if ( currentTime - previousTime_2 >= eventTime_2) {             //Slows down Encoder reading to prevent double-read
        encoder( RotaryEncoder.read() );
        previousTime_2 = currentTime;
    }

    Input = getTemp();                  //Input to PID
    Setpoint = setTemp;                 //Desired output for PID

    if( isRunning == true)
    {
        myPID.Compute();                          //PID Math stuff in here
        analogWrite(HeaterPIN, Output);     //PID Output to heater
    }

    #ifdef PLOTTER
    w = map(Output, 0, 255, 0, 100);    //Plotter Stuff to see on the computer
    x = Setpoint;   
    y = getTemp();
    z = Setpoint - Input;
    p.Plot();                           // usually called within loop()
    #endif

    button.check();                     //Read Encoder Button Presses
    // digitalWrite(LED_BUILTIN, blink(timeOn, timeOff));      //Not required, but shows any hiccups in the MCU if there's inconsistent blinking
    

    /* Choose one below*/

    nav.poll();              //Polling based, laggy inputs but better for time sensitive application
    display.display(); 

    // nav.doInput();              //Event Based. This display method is more responsive, but lags background process
    // if (nav.changed(0)) {       //only draw if changed
    //     nav.doOutput();
    //     display.display();
    // }
}
