using namespace ::ace_button;

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