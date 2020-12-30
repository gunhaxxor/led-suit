
#include "button.h"

Button_Class::Button_Class(int pin, bool toggle, void(*interrupt)(), int duration)
: _pin(pin), _interrupt(interrupt), _pressedDown(false), toggle(toggle), value(0), bounce(pin, duration){

}


//USAGE
void Button_Class::init(){
	pinMode(_pin, INPUT_PULLUP);
	// digitalWrite(_pin, INPUT_PULLUP);
	attachInterrupt(_pin, _interrupt, CHANGE);
	//Make sure we have a proper initial state for the button!
	for (int i = 0; i < 5; i++) {
		bounce.update();
		delayMicroseconds(700);
	}
}

void Button_Class::interrupt(){
	if(this->bounce.update()){//This is where we check if it's an actual update
		_updated = true;
		updatePressedState();
		// return;
		if(toggle){
			if(this->bounce.risingEdge()){
				value++;
			}
		}
		else{
			if(this->bounce.fallingEdge()){
				value++;
			}
			else if(this->bounce.risingEdge()){
				value++;
			}
		}
	}
}

void Button_Class::updatePressedState(){//Should be called directly after a bounce.update() check.
	if(this->bounce.fallingEdge()){///These two if statements updates the pressed state
		// DEBUG_PRINTLN("falling edge");
		_pressedDown = true;
	}
	else if(this->bounce.risingEdge()){
		// DEBUG_PRINTLN("rising edge");
		_pressedDown = false;
		// pressedDownAlreadyChecked = false;
	}
}

bool Button_Class::isActive(){
	return value % 2;
}

bool Button_Class::isPressedDown(){
	return _pressedDown;
}

bool Button_Class::isUpdated(){
	return _updated;
}

void Button_Class::clearUpdateFlag(){
	_updated = false;
}
