#ifndef BUTTONS_H
#define BUTTONS_H

#include <Arduino.h>
#include <Bounce.h>
#define DEBUG
#include "debug.h"

#define BOUNCEDURATION 8

class Button_Class{
	private:
		const int _pin;
		volatile bool _updated;
		// volatile bool _active;
		void (*_interrupt)();
		volatile bool _pressedDown;

	public:
		bool toggle;
		volatile uint8_t value;

		Button_Class(int pin , bool toggle, void(*interrupt)(), int duration = BOUNCEDURATION);
		Bounce bounce;

		void init();
		void interrupt();
		void updatePressedState();
		bool isActive();
		bool isPressedDown();

		bool isUpdated();
		void clearUpdateFlag();

};


#endif
