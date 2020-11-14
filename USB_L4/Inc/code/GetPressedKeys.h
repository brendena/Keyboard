#pragma once

#include "main.h"

typedef struct KeysPressedStruct{
	unsigned char pressedKeys[20];
	unsigned char numKeysPressed;
} KeysPressed;


extern const unsigned char ButtonLayout[20];
extern unsigned char charToUSBKey[255];

void setupUSBConversionTable();
KeysPressed  * getKeyPadPressedKeys();
void calculatePressedKeys();

