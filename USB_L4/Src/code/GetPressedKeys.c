#include "code/GetPressedKeys.h"

static KeysPressed keyPadPressedKeys;

const unsigned char ButtonLayout[20] = {
		'-','-','-','-',
		'F','E','D','C',
		'B','A','9','8',
		'7','6','5','4',
		'3','2','1','0'
};


unsigned char charToUSBKey[255];

void setupUSBConversionTable(){
	charToUSBKey['-'] = 0x2D;
	charToUSBKey['F'] = 0x09;
	charToUSBKey['f'] = 0x09;

	charToUSBKey['E'] = 0x08;
	charToUSBKey['e'] = 0x08;

	charToUSBKey['D'] = 0x07;
	charToUSBKey['d'] = 0x07;

	charToUSBKey['C'] = 0x06;
	charToUSBKey['c'] = 0x06;

	charToUSBKey['B'] = 0x05;
	charToUSBKey['b'] = 0x05;

	charToUSBKey['A'] = 0x04;
	charToUSBKey['a'] = 0x04;

	charToUSBKey['0'] = 0x62;
	charToUSBKey['9'] = 0x61;
	charToUSBKey['8'] = 0x60;
	charToUSBKey['7'] = 0x5F;
	charToUSBKey['6'] = 0x5E;
	charToUSBKey['5'] = 0x5D;
	charToUSBKey['4'] = 0x5C;
	charToUSBKey['3'] = 0x5B;
	charToUSBKey['2'] = 0x5A;
	charToUSBKey['1'] = 0x59;


}


KeysPressed  * getKeyPadPressedKeys()
{
	return &keyPadPressedKeys;
}

void calculatePressedKeys()
{
	static GPIO_TypeDef * rowsPorts[] = {
			ROW_0_OUTPUT_GPIO_Port,
			ROW_1_OUTPUT_GPIO_Port,
			ROW_2_OUTPUT_GPIO_Port,
			ROW_3_OUTPUT_GPIO_Port,
			ROW_4_OUTPUT_GPIO_Port
	};

	static uint16_t rowsPins[] = {
			ROW_0_OUTPUT_Pin,
			ROW_1_OUTPUT_Pin,
			ROW_2_OUTPUT_Pin,
			ROW_3_OUTPUT_Pin,
			ROW_4_OUTPUT_Pin
	};

	static GPIO_TypeDef * colsPorts[] = {
			COL_0_INPUT_GPIO_Port,
			COL_1_INPUT_GPIO_Port,
			COL_2_INPUT_GPIO_Port,
			COL_3_INPUT_GPIO_Port
	};

	static uint16_t colsPins[] = {
			COL_0_INPUT_Pin,
			COL_1_INPUT_Pin,
			COL_2_INPUT_Pin,
			COL_3_INPUT_Pin
	};

	keyPadPressedKeys.numKeysPressed = 0;

	unsigned char i = 0;
	unsigned char j = 0;
	GPIO_PinState statePin;
	for(i=0;i < 5; i++)
	{
		  HAL_GPIO_WritePin(rowsPorts[i], rowsPins[i], GPIO_PIN_SET);
		  for(j=0; j < 4; j++)
		  {
			  statePin =  HAL_GPIO_ReadPin(colsPorts[j],colsPins[j]);
			  if(statePin == GPIO_PIN_SET)
			  {
				  keyPadPressedKeys.pressedKeys[keyPadPressedKeys.numKeysPressed] = ButtonLayout[i * 4 + j];
				  keyPadPressedKeys.numKeysPressed++;
			  }

		  }
		  HAL_GPIO_WritePin(rowsPorts[i], rowsPins[i], GPIO_PIN_RESET);
	}
}
