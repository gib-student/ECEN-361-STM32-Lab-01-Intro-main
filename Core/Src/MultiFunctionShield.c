#include "main.h"


#define LSBFIRST 0
#define MSBFIRST 1
#define LOW 0
#define HIGH 1
#define SevenSeg_LATCH_Port

void shiftOut(	GPIO_TypeDef* dataPort,uint16_t dataPin,
				GPIO_TypeDef* clockPort, uint16_t clockPin,
				uint8_t bitOrder, uint8_t val)
{
	uint8_t i;

	for (i = 0; i < 8; i++)  {
		if (bitOrder == LSBFIRST) {
			HAL_GPIO_WritePin(dataPort, dataPin,val & 1);
			val >>= 1;
		} else {	
			HAL_GPIO_WritePin(dataPort, dataPin, (val & 128) != 0);
			val <<= 1;
		}
			
		HAL_GPIO_WritePin(clockPort, clockPin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(clockPort, clockPin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(clockPort, clockPin,GPIO_PIN_RESET);
	}
}



const uint8_t SEGMENT_MAP[] = {0xC0,0xF9,0xA4,0xB0,0x99,0x92,0x82,0xF8,0x80,0x90};    // Segmente, die leuchten sollen pro Zahlwert (Low-Aktiv), & 0x7F Verknüpfen fuer Dezimalpunkt
const uint8_t SEGMENT_BLANK = 0xFF;
const uint8_t SEGMENT_MINUS = 0xBF;
const uint8_t SEGMENT_SELECT[] = {0xF1,0xF2,0xF4,0xF8};                               // Ziffernposition (gemeinsame Anode, LSB)
volatile uint8_t ActDigit = 0;
volatile uint8_t SEGMENT_VALUE[4];

//static MultiFunctionShield *instance;

void MultiFunctionShield_Display (int16_t value)
{
  if ((value > 9999) || (value < -999))   // out of range
  {
    SEGMENT_VALUE[0] = SEGMENT_MINUS;
    SEGMENT_VALUE[1] = SEGMENT_MINUS;
    SEGMENT_VALUE[2] = SEGMENT_MINUS;
    SEGMENT_VALUE[3] = SEGMENT_MINUS;
  }
  else    // possible range
  {
    if (value >= 0)   // positive values
    {
      if (value > 999)
        SEGMENT_VALUE[0] = SEGMENT_MAP [(uint8_t) (value / 1000)];
      else
        SEGMENT_VALUE[0] = SEGMENT_BLANK;

      if (value > 99)
        SEGMENT_VALUE[1] = SEGMENT_MAP [(uint8_t) ((value / 100) % 10)];
      else
        SEGMENT_VALUE[1] = SEGMENT_BLANK;

      if (value > 9)
        SEGMENT_VALUE[2] = SEGMENT_MAP [(uint8_t) ((value / 10) % 10)];
      else
        SEGMENT_VALUE[2] = SEGMENT_BLANK;

      SEGMENT_VALUE[3] = SEGMENT_MAP [(uint8_t) (value % 10)];

    }
    if (value < 0)      // negative values: "-" left
    {
      value *= -1;
      SEGMENT_VALUE[0] = SEGMENT_MINUS;

      if (value > 99)
        SEGMENT_VALUE[1] = SEGMENT_MAP [(uint8_t) ((value / 100) % 10)];
      else
        SEGMENT_VALUE[1] = SEGMENT_BLANK;

      if (value > 9)
        SEGMENT_VALUE[2] = SEGMENT_MAP [(uint8_t) ((value / 10) % 10)];
      else
        SEGMENT_VALUE[2] = SEGMENT_BLANK;

      SEGMENT_VALUE[3] = SEGMENT_MAP [(uint8_t) (value % 10)];
    }
  }
}

void MultiFunctionShield_Clear(void)
{
  SEGMENT_VALUE[0] = SEGMENT_BLANK;
  SEGMENT_VALUE[1] = SEGMENT_BLANK;
  SEGMENT_VALUE[2] = SEGMENT_BLANK;
  SEGMENT_VALUE[3] = SEGMENT_BLANK;
}

void MultiFunctionShield_WriteNumberToSegment(uint8_t digit)
	{
	HAL_GPIO_WritePin(SevenSeg_LATCH_GPIO_Port, SevenSeg_LATCH_Pin,GPIO_PIN_RESET);
	shiftOut(SevenSeg_DATA_GPIO_Port,SevenSeg_DATA_Pin,SevenSeg_CLK_GPIO_Port,SevenSeg_CLK_Pin ,
			MSBFIRST, SEGMENT_VALUE[digit]);
	shiftOut(SevenSeg_DATA_GPIO_Port,SevenSeg_DATA_Pin,SevenSeg_CLK_GPIO_Port,SevenSeg_CLK_Pin ,
				MSBFIRST, SEGMENT_SELECT[digit]);
	HAL_GPIO_WritePin(SevenSeg_LATCH_GPIO_Port, SevenSeg_LATCH_Pin,GPIO_PIN_SET);
	}

void MultiFunctionShield__ISRFunc(void)
{
  switch (++ActDigit)
  {
    case 1 : MultiFunctionShield_WriteNumberToSegment(0); break;
    case 2 : MultiFunctionShield_WriteNumberToSegment(1); break;
    case 3 : MultiFunctionShield_WriteNumberToSegment(2); break;
    case 4 : MultiFunctionShield_WriteNumberToSegment(3); ActDigit = 0; break;
  }
}

/*
ISR(TIMER1_COMPA_vect)
{
  instance->ISRFunc();
}
*/
