#include "printing.h"
#include <stdio.h>

// from https://forum.digikey.com/t/easily-use-printf-on-stm32/20157

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
