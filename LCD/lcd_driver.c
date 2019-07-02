/***************************************************************************//**
 * @file
 * @brief lcd_driver.c
 * @edited: huekh
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include <stdio.h>
#include <string.h>
#include "graphics.h"
#include "lcd_driver.h"

#if (HAL_SPIDISPLAY_ENABLE == 1)

static char LCD_data[LCD_ROW_MAX][LCD_ROW_LEN];   /* 2D array for storing the LCD content */

static char header[HEADER_LINE * LCD_ROW_LEN];    /* 2D array for storing the LCD header */

//This is needed by the LCD driver
int rtcIntCallbackRegister(void (*pFunction)(void*),
                           void* argument,
                           unsigned int frequency) {
  return 0;
}

/*
 * LCD initialization, called once at startup.
 * This functions will be initialize display and set header.
*/
void LCD_init(char* str) {
  int i = 0;
  int count_endline = 0;

  memset(&LCD_data, 0, sizeof(LCD_data));

  while (count_endline < HEADER_LINE) {
	  header[i] = *(str + i);
	  if (*(str + i) == '\n') count_endline++;
	  else if (*(str + i) == '\0') {
		  header[i] = '\n';
		  count_endline++;
	  }
	  i++;
  }
  graphInit(header);
}

/*
 * This function is used to write one line in the LCD.
 * The parameter 'row' selects which line is written,
 * possible values are defined as LCD_ROW_xx.
*/
void LCD_write(char *str, uint8 row) {
  int i = 0;
  char *pRow;
  char LCD_message[LCD_ROW_MAX * LCD_ROW_LEN];

  char new_str[ROW_LINE * LCD_ROW_LEN];

  if (row > LCD_ROW_MAX) {
    return;
  }

  while (*(str + i) != '\n' && *(str + i) != '\0') {
	  new_str[i] = *(str + i);
	  i++;
  }

  pRow  = &(LCD_data[row - 1][0]);

  snprintf(pRow, i + 1, new_str);

  LCD_message[0] = 0;

  for (i = 0; i < LCD_ROW_MAX; i++) {
    pRow  = &(LCD_data[i][0]);
    strcat(LCD_message, pRow);
    strcat(LCD_message, "\n"); // add newline at end of reach row
  }

  graphWriteString(LCD_message);
}
#endif /* HAL_SPIDISPLAY_ENABLE */
