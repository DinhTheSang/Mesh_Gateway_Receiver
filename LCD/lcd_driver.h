/***************************************************************************//**
 * @file
 * @brief lcd_driver.h
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

#ifndef LCD_DRIVER_H_
#define LCD_DRIVER_H_

#include "hal-config.h"

#if (HAL_SPIDISPLAY_ENABLE == 1)

#include "bg_types.h"

/**
 *  LCD content can be updated one row at a time using function LCD_write().
 *  Row number is passed as parameter,the possible values are defined below.
 */

/*************************************************
 ************ LCD DISPLAY FORMAT *****************
 -------------------------------------------------
 --------------- HEADER LINE ---------------------
 --------------- HEADER LINE ---------------------
 --------------- HEADER LINE ---------------------
 --------------- HEADER LINE ---------------------
 --------------- HEADER LINE ---------------------
 --------------- FIRST ROW   ---------------------
 --------------- SECOND ROW  ---------------------
 --------------- THIRD ROW   ---------------------
 --------------- FOURTH ROW  ---------------------
 --------------- FIFTH ROW   ---------------------
 --------------- SIXTH ROW   ---------------------
 --------------- SEVENTH ROW ---------------------
**************************************************/

//Define the number line of header and one row
#define HEADER_LINE 	5
#define ROW_LINE		1

#define LCD_ROW_01   	1
#define LCD_ROW_02   	2
#define LCD_ROW_03 		3
#define LCD_ROW_04 		4
#define LCD_ROW_05  	5
#define LCD_ROW_06   	6
#define LCD_ROW_07 		7

#define LCD_ROW_NAME           1
#define LCD_ROW_INFO           2
#define LCD_ROW_BLINK          3
#define LCD_ROW_ERR            3
#define LCD_ROW_SENSOR_00      4
#define LCD_ROW_SENSOR_01      5
#define LCD_ROW_FRIEND_INFOR   6
#define LCD_ROW_CONNECTION     6

#define LCD_ROW_MAX    	7

#define LCD_ROW_LEN  	32   /* up to 32 characters per each row */

void LCD_init(char* str);
void LCD_write(char *str, uint8 row);

#endif /* HAL_SPIDISPLAY_ENABLE */

#endif /* LCD_DRIVER_H_ */
