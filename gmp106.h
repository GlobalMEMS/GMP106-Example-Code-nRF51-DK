/*
 *
 ****************************************************************************
 * Copyright (C) 2018 GlobalMEMS, Inc. <www.globalmems.com>
 * All rights reserved.
 *
 * File : gmp106.h
 *
 * Date : 2018/2/08
 *
 * Usage: GMP106 sensor driver header file
 *
 ****************************************************************************
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 **************************************************************************/

/*! @file gmp106.h
 *  @brief  GMP106 Sensor Driver Header File 
 *  @author Joseph FC Tseng
 */
 
#ifndef __GMP106_H__
#define __GMP106_H__

#include "bus_support.h"

#define GMP106_7BIT_I2C_ADDR		0x6D
#define GMP106_PRESSURE_SENSITIVITY (64)  //1 Pa = 64 code
#define GMP106_TEMPERATURE_SENSITIVITY (256)  //1 Celsius = 256 code
#define GMP106_P_CODE_TO_PA(pCode) (((float)(pCode)) / GMP106_PRESSURE_SENSITIVITY)
#define GMP106_T_CODE_TO_CELSIUS(tCode) (((float)(tCode)) / GMP106_TEMPERATURE_SENSITIVITY)

//Registers Address
#define GMP106_REG_RESET	  0x00
#define GMP106_REG_PID 	          0x01
#define GMP106_REG_STATUS 	  0x02
#define GMP106_REG_PRESSH	  0x06
#define GMP106_REG_PRESSM	  0x07
#define GMP106_REG_PRESSL	  0x08
#define GMP106_REG_TEMPH	  0x09
#define GMP106_REG_TEMPL	  0x0A
#define GMP106_REG_CMD	 	  0x30
#define GMP106_REG_CONFIG2 	  0xA6
#define GMP106_REG_CONFIG3 	  0xA7
//Soft reset
#define GMP106_SW_RST_SET_VALUE		0x24
//T-Forced mode
#define GMP106_T_FORCED_MODE_SET_VALUE	0x08
//P-Forced mode
#define GMP106_P_FORCED_MODE_SET_VALUE	0x09
//Continuous mode
#define GMP106_CONT_MODE_SET_VALUE	0x0B

/* PID */
#define GMP106_PID__REG GMP106_REG_PID
/* Soft Rest bit */
#define GMP106_RST__REG		GMP106_REG_RESET
#define GMP106_RST__MSK		0x24
#define GMP106_RST__POS		0
/* DRDY bit */
#define GMP106_DRDY__REG	GMP106_REG_STATUS
#define GMP106_DRDY__MSK	0x01
#define GMP106_DRDY__POS	0
/* Measure CTRL bits */
#define GMP106_MEAS_CTRL__REG	GMP106_REG_CMD
#define GMP106_MEAS_CTRL__MSK	0x0F
#define GMP106_MEAS_CTRL__POS	0
/* Standby time bits */
#define GMP106_STANDBY_TIME__REG	GMP106_REG_CMD
#define GMP106_STANDBY_TIME__MSK	0xF0
#define GMP106_STANDBY_TIME__POS	4
/* P OSR bits */
#define GMP106_P_OSR__REG       GMP106_REG_CONFIG2
#define GMP106_P_OSR__MSK       0x07
#define GMP106_P_OSR__POS       0
/* T OSR bits */
#define GMP106_T_OSR__REG       GMP106_REG_CONFIG3
#define GMP106_T_OSR__MSK       0x07
#define GMP106_T_OSR__POS       0

#define GMP106_GET_BITSLICE(regvar, bitname)	\
  ((regvar & bitname##__MSK) >> bitname##__POS)

#define GMP106_SET_BITSLICE(regvar, bitname, val)			\
  ((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))

typedef enum {
  GMP106_P_OSR_256 = 0x04,
  GMP106_P_OSR_512 = 0x05,
  GMP106_P_OSR_1024 = 0x00,
  GMP106_P_OSR_2048 = 0x01,
  GMP106_P_OSR_4096 = 0x02,
  GMP106_P_OSR_8192 = 0x03,
  GMP106_P_OSR_16384 = 0x06,
  GMP106_P_OSR_32768 = 0x07,	
} GMP106_P_OSR_Type;

typedef enum {
  GMP106_T_OSR_256 = 0x04,
  GMP106_T_OSR_512 = 0x05,
  GMP106_T_OSR_1024 = 0x00,
  GMP106_T_OSR_2048 = 0x01,
  GMP106_T_OSR_4096 = 0x02,
  GMP106_T_OSR_8192 = 0x03,
  GMP106_T_OSR_16384 = 0x06,
  GMP106_T_OSR_32768 = 0x07,	
} GMP106_T_OSR_Type;

typedef enum {
  GMP106_STANDBY_TIME_0ms = 0,
  GMP106_STANDBY_TIME_63ms = 1,
  GMP106_STANDBY_TIME_125ms = 2,
  GMP106_STANDBY_TIME_188ms = 3,
  GMP106_STANDBY_TIME_250ms = 4,
  GMP106_STANDBY_TIME_313ms = 5,
  GMP106_STANDBY_TIME_375ms = 6,
  GMP106_STANDBY_TIME_438ms = 7,
  GMP106_STANDBY_TIME_500ms = 8,
  GMP106_STANDBY_TIME_563ms = 9,
  GMP106_STANDBY_TIME_625ms = 10,
  GMP106_STANDBY_TIME_688ms = 11,
  GMP106_STANDBY_TIME_750ms = 12,
  GMP106_STANDBY_TIME_813ms = 13,
  GMP106_STANDBY_TIME_875ms = 14,
  GMP106_STANDBY_TIME_938ms = 15,
} GMP106_STANDBY_TIME_Type;

/*!
 * @brief Read multiple data from the starting regsiter address
 *
 * @param u8Addr Starting register address
 * @param pu8Data The data array of values read
 * @param u8Len Number of bytes to read
 * 
 * @return Result from the burst read function
 * @retval >= 0 Success, number of bytes read
 * @retval -127 Error null bus
 * @retval -1   Bus communication error
 *
 */
s8 gmp106_burst_read(u8 u8Addr, u8* pu8Data, u8 u8Len);

/*!
 * @brief Write multiple data to the starting regsiter address
 *
 * @param u8Addr Starting register address
 * @param pu8Data The data array of values to write
 * @param u8Len Number of bytes to write
 * 
 * @return Result from the burst write function
 * @retval >= 0 Success, number of bytes write
 * @retval -127 Error null bus
 * @retval -1   Communication error
 *
 */
s8 gmp106_burst_write(u8 u8Addr, u8* pu8Data, u8 u8Len);

/*!
 * @brief gmp106 initialize communication bus
 *
 * @param pbus Pointer to the I2C/SPI read/write bus support struct
 * 
 * @return Result from bus communication function
 * @retval 0 Success
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gmp106_bus_init(bus_support_t* pbus);

/*!
 * @brief gmp106 soft reset
 *
 * @param None
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gmp106_soft_reset(void);

/*!
 * @brief gmp106 T-Forced mode measure temperature
 *
 * @param *ps16T calibrated temperature code returned to caller
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gmp106_measure_T(s16* ps16T);

/*!
 * @brief gmp106 P-Forced mode measure pressure
 *
 * @param *ps32P calibrated pressure in code returned to caller
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gmp106_measure_P(s32* ps32P);

/*!
 * @brief gmp106 read calibrated pressure and temperature
 *        This function just read data registers, thus should 
 *        be called when GMP106 is in the continuous mode that
 *        data conversion is periodically conducted.
 *        
 * @param *ps32P calibrated pressure in code returned to caller
 * @param *ps16T calibrated temperature code returned to caller
 *
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gmp106_read_P_T(s32* ps32P, s16* ps16T);

/*!
 * @brief gmp106 set to continuous mode
 *
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gmp106_set_continuous_mode();

/*!
 * @brief gmp106 set standby-time for continuous mode
 *
 * @param stbyTime standby time to set
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gmp106_set_standby_time(GMP106_STANDBY_TIME_Type stbyTime);

/*!
 * @brief gmp106 set pressure OSR
 *
 * @param osrP OSR to set
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gmp106_set_P_OSR(GMP106_P_OSR_Type osrP);

/*!
 * @brief gmp106 set temperature OSR
 *
 * @param osrT OSR to set
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gmp106_set_T_OSR(GMP106_T_OSR_Type osrT);

#endif // __GMP106_H__
