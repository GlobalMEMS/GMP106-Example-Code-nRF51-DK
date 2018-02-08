/*
 *
 ****************************************************************************
 * Copyright (C) 2018 GlobalMEMS, Inc. <www.globalmems.com>
 * All rights reserved.
 *
 * File : gmp106.c
 *
 * Date : 2018/2/08
 *
 * Usage: GMP106 sensor driver file
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
 
/*! @file gmp106.c
 *  @brief  GMP106 Sensor Driver File 
 *  @author Joseph FC Tseng
 */
 
#include <stddef.h>
#include "nrf_error.h"
#include "gmp106.h"
 
#define WAIT_FOR_DRDY_LOOP_DELAY(count) {int i;for(i = 0; i < (count); ++i);}
 
bus_support_t* pGMP106Bus = 0;

/*!
 * @brief Read multiple data from the starting regsiter address
 *
 * @param u8Addr Starting register address
 * @param pu8Data The data array of values read
 * @param u8Len Number of bytes to read
 * 
 * @return Result from the burst read function
 * @retval >= 0 Success
 * @retval -127 Error null bus
 * @retval < 0  Communication error
 *
 */
s8 gmp106_burst_read(u8 u8Addr, u8* pu8Data, u8 u8Len){
	
  s8 comRslt = -1;
  if(pGMP106Bus == NULL){
    return -127;
  }
  else{
    comRslt = pGMP106Bus->bus_read(pGMP106Bus->p_app_twi, pGMP106Bus->u8DevAddr, u8Addr, pu8Data, u8Len);
    if(comRslt == NRF_SUCCESS) //success, return # of bytes read
      comRslt = u8Len;
    else //return the nRF51 error code
      comRslt = -comRslt;
  }
	
  return comRslt;
}
 

/*!
 * @brief Write multiple data to the starting regsiter address
 *
 * @param u8Addr Starting register address
 * @param pu8Data The data array of values to write
 * @param u8Len Number of bytes to write
 * 
 * @return Result from the burst write function
 * @retval >= 0 Success
 * @retval -127 Error null bus
 * @retval < 0   Communication error
 *
 */
s8 gmp106_burst_write(u8 u8Addr, u8* pu8Data, u8 u8Len){
	
  s8 comRslt = -1;
  if(pGMP106Bus == NULL){
    return -127;
  }
  else{
    comRslt = pGMP106Bus->bus_write(pGMP106Bus->p_app_twi, pGMP106Bus->u8DevAddr, u8Addr, pu8Data, u8Len);
    if(comRslt == NRF_SUCCESS) //success, return # of bytes write
      comRslt = u8Len;
    else //return the nRF51 error code
      comRslt = -comRslt;
  }
	
  return comRslt;	
}

/*!
 * @brief GMP106 initialize communication bus
 *
 * @param pbus Pointer to the I2C/SPI read/write bus support struct
 * 
 * @return Result from bus communication function
 * @retval 0 Success
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gmp106_bus_init(bus_support_t* pbus){
	
  s8 comRslt = -1;
  u8 u8Data;
	
  //assign the I2C/SPI bus
  if(pbus == NULL)
    return -127;
  else
    pGMP106Bus = pbus;
	
  //Read chip ID
  comRslt = gmp106_burst_read(GMP106_REG_PID, &u8Data, 1);
	
  return comRslt;
}
 
/*!
 * @brief GMP106 soft reset
 *
 * @param None
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gmp106_soft_reset(void){
	
  s8 comRslt = -1;
  u8 u8Data = GMP106_SW_RST_SET_VALUE;
	
  //Set 00h = 0x24
  comRslt = gmp106_burst_write(GMP106_RST__REG, &u8Data, 1);
	
  return comRslt;
}

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
s8 gmp106_measure_T(s16* ps16T){
	
  s8 comRslt = 0, s8Tmp;
  u8 u8Data[2];

  // Set 30h = 0x08, T-Forced mode
  u8Data[0] = GMP106_T_FORCED_MODE_SET_VALUE;
  s8Tmp = gmp106_burst_write(GMP106_REG_CMD, u8Data, 1);

  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;
	
  // Wait for 02h[0] DRDY bit set
  do{

    //wait a while
    WAIT_FOR_DRDY_LOOP_DELAY(1000)
		
    s8Tmp = gmp106_burst_read(GMP106_REG_STATUS, u8Data, 1);

    if(s8Tmp < 0){ //communication error
      comRslt = s8Tmp;
      goto EXIT;
    }
    comRslt += s8Tmp;		
		
  } while( GMP106_GET_BITSLICE(u8Data[0], GMP106_DRDY) != 1);
	
  // Read 09h~0Ah
  s8Tmp = gmp106_burst_read(GMP106_REG_TEMPH, u8Data, 2);

  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;	
	
  // Get the calibrated temperature in code
  *ps16T = (u8Data[0] << 8) + u8Data[1];
	
 EXIT:
  return comRslt;
}

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
s8 gmp106_measure_P(s32* ps32P){

  s8 comRslt = 0, s8Tmp;
  u8 u8Data[3];

  // Set 30h = 0x09, P-Forced mode
  u8Data[0] = GMP106_P_FORCED_MODE_SET_VALUE;
  s8Tmp = gmp106_burst_write(GMP106_REG_CMD, u8Data, 1);

  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;
	
  // Wait for 02h[0] DRDY bit set
  do{

    //wait a while
    WAIT_FOR_DRDY_LOOP_DELAY(1000)
		
    s8Tmp = gmp106_burst_read(GMP106_REG_STATUS, u8Data, 1);

    if(s8Tmp < 0){ //communication error
      comRslt = s8Tmp;
      goto EXIT;
    }
    comRslt += s8Tmp;		
		
  } while( GMP106_GET_BITSLICE(u8Data[0], GMP106_DRDY) != 1);
	
  // Read 06h~08h
  s8Tmp = gmp106_burst_read(GMP106_REG_PRESSH, u8Data, 3);

  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;	
	
  s8Tmp = sizeof(*ps32P)*8 - 24;
  // Get the raw pressure in code
  *ps32P = (u8Data[0] << 16) + (u8Data[1] << 8) + u8Data[2];
  *ps32P = (*ps32P << s8Tmp) >> s8Tmp; //24 bit sign extension
	
 EXIT:
  return comRslt;
}

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
s8 gmp106_read_P_T(s32* ps32P, s16* ps16T){

  s8 comRslt = 0, s8Tmp;
  u8 u8Data[5];
		
  // Read data registers 06h~0Ah
  s8Tmp = gmp106_burst_read(GMP106_REG_PRESSH, u8Data, 5);

  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;	
	
  s8Tmp = sizeof(*ps32P)*8 - 24;
  // Get the calibrated pressure in code
  *ps32P = (u8Data[0] << 16) + (u8Data[1] << 8) + u8Data[2];
  *ps32P = (*ps32P << s8Tmp) >> s8Tmp; //24 bit sign extension	
	
  // Get the calibrated temperature in code
  *ps16T = (u8Data[3] << 8) + u8Data[4];
	
 EXIT:
  return comRslt;
}

/*!
 * @brief gmp106 set to continuous mode
 *
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gmp106_set_continuous_mode(){

  s8 comRslt = 0, s8Tmp;
  u8 u8Data;
	
  //Read 30h
  s8Tmp = gmp106_burst_read(GMP106_REG_CMD, &u8Data, 1);
	
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;	

  //Set the 30h[3:0] Measure_CTRL bits
  u8Data = GMP106_SET_BITSLICE(u8Data, GMP106_MEAS_CTRL, GMP106_CONT_MODE_SET_VALUE);
  s8Tmp = gmp106_burst_write(GMP106_REG_CMD, &u8Data, 1);
	
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;
	
 EXIT:
  return comRslt;
  
}

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
s8 gmp106_set_standby_time(GMP106_STANDBY_TIME_Type stbyTime){

  s8 comRslt = 0, s8Tmp;
  u8 u8Data;
	
  //Read 30h
  s8Tmp = gmp106_burst_read(GMP106_REG_CMD, &u8Data, 1);
	
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;	

  //Set the 30h[7:4] Standby_Time bits
  u8Data = GMP106_SET_BITSLICE(u8Data, GMP106_STANDBY_TIME, stbyTime);
  s8Tmp = gmp106_burst_write(GMP106_REG_CMD, &u8Data, 1);
	
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;
	
 EXIT:
  return comRslt;
  
}

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
s8 gmp106_set_P_OSR(GMP106_P_OSR_Type osrP){
	
  s8 comRslt = 0, s8Tmp;
  u8 u8Data;
	
  //Read A6h
  s8Tmp = gmp106_burst_read(GMP106_REG_CONFIG2, &u8Data, 1);
	
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;	

  //Set the A6h[2:0] OSR bits
  u8Data = GMP106_SET_BITSLICE(u8Data, GMP106_P_OSR, osrP);
  s8Tmp = gmp106_burst_write(GMP106_REG_CONFIG2, &u8Data, 1);
	
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;
	
 EXIT:
  return comRslt;
}


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
s8 gmp106_set_T_OSR(GMP106_T_OSR_Type osrT){
	
  s8 comRslt = 0, s8Tmp;
  u8 u8Data;
	
  //Read A7h
  s8Tmp = gmp106_burst_read(GMP106_REG_CONFIG3, &u8Data, 1);
	
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;	

  //Set the A7h[2:0] OSR bits
  u8Data = GMP106_SET_BITSLICE(u8Data, GMP106_T_OSR, osrT);
  s8Tmp = gmp106_burst_write(GMP106_REG_CONFIG3, &u8Data, 1);
	
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;
	
 EXIT:
  return comRslt;
}
