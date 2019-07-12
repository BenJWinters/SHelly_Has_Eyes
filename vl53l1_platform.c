/*******************************************************************************

This file is derived from vl53l1_platform.c in the STSW-IMG007 VL53L1X API.

********************************************************************************
*
* Copyright (c) 2017, STMicroelectronics - All Rights Reserved
* Copyright (c) 2018 Pololu Corporation
*
* This file is part of VL53L1 Core and is dual licensed,
* either 'STMicroelectronics
* Proprietary license'
* or 'BSD 3-clause "New" or "Revised" License' , at your option.
*
********************************************************************************
*
* 'STMicroelectronics Proprietary license'
*
********************************************************************************
*
* License terms: STMicroelectronics Proprietary in accordance with licensing
* terms at www.st.com/sla0081
*
* STMicroelectronics confidential
* Reproduction and Communication of this document is strictly prohibited unless
* specifically authorized in writing by STMicroelectronics.
*
*
********************************************************************************
*
* Alternatively, VL53L1 Core may be distributed under the terms of
* 'BSD 3-clause "New" or "Revised" License', in which case the following
* provisions apply instead of the ones mentioned above :
*
********************************************************************************
*
* License terms: BSD 3-clause "New" or "Revised" License.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*
********************************************************************************
*
*/

#define F_CPU 14745600UL
#define __DELAY_BACKWARD_COMPATIBLE__
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "twimaster.h"


#include "vl53l1_platform.h"
// #include "vl53l1_platform_log.h"
#include "vl53l1_api.h"

// #include "stm32xxx_hal.h"
#include <string.h>
// #include <time.h>
// #include <math.h>


// #define I2C_TIME_OUT_BASE   10
// #define I2C_TIME_OUT_BYTE   1

// #ifdef VL53L1_LOG_ENABLE
// #define trace_print(level, ...) VL53L1_trace_print_module_function(VL53L1_TRACE_MODULE_PLATFORM, level, VL53L1_TRACE_FUNCTION_NONE, ##__VA_ARGS__)
// #define trace_i2c(...) VL53L1_trace_print_module_function(VL53L1_TRACE_MODULE_NONE, VL53L1_TRACE_LEVEL_NONE, VL53L1_TRACE_FUNCTION_I2C, ##__VA_ARGS__)
// #endif

// #ifndef HAL_I2C_MODULE_ENABLED
// #warning "HAL I2C module must be enable "
// #endif

//extern I2C_HandleTypeDef hi2c1;
//#define VL53L0X_pI2cHandle    (&hi2c1)

/* when not customized by application define dummy one */
// #ifndef VL53L1_GetI2cBus
/** This macro can be overloaded by user to enforce i2c sharing in RTOS context
 */
// #   define VL53L1_GetI2cBus(...) (void)0
// #endif

// #ifndef VL53L1_PutI2cBus
/** This macro can be overloaded by user to enforce i2c sharing in RTOS context
 */
// #   define VL53L1_PutI2cBus(...) (void)0
// #endif

// uint8_t _I2CBuffer[256];

// int _I2CWrite(VL53L1_DEV Dev, uint8_t *pdata, uint32_t count) {
//     int status = 0;
//     return status;
// }

// int _I2CRead(VL53L1_DEV Dev, uint8_t *pdata, uint32_t count) {
//    int status = 0;
//    return Status;
// }

VL53L1_Error VL53L1_WriteMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
	int i2cWriteStatus = 0;
	
	while (count > 0)
	{
		i2c_start_wait(Dev->I2cDevAddr | I2C_WRITE);
		//Wire.beginTransmission(Dev->I2cDevAddr >> 1);
		i2cWriteStatus += i2c_write((index >> 8) & 0xff);
		//Wire.write((index >> 8) & 0xff);
		i2cWriteStatus += i2c_write(index & 0xff);
		//Wire.write(index & 0xff);

		uint8_t writing = 0;
		//printf("WriteMulti, Count: %lu, Index: %u\n",count,index);

		//while (count > 0 && Wire.write(*pdata) != 0)
		while ( (count > 0) && (i2c_write(*pdata) == 0))
		{
			pdata++;
			writing++;
			count--;
			i2cWriteStatus += 0;
		}
		//i2cWriteStatus += i2c_write(*pdata);
		//printf("WriteMulti, Count: %lu, Index: %u\n",count,index);
		
		if (writing == 0 || i2cWriteStatus !=0) { return VL53L1_ERROR_CONTROL_INTERFACE; }
		//if (writing == 0 || Wire.endTransmission() != 0) { return VL53L1_ERROR_CONTROL_INTERFACE; }
		
		index += writing;
		
		//_delay_ms(1000);
		i2c_stop();
	}
	
	return VL53L1_ERROR_NONE;
}


VL53L1_Error VL53L1_ReadMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
  int i2cWriteStatus = 0;
   //printf("ReadMulti, Count: %lu, Index: %u\n",count,index);
	
  i2c_start_wait(Dev->I2cDevAddr | I2C_WRITE);
  //Wire.beginTransmission(Dev->I2cDevAddr >> 1);
  i2cWriteStatus += i2c_write((index >> 8) & 0xff);
  //Wire.write((index >> 8) & 0xff);
  i2cWriteStatus = i2c_write(index & 0xff);
  //Wire.write(index & 0xff);
  
  if (i2cWriteStatus != 0) { return VL53L1_ERROR_CONTROL_INTERFACE; }
  //if (Wire.endTransmission() != 0) { return VL53L1_ERROR_CONTROL_INTERFACE; }
	
  uint8_t reading = i2c_rep_start(Dev->I2cDevAddr + I2C_READ);
  if (reading != 0) { return VL53L1_ERROR_CONTROL_INTERFACE; }
  
  while (count > 0)
    {	
    //uint8_t reading = Wire.requestFrom(Dev->I2cDevAddr >> 1, count);   
    count--;
	//if (reading == 0) { return VL53L1_ERROR_CONTROL_INTERFACE; }
	//count -= reading; 
    //while (reading-- > 0)
    //{
      if (count!=0) {*pdata = i2c_readAck();pdata++;}
		  else{*pdata = i2c_readNak();}
	  //*pdata++ = Wire.read();
    //}
  }
		
  i2c_stop();
	//printf("ReadMulti: %lu\n",count);
  return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WrByte(VL53L1_DEV Dev, uint16_t index, uint8_t data)
{
	int i2cWriteStatus = 0;
	
	i2c_start_wait(Dev->I2cDevAddr | I2C_WRITE);
	//Wire.beginTransmission(Dev->I2cDevAddr >> 1);
	i2cWriteStatus += i2c_write((index >> 8) & 0xff);
	//Wire.write((index >> 8) & 0xff);
	i2cWriteStatus += i2c_write(index & 0xff);
	//Wire.write(index & 0xff);  
  
	i2cWriteStatus = i2c_write(data);
	//Wire.write(data);
	
	i2c_stop();	
	
  return (i2cWriteStatus == 0 ? VL53L1_ERROR_NONE : VL53L1_ERROR_CONTROL_INTERFACE);
  //return (Wire.endTransmission() == 0 ? VL53L1_ERROR_NONE : VL53L1_ERROR_CONTROL_INTERFACE);
}

VL53L1_Error VL53L1_WrWord(VL53L1_DEV Dev, uint16_t index, uint16_t data)
{
  	int i2cWriteStatus = 0;
  	
  	i2c_start_wait(Dev->I2cDevAddr | I2C_WRITE);
  	//Wire.beginTransmission(Dev->I2cDevAddr >> 1);
  	i2cWriteStatus += i2c_write((index >> 8) & 0xff);
  	//Wire.write((index >> 8) & 0xff);
  	i2cWriteStatus += i2c_write(index & 0xff);
  	//Wire.write(index & 0xff);

  	i2cWriteStatus += i2c_write((data >> 8) & 0xff);
  	//Wire.write((data >> 8) & 0xff);
	i2cWriteStatus += i2c_write(data & 0xff); 
	//Wire.write(data & 0xff);

	i2c_stop();
	
  return (i2cWriteStatus == 0 ? VL53L1_ERROR_NONE : VL53L1_ERROR_CONTROL_INTERFACE);
  //return (Wire.endTransmission() == 0 ? VL53L1_ERROR_NONE : VL53L1_ERROR_CONTROL_INTERFACE);
}

VL53L1_Error VL53L1_WrDWord(VL53L1_DEV Dev, uint16_t index, uint32_t data)
{	
  	int i2cWriteStatus = 0;
  	
  	i2c_start_wait(Dev->I2cDevAddr | I2C_WRITE);
  	//Wire.beginTransmission(Dev->I2cDevAddr >> 1);
  	i2cWriteStatus += i2c_write((index >> 8) & 0xff);
  	//Wire.write((index >> 8) & 0xff);
  	i2cWriteStatus += i2c_write(index & 0xff);
  	//Wire.write(index & 0xff);

  	i2cWriteStatus += i2c_write((data >> 24) & 0xff);
  	//Wire.write((data >> 24) & 0xff);
	i2cWriteStatus += i2c_write((data >> 16) & 0xff);
	//Wire.write((data >> 16) & 0xff);
  	i2cWriteStatus += i2c_write((data >> 8) & 0xff);
  	//Wire.write((data >> 8) & 0xff);
  	i2cWriteStatus += i2c_write(data & 0xff);
  	//Wire.write(data & 0xff);	
	    
    i2c_stop();
	
  return (i2cWriteStatus == 0 ? VL53L1_ERROR_NONE : VL53L1_ERROR_CONTROL_INTERFACE);
  //return (Wire.endTransmission() == 0 ? VL53L1_ERROR_NONE : VL53L1_ERROR_CONTROL_INTERFACE);
}

VL53L1_Error VL53L1_UpdateByte(VL53L1_DEV Dev, uint16_t index, uint8_t AndData, uint8_t OrData)
{
  uint8_t data;
  VL53L1_Error status = VL53L1_RdByte(Dev, index, &data);
  if (status != VL53L1_ERROR_NONE) { return status; }
  data &= AndData;
  data |= OrData;
  return VL53L1_WrByte(Dev, index, data);
}

VL53L1_Error VL53L1_RdByte(VL53L1_DEV Dev, uint16_t index, uint8_t *data)
{
	int i2cWriteStatus = 0;
	  	
	i2c_start_wait(Dev->I2cDevAddr | I2C_WRITE);
	//Wire.beginTransmission(Dev->I2cDevAddr >> 1);	  	
	i2cWriteStatus += i2c_write((index >> 8) & 0xff);
	//printf("I2C write status: %d\n\r", i2cWriteStatus);
	//Wire.write((index >> 8) & 0xff);	  	
	i2cWriteStatus += i2c_write(index & 0xff);
	//printf("I2C write status: %d\n\r", i2cWriteStatus);
	//Wire.write(index & 0xff);	
	if (i2cWriteStatus != 0) { return VL53L1_ERROR_CONTROL_INTERFACE; }
	//if (Wire.endTransmission() != 0) { return VL53L1_ERROR_CONTROL_INTERFACE; }	
			
	if (i2c_rep_start(Dev->I2cDevAddr | I2C_READ) != 0) { return VL53L1_ERROR_CONTROL_INTERFACE; }
	//if (Wire.requestFrom(Dev->I2cDevAddr >> 1, 1) != 1) { return VL53L1_ERROR_CONTROL_INTERFACE; }
  	*data = i2c_readNak();
	//*data = Wire.read();
    
	i2c_stop();
  
	return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_RdWord(VL53L1_DEV Dev, uint16_t index, uint16_t *data)
{
  	int i2cWriteStatus = 0;
	  	
	i2c_start_wait(Dev->I2cDevAddr | I2C_WRITE);
	//Wire.beginTransmission(Dev->I2cDevAddr >> 1);	  	
	i2cWriteStatus += i2c_write((index >> 8) & 0xff);
	//Wire.write((index >> 8) & 0xff);	  	
	i2cWriteStatus += i2c_write(index & 0xff);
	//Wire.write(index & 0xff);	
	if (i2cWriteStatus != 0) { return VL53L1_ERROR_CONTROL_INTERFACE; }
	//if (Wire.endTransmission() != 0) { return VL53L1_ERROR_CONTROL_INTERFACE; }	
			
	if (i2c_rep_start(Dev->I2cDevAddr | I2C_READ) != 0) { return VL53L1_ERROR_CONTROL_INTERFACE; }
	//if (Wire.requestFrom(Dev->I2cDevAddr >> 1, 2) != 2) { return VL53L1_ERROR_CONTROL_INTERFACE; }
	*data = (uint16_t)i2c_readAck() << 8;
	//*data = (uint16_t)Wire.read() << 8;  
	*data |= i2c_readNak();
	//*data |= Wire.read();

	i2c_stop();     
  
  return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_RdDWord(VL53L1_DEV Dev, uint16_t index, uint32_t *data)
{
  	int i2cWriteStatus = 0;
	  	
	i2c_start_wait(Dev->I2cDevAddr | I2C_WRITE);
	//Wire.beginTransmission(Dev->I2cDevAddr >> 1);	  	
	i2cWriteStatus += i2c_write((index >> 8) & 0xff);
	//Wire.write((index >> 8) & 0xff);	  	
	i2cWriteStatus += i2c_write(index & 0xff);
	//Wire.write(index & 0xff);	
	if (i2cWriteStatus != 0) { return VL53L1_ERROR_CONTROL_INTERFACE; }
	//if (Wire.endTransmission() != 0) { return VL53L1_ERROR_CONTROL_INTERFACE; }	
			
	if (i2c_rep_start(Dev->I2cDevAddr | I2C_READ) != 0) { return VL53L1_ERROR_CONTROL_INTERFACE; }
	//if (Wire.requestFrom(Dev->I2cDevAddr >> 1, 4) != 4) { return VL53L1_ERROR_CONTROL_INTERFACE; }
	*data = (uint32_t)i2c_readAck() << 24;
	//*data = (uint32_t)Wire.read() << 24;	
	*data |= (uint32_t)i2c_readAck() << 16;
	//*data |= (uint32_t)Wire.read() << 16;	
	*data |= (uint32_t)i2c_readAck() << 8;
	//*data |= (uint32_t)Wire.read() << 8;  
	*data |= i2c_readNak();
	//*data |= Wire.read();
  
	i2c_stop(); 
	
  return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GetTickCount(uint32_t *ptick_count_ms)
{	
  *ptick_count_ms = TCNT1;  //ENSURE YOUR TIMER IS SETUP CORRECTLY!!
  //printf("test\n");
	return VL53L1_ERROR_NONE;
}

//#define trace_print(level, ...) \
//	_LOG_TRACE_PRINT(VL53L1_TRACE_MODULE_PLATFORM, \
//	level, VL53L1_TRACE_FUNCTION_NONE, ##__VA_ARGS__)

//#define trace_i2c(...) \
//	_LOG_TRACE_PRINT(VL53L1_TRACE_MODULE_NONE, \
//	VL53L1_TRACE_LEVEL_NONE, VL53L1_TRACE_FUNCTION_I2C, ##__VA_ARGS__)

VL53L1_Error VL53L1_GetTimerFrequency(int32_t *ptimer_freq_hz)
{
	return VL53L1_ERROR_NOT_IMPLEMENTED;
}

VL53L1_Error VL53L1_WaitMs(VL53L1_Dev_t *pdev, int32_t wait_ms)
{
  _delay_ms(wait_ms);
	return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WaitUs(VL53L1_Dev_t *pdev, int32_t wait_us)
{
  _delay_us(wait_us);
	return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WaitValueMaskEx(
	VL53L1_Dev_t *pdev,
	uint32_t      timeout_ms,
	uint16_t      index,
	uint8_t       value,
	uint8_t       mask,
	uint32_t      poll_delay_ms)
{
  uint8_t data;
  VL53L1_Error status;

  while (timeout_ms > 0)
  {
    status = VL53L1_RdByte(pdev, index, &data);
    if (status != VL53L1_ERROR_NONE) { return status; }
    if ((data & mask) == value) { return VL53L1_ERROR_NONE; }
    _delay_ms(poll_delay_ms);
    timeout_ms -= MIN(poll_delay_ms, timeout_ms);
	//timeout_ms -= min(poll_delay_ms, timeout_ms);
  }

  return VL53L1_ERROR_TIME_OUT;
}




