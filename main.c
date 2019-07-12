/*
 * Created: 2019-04-19 4:03:44 PM
 * Author : SiebenLab
 */ 

//Definitions
#define F_CPU 14745600UL

//Standard Includes
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>

//Custom Includes
#include "vl53l1x/uart.h"				//Header includes setup for UART
#include "vl53l1x/twimaster.h"
#include "vl53l1x/vl53l1_api.h"			//LiDAR API Header

//Setup LIDAR Device
VL53L1_Dev_t	dev1, dev2;//, dev3;				//declare new objets for each sensor 
VL53L1_DEV	Dev1 = &dev1;				
VL53L1_DEV	Dev2 = &dev2;
//VL53L1_DEV	Dev3 = &dev3;

int status;
VL53L1_RangingMeasurementData_t RangingData1, RangingData2;//RangingData3;	//declare results struct for each sensor

int setup_lidar(VL53L1_DEV Dev, int addr)
{
	uint8_t byteData;
	uint16_t wordData;
	
	Dev->I2cDevAddr = 0x52;
	VL53L1_software_reset(Dev);
	VL53L1_RdByte(Dev, 0x010F, &byteData);
	printf("VL53L1X Model_ID: %X\n\r", byteData);		
	VL53L1_RdByte(Dev, 0x0110, &byteData);
	printf("VL53L1X Module_Type: %X\n\r", byteData);
	VL53L1_RdWord(Dev, 0x010F, &wordData);
	printf("VL53L1X: %X\n\r",wordData);
	printf("Autonomous Ranging Test\n\r");
	status = VL53L1_WaitDeviceBooted(Dev);
	status = VL53L1_SetDeviceAddress(Dev, addr);
	printf("ADDR Set to 0x%03X! Status: %d\n\r", addr, status);
	status = VL53L1_DataInit(Dev);
	printf("Data Init'd: %d\n\r", status);
	status = VL53L1_StaticInit(Dev);
	printf("Static Init'd: %d\n\r", status);
	status = VL53L1_SetDistanceMode(Dev, VL53L1_DISTANCEMODE_LONG);
	printf("Mode Set: %d\n\r", status);
	status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(Dev, 50000);
	printf("Timing1 Set: %d\n\r", status);
	status = VL53L1_SetInterMeasurementPeriodMilliSeconds(Dev, 50);
	printf("Timing2 Set: %d\n\r", status); 
	VL53L1_UserRoi_t roiConfig;
		roiConfig.TopLeftX  = 1;
		roiConfig.TopLeftY  = 15;
		roiConfig.BotRightX = 15;
		roiConfig.BotRightY = 1;
	status = VL53L1_SetUserROI(Dev, &roiConfig);
	printf("Rio Configured: %d\n\r", status); 
	//status = VL53L1_StartMeasurement(Dev);	
	//printf("Measure began: %d\n\r", status); 
	if(status)
	{
		printf("Status: %i\n\r",status);
		printf("VL53L1_StartMeasurement failed... Halting program execution.\n\r");
		while(1);	//... and kill progression
	}
	return status;
}

int main(void)
{
	//Interrupt-based initializations
	cli();								//Disable Interrupts
	init_uart();						//Initialize UART as per uart.h	(baud/etc)	
	i2c_init();							//Init I2C bus
	sei();								//Enable Interrupts
	//set up IOs for XSHUNT control
	DDRA |= (1<<DDA0) | (1<<DDA1);		//enable PA0&PA1 as output
	PORTA = 0x00;						//disable all sensor
	
	PORTA |= (1<<PA0);					//enable Sensor 1;	
	_delay_ms(10);					
	printf("LiDAR 1 Enabled\n\r");
	//Initialize the 1st LIDAR with ADDR 0x39
	setup_lidar(Dev1, 0x38);
	printf("LiDAR 1 Set\n\r");
	
	PORTA |= (1<<PA1);		//enable Sensor 1&2;
	_delay_ms(10);	
	printf("LiDAR 2 Enabled\n\r");
	//Initialize the 2nd LIDAR with ADDR 0x40
	setup_lidar(Dev2, 0x39);
	printf("LiDAR 2 Set\n\r");
	
	/*PORTA |= (1<<PA2);		//enable Sensor 1&2;
	_delay_ms(10);
	printf("LiDAR 3 Enabled\n\r");
	//Initialize the 3rd LIDAR with ADDR 0x41
	setup_lidar(Dev3, 0x40);
	printf("LiDAR 3 Set\n\r");
	*/
	//continuous measurements start
	VL53L1_StartMeasurement(Dev1);
	VL53L1_StartMeasurement(Dev2);
	//VL53L1_StartMeasurement(Dev3);
	
	printf("-----------Starting to read ranges:\n\r");
	while (1)
	{ 
		 status = VL53L1_WaitMeasurementDataReady(Dev1) + VL53L1_WaitMeasurementDataReady(Dev2);// + VL53L1_WaitMeasurementDataReady(Dev3);	  
		 if(!status)
		  {
			  status = VL53L1_GetRangingMeasurementData(Dev1, &RangingData1) + VL53L1_GetRangingMeasurementData(Dev2, &RangingData2);// + VL53L1_GetRangingMeasurementData(Dev3, &RangingData3);		  
			  if(status==0)
			  {
				  printf("Range1: %i mm, ", RangingData1.RangeMilliMeter);		//print data from sensor 1
				  printf("Range2: %i mm, \n\r", RangingData2.RangeMilliMeter);	//print data from sensor 2
				  // printf("Range3: %i mm, \n\r", RangingData3.RangeMilliMeter);
				  //printf("Range2: %i mm, \n\r", RangingData2.RangeMilliMeter);
				  //printf("Status: %u, ", RangingData.RangeStatus);
				  //printf("Signal Rate: %3.2lf, ", RangingData.SignalRateRtnMegaCps/65536.0);
				  //printf("Ambient Rate: %3.2lf",RangingData.AmbientRateRtnMegaCps/65336.0);
			  }			  
			 status = VL53L1_ClearInterruptAndStartMeasurement(Dev1) + VL53L1_ClearInterruptAndStartMeasurement(Dev2);// + VL53L1_ClearInterruptAndStartMeasurement(Dev3);
		  }
		  else{printf("Error waiting for data ready: %i\n\r",status);}			  
		//_delay_ms(1000);				//one second delay
	}
}

