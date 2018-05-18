/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @brief          :
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"
/* USER CODE BEGIN INCLUDE */
/* USER CODE END INCLUDE */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_CDC 
  * @brief usbd core module
  * @{
  */ 

/** @defgroup USBD_CDC_Private_TypesDefinitions
  * @{
  */ 
/* USER CODE BEGIN PRIVATE_TYPES */
/* USER CODE END PRIVATE_TYPES */ 
/**
  * @}
  */ 

/** @defgroup USBD_CDC_Private_Defines
  * @{
  */ 
/* USER CODE BEGIN PRIVATE_DEFINES */
/* Define size for the receive and transmit buffer over CDC */
/* It's up to user to redefine and/or remove those define */
//this is a goofy linear buffer as required by the USB middleware, much in the
//same way as for the UART stuff (there, the goofy buffer is 1 byte; here it is
//the maximum transfer size for USB FS, 64 bytes.  I don't see any reason to
//make it bigger.  I also don't see any reason to make it smaller, since there
//is no provision for communicating the size of UserRxBufferFS, so it seems
//like a buffer overflow fest to me in that regards.
#define APP_RX_DATA_SIZE  CDC_DATA_FS_MAX_PACKET_SIZE
#define APP_TX_DATA_SIZE  CDC_DATA_FS_MAX_PACKET_SIZE
__weak void USBCDC_DataAvailable ( void );
__weak void USBCDC_TransmitEmpty ( void );
/* USER CODE END PRIVATE_DEFINES */
/**
  * @}
  */ 

/** @defgroup USBD_CDC_Private_Macros
  * @{
  */ 
/* USER CODE BEGIN PRIVATE_MACRO */
/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */ 
  
/** @defgroup USBD_CDC_Private_Variables
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/* Received Data over USB are stored in this buffer       */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/* Send Data over USB CDC are stored in this buffer       */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */

/*******************************************************************************/
/* Line Coding Structure                                                       */
/*-----------------------------------------------------------------------------*/
/* Offset | Field       | Size | Value  | Description                          */
/* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
/* 4      | bCharFormat |   1  | Number | Stop bits                            */
/*                                        0 - 1 Stop bit                       */
/*                                        1 - 1.5 Stop bits                    */
/*                                        2 - 2 Stop bits                      */
/* 5      | bParityType |  1   | Number | Parity                               */
/*                                        0 - None                             */
/*                                        1 - Odd                              */ 
/*                                        2 - Even                             */
/*                                        3 - Mark                             */
/*                                        4 - Space                            */
/* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
/*******************************************************************************/
//XXX
static USBD_CDC_LineCodingTypeDef __LineCoding =
{
	115200,	//baud rate
	0x00,	//stop bits-1
	0x00,	//parity - none
	0x08	//nb. of bits 8
};


/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */ 
  
/** @defgroup USBD_CDC_IF_Exported_Variables
  * @{
  */ 
  extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE BEGIN EXPORTED_VARIABLES */
/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */ 
  
/** @defgroup USBD_CDC_Private_FunctionPrototypes
  * @{
  */
static int8_t CDC_Init_FS     (void);
static int8_t CDC_DeInit_FS   (void);
static int8_t CDC_Control_FS  (uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS  (uint8_t* pbuf, uint32_t *Len);
/* USER CODE BEGIN MyCDCExt */
static void CDC_TsComplete_FS (uint8_t* pbuf, uint32_t Len);
//this is a little hack to work around the fact that re-generating code with
//STM32CubeMX will overwrite our changes (since they have to be in a
//non-"USER CODE BEGIN" demarcated block.  Further, when it does overwrite
//those changes, the project will still build, but just not work.  This
//presence hack will force the linkage to fail, making it obvious that the
//changes need to be re-applied.
void XXX_USBCDC_PresenceHack ( void )
{
	volatile int i = 0;	//thou shalt not optimize away
	(void)i;	//thou shalt not cry
}
/* USER CODE END MyCDCExt */

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */
/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */ 
  
USBD_CDC_ItfTypeDef USBD_Interface_fops_FS = 
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,  
  CDC_Receive_FS
/* USER CODE BEGIN MyCDCExt */
  , CDC_TsComplete_FS
/* USER CODE END MyCDCExt */
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  CDC_Init_FS
  *         Initializes the CDC media low layer over the FS USB IP
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{ 
  /* USER CODE BEGIN 3 */ 
	/* Set Application Buffers */
	//for some reason we bind the TX buffer of zero length, but the generated
	//code never uses that buffer again (it instead binds user buffers hoping
	//they will remain stable for the lifetime of the transfer).
	USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
	USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
	
	//immediately 'arm' reception of data to prime the pump
	USBD_CDC_ReceivePacket(&hUsbDeviceFS);
	
	return (USBD_OK);
  /* USER CODE END 3 */ 
}

/**
  * @brief  CDC_DeInit_FS
  *         DeInitializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */ 
	return (USBD_OK);
  /* USER CODE END 4 */ 
}

/**
  * @brief  CDC_Control_FS
  *         Manage the CDC class requests
  * @param  cmd: Command code            
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS  (uint8_t cmd, uint8_t* pbuf, uint16_t length)
{ 
  /* USER CODE BEGIN 5 */
	switch (cmd)
	{
		case CDC_SEND_ENCAPSULATED_COMMAND:
		break;

		case CDC_GET_ENCAPSULATED_RESPONSE:
		break;

		case CDC_SET_COMM_FEATURE:
		break;

		case CDC_GET_COMM_FEATURE:
		break;

		case CDC_CLEAR_COMM_FEATURE:
		break;

		/*******************************************************************************/
		/* Line Coding Structure                                                       */
		/*-----------------------------------------------------------------------------*/
		/* Offset | Field       | Size | Value  | Description                          */
		/* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
		/* 4      | bCharFormat |   1  | Number | Stop bits                            */
		/*                                        0 - 1 Stop bit                       */
		/*                                        1 - 1.5 Stop bits                    */
		/*                                        2 - 2 Stop bits                      */
		/* 5      | bParityType |  1   | Number | Parity                               */
		/*                                        0 - None                             */
		/*                                        1 - Odd                              */ 
		/*                                        2 - Even                             */
		/*                                        3 - Mark                             */
		/*                                        4 - Space                            */
		/* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
		/*******************************************************************************/
		case CDC_SET_LINE_CODING:
		/*	//XXX do I even want to change it?
			__LineCoding.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) |\
					(pbuf[2] << 16) | (pbuf[3] << 24));
			__LineCoding.format     = pbuf[4];
			__LineCoding.paritytype = pbuf[5];
			__LineCoding.datatype   = pbuf[6];
		*/
		break;

		case CDC_GET_LINE_CODING:
			pbuf[0] = (uint8_t)(__LineCoding.bitrate);
			pbuf[1] = (uint8_t)(__LineCoding.bitrate >> 8);
			pbuf[2] = (uint8_t)(__LineCoding.bitrate >> 16);
			pbuf[3] = (uint8_t)(__LineCoding.bitrate >> 24);
			pbuf[4] = __LineCoding.format;
			pbuf[5] = __LineCoding.paritytype;
			pbuf[6] = __LineCoding.datatype;
		break;

		case CDC_SET_CONTROL_LINE_STATE:
		//XXX may be interesting to handle this; presumably flow control stuff
		break;

		case CDC_SEND_BREAK:
		break;

		default:
		break;
	}

	return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  CDC_Receive_FS
  *         Data received over USB OUT endpoint are sent over CDC interface 
  *         through this function.
  *           
  *         @note
  *         This function will block any OUT packet reception on USB endpoint 
  *         untill exiting this function. If you exit this function before transfer
  *         is complete on CDC interface (ie. using DMA controller) it will result 
  *         in receiving more data while previous ones are still not sent.
  *                 
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS (uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */

	//XXX this is the original stuff; this is a static function, so I think it
	//is called-back from the driver?  It seems that this method takes a
	//length, so I guess we are to hope we can take it all?  The param is a
	//pointer, so are we meant to communicate back how much we could take in
	//case we can't take it all?  Would that scenario violate the @note
	//caveat?  hmm...
	/*
	USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
	USBD_CDC_ReceivePacket(&hUsbDeviceFS);
	return (USBD_OK);
	*/
	
	//new implementation; get it, and push it out of here for others to
	//deal with.  This
	USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
	USBD_CDC_ReceivePacket(&hUsbDeviceFS);
	//YYY this is in usbd_cdc_if.c
	extern size_t XXX_Push_USBCDC_RxData ( const uint8_t* pbyBuffer, const size_t nAvail );
	size_t nPushed = XXX_Push_USBCDC_RxData ( &Buf[0], (size_t)*Len );
	if ( nPushed != *Len )
	{
		//horror; dropped data
	}
	
	USBCDC_DataAvailable();	//notify data is available

	return (USBD_OK);
	
  /* USER CODE END 6 */ 
}

/**
  * @brief  CDC_Transmit_FS
  *         Data send over USB IN endpoint are sent over CDC interface 
  *         through this function.           
  *         @note
  *         
  *                 
  * @param  Buf: Buffer of data to be send
  * @param  Len: Number of data to be send (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */ 

	//XXX this is the original stuff.  It seems really goofy that the
	//USBD_CDC_SetTxBuffer call sets up with the caller's buffer, instead of
	//referencing the UserTxBufferFS, which as best as I can tell serves no
	//useful function in this generated implementation (it is bound at init
	//with a zero size, but we are now binding directly to user buffers for
	//actual use of the driver, and there are no other references to the
	//UserTxBufferFS.  hmmm.)
	/*
	USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
	if (hcdc->TxState != 0){
		return USBD_BUSY;
	}
	USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
	result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
	*/
	
	//XXX new implementation:
	//If we are not 'busy', try to kickstart the transmission by dequeueing
	//as much as possible into our (goofy) private /linear buffer, and
	//invoking USBD_CDC_SetTxBuffer and USBD_CDC_TransmitPacket.
	//If we are 'busy', then we don't need to do anything special, because
	//the CDC_TsComplete_FS callback will continue the process until the
	//data is exhausted.
	//There.  No goofy polling timers.  It's tacky we have to have these
	//goofy private linear buffers, so maybe I'll eventually rewrite this
	//stuff, but right now I'm going to leave well enough alone.
	USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
	if (hcdc->TxState != 0){
		return USBD_BUSY;
	}
	//YYY this is in usbd_cdc_if.c
	extern size_t XXX_Pull_USBCDC_TxData ( uint8_t* pbyBuffer, const size_t nMax );
	size_t nPulled = XXX_Pull_USBCDC_TxData ( UserTxBufferFS, APP_TX_DATA_SIZE );
	if ( 0 != nPulled )
	{
		USBD_CDC_SetTxBuffer ( &hUsbDeviceFS, UserTxBufferFS, nPulled );
		result = USBD_CDC_TransmitPacket ( &hUsbDeviceFS );
	}
	else
	{
		USBCDC_TransmitEmpty();	//notify transmit is empty
	}
	UNUSED(Buf);
	UNUSED(Len);
	
  /* USER CODE END 7 */ 
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

//I added this callback (lower driver into us) to get notification that a
//transmission has completed, and that now we can start some more.
static void CDC_TsComplete_FS (uint8_t* pbuf, uint32_t Len)
{
	//just kick off a new transmission if we can.
	CDC_Transmit_FS(NULL,0);	//Note, these parameters no longer have meaning
	UNUSED(pbuf);
	UNUSED(Len);
}



//the DAV callback (we make to the user) is optional
__weak void USBCDC_DataAvailable ( void )
{
}

//the TBMT callback (we make to the user) is optional
__weak void USBCDC_TransmitEmpty ( void )
{
}



/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

