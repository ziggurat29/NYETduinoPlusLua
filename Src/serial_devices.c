//==============================================================
//This realizes the various serial devices in the system.
//This module is part of the BluePill82240Interface project.
//Some parts of this logical component are realized in main.c, as a
//consequence of the STM32CubeMX code generator.
//Additionally, there are many modifications to the machine-generated
//USB CDC middleware that are required for that stuff to work as well.

#include "serial_devices.h"

#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

//#include "lamps.h"
//#include "task_notification_bits.h"

#include "util_circbuff2.h"



//Because of the peculiarities of the STM32CubeMX, we are leaving these things
//in main.c.  They come from generated code, and if we fight the system, we
//will be in an eternal struggle of light against darkness. So we go to the
//dark side and leave them there.
extern UART_HandleTypeDef huart6;
//NOTE there is not handle-y thing for the CDC for us (well, sort of, there is
//an object buried in the 'middleware', but we don't need it.


//UART transmit/receive circular buffers
CIRCBUF(UART6_txbuff,uint8_t,128);
CIRCBUF(UART6_rxbuff,uint8_t,128);

//USB CDC transmit/receive circular buffers
CIRCBUF(CDC_txbuff,uint8_t,128);
CIRCBUF(CDC_rxbuff,uint8_t,128);





//these are debug methods for tuning buffer sizes
#ifdef DEBUG

unsigned int UART6_txbuff_max ( void )
{
	return circbuff_max ( &UART6_txbuff );
}

unsigned int UART6_rxbuff_max ( void )
{
	return circbuff_max ( &UART6_rxbuff );
}

unsigned int CDC_txbuff_max ( void )
{
	return circbuff_max ( &CDC_txbuff );
}

unsigned int CDC_rxbuff_max ( void )
{
	return circbuff_max ( &CDC_rxbuff );
}

#endif


//========================================================================



static void UART6_flushTtransmit ( const IOStreamIF* pthis );
static size_t UART6_transmitFree ( const IOStreamIF* pthis );
static size_t UART6_transmit ( const IOStreamIF* pthis, const void* pv, size_t nLen );
static void UART6_flushReceive ( const IOStreamIF* pthis );
static size_t UART6_receiveAvailable ( const IOStreamIF* pthis );
static size_t UART6_receive ( const IOStreamIF* pthis, void* pv, const size_t nLen );


static void USBCDC_flushTtransmit ( const IOStreamIF* pthis );
static size_t USBCDC_transmitFree ( const IOStreamIF* pthis );
static void USBCDC_flushReceive ( const IOStreamIF* pthis );
static size_t USBCDC_receiveAvailable ( const IOStreamIF* pthis );
static size_t USBCDC_transmit ( const IOStreamIF* pthis, const void* pv, size_t nLen );
static size_t USBCDC_receive ( const IOStreamIF* pthis, void* pv, const size_t nLen );


static int Serial_transmitCompletely ( const IOStreamIF* pcom, const void* pv, size_t nLen, uint32_t to );
static int Serial_receiveCompletely ( const IOStreamIF* pcom, void* pv, const size_t nLen, uint32_t to );


const IOStreamIF g_pifUART6 = {
	UART6_flushTtransmit,
	UART6_transmitFree,
	UART6_transmit,
	UART6_flushReceive,
	UART6_receiveAvailable,
	UART6_receive,
	Serial_transmitCompletely,
	Serial_receiveCompletely,
	&huart6
};


const IOStreamIF g_pifCDC = {
	USBCDC_flushTtransmit,
	USBCDC_transmitFree,
	USBCDC_transmit,
	USBCDC_flushReceive,
	USBCDC_receiveAvailable,
	USBCDC_receive,
	Serial_transmitCompletely,
	Serial_receiveCompletely,
	NULL
};




//====================================================
//UART support; this glues the UART to the relevant circular buffers


//goofy kickstarters; the STM HAL functions take a pointer to a buffer which
//needs to be stable for later processing at ISR time, so we have these
//knumbskull 1-byte buffers to satisfy that.  There could be a way for me
//to point directly into the circular queue space, but it would be messy and
//bug-prone, so I am punting on that now.  Later, I'll probably write my own
//low-level UART code, since I'm not in love with the HAL API for that, anyway.

static volatile uint8_t _byTxNow;	//knumbskull TX buffer for UART6
static void __kickstartTransmitUART6()
{
	//XXX_byTxNow = UART6_txbuff_dequeue();	//
	circbuff_dequeue(&UART6_txbuff,(void*)&_byTxNow);	//
	if(HAL_UART_Transmit_IT(&huart6, (uint8_t*)&_byTxNow, sizeof(_byTxNow)) != HAL_OK)
	{
		//XXX horror
		//LightLamp ( 2000, &g_lltOr, _ledOnWh );
	}
}

volatile uint8_t _byRxNow;	//knumbskull RX buffer for UART6
static void __kickstartReceiveUART6()
{
	//set up to receive more
	//if ( HAL_UART_STATE_BUSY_RX == huart6.State || HAL_UART_STATE_BUSY_TX_RX == huart6.State )	//must grope for RX only state
	if(HAL_UART_Receive_IT(&huart6, (uint8_t*)&_byRxNow, sizeof(_byRxNow)) != HAL_OK)
	{
		//XXX horror
		//LightLamp ( 2000, &g_lltOr, _ledOnWh );
	}
}



//our stub implementation of the optional notification callbacks
__weak void UART6_DataAvailable ( void ){}
__weak void UART6_TransmitEmpty ( void ){}



//A UART has completed transmission.  Push more if we've got it.
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if ( USART2 == huart->Instance )
	{
		int bEmpty;
UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();	//lock queue XXX heavy handed
		//if there is more in the queue, pluck and transmit
		//XXX if ( ! UART6_txbuff_empty() )
		if ( ! circbuff_empty(&UART6_txbuff) )
		{
			__kickstartTransmitUART6();
			bEmpty = 0;
		}
		else
		{
			bEmpty = 1;
		}
taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);	//unlock queue XXX heavy handed
		if ( bEmpty )
			UART6_TransmitEmpty();	//notify anyone interested
	}
	else if ( USART1 == huart->Instance )
	{
		//XXX III
	}
}



//A UART has completed reception.  Stick it in our queue if we can.
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if ( USART2 == huart->Instance )
	{
UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();	//lock queue XXX heavy handed
		//XXX if ( ! UART6_rxbuff_full() )
		if ( ! circbuff_full(&UART6_rxbuff) )
		{
			//XXX UART6_rxbuff_enqueue ( _byRxNow );
			circbuff_enqueue ( &UART6_rxbuff, (void*)&_byRxNow );
		}
		else
		{
			//XXX horror; buffer overrun
			//LightLamp ( 2000, &g_lltOr, _ledOnWh );
		}
		//either way, set up to receive more
		__kickstartReceiveUART6();
taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);	//unlock queue XXX heavy handed
		UART6_DataAvailable();	//notify anyone interested
	}
	else if ( USART1 == huart->Instance )
	{
		//XXX III
	}
}



//UART error
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if ( USART2 == huart->Instance )
	{
		//XXX III
	}
	else if ( USART1 == huart->Instance )
	{
		//XXX III
	}
}



//====================================================
//USB CDC support


//The ST USB CDC 'middleware' is also a bit goofy (like the UART
//implementation.  Actually, I think it's worse).  We have attempted to spackle
//over the oddities with a different API that uses circular buffers to push and
//pull data from the USB CDC channel.  The modified middleware implementation
//needs to get data from these circular buffers; it does this by way of these
//callbacks.
//Eventually, I may move this internal to the USB CDC implementation, but it is
//not yet clear that this will improve things, so I am keeping them here for
//the moment, ugly though that may be.
size_t XXX_Pull_USBCDC_TxData ( uint8_t* pbyBuffer, const size_t nMax )
{
	size_t nPulled;
UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();	//lock queue XXX heavy handed
	//XXX size_t nToPull = CDC_txbuff_count();	//max you could pull
	size_t nToPull = circbuff_count(&CDC_txbuff);	//max you could pull
	if ( nMax < nToPull )	//no buffer overruns, please
		nToPull = nMax;
	for ( nPulled = 0; nPulled < nToPull; ++nPulled )
	{
		//XXX pbyBuffer[nPulled] = CDC_txbuff_dequeue();
		circbuff_dequeue(&CDC_txbuff,&pbyBuffer[nPulled]);
	}
taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);	//unlock queue XXX heavy handed
	return nPulled;
}

size_t XXX_Push_USBCDC_RxData ( const uint8_t* pbyBuffer, const size_t nAvail )
{
	size_t nPushed;
UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();	//lock queue XXX heavy handed
	//XXX size_t nToPush = CDC_rxbuff_capacity() - CDC_rxbuff_count();	//max you could push
	size_t nToPush = circbuff_capacity(&CDC_rxbuff) - circbuff_count(&CDC_rxbuff);	//max you could push
	if ( nAvail < nToPush )	//no buffer overruns, please
		nToPush = nAvail;
	for ( nPushed = 0; nPushed < nToPush; ++nPushed )
	{
		//XXX CDC_rxbuff_enqueue ( pbyBuffer[nPushed] );
		circbuff_enqueue ( &CDC_rxbuff, &pbyBuffer[nPushed] );
	}
taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);	//unlock queue XXX heavy handed
	return nPushed;
}



//====================================================
//UART read/write API



static void UART6_flushTtransmit ( const IOStreamIF* pthis )
{
UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();	//lock queue XXX heavy handed
	//XXX UART6_txbuff_init();
	circbuff_init(&UART6_txbuff);
taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);	//unlock queue XXX heavy handed
}


static void UART6_flushReceive ( const IOStreamIF* pthis )
{
UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();	//lock queue XXX heavy handed
	//XXX UART6_rxbuff_init();
	circbuff_init(&UART6_rxbuff);
taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);	//unlock queue XXX heavy handed
}



static size_t UART6_transmit ( const IOStreamIF* pthis, const void* pv, size_t nLen )
{
	size_t nPushed;
UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();	//lock queue XXX heavy handed
	//XXX size_t nToPush = UART6_txbuff_capacity() - UART6_txbuff_count();	//max you could push
	size_t nToPush = circbuff_capacity(&UART6_txbuff) - circbuff_count(&UART6_txbuff);	//max you could push
	if ( nLen < nToPush )	//no buffer overruns, please
		nToPush = nLen;
	for ( nPushed = 0; nPushed < nToPush; ++nPushed )
	{
		//XXX UART6_txbuff_enqueue ( ((uint8_t*)pv)[nPushed] );
		circbuff_enqueue ( &UART6_txbuff, &((uint8_t*)pv)[nPushed] );
	}
	//if the transmitter is idle, we will need to kickstart it
//old HAL lib had one state var
//	if ( HAL_UART_STATE_READY == huart6.State ||
//			HAL_UART_STATE_BUSY_RX == huart6.State
//new HAL lib split state into two vars
	if ( HAL_UART_STATE_READY == huart6.gState ||
			HAL_UART_STATE_BUSY_RX == huart6.RxState
		)	//must grope for TX only ready state
	{
		__kickstartTransmitUART6();
	}
	else
	{
		//dummy = 0;
	}
taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);	//unlock queue XXX heavy handed
	return nPushed;
}



static size_t UART6_receive ( const IOStreamIF* pthis, void* pv, const size_t nLen )
{
	size_t nPulled;
UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();	//lock queue XXX heavy handed
	//XXX size_t nToPull = UART6_rxbuff_count();	//max you could pull
	size_t nToPull = circbuff_count(&UART6_rxbuff);	//max you could pull
	if ( nLen < nToPull )	//no buffer overruns, please
		nToPull = nLen;
	for ( nPulled = 0; nPulled < nToPull; ++nPulled )
	{
		//XXX ((uint8_t*)pv)[nPulled] = UART6_rxbuff_dequeue();
		circbuff_dequeue(&UART6_rxbuff, &((uint8_t*)pv)[nPulled]);
	}
taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);	//unlock queue XXX heavy handed
	return nPulled;
}



//what are the number of bytes available to be read now
static size_t UART6_receiveAvailable ( const IOStreamIF* pthis )
{
	size_t n;
UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();	//lock queue XXX heavy handed
	//XXX n = UART6_rxbuff_count();
	n = circbuff_count(&UART6_rxbuff);
taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);	//unlock queue XXX heavy handed
	return n;
}



//how much can be pushed into the transmitter buffers now
static size_t UART6_transmitFree ( const IOStreamIF* pthis )
{
	size_t n;
UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();	//lock queue XXX heavy handed
	//XXX n = UART6_txbuff_capacity() - UART6_txbuff_count();
	n = circbuff_capacity(&UART6_txbuff) - circbuff_count(&UART6_txbuff);
taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);	//unlock queue XXX heavy handed
	return n;
}



//====================================================
//USB CDC read/write API



static void USBCDC_flushTtransmit ( const IOStreamIF* pthis )
{
UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();	//lock queue XXX heavy handed
	//XXX CDC_txbuff_init();
	circbuff_init(&CDC_txbuff);
taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);	//unlock queue XXX heavy handed
}


static void USBCDC_flushReceive ( const IOStreamIF* pthis )
{
UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();	//lock queue XXX heavy handed
	//XXX CDC_rxbuff_init();
	circbuff_init(&CDC_rxbuff);
taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);	//unlock queue XXX heavy handed
}


static size_t USBCDC_transmit ( const IOStreamIF* pthis, const void* pv, size_t nLen )
{
	size_t nPushed;
UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();	//lock queue XXX heavy handed
	//XXX size_t nToPush = CDC_txbuff_capacity() - CDC_txbuff_count();	//max you could push
	size_t nToPush = circbuff_capacity(&CDC_txbuff) - circbuff_count(&CDC_txbuff);	//max you could push
	if ( nLen < nToPush )	//no buffer overruns, please
		nToPush = nLen;
	for ( nPushed = 0; nPushed < nToPush; ++nPushed )
	{
		//XXX CDC_txbuff_enqueue ( ((uint8_t*)pv)[nPushed] );
		circbuff_enqueue ( &CDC_txbuff, &((uint8_t*)pv)[nPushed] );
	}
taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);	//unlock queue XXX heavy handed
	//notify to kick-start transmission, if needed
	CDC_Transmit_FS(NULL, 0);
	return nPushed;
}



static size_t USBCDC_receive ( const IOStreamIF* pthis, void* pv, const size_t nLen )
{
	size_t nPulled;
UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();	//lock queue XXX heavy handed
	//XXX size_t nToPull = CDC_rxbuff_count();	//max you could pull
	size_t nToPull = circbuff_count(&CDC_rxbuff);	//max you could pull
	if ( nLen < nToPull )	//no buffer overruns, please
		nToPull = nLen;
	for ( nPulled = 0; nPulled < nToPull; ++nPulled )
	{
		//XXX ((uint8_t*)pv)[nPulled] = CDC_rxbuff_dequeue();
		circbuff_dequeue(&CDC_rxbuff,&((uint8_t*)pv)[nPulled]);
	}
taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);	//unlock queue XXX heavy handed
	return nPulled;
}



//what are the number of bytes available to be read now
static size_t USBCDC_receiveAvailable ( const IOStreamIF* pthis )
{
	size_t n;
UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();	//lock queue XXX heavy handed
	//XXX n = CDC_rxbuff_count();
	n = circbuff_count(&CDC_rxbuff);
taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);	//unlock queue XXX heavy handed
	return n;
}



//how much can be pushed into the transmitter buffers now
static size_t USBCDC_transmitFree ( const IOStreamIF* pthis )
{
	size_t n;
UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();	//lock queue XXX heavy handed
	//XXX n = CDC_txbuff_capacity() - CDC_txbuff_count();
	n = circbuff_capacity(&CDC_txbuff) - circbuff_count(&CDC_txbuff);
taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);	//unlock queue XXX heavy handed
	return n;
}



//our stub implementation of the optional notification callbacks
__weak void USBCDC_DataAvailable ( void ){}
__weak void USBCDC_TransmitEmpty ( void ){}




//====================================================
//Generalized blocking serial transmit/receive functions; these take function
//pointers to the send/receive methods for particular serial ports



//A simple blocking transmit function.  This just does a sleep when the
//transmit buffer cannot accept more.  Not sophisticated, but still better than
//polling.
//The return value is the portion of nLen that has /not/ been processed; so
//0 means success, and non-zero means failure, and nLen-(return) means how
//much /was/ processed.
static int Serial_transmitCompletely ( const IOStreamIF* pcom, const void* pv, size_t nLen, uint32_t to )
{
	uint32_t tsStart;
	size_t nIdxNow;
	size_t nRemaining;
	size_t nDone;
	
	tsStart = HAL_GetTick();
	
	nIdxNow = 0;
	while ( nRemaining = nLen - nIdxNow, 0 != nRemaining )
	{
		nDone = pcom->_transmit ( pcom, &((const uint8_t*)pv)[nIdxNow], nRemaining );
		nIdxNow += nDone;
		if ( nDone != nRemaining )
		{
			if ( ( HAL_GetTick() - tsStart ) > to )
			{
				return nLen - nIdxNow;	//(must recompute since we're at this point)
			}
			osDelay(1);
		}
	}
	
/*
	//Reeeeally completely
	while ( ! UART6_txbuff_empty() )
	{
		osDelay ( 1 );
	}
*/
	return 0;	//tada!
}



//A simple blocking receive function.  This just does a sleep when the receive
//buffer cannot produce what is requested.  Not sophisticated, but still better
//than polling.
static int Serial_receiveCompletely ( const IOStreamIF* pcom, void* pv, const size_t nLen, uint32_t to )
{
	uint32_t tsStart;
	size_t nIdxNow;
	size_t nRemaining;
	size_t nDone;
	
	tsStart = HAL_GetTick();
	
	nIdxNow = 0;
	while ( nRemaining = nLen - nIdxNow, 0 != nRemaining )
	{
		nDone = pcom->_receive ( pcom, &((uint8_t*)pv)[nIdxNow], nRemaining );
		nIdxNow += nDone;
		if ( nDone != nRemaining )
		{
			if ( ( HAL_GetTick() - tsStart ) > to )
			{
				return nLen - nIdxNow;	//(must recompute since we're at this point)
			}
			osDelay(1);
		}
	}
	return 0;	//tada!
}



//========================================================================



void UART6_Init ( void )
{
	//XXX UART6_txbuff_init();
	//XXX UART6_rxbuff_init();
	circbuff_init(&UART6_txbuff);
	circbuff_init(&UART6_rxbuff);
	//set up the receive action on UART 2
	__kickstartReceiveUART6();
}


void USBCDC_Init ( void )
{
	//XXX CDC_txbuff_init();
	//XXX CDC_rxbuff_init();
	circbuff_init(&CDC_txbuff);
	circbuff_init(&CDC_rxbuff);
}


