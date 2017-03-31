/********************************************************************************
* @file    CUartConsole.cpp
* @author  Chenxx
* @version V1.0
* @date    2016-04-21
* @brief   this file defines the Console function that can printf with uart.
*	   This is for STM32F4.
********************************************************************************/
#include "CUartConsole.h"

uint8_t CUartConsole::TxDmaBuf_[TX_DMA_SIZE];	//for txDma
uint8_t CUartConsole::TxBuf_[TXBUF_SIZE];		//buffer for None DMA Mode txQueue_
/* emulate dsp sci FIFO buffer */
uint8_t fake_rxbuf;//this is useless
CUsart consoleUsart(CONSOLE_UART, &fake_rxbuf, 1);	

/**
  * @brief  Constructor
	* @param  None
  * @retval None
  */
CUartConsole::CUartConsole()
	:txQueue_(TxBuf_, TXBUF_SIZE),
	overflowCounter_(0)
{
	
}

/**
  * @brief  run UART transmitter, in another word TXD
  * @param  None
  * @retval None
  */
void CUartConsole::runTransmitter()
{
	if(0 != DMA_GetCurrDataCounter(CUsart::TxDMA(CONSOLE_UART)))
		return;

	uint8_t sendLen = txQueue_.pop_array(TxDmaBuf_, TX_DMA_SIZE);
	consoleUsart.send_Array((uint8_t*)TxDmaBuf_, sendLen);
}

/**
  * @brief  write data to UartConsole device
  * @param  srcBuf
	* @param  len
  * @retval bytes that actually wrote into it
  */
uint16_t CUartConsole::write(uint8_t* srcBuf, uint16_t srcLen)
{
	return txQueue_.push_array(srcBuf, srcLen);
}

/**
  * @brief  Initialize the Usart of console
  * @param  None
  * @retval None
  */
bool CUartConsole::open()
{
	consoleUsart.InitSciGpio();
	consoleUsart.InitSci();
	return true;
}

/**
* @brief  Is Transmitter idel or not
* @param  None
* @retval If is idel
*/
bool CUartConsole::isIdel()
{
	runTransmitter();
	return (0 == txQueue_.elemsInQue() 
		&& 0 == DMA_GetCurrDataCounter(CUsart::TxDMA(CONSOLE_UART))
		&& 1 == USART_GetFlagStatus(CONSOLE_UART, USART_FLAG_TXE));
}

//end of file
