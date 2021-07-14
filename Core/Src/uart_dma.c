/*
 * uart_dma.c
 *
 *  Created on: Dec 10, 2020
 *      Author: acapone13
 */

#include "uart_dma.h"


void UARTDMA_UART_IRQHandler(UARTDMA_HandleTypeDef *huartdma)
{
	if (huartdma->huart->Instance->SR & UART_FLAG_IDLE)	//	Check if Idle flag is set
	{
		volatile uint32_t tmp;
		tmp = huartdma->huart->Instance->SR;	//	Read Status Register
		tmp = huartdma->huart->Instance->DR;	//	Read Data Register

		//	If DMA_CCR is enabled it will force Transfer Complete interrupt
		huartdma->huart->hdmarx->Instance->CCR &= ~DMA_CCR_EN;	//	Disable DMA CCR

		tmp = tmp;	//	For unused warning
		UARTDMA_DMA_IRQHandler(huartdma);	//	Since DMA IRQ won't start independently for Channels, we have to handle it manually
	}
}

void UARTDMA_DMA_IRQHandler(UARTDMA_HandleTypeDef *huartdma)
{
	uint8_t *UartBufferPointer, *DmaBufferPointer;
	uint32_t length;
	uint16_t i, tmpHead;

	typedef struct
	{
		__IO uint32_t ISR;	//	DMA Interrupt Status Register
		__IO uint32_t IFCR;	//	DMA Interrupt Flag clear Register
	}DMA_Base_Registers;

	DMA_Base_Registers *DmaRegisters = (DMA_Base_Registers * ) huartdma->huart->hdmarx->DmaBaseAddress;	//	Take registers Base Address

	if (__HAL_DMA_GET_IT_SOURCE(huartdma->huart->hdmarx, DMA_IT_TC)  != RESET)	//	Check if interrupt source is Transfer Complete
	{
		DmaRegisters->IFCR = DMA_IFCR_CTCIF1 << huartdma->huart->hdmarx->ChannelIndex;	//	Clear Transfer Complete Flag

		length = DMA_RX_BUFFER_SIZE - huartdma->huart->hdmarx->Instance->CNDTR;	//	Get the length of transferred data

		UartBufferPointer = huartdma->UART_Buffer;
		DmaBufferPointer = huartdma->DMA_RX_Buffer;

		//	Write received data for UART main buffer
		for (i = 0; i < length; i++)
		{
			tmpHead =(huartdma->uartBufferHead + 1) % UART_BUFFER_SIZE;
			if (tmpHead == huartdma->uartBufferTail)
			{
				huartdma->uartBufferHead = huartdma->uartBufferTail;	//	No space for new data
			}
			else
			{
				UartBufferPointer[tmpHead] = DmaBufferPointer[i];
				if (UartBufferPointer[tmpHead] == '\n')
				{
					huartdma->uartBufferLines++;
				}
				huartdma->uartBufferHead = tmpHead;
			}
		}
		DmaRegisters->IFCR = 0x0FU << huartdma->huart->hdmarx->ChannelIndex;	//	Clear all Interrupts
		huartdma->huart->hdmarx->Instance->CMAR = (uint32_t) huartdma->DMA_RX_Buffer;	//	Set Memory address for DMA again
		huartdma->huart->hdmarx->Instance->CNDTR = DMA_RX_BUFFER_SIZE;	//	Set number of bytes to receive
		huartdma->huart->hdmarx->Instance->CCR |= DMA_CCR_EN;	//	Start DMA Transfer
	}
}

int UARTDMA_GetCharFromBuffer(UARTDMA_HandleTypeDef *huartdma)
{
	if (huartdma->uartBufferHead == huartdma->uartBufferTail)
	{
		return -1;	//	Error - no char to return
	}
	huartdma->uartBufferTail = (huartdma->uartBufferTail + 1) % UART_BUFFER_SIZE;

	return huartdma->UART_Buffer[huartdma->uartBufferTail];
}

uint8_t UARTDMA_IsDataReady(UARTDMA_HandleTypeDef *huartdma)
{
	if (huartdma->uartBufferLines)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

int UARTDMA_GetLineFromBuffer(UARTDMA_HandleTypeDef *huartdma, char *outBuffer)
{
	char tmpChar;
	char *linePointer = outBuffer;
	if (huartdma->uartBufferLines)
	{
		while ((tmpChar = UARTDMA_GetCharFromBuffer(huartdma)))
		{
			if (tmpChar == '\n')
			{
				break;
			}
			*linePointer = tmpChar;
			linePointer++;
		}
		*linePointer = 0;	//	end of cstring
		huartdma->uartBufferLines--;	//	Decrement line counter
	}

	return 0;
}

void UARTDMA_Init(UARTDMA_HandleTypeDef *huartdma, UART_HandleTypeDef *huart)
{
	huartdma->huart = huart;

	__HAL_UART_ENABLE_IT(huartdma->huart, UART_IT_IDLE);		//	UART Idle Line Interrupt
	__HAL_DMA_ENABLE_IT(huartdma->huart->hdmarx, DMA_IT_TC);	//	UART DMA Transfer Complete Interrupt

	HAL_UART_Receive_DMA(huartdma->huart, huartdma->DMA_RX_Buffer, DMA_RX_BUFFER_SIZE);	//	Run DMA for whole DMA Buffer

	huartdma->huart->hdmarx->Instance->CCR &= ~DMA_CCR_HTIE;	//	Disable DMA Half Complete Interrupt
}
