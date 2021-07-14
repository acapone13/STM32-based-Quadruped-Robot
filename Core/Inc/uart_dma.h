/*
 * uart_dma.h
 *
 *  Created on: Dec 10, 2020
 *      Author: acapone13
 */

#ifndef __UART_DMA_H_
#define __UART_DMA_H_

#include "main.h"

#define DMA_RX_BUFFER_SIZE	64
#define UART_BUFFER_SIZE	256

typedef struct
{
	UART_HandleTypeDef *huart;	// UART Handler

	uint8_t DMA_RX_Buffer[DMA_RX_BUFFER_SIZE];	// DMA direct buffer
	uint8_t UART_Buffer[UART_BUFFER_SIZE];	// UART Working circular buffer

	uint16_t uartBufferHead;
	uint16_t uartBufferTail;
	uint16_t uartBufferLines;
}UARTDMA_HandleTypeDef;


void UARTDMA_UART_IRQHandler(UARTDMA_HandleTypeDef *huartdma);
void UARTDMA_DMA_IRQHandler(UARTDMA_HandleTypeDef *huartdma);

uint8_t UARTDMA_IsDataReady(UARTDMA_HandleTypeDef *huartdma);
int UARTDMA_GetLineFromBuffer(UARTDMA_HandleTypeDef *huartdma, char *outBuffer);

void UARTDMA_Init(UARTDMA_HandleTypeDef *huartdma, UART_HandleTypeDef *huart);


#endif /* __UART_DMA_H_ */
