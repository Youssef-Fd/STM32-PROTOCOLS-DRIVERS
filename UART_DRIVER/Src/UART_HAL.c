/*
 * UART_HAL.c
 *
 *  Created on: Jan 19, 2026
 *      Author: Lenovo
 */
#include <stdio.h>
#include <stdint.h>
#include "UART_HAL.h"

void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms * 4000; i++) {
        __asm__("nop");
    }
}

/* ------------Enable USART peripheral------------- */
static void hal_uart_enable(USART_TypeDef *uartx){
	uartx -> CR1 |= USART_REG_CR1_USART_EN;
}

/* ------------Disable USART peripheral------------- */
static void hal_uart_disable(USART_TypeDef *uartx){
	uartx -> CR1 &= ~USART_REG_CR1_USART_EN;
}

/* ------------Enable or disable transmitter (TE bit)------------- */
static void hal_uart_enable_disable_tx(USART_TypeDef *uartx, uint32_t te){

	if(te & USART_REG_CR1_TE){
		uartx -> CR1 |= USART_REG_CR1_TE; /* Transmitter Enabled */
	}
	else{
		uartx -> CR1 &= ~USART_REG_CR1_TE; /* Transmitter Disabled */
	}
}

/* ------------Enable or disable receiver (RE bit)------------- */
static void hal_uart_enable_disable_rx(USART_TypeDef *uartx, uint32_t re){

	if(re & USART_REG_CR1_RE){
		uartx -> CR1 |= USART_REG_CR1_RE; /* Receiver Enabled */
	}
	else{
		uartx -> CR1 &= ~USART_REG_CR1_RE; /* Receiver Disabled */
	}
}

/* ---------------Configure UART word length (8 or 9 bits)-------------------- */
static void hal_uart_configure_word_length(USART_TypeDef *uartx, uint32_t wordLength){

	if(wordLength){
		uartx -> CR1 |= USART_REG_CR1_UART_WL; /* 1 Start bit, 8 Data bits, n Stop bit */
	}
	else{
		uartx -> CR1 &= ~USART_REG_CR1_UART_WL; /* 1 Start bit, 9 Data bits, n Stop bit */
	}
}

/* ------------- Configure UART number of stop bits (8 or 9 bits)------------------- */
static void hal_uart_configure_stop_bits(USART_TypeDef *uartx, uint32_t nstop){

	uartx -> CR2 &= ~( 0x3U << USART_REG_CR2_STOP_Bit ); /* Clear STOP bits */

	if(nstop == USART_CR2_0_5StopBit ){
		uartx -> CR2 |= ( 0x01 << USART_REG_CR2_STOP_Bit ); /* 0.5 Stop bit */
	}
	else if (nstop == USART_CR2_2StopBit){
		uartx -> CR2 |= ( 0x02 << USART_REG_CR2_STOP_Bit );  /* 2 Stop bit */
	}
	else if (nstop == USART_CR2_1_5StopBit){
		uartx -> CR2 |= ( 0x03 << USART_REG_CR2_STOP_Bit );  /* 1.5 Stop bit */
	}
	else{
		uartx -> CR2 |= ( 0x00 << USART_REG_CR2_STOP_Bit );  /* 1 Stop bit */
	}
}

/* ------------- Configure UART oversampling mode (8 or 16)  ------------------- */
static void hal_uart_configure_over_sampling(USART_TypeDef *uartx, uint32_t over8){

	if(over8){
		uartx -> CR1 |= USART_REG_CR1_OVER8; /* oversampling by 8 */
	}
	else{
		uartx -> CR1 &= ~USART_REG_CR1_OVER8; /* oversampling by 16 */
	}

}

/* ------------------ Configure UART Baud rate --------------------- */
static void hal_uart_set_baud_rate(USART_TypeDef *uartx, uint32_t baudRate){

	uint32_t baud;
	if( baudRate == USART_BAUD_9600){
		baud = 0x0683;     /* 9600 Baud Rate */
	}
	else if( baudRate == USART_BAUD_19200){
		baud = 0x341;      /* 19200 Baud Rate */
	}
	else if( baudRate == USART_BAUD_115200){
		baud = 0x8A;      /* 115200 Baud Rate */
	}
	else{
		baud = 0x11;      /* 921600 Baud Rate */
	}
	uartx -> BRR = baud;  /* Set the baud rate in the baud rate regiter */

}

/* --------------------- Enable or disable TXE interrupt ------------------------- */
static void hal_uart_configure_txe_interrupt(USART_TypeDef *uartx, uint32_t txe_en){

	if(txe_en){
		/* TXE interrupt enabled */
		uartx -> CR1 |= USART_REG_CR1_TXE_INT_EN;
	}
	else{
		/* TXE interrupt disabled */
		uartx -> CR1 &= ~USART_REG_CR1_TXE_INT_EN;
	}
}


/* --------------------- Enable or disable RXNE interrupt ------------------------- */
static void hal_uart_configure_rxne_interrupt(USART_TypeDef *uartx, uint32_t rxne_en){

	if(rxne_en){
		/* RXNE interrupt enabled */
		uartx -> CR1 |= USART_REG_CR1_RXNE_INT_EN;
	}
	else{
		/* RXNE interrupt disabled */
		uartx -> CR1 &= ~USART_REG_CR1_RXNE_INT_EN;
	}
}

/* --------------------- Enable or disable UART error interrupts ------------------------- */
static void hal_uart_configure_error_interrupt(USART_TypeDef *uartx, uint32_t er_en){

	if(er_en){
		/* Error interrupt Enabled */
		uartx -> CR3 |= USART_REG_CR3_ERROR_IN_EN;
	}
	else{
		/* Error interrupt Disabled */
		uartx -> CR3 &= ~USART_REG_CR3_ERROR_IN_EN;
	}
}

/* --------------------- Enable or disable parity error interrupt ------------------------- */
static void hal_uart_configure_parrity_error_interrupt(USART_TypeDef *uartx, uint32_t pe_en){

	if(pe_en){
		/* Parrity Error interrupt Enabled */
		uartx -> CR1 |= USART_REG_CR1_PEIE_INT_EN;
	}
	else{
		/* Parrity Error interrupt Disabled */
		uartx -> CR1 &= ~USART_REG_CR1_PEIE_INT_EN;
	}
}


/* -------------- Clear UART error flags --------------------- */
static void hal_uart_clear_error_flag(uart_handle_t *huart)
{
    volatile uint32_t tmpreg;
    tmpreg = huart->Instance->SR;  /* Read Status Register */
    tmpreg = huart->Instance->DR;  /* Read Data Register to clear flags */
}

/* ----- Error callback: indicates fatal UART error (LED blinking) ----------- */
static void hal_uart_error_cb(uart_handle_t *uart_handle)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN; //GPIOG Clock
	GPIOG->MODER &= ~(3 << 13*2);    //Clear pins PG13
	GPIOG->MODER |= (1 << 13*2);     //Set pins PG13

	while(1)
	{
		GPIOG->ODR ^= (1 << 13); // Toggle LED in pin PG13
		for(volatile int i = 0; i < 500000; i++);
	}
}


/* ----------- Handle TXE interrupt: send next byte --------------- */
static void hal_uart_handle_TXE_interrupt(uart_handle_t *huart)
{
    uint32_t tmp1 = 0;
    uint8_t val;

    tmp1 = huart->tx_state;

    if(tmp1 == HAL_UART_STATE_BUSY_TX)
    {
        /* Load data into the DR register */
        val = (uint8_t)(*huart->pTxBuffPtr++ & (uint8_t)0x00FF);
        huart->Instance->DR = val;

        /* Check if all data is transmitted */
        if(--huart->TxXferCount == 0)
        {
            /* Disable the UART TXE Interrupt */
            huart->Instance->CR1 &= ~USART_REG_CR1_TXE_INT_EN;

            /* Enable the UART Transmit Complete Interrupt */
            huart->Instance->CR1 |= USART_REG_CR1_TCIE_INT_EN;
        }
    }
}


/* --------- Handle Transmission Complete interrupt ------------- */
static void hal_uart_handle_TC_interrupt(uart_handle_t *huart)
{
    /* Disable the UART Transmit Complete Interrupt */
    huart->Instance->CR1 &= ~USART_REG_CR1_TCIE_INT_EN;

    /* Reset the TX state to Ready */
    huart->tx_state = HAL_UART_STATE_READY;

    /* Call the application callback */
    if(huart->tx_cmp_cb)
    {
        huart->tx_cmp_cb(&huart->TxXferSize);
    }
}


/*---------------brief Handle the RXNE interrupt ------------------*/
static void hal_uart_handle_RXNE_interrupt(uart_handle_t *huart)
{
    uint32_t tmpl = 0;

    tmpl = huart->rx_state;

    if (tmpl == HAL_UART_STATE_BUSY_RX)
    {
        /* Is application using parity? */
        if (huart->Init.Parity == UART_PARITY_NONE)
        {
            /* No parity: read all 8 bits from the data register */
            *huart->pRxBuffPtr++ = (uint8_t)(huart->Instance->DR & (uint8_t)0x00FF);
        }
        else
        {
            /* Yes: don't read the most significant bit, because it's a parity bit */
            /* Mask with 0x007F to get only the lower 7 data bits */
            *huart->pRxBuffPtr++ = (uint8_t)(huart->Instance->DR & (uint8_t)0x007F);
        }

        /* Check if we are done with the reception */
        if (--huart->RxXferCount == 0)
        {
            /* Disable RXNE and Error interrupts */
            huart->Instance->CR1 &= ~USART_REG_CR1_RXNE_INT_EN;
            huart->Instance->CR1 &= ~USART_REG_CR1_PEIE_INT_EN;
            huart->Instance->CR3 &= ~USART_REG_CR3_ERROR_IN_EN;

            /* Set state back to Ready */
            huart->rx_state = HAL_UART_STATE_READY;

            /* Call application callback */
            if (huart->rx_cmp_cb)
            {
                huart->rx_cmp_cb(&huart->RxXferSize);
            }
        }
    }
}


/**********************************************************************************/
/*----------------------------- USART FUNCTIONS ----------------------------------*/
/**********************************************************************************/


/**
 * @brief  Initialize UART peripheral.
 *
 * This function configures the UART peripheral parameters such as:
 * - Word length
 * - Stop bits
 * - Oversampling mode
 * - Baud rate
 * - Transmitter and receiver enable
 *
 * It also initializes the UART internal state machine and enables the UART.
 *
 * @param  uart_handle Pointer to a uart_handle_t structure that contains
 *         the configuration information for the UART module.
 */
void hal_uart_init(uart_handle_t *uart_handle){

	hal_uart_configure_word_length(uart_handle -> Instance, uart_handle -> Init.WordLength);
	hal_uart_configure_stop_bits(uart_handle -> Instance, uart_handle -> Init.StopBits);
	hal_uart_configure_over_sampling(uart_handle -> Instance, uart_handle -> Init.OverSampling);
	hal_uart_set_baud_rate(uart_handle -> Instance, uart_handle -> Init.BaudRate);
	hal_uart_enable_disable_tx(uart_handle -> Instance, uart_handle -> Init.Mode);
	hal_uart_enable_disable_rx(uart_handle -> Instance, uart_handle -> Init.Mode);

	uart_handle -> tx_state  = HAL_UART_STATE_READY;
	uart_handle -> rx_state  = HAL_UART_STATE_READY;
	uart_handle -> ErrorCode = HAL_UART_ERROR_NONE;

	hal_uart_enable(uart_handle->Instance);
}


/**
 * @brief  Start UART transmission in interrupt mode.
 *
 * This function starts a non-blocking UART transmission using interrupts.
 * Data is transmitted byte-by-byte using the TXE interrupt.
 *
 * @param  uart_handle Pointer to a uart_handle_t structure.
 * @param  buffer Pointer to the data buffer to be transmitted.
 * @param  len Number of bytes to transmit.
 */
void hal_uart_tx(uart_handle_t *uart_handle, uint8_t *buffer, uint32_t len){

	// Set the buffer containing the data to transmit and the number of bytes to transmit
	uart_handle -> pTxBuffPtr  = buffer;
	uart_handle -> TxXferCount = len;
	uart_handle -> TxXferSize  = len;

	// Mark UART as busy transmitting and Clear TC flag
	uart_handle -> tx_state = HAL_UART_STATE_BUSY_TX;
	uart_handle->Instance->SR &= ~USART_REG_SR_TC_FLAG;

	// Enable the UART peripheral
	hal_uart_enable(uart_handle -> Instance);

	// Enable the TX buffer empty interrupt to start transmission
	hal_uart_configure_txe_interrupt(uart_handle -> Instance, 1);

}


/**
 * @brief  Start UART reception in interrupt mode.
 *
 * This function starts a non-blocking UART reception using interrupts.
 * Received data is stored in the provided buffer until the requested
 * number of bytes is received.
 *
 * @param  uart_handle Pointer to a uart_handle_t structure.
 * @param  buffer Pointer to the buffer where received data will be stored.
 * @param  len Number of bytes to receive.
 */
void hal_uart_rx(uart_handle_t *uart_handle, uint8_t *buffer, uint32_t len){

	// Set the buffer where received data will be stored and the number of bytes to recieve
	uart_handle -> pRxBuffPtr  = buffer;
	uart_handle -> RxXferCount = len;
	uart_handle -> RxXferSize  = len;

	// Mark UART as busy receiving
	uart_handle -> rx_state = HAL_UART_STATE_BUSY_RX;

	// Enable interrupts for parity errors and general error
	hal_uart_configure_parrity_error_interrupt(uart_handle -> Instance, 1);
	hal_uart_configure_error_interrupt(uart_handle -> Instance, 1);

	// Enable the RX buffer not empty interrupt to start receiving data
	hal_uart_configure_rxne_interrupt(uart_handle -> Instance, 1);
}


/**
 * @brief  Handle UART interrupt requests.
 *
 * This function processes all UART-related interrupts including:
 * - Parity error (PE)
 * - Framing error (FE)
 * - Noise error (NE)
 * - Overrun error (ORE)
 * - Receive data register not empty (RXNE)
 * - Transmit data register empty (TXE)
 * - Transmission complete (TC)
 *
 * It updates the UART error code, manages transmit and receive state
 * machines, and calls the corresponding internal handlers or user
 * callbacks when required.
 *
 * @param  huart Pointer to a uart_handle_t structure that contains
 *         the configuration and state information for the UART instance.
 */
void hal_uart_handle_interrupt(uart_handle_t *huart)
{
    uint32_t tmp1 = 0;
    uint32_t tmp2 = 0;

    /*-------------------- Parity Error Interrupt --------------------*/
    tmp1 = huart->Instance->SR & USART_REG_SR_PE_FLAG;        /* Check PE flag */
    tmp2 = huart->Instance->CR1 & USART_REG_CR1_PEIE_INT_EN;  /* Check PE interrupt enable */
    if ((tmp1) && (tmp2))
    {
        hal_uart_clear_error_flag(huart);                     /* Clear error flags */
        huart->ErrorCode |= HAL_UART_ERROR_PE;                /* Update error code with PE */
    }

    /*-------------------- Framing Error Interrupt -------------------*/
    tmp1 = huart->Instance->SR & USART_REG_SR_FE_FLAG;        /* Check FE flag */
    tmp2 = huart->Instance->CR3 & USART_REG_CR3_ERROR_IN_EN; /* Check error interrupt enable */
    if ((tmp1) && (tmp2))
    {
        hal_uart_clear_error_flag(huart);
        huart->ErrorCode |= HAL_UART_ERROR_FE; /* Update error code with framing error */
    }

    /*-------------------- Noise Error Interrupt ---------------------*/
    tmp1 = huart->Instance->SR & USART_REG_SR_NE_FLAG;        /* Check NE flag */
    tmp2 = huart->Instance->CR3 & USART_REG_CR3_ERROR_IN_EN;
    if ((tmp1) && (tmp2))
    {
        hal_uart_clear_error_flag(huart);
        huart->ErrorCode |= HAL_UART_ERROR_NE;  /* Update error code with noise error */
    }

    /*-------------------- Overrun Error Interrupt -------------------*/
    tmp1 = huart->Instance->SR & USART_REG_SR_ORE_FLAG;       /* Check ORE flag */
    tmp2 = huart->Instance->CR3 & USART_REG_CR3_ERROR_IN_EN;
    if ((tmp1 != 0U) && (tmp2 != 0U))
    {
        hal_uart_clear_error_flag(huart);
        huart->ErrorCode |= HAL_UART_ERROR_ORE; /* Update error code with overrun error */
    }

    /*-------------------- RXNE Interrupt ----------------------------*/
    tmp1 = huart->Instance->SR & USART_REG_SR_RXNE_FLAG;     // Check if Transmit Data Register Empty (TXE) flag is set
    tmp2 = huart->Instance->CR1 & USART_REG_CR1_RXNE_INT_EN; // Check if RXNE interrupt is enabled
    if ((tmp1) && (tmp2))
    {
        hal_uart_handle_RXNE_interrupt(huart);  // Call RXNE interrupt handler to read received data
    }

    /*-------------------- TXE Interrupt -----------------------------*/
    tmp1 = huart->Instance->SR & USART_REG_SR_TXE_FLAG;      // Check if Transmit Data Register Empty (TXE) flag is set
    tmp2 = huart->Instance->CR1 & USART_REG_CR1_TXE_INT_EN;  // Check if TXE interrupt is enabled
    if ((tmp1) && (tmp2))
    {
        hal_uart_handle_TXE_interrupt(huart);     // Call TXE interrupt handler to write next byte
    }

    /*-------------------- TC Interrupt ------------------------------*/
    tmp1 = huart->Instance->SR & USART_REG_SR_TC_FLAG;        // Check if Transmission Complete (TC) flag is set
    tmp2 = huart->Instance->CR1 & USART_REG_CR1_TCIE_INT_EN;  // Check if TC interrupt is enabled
    if ((tmp1) && (tmp2))
    {
        hal_uart_handle_TC_interrupt(huart);    // Call TC interrupt handler to finish transmission
    }

    /*-------------------- Error Handling ----------------------------*/
    if (huart->ErrorCode != HAL_UART_ERROR_NONE)
    {
        /* Reset UART state */
        huart->tx_state = HAL_UART_STATE_READY;
        huart->rx_state = HAL_UART_STATE_READY;

        /* Call application error callback */
        hal_uart_error_cb(huart);
    }
}

