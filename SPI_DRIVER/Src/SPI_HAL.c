#include <stdint.h>
#include "SPI_HAL.h"


// ENABLE THE SPI DEVICE
static void hal_spi_enable(SPI_TypeDef *SPIx)
{
    if( !(SPIx->CR1 & SPI_REG_CR1_SPE))
		SPIx->CR1 |= SPI_REG_CR1_SPE;
}

// DISABLE THE SPI DEVICE
static void hal_spi_disable(SPI_TypeDef *SPIx)
{
	SPIx->CR1 &= ~SPI_REG_CR1_SPE;
}



/**
 * @brief  Configures SPI clk phase and polarity
 * @param  *SPIx    : Base address of the SPI
 * @param  phase    : configures phase
 * @param  polarity : configures polarity
 * @retval None
 */
static void hal_spi_configure_phase_and_polarity(SPI_TypeDef *SPIx, uint32_t phase_value, uint32_t polarity)
{
    if(phase_value)
    {
        SPIx->CR1 |= SPI_REG_CR1_CPHA;
    }
    else
    {
        SPIx->CR1 &= ~SPI_REG_CR1_CPHA;
    }

    if(polarity)
    {
        SPIx->CR1 |= SPI_REG_CR1_CPOL;
    }
    else
    {
        SPIx->CR1 &= ~SPI_REG_CR1_CPOL;
    }
}

/**
 * @brief  Configures master or slave mode
 * @param  *SPIx  : Base address of the SPI
 * @param  master : if 1, then configured for master
 * @retval None
 */
static void hal_spi_configure_device_mode(SPI_TypeDef *SPIx, uint32_t master)
{
    if(master)
    {
        SPIx->CR1 |= SPI_REG_CR1_MSTR;
    }
    else
    {
        SPIx->CR1 &= ~SPI_REG_CR1_MSTR;
    }
}

/**
 * @brief  Configures SPI datasize and direction
 * @param  *SPIx       : Base address of the SPI
 * @param  datasize_16 : data size to be configured
 * @param  lsbmbsfirst : if 1, lsb will be sent first.
 * @retval None
 */
static void hal_spi_configure_datasize_direction(SPI_TypeDef *SPIx, uint32_t datasize_16, uint32_t lsbfirst)
{
    if(datasize_16)
    {
        SPIx->CR1 |= SPI_REG_CR1_DFF;
    }
    else
    {
        SPIx->CR1 &= ~SPI_REG_CR1_DFF;
    }

    if(lsbfirst)
    {
        SPIx->CR1 |= SPI_CR1_LSBFRIST;
    }
    else
    {
        SPIx->CR1 &= ~SPI_CR1_LSBFRIST;
    }
}

/**
 * @brief  Configures the NSS pin of the master
 * @param  *SPIx       : Base address of the SPI
 * @param  ssm_enable : if 1, software slave management is enabled
 * @retval None
 */
static void hal_spi_configure_nss_master(SPI_TypeDef *SPIx, uint32_t ssm_enable)
{
    if(ssm_enable)
    {
        SPIx->CR1 |= SPI_REG_CR1_SSM;
        SPIx->CR1 |= SPI_REG_CR1_SSI;
    }
    else
    {
        SPIx->CR1 &= ~SPI_REG_CR1_SSM;
    }
}

/**
 * @brief  Configures the NSS pin of the master
 * @param  *SPIx       : Base address of the SPI
 * @param  ssm_enable : if 1, software slave management is enabled
 * @retval None
 */
static void hal_spi_configure_nss_slave(SPI_TypeDef *SPIx, uint32_t ssm_enable)
{
    if(ssm_enable)
    {
        SPIx->CR1 |= SPI_REG_CR1_SSM;
        SPIx->CR1 |= SPI_REG_CR1_SSI;
    }
    else
    {
        SPIx->CR1 &= ~SPI_REG_CR1_SSM;
    }
}


void hal_spi_init(spi_handle_t *spi_handle)
{
    /* configure the phase and polarity */
    hal_spi_configure_phase_and_polarity(spi_handle->Instance, spi_handle->Init.CLKPhase, spi_handle->Init.CLKPolarity);

    /* Configure the spi device mode */
    hal_spi_configure_device_mode(spi_handle->Instance, spi_handle->Init.Mode);

    /* Configure the spi data size and device direction */
    hal_spi_configure_datasize_direction(spi_handle->Instance, spi_handle->Init.DataSize, spi_handle->Init.FirstBit);

    /* Configure the slave select line */
    if (spi_handle->Init.Mode == SPI_MASTER_MODE_SEL)
	{
	    hal_spi_configure_nss_master(spi_handle->Instance, spi_handle->Init.NSS);
	}
	else
	{
	    hal_spi_configure_nss_slave(spi_handle->Instance, spi_handle->Init.NSS);
	}

    /* Configure the SPI device speed */
    spi_handle->Instance->CR1 &= ~( (uint32_t)0x7 << 3 ); // We clear the BR bits first (bits 3, 4, 5) then set the new value
    spi_handle->Instance->CR1 |= spi_handle->Init.BaudRatePrescaler;

}


/**
 * @brief  Enables the Tx buffer empty interrupt (TXE)
 * @param  *SPIx : Base address of the SPI
 * @retval None
 */
static void hal_spi_enable_txe_interrupt(SPI_TypeDef *SPIx)
{
    SPIx->CR2 |= SPI_REG_CR2_TXEIE_ENABLE;
}

/**
 * @brief  Disables the Tx buffer empty interrupt (TXE)
 * @param  *SPIx : Base address of the SPI
 * @retval None
 */
static void hal_spi_disable_txe_interrupt(SPI_TypeDef *SPIx)
{
    SPIx->CR2 &= ~SPI_REG_CR2_TXEIE_ENABLE;
}

/**
 * @brief  Enables the Rx buffer empty interrupt (RXNE)
 * @param  *SPIx : Base address of the SPI
 * @retval None
 */
static void hal_spi_enable_rxne_interrupt(SPI_TypeDef *SPIx)
{
    SPIx->CR2 |= SPI_REG_CR2_RXNEIE_ENABLE;
}

/**
 * @brief  Disables the Rx buffer empty interrupt (RXNE)
 * @param  *SPIx : Base address of the SPI
 * @retval None
 */
static void hal_spi_disable_rxne_interrupt(SPI_TypeDef *SPIx)
{
    SPIx->CR2 &= ~SPI_REG_CR2_RXNEIE_ENABLE;
}


void hal_spi_master_tx(spi_handle_t *SPI_Handle, uint8_t *buffer, uint32_t len)
{
    /* Populate the TxBuffer pointer address along with the size in the SPI_Handle */
    SPI_Handle->pTxBuffPtr      = buffer;
    SPI_Handle->TxTransferCount = len;
    SPI_Handle->TxTranferSize   = len;

    /* Driver is Busy in TXing data */
    SPI_Handle->State = HAL_SPI_STATE_BUSY_TX;

    /* Enable the peripherals if not already enabled */
    hal_spi_enable(SPI_Handle->Instance);

    /* Enable TXE interrupt so the master can transmit data */
    hal_spi_enable_txe_interrupt(SPI_Handle->Instance);
}



void hal_spi_slave_rx(spi_handle_t *SPI_Handle, uint8_t *RxBuffer, uint32_t len)
{
    /* Populate the RxBuffer pointer address along with the size in the SPI_Handle */
    SPI_Handle->pRxBuffPtr = RxBuffer;
    SPI_Handle->RxTransferCount = len;
    SPI_Handle->RxTranferSize = len;

    /* Driver is Busy in receiving data */
    SPI_Handle->State = HAL_SPI_STATE_BUSY_RX;

    hal_spi_enable(SPI_Handle->Instance);

    /* The slave needs to receive data, so enable the RXNE Interrupt */
    hal_spi_enable_rxne_interrupt(SPI_Handle->Instance);
}



void hal_spi_master_rx(spi_handle_t *SPI_Handle, uint8_t *RxBuffer, uint32_t len)
{

	uint32_t val;
    /* Place dummy data in the Tx Buffer so the master can produce a clock */
	static uint8_t dummy[256] = {0xFF};
	SPI_Handle->pTxBuffPtr = &dummy[0];
	SPI_Handle->TxTranferSize = len;
    SPI_Handle->TxTransferCount = len;

    /* Put data in the RxBuffer */
    SPI_Handle->pRxBuffPtr = RxBuffer;
    SPI_Handle->RxTranferSize = len;
    SPI_Handle->RxTransferCount = len;

    /* Driver is Busy in receiving data */
    SPI_Handle->State = HAL_SPI_STATE_BUSY_RX;

    hal_spi_enable(SPI_Handle->Instance);

	val = SPI_Handle->Instance->DR;
	(void)val;

    /* Enable both TXE and RXNE interrupts */
    hal_spi_enable_rxne_interrupt(SPI_Handle->Instance);
    hal_spi_enable_txe_interrupt(SPI_Handle->Instance);
}



void hal_spi_slave_tx(spi_handle_t *SPI_Handle, uint8_t *TxBuffer, uint32_t len)
{
    /* Place the data in the Tx Buffer for the slave */
    SPI_Handle->pTxBuffPtr = TxBuffer;
    SPI_Handle->TxTranferSize = len;
    SPI_Handle->TxTransferCount = len;

    /* Setup the dummy Rx (slave receives while transmitting) */
    SPI_Handle->pRxBuffPtr = TxBuffer;
    SPI_Handle->RxTranferSize = len;
    SPI_Handle->RxTransferCount = len;

    /* Driver is busy doing TX */
    SPI_Handle->State = HAL_SPI_STATE_BUSY_TX;

    hal_spi_enable(SPI_Handle->Instance);

    /* Enable both TXE and RXNE interrupts */
    hal_spi_enable_rxne_interrupt(SPI_Handle->Instance);
    hal_spi_enable_txe_interrupt(SPI_Handle->Instance);
}


void hal_spi_irq_handler(spi_handle_t *hspi)
{
    /* Get status and control registers */
    uint32_t sr  = hspi->Instance->SR;
    uint32_t cr2 = hspi->Instance->CR2;

    /* Check if RXNE is set and RXNEIE is enabled */
    if( (sr & SPI_REG_SR_RXNE_FLAG) && (cr2 & SPI_REG_CR2_RXNEIE_ENABLE) )
    {
        hal_spi_handle_rx_interrupt(hspi);
        return;
    }

    /* Check if TXE is set and TXEIE is enabled */
    if( (sr & SPI_REG_SR_TXE_FLAG) && (cr2 & SPI_REG_CR2_TXEIE_ENABLE) )
    {
        hal_spi_handle_tx_interrupt(hspi);
        return;
    }
}


/**
 * @brief  Close the SPI Tx interrupt
 * @param  *SPIx : Base address of the SPI
 * @retval None
 */
static void hal_spi_tx_close_interrupt(spi_handle_t *hspi)
{
	/* Disable TXE interrupt*/
	hal_spi_disable_txe_interrupt(hspi->Instance);

	if(hspi->Init.Mode && (hspi->State != HAL_SPI_STATE_BUSY_RX))
		hspi->State = HAL_SPI_STATE_READY;
}

/**
 * @brief  handles SPI TXE interrupt.
 * @param  hspi: pointer to a spi_handle_t structure that contains the configuration information for SPI module.
 * @retval none
 */
void hal_spi_handle_tx_interrupt(spi_handle_t *hspi)
{
    /* Transmit data in 8-bit mode */
    if(hspi->Init.DataSize == SPI_8BIT_DF_ENABLE)
    {
        hspi->Instance->DR = (*hspi->pTxBuffPtr++);
        hspi->TxTransferCount--; // sent 1 byte
    }

	/* transmit data in 16-bit mode*/
    else
    {
        hspi->Instance->DR = *((uint16_t*)hspi->pTxBuffPtr);
        hspi->pTxBuffPtr += 2;
        hspi->TxTransferCount -= 2; // sent 2 bytes in one go
    }

    if(hspi->TxTransferCount == 0)
    {
        /* Transmission complete, close the TXE interrupt */
        hal_spi_tx_close_interrupt(hspi);
    }
}


/**
 * @brief  Close the SPI Rx interrupt
 * @param  *SPIx : Base address of the SPI
 * @retval None
 */
static void hal_spi_rx_close_interrupt(spi_handle_t *hspi)
{
	/* Disable TXE interrupt*/
	hal_spi_disable_rxne_interrupt(hspi->Instance);

	hspi->State = HAL_SPI_STATE_READY;
}

/**
 * @brief handles SPI RX interrupt request.
 * @param  hspi: pointer to a spi_handle_t structure that contains the configuration information for SPI module.
 * @retval none
 */
void hal_spi_handle_rx_interrupt(spi_handle_t *hspi)
{
    /* Receive data in 8-bit mode*/
    if(hspi->Init.DataSize == SPI_8BIT_DF_ENABLE)
    {
        (*hspi->pRxBuffPtr++) = hspi->Instance->DR;
        hspi->RxTransferCount--;
    }
    /* Receive data in 16-bit mode*/
    else
    {
        *((uint16_t*)hspi->pRxBuffPtr) = hspi->Instance->DR;
        hspi->pRxBuffPtr += 2;
        hspi->RxTransferCount -= 2;
    }

    if(hspi->RxTransferCount == 0)
    {
        hal_spi_rx_close_interrupt(hspi);
    }
}
