#ifndef SPI_HAL_H
#define SPI_HAL_H

#include <Include/stm32f429xx.h>
#include <stdint.h>

/* ================= SPI State ================= */
typedef enum
{
    HAL_SPI_STATE_RESET      = 0x00,
    HAL_SPI_STATE_READY      = 0x01,
    HAL_SPI_STATE_BUSY_TX    = 0x12,
    HAL_SPI_STATE_BUSY_RX    = 0x22,
    HAL_SPI_STATE_BUSY_TX_RX = 0x32,
    HAL_SPI_STATE_ERROR      = 0x03,
} hal_spi_state_t;

/* ================= SPI Init ================= */
typedef struct
{
    uint32_t Mode;                 // Specifies the SPI operating mode: SPI_MASTER_MODE / SPI_SLAVE_MODE
	uint32_t Direction;            // Specifies the SPI Directional mode state
    uint32_t DataSize;             // Specifies the SPI data size: SPI_8Bit_DF_ENABLE / SPI_16Bit_DF_ENABLE
    uint32_t CLKPolarity;          // Specifies the serial clock steady state: SPI_CPOL_LOW / SPI_CPOL_HIGH
    uint32_t CLKPhase;             // Specifies the clock active edge for the bit capture: SPI_FIRST_CLOCK_TRANS / SPI_SECOND_CLOCK_TRANS
    uint32_t NSS;                  // SPI_SSM_ENABLE / SPI_SSM_DISABLE
    uint32_t BaudRatePrescaler;    // Specifies the Baud Rate prescaler value:  SPI_CR1_BR_x
    uint32_t FirstBit;             // SPI_TX_MSB_FIRST / SPI_TX_LSB_FIRST
} spi_init_t;

/* ================= SPI Handle ================= */
typedef struct
{
    SPI_TypeDef         *Instance;       /* SPI registers base address */
    spi_init_t          Init;            /* SPI communication parameters */
    uint8_t             *pTxBuffPtr;     /* Pointer to SPI Tx transfer Buffer */
    uint16_t            TxTranferSize;   /* SPI Tx transfer size */
    uint16_t            TxTransferCount; /* SPI Tx Transfer Counter */
    uint8_t             *pRxBuffPtr;     /* Pointer to SPI Rx transfer Buffer */
    uint16_t            RxTranferSize;   /* SPI Rx transfer size */
    uint16_t            RxTransferCount; /* SPI Rx Transfer Counter */
    hal_spi_state_t     State;           /* SPI communication state */
} spi_handle_t;

/* ******************** Bit definition for SPI_CR1 register ******************** */

#define SPI_REG_CR1_BIDIMODE               ( (uint32_t) 1 << 15 )
#define SPI_ENABLE_2_LINE_UNI_DIR          0
#define SPI_ENABLE_1_LINE_BIDI             1

#define SPI_REG_CR1_DFF                    ( (uint32_t) 1 << 11 )
#define SPI_8BIT_DF_ENABLE                 0
#define SPI_16_BIT_DF_ENABLE               1

#define SPI_REG_CR1_SSM                    ( (uint32_t) 1 << 9 )
#define SPI_SSM_ENABLE                     1
#define SPI_SSM_DISABLE                    0

#define SPI_REG_CR1_SSI                    ( (uint32_t) 1 << 8 )

#define SPI_CR1_LSBFRIST                   ( (uint32_t) 1 << 7 )
#define SPI_TX_MSB_FIRST                   0
#define SPI_TX_LSB_FIRST                   1

#define SPI_REG_CR1_SPE                    ( (uint32_t) 1 << 6 )

#define SPI_REG_CR1_BR_PCLK_DIV_2          ( (uint32_t) 0 << 3 )
#define SPI_REG_CR1_BR_PCLK_DIV_4          ( (uint32_t) 1 << 3 )
#define SPI_REG_CR1_BR_PCLK_DIV_8          ( (uint32_t) 2 << 3 )
#define SPI_REG_CR1_BR_PCLK_DIV_16         ( (uint32_t) 3 << 3 )
#define SPI_REG_CR1_BR_PCLK_DIV_32         ( (uint32_t) 4 << 3 )
#define SPI_REG_CR1_BR_PCLK_DIV_64         ( (uint32_t) 5 << 3 )
#define SPI_REG_CR1_BR_PCLK_DIV_128        ( (uint32_t) 6 << 3 )
#define SPI_REG_CR1_BR_PCLK_DIV_256        ( (uint32_t) 7 << 3 )

#define SPI_REG_CR1_MSTR                   ( (uint32_t) 1 << 2 )
#define SPI_MASTER_MODE_SEL                1
#define SPI_SLAVE_MODE_SEL                 0

#define SPI_REG_CR1_CPOL                   ( (uint32_t) 1 << 1 )
#define SPI_CPOL_LOW                       0
#define SPI_CPOL_HIGH                      1

#define SPI_REG_CR1_CPHA                   ( (uint32_t) 1 << 0 )
#define SPI_FIRST_CLOCK_TRANS              0
#define SPI_SECOND_CLOCK_TRANS             1

/* ******************** Bit definition for SPI_CR2 register ******************** */

#define SPI_REG_CR2_TXEIE_ENABLE           ( ( uint32_t) 1 << 7)
#define SPI_REG_CR2_RXNEIE_ENABLE          ( ( uint32_t) 1 << 6)
#define SPI_REG_CR2_ERRIE_ENABLE           ( ( uint32_t) 1 << 5)

#define SPI_REG_CR2_FRAME_FORMAT           ( ( uint32_t) 1 << 4)
#define SPI_MOTOROLA_MODE                  0
#define SPI_TI_MODE                        1

#define SPI_REG_CR2_SSOE                   ( ( uint32_t) 1 << 2)

/* ******************** Bit definition for SPI_SR register ******************** */
#define SPI_REG_SR_FRE_FLAG                ( ( uint32_t) 1 << 8)
#define SPI_REG_SR_BUSY_FLAG               ( ( uint32_t) 1 << 7)
#define SPI_REG_SR_TXE_FLAG                ( ( uint32_t) 1 << 1)
#define SPI_REG_SR_RXNE_FLAG               ( ( uint32_t) 1 << 0)

/* SPI device base address */
#define SPI_1                              SPI1
#define SPI_2                              SPI2
#define SPI_3                              SPI3

#define SPI_IS_BUSY                        1
#define SPI_IS_NOT_BUSY                    0

/* Macros to Enable Clock for diffrent SPI devices */
#define _HAL_RCC_SPI1_CLK_ENABLE()        ( RCC->APB2ENR |=  (1 << 12) )
#define _HAL_RCC_SPI2_CLK_ENABLE()        ( RCC->APB1ENR |=  (1 << 14) )
#define _HAL_RCC_SPI3_CLK_ENABLE()        ( RCC->APB1ENR |=  (1 << 15) )

#define RESET                              0
#define SET                                !RESET


/* ================= API ================= */

/**
 * @brief  API used to do initialize the given SPI device
 * @param  *spi_handle : pointer to the spi_handle_t structure
 * @retval none
 */
void hal_spi_init(spi_handle_t *spi_handle);


/**
 * @brief  API used to do master data transmission
 * @param  *spi_handle : pointer to the spi_handle_t structure
 * @param  *buffer     : pointer to the tx buffer
 * @param  len         : len of tx data
 * @retval none
 */
void hal_spi_master_tx(spi_handle_t *spi_handle, uint8_t *buffer, uint32_t len);

/**
 * @brief  API used to do slave data reception
 * @param  *spi_handle : pointer to the spi_handle_t structure
 * @param  *rcv_buffer : pointer to the rx buffer
 * @param  len         : len of rx data
 * @retval none
 */
void hal_spi_slave_rx(spi_handle_t *spi_handle, uint8_t *rcv_buffer, uint32_t len);

/**
 * @brief  API used to do master data reception
 * @param  *spi_handle : pointer to the spi_handle_t structure
 * @param  *buffer     : pointer to the rx buffer
 * @param  len         : len of rx data
 * @retval none
 */
void hal_spi_master_rx(spi_handle_t *spi_handle, uint8_t *buffer, uint32_t len);

/**
 * @brief  API used to do slave data transmission
 * @param  *spi_handle : pointer to the spi_handle_t structure
 * @param  *rcv_buffer : pointer to the tx buffer
 * @param  len         : len of tx data
 * @retval none
 */
void hal_spi_slave_tx(spi_handle_t *spi_handle, uint8_t *rcv_buffer, uint32_t len);


/**
 * @brief  handles SPI TXE interrupt.
 * @param  hspi: pointer to a spi_handle_t structure that contains the configuration information for SPI module.
 * @retval none
 */
void hal_spi_handle_tx_interrupt(spi_handle_t *hspi);

/**
 * @brief handles SPI RX interrupt request.
 * @param  hspi: pointer to a spi_handle_t structure that contains the configuration information for SPI module.
 * @retval none
 */
void hal_spi_handle_rx_interrupt(spi_handle_t *hspi);

/**
 * @brief  This function handles SPI interrupt request.
 * @param  hspi: pointer to a spi_handle_t structure that contains the configuration information for SPI module.
 * @retval none
 */
void hal_spi_irq_handler(spi_handle_t *hspi);

#endif /* INC_SPI_HAL_H_ */
