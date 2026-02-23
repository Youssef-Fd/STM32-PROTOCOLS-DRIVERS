#include "SPI_HAL.h"

/* Define SPI handle and init structure */
spi_handle_t hspi1, hspi2;

/* SPI1 IRQ Handler - required by startup file */
void SPI1_IRQHandler(void)
{
	hal_spi_irq_handler(&hspi1);
}

void SPI2_IRQHandler(void)
{
	hal_spi_irq_handler(&hspi2);
}

/* Configure GPIO pins for SPI1 */
static void SPI_GPIO_Init(void)
{
    /* Enable GPIOA and GPIOC Clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    /* Configure PA5(CLK), PA6(MISO), PA7(MOSI) as Alternate Function */
    GPIOA->MODER &= ~(3U << (5*2) | 3U << (6*2) | 3U << (7*2));
    GPIOA->MODER |=  (2U << (5*2) | 2U << (6*2) | 2U << (7*2));

    /* Configure PB10(CLK), PB14(MISO), PB15(MOSI) as Alternate Function */
    GPIOB->MODER &= ~(3U << (10*2) | 3U << (14*2) | 3U << (15*2));
    GPIOB->MODER |=  (2U << (10*2) | 2U << (14*2) | 2U << (15*2));

    /* Set Alternate Function to AF5 (SPI1) */
    GPIOA->AFR[0] &= ~(0xFU << (5*4) | 0xFU << (6*4) | 0xFU << (7*4));
    GPIOA->AFR[0] |=  (5U << (5*4) | 5U << (6*4) | 5U << (7*4));

    /* Set Alternate Function to AF5 (SPI2) */
    GPIOB->AFR[1] &= ~(0xFU << (2*4) | 0xFU << (6*4) | 0xFU << (7*4));
    GPIOB->AFR[1] |=  (5U << (2*4)   | 5U << (6*4)   | 5U << (7*4));

    /* Set GPIO speed to High Speed */
    GPIOA->OSPEEDR |= (3U << (5*2)  | 3U << (6*2)  | 3U << (7*2));
    GPIOB->OSPEEDR |= (3U << (10*2) | 3U << (14*2) | 3U << (15*2));

    /* No pull-up/pull-down */
    GPIOA->PUPDR &= ~(3U << (5*2)  | 3U << (6*2)  | 3U << (7*2));
    GPIOB->PUPDR &= ~(3U << (10*2) | 3U << (14*2) | 3U << (15*2));

}

/* Initialize SPI1 peripheral */
static void SPI1_Init(void)
{
    /* Enable SPI1 Clock on APB2 */
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    /* Configure SPI initialization structure */
    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MASTER_MODE_SEL;
    hspi1.Init.DataSize          = SPI_8BIT_DF_ENABLE;
    hspi1.Init.Direction         = SPI_ENABLE_2_LINE_UNI_DIR;
    hspi1.Init.CLKPolarity       = SPI_CPOL_LOW;
    hspi1.Init.CLKPhase          = SPI_SECOND_CLOCK_TRANS;
    hspi1.Init.NSS               = SPI_SSM_ENABLE;
    hspi1.Init.BaudRatePrescaler = SPI_REG_CR1_BR_PCLK_DIV_32;
    hspi1.Init.FirstBit          = SPI_TX_MSB_FIRST;

    /* Call HAL initialization function */
    hal_spi_init(&hspi1);

    /* Enable NVIC interrupt for SPI1 */
    NVIC_EnableIRQ(SPI1_IRQn);
    NVIC_SetPriority(SPI1_IRQn, 1);
}

static void SPI2_Init(void)
{
    /* Enable SPI2 Clock on APB1 */
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

    /* Configure SPI initialization structure */
    hspi2.Instance               = SPI2;
    hspi2.Init.Mode              = SPI_MASTER_MODE_SEL;
    hspi2.Init.DataSize          = SPI_8BIT_DF_ENABLE;
    hspi2.Init.Direction         = SPI_ENABLE_2_LINE_UNI_DIR;
    hspi2.Init.CLKPolarity       = SPI_CPOL_LOW;
    hspi2.Init.CLKPhase          = SPI_SECOND_CLOCK_TRANS;
    hspi2.Init.NSS               = SPI_SSM_ENABLE;
    hspi2.Init.BaudRatePrescaler = SPI_REG_CR1_BR_PCLK_DIV_32;
    hspi2.Init.FirstBit          = SPI_TX_MSB_FIRST;

    /* Call HAL initialization function */
    hal_spi_init(&hspi2);

    /* Enable NVIC interrupt for SPI2 */
    NVIC_EnableIRQ(SPI2_IRQn);
    NVIC_SetPriority(SPI2_IRQn, 1);
}

void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms * 4000; i++) __asm__("nop");
}

uint8_t txData[] = "HI YS";
uint8_t rxData[6];
int main(void)
{

    /* Initialize SPI GPIO and peripheral */
	SPI_GPIO_Init();
    SPI1_Init();
    SPI2_Init();

    hal_spi_master_rx(&hspi2, rxData, 6);
    delay_ms(10);
    hal_spi_master_tx(&hspi1, txData, 6);

    while(hspi1.State != HAL_SPI_STATE_READY);
    while(hspi2.State != HAL_SPI_STATE_READY);

    delay_ms(100);

    while (1)
    {
    	delay_ms(1000);
    }
}
