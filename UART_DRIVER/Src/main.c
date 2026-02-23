#include <stdint.h>
#include <string.h>
#include "UART_HAL.h"

uart_handle_t huart1;
uint8_t tx_data[] = "Hello from uart1!\r\n";
uint8_t rx_data[1];

void my_rx_callback(void *ptr) {
    hal_uart_tx(&huart1, rx_data, 1);
    hal_uart_rx(&huart1, rx_data, 1);
}

void GPIO_Init_USART1(void) {

    RCC->AHB1ENR |= (1 << 0); // GPIOA Clock
    __HAL_RCC_USART1_CLK_ENABLE();

    // PA9 (TX) and PA10 (RX) as Alternate Function
    GPIOA->MODER &= ~((3 << 18) | (3 << 20));
    GPIOA->MODER |=  ((2 << 18) | (2 << 20));

    // Set AF7 for PA9 and PA10 (AFRH for pins 8-15)
    GPIOA->AFRH &= ~((0xF << 4) | (0xF << 8));
    GPIOA->AFRH |=  ((0x7 << 4) | (0x7 << 8));
}

int main(void) {

    GPIO_Init_USART1();

    huart1.Instance          = USART_1;
    huart1.Init.BaudRate     = USART_BAUD_9600;
    huart1.Init.WordLength   = USART_WL_1S8B;
    huart1.Init.StopBits     = USART_CR2_1StopBit;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = (USART_REG_CR1_TE | USART_REG_CR1_RE);
    huart1.Init.OverSampling = USART_OVER16_ENABLE;

    huart1.rx_cmp_cb = my_rx_callback;

    hal_uart_init(&huart1);

    // Enable USART1 Interrupt in NVIC (IRQ 37)
    NVIC_ENABLE_IRQ(37);

    hal_uart_tx(&huart1, tx_data, strlen((char*)tx_data));
    hal_uart_rx(&huart1, rx_data, 1);

    while(1);
}


void USART1_IRQHandler(void) {
    hal_uart_handle_interrupt(&huart1);
}
