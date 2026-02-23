#include <stdint.h>
#include "I2C_HAL.h"

i2c_handle_t hi2c1, hi2c2;

void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms * 4000; i++) __asm__("nop");
}

void GPIO_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    // PB8, PB9 (I2C1), PB10, PB11 (I2C2) - Alternate function
    GPIOB->MODER &= ~((3 << 16) | (3 << 18) | (3 << 20) | (3 << 22));
    GPIOB->MODER |= ((2 << 16) | (2 << 18) | (2 << 20) | (2 << 22));

    // AF4
    GPIOB->AFR[1] |= ((4 << 0) | (4 << 4) | (4 << 8) | (4 << 12));

    // Open drain, pull-up
    GPIOB->OTYPER |= ((1 << 8) | (1 << 9) | (1 << 10) | (1 << 11));
    GPIOB->PUPDR |= ((1 << 16) | (1 << 18) | (1 << 20) | (1 << 22));
}

void I2C_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN | RCC_APB1ENR_I2C2EN;

    NVIC_EnableIRQ(I2C1_EV_IRQn);
    NVIC_EnableIRQ(I2C1_ER_IRQn);
    NVIC_EnableIRQ(I2C2_EV_IRQn);
    NVIC_EnableIRQ(I2C2_ER_IRQn);

    // I2C1 Master
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_FM_DUTY_2;
    hi2c1.Init.OwnAddress1 = 0x60;
    hi2c1.Init.AddressingMode = I2C_ADDRMODE_7BIT;
    hi2c1.Init.NoStretchMode = 0;
    hi2c1.Init.ack_enable = I2C_ACK_ENABLE;
    hal_i2c_init(&hi2c1);

    // I2C2 Slave
    hi2c2.Instance = I2C2;
    hi2c2.Init.ClockSpeed = 100000;
    hi2c2.Init.DutyCycle = I2C_FM_DUTY_2;
    hi2c2.Init.OwnAddress1 = 0x50;
    hi2c2.Init.AddressingMode = I2C_ADDRMODE_7BIT;
    hi2c2.Init.NoStretchMode = 0;
    hi2c2.Init.ack_enable = I2C_ACK_ENABLE;
    hal_i2c_init(&hi2c2);
}

void I2C1_EV_IRQHandler(void) {
    hal_i2c_handle_evt_interrupt(&hi2c1);
}

void I2C1_ER_IRQHandler(void) {
    hal_i2c_handle_error_interrupt(&hi2c1);
}

void I2C2_EV_IRQHandler(void) {
    hal_i2c_handle_evt_interrupt(&hi2c2);
}

void I2C2_ER_IRQHandler(void) {
    hal_i2c_handle_error_interrupt(&hi2c2);
}

uint8_t tx_data[] = "HELLO using I2C";
uint8_t rx_data[16];

int main(void) {
    GPIO_Init();
    I2C_Init();
    delay_ms(100);

    // Master TX, Slave RX
    hal_i2c_slave_rx(&hi2c2, rx_data, 16);
    delay_ms(10);
    hal_i2c_master_tx(&hi2c1, 0x50, tx_data, 16);

    delay_ms(500);

    while(1) {
        delay_ms(1000);
    }

    return 0;
}
