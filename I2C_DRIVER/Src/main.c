#include <stdint.h>
#include <string.h>
#include "I2C_HAL.h"

i2c_handle_t hi2c1, hi2c2, hi2c3;

void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms * 4000; i++) __asm__("nop");
}

void GPIO_Init(void)
{
    // Enable GPIOA, GPIOB and GPIOC clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

    // I2C1(PB8, PB9) , I2C2(PB10, PB11) -> Alternate Function
    GPIOB->MODER &= ~((3U << 16) | (3U << 18) | (3U << 20) | (3U << 22));
    GPIOB->MODER |=  ((2U << 16) | (2U << 18) | (2U << 20) | (2U << 22));

    // PA8 -> Alternate Function (I2C3 SCL)
    GPIOA->MODER &= ~(3U << 16);
    GPIOA->MODER |=  (2U << 16);

    // PC9 -> Alternate Function (I2C3 SDA)
    GPIOC->MODER &= ~(3U << 18);
    GPIOC->MODER |=  (2U << 18);

    // Configure AF4 on PB8-PB11
    GPIOB->AFR[1] &= ~0xFFFFU;
    GPIOB->AFR[1] |=  0x4444U;

    // Configure AF4 on PA8
    GPIOA->AFR[1] &= ~(0xFU << 0);
    GPIOA->AFR[1] |=  (4U << 0);

    // Configure AF4 on PC9
    GPIOC->AFR[1] &= ~(0xFU << 4);
    GPIOC->AFR[1] |=  (4U << 4);

    // Configure all I2C pins as open-drain
    GPIOB->OTYPER |= (0xFU << 8);
    GPIOA->OTYPER |= (1U << 8);
    GPIOC->OTYPER |= (1U << 9);

    // External pull-ups are used: disable internal pulls
    GPIOB->PUPDR &= ~(0xFFU << 16);
    GPIOA->PUPDR &= ~(3U << 16);
    GPIOC->PUPDR &= ~(3U << 18);
}

void I2C_Init(void) {

	_HAL_RCC_I2C1_CLK_ENABLE();
	_HAL_RCC_I2C2_CLK_ENABLE();
	_HAL_RCC_I2C3_CLK_ENABLE();

    NVIC_EnableIRQ(I2C1_EV_IRQn);
    NVIC_EnableIRQ(I2C1_ER_IRQn);

    NVIC_EnableIRQ(I2C2_EV_IRQn);
    NVIC_EnableIRQ(I2C2_ER_IRQn);

    NVIC_EnableIRQ(I2C3_EV_IRQn);
    NVIC_EnableIRQ(I2C3_ER_IRQn);

    // I2C1 Master
    hi2c1.Instance = I2C_1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_FM_DUTY_2;
    hi2c1.Init.OwnAddress1 = 0x60;
    hi2c1.Init.AddressingMode = I2C_ADDRMODE_7BIT;
    hi2c1.Init.NoStretchMode = 0;
    hi2c1.Init.ack_enable = I2C_ACK_ENABLE;
    hal_i2c_init(&hi2c1, I2C_PERIPHERAL_CLK_FREQ_16MHZ);

    // I2C2 Slave
    hi2c2.Instance = I2C_2;
    hi2c2.Init.ClockSpeed = 100000;
    hi2c2.Init.DutyCycle = I2C_FM_DUTY_2;
    hi2c2.Init.OwnAddress1 = 0x50;
    hi2c2.Init.AddressingMode = I2C_ADDRMODE_7BIT;
    hi2c2.Init.NoStretchMode = 0;
    hi2c2.Init.ack_enable = I2C_ACK_ENABLE;
    hal_i2c_init(&hi2c2, I2C_PERIPHERAL_CLK_FREQ_16MHZ);

    // I2C3 Slave
    hi2c3.Instance = I2C_3;
    hi2c3.Init.ClockSpeed = 100000;
    hi2c3.Init.DutyCycle = I2C_FM_DUTY_2;
    hi2c3.Init.OwnAddress1 = 0x40;
    hi2c3.Init.AddressingMode = I2C_ADDRMODE_7BIT;
    hi2c3.Init.NoStretchMode = 0;
    hi2c3.Init.ack_enable = I2C_ACK_ENABLE;
    hal_i2c_init(&hi2c3, I2C_PERIPHERAL_CLK_FREQ_16MHZ);
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

void I2C3_EV_IRQHandler(void) {
    hal_i2c_handle_evt_interrupt(&hi2c3);
}

void I2C3_ER_IRQHandler(void) {
    hal_i2c_handle_error_interrupt(&hi2c3);
}

uint8_t tx_data1[]  = "HELLO1";
uint8_t rx1_data[7] = {0};
uint8_t tx_data2[]  = "HELLO2";
uint8_t rx2_data[7] = {0};


int main(void) {

    GPIO_Init();
    I2C_Init();
    delay_ms(100);

    // Master TX, Slave RX
    hal_i2c_slave_rx(&hi2c2, rx1_data, strlen((char*)tx_data1));
    delay_ms(10);

    hal_i2c_master_tx(&hi2c1, 0x50, tx_data1, strlen((char*)tx_data1));

    hal_i2c_slave_rx(&hi2c3, rx2_data, strlen((char*)tx_data2));
    delay_ms(10);
    hal_i2c_master_tx(&hi2c1, 0x40, tx_data2, strlen((char*)tx_data2));

    while(1) {
        delay_ms(1000);
    }

    return 0;
}
