#include <stdint.h>
#include "I2C_HAL.h"


/* ======================== Helper Functions to write the APIs for I2C ============================ */
/*
 * @brief  Enables the given I2C module
 * @param  *i2cx : Base address the I2C Peripheral
 * @retval None
 */
static void hal_i2c_enable_peripheral(I2C_TypeDef *i2cx)
{
    i2cx->CR1 |= I2C_REG_CR1_ENABLE_I2C;
}

/*
 * @brief  Disables the given I2C module
 * @param  *i2cx : Base address the I2C Peripheral
 * @retval None
 */
static void hal_i2c_disable_peripheral(I2C_TypeDef *i2cx)
{
    i2cx->CR1 &= ~I2C_REG_CR1_ENABLE_I2C;
}


/**
 * @brief  I2C Clock Control Register (CCR) configuration
 * @param  *i2cx      : Base address the I2C Peripheral
 * @param  pclk       : Peripheral clock frequency in Mhz
 * @param  clkspeed   : Desire clock speed in Hz
 * @param  duty_cycle : Fast mode duty cycle selection (I2C_FM_DUTY_2 or I2C_FM_DUTY_16_9)
 * @retval None
 */
static void hal_i2c_configure_ccr(I2C_TypeDef *i2cx, uint32_t pclk, uint32_t clkspeed, uint32_t duty_cycle)
{
    uint32_t ccr_val = 0;

    if(clkspeed <= 100000) // Standard Mode
    {
        ccr_val = pclk * 1000000 / (clkspeed * 2); // CCR = Fpclk / (2*Fscl)
        i2cx->CCR &= ~0xFFF;
        i2cx->CCR |= ccr_val & 0xFFF;
    }
    else // Fast Mode
    {
        if(duty_cycle == I2C_FM_DUTY_2)
            ccr_val = pclk * 1000000 / (3 * clkspeed);  // duty 2
        else
            ccr_val = pclk * 1000000 / (25 * clkspeed); // duty 16/9

        i2cx->CCR &= ~0xFFF;
        i2cx->CCR |= ccr_val & 0xFFF;
        i2cx->CCR |= I2C_REG_CCR_ENABLE_FM; // Set FM bit
    }
}

/**
 * @brief  I2C Rise Time (TRISE) configuration
 * @param  *i2cx    : Base address the I2C Peripheral
 * @param  pclk     : Peripheral clock frequency in Mhz
 * @param  clkspeed : Desire clock speed in Hz
 * @retval None
 */
static void hal_i2c_rise_time_configuration(I2C_TypeDef *i2cx, uint32_t pclk, uint32_t clkspeed)
{
    uint32_t trise = 0;

    if(clkspeed <= 100000) // Standard mode
    {
        trise = (pclk * 1000000 / 1000000) + 1; // TRISE = Fpclk(MHz) + 1
    }
    else // Fast mode
    {
        trise = ((pclk * 1000000 * 300) / 1000000000) + 1; // TRISE = Fpclk*300ns +1
    }

    i2cx->TRISE = trise & 0x3F; // TRISE is 6-bit
}

/**
 * @brief  Enables/Disables clock strectching
 * @param  *i2cx : Base address the I2C Peripheral
 * @param  no_stretch : clock strecting enable/disable
 * @retval None
 */
static void hal_i2c_manage_clock_stretch(I2C_TypeDef *i2cx, uint32_t no_stretch)
{
    if(no_stretch)
    {
        //clock streching disabled
        i2cx->CR1 |= I2C_REG_CR1_NOSTRETCH;
    }else
    {
        i2cx->CR1 &= ~I2C_REG_CR1_NOSTRETCH;
    }
}

/*
 * @brief  Configures the own I2C device address
 * @param  *i2cx : Base address the I2C Peripheral
 * @param  own_address : Address of the I2C Device to be configured
 */
static void hal_i2c_set_own_address1(I2C_TypeDef *i2cx, uint32_t own_address)
{
    i2cx->OAR1 &= ~( 0x7f << 1);
    i2cx->OAR1 |=  (own_address << 1);
}

/**
 * @brief  Configures I2C addressing mode either 7 bit or 10 bit
 * @param  *i2cx : Base address the I2C Peripheral
 * @param  adr_mode : addressing mode to be configured
 * @retval None
 */
static void hal_i2c_set_addressing_mode(I2C_TypeDef *i2cx, uint32_t adr_mode)
{
    if(adr_mode == I2C_ADDRMODE_10BIT)
        i2cx->OAR1 |= I2C_REG_OAR1_ADDRMODE;
    else
        i2cx->OAR1 &= ~I2C_REG_OAR1_ADDRMODE;
}

/**
 * @brief  Configures I2C Clock duty clycle in FM mode
 * @param  *i2cx : Base address the I2C Peripheral
 * @param  duty_cycle : duty cycle to be configured it couble be either I2C_FM_DUTY_16BY9 or I2C_FM_DUTY_2
 * @retval None
 */
static void hal_i2c_set_fm_mode_duty_cycle(I2C_TypeDef *i2cx, uint32_t duty_cycle)
{
    if(duty_cycle == I2C_FM_DUTY_16BY9 )
    {
        i2cx->CCR |= I2C_REG_CCR_DUTY;
    }else
    {
        i2cx->CCR &= ~I2C_REG_CCR_DUTY;
    }
}

/**
 * @brief  Does I2C clock realted intitialzations
 * @param  *i2cx : Base address the I2C Peripheral
 * @param  clkspeed : I2C clock speed
 * @param  duty_cycle : I2C clock duty_cycle
 * @retval None
 */
static void hal_i2c_clk_init(I2C_TypeDef *i2cx, uint32_t clkspeed, uint32_t duty_cycle)
{
    uint32_t pclk = I2C_PERIPHERAL_CLK_FREQ_10MHZ;
    i2cx->CR2 &= ~(0x3F);
    i2cx->CR2 |= ( pclk & 0x3F );
    hal_i2c_configure_ccr(i2cx,pclk,clkspeed,duty_cycle);
    hal_i2c_rise_time_configuration(i2cx,pclk, clkspeed);
}

/**
 * @brief  Generated start condition , used only by master
 * @param  *i2cx : Base address the I2C Peripheral
 * @retval None
 */
static void hal_i2c_generate_start_condition(I2C_TypeDef *i2cx)
{
    i2cx->CR1 |= I2C_REG_CR1_START_GEN;
}

/**
 * @brief  Generated stop condition , used only by master
 * @param  *i2cx : Base address the I2C Peripheral
 * @retval None
 */
static void hal_i2c_generate_stop_condition(I2C_TypeDef *i2cx)
{
    i2cx->CR1 |= I2C_REG_CR1_STOP_GEN;
}

/**
 * @brief  Enables/Disables TXE and RXNE (buffer) interrupt
 * @param  *i2cx  : Base address the I2C Peripheral
 * @param  enable : if this is set to zero, then tx, rx interrupts will be disabled
 * @retval None
 */
static void hal_i2c_configure_buffer_interrupt(I2C_TypeDef *i2cx, uint32_t enable)
{
    if(enable)
        i2cx->CR2 |= I2C_REG_CR2_BUF_INT_ENABLE;
    else
        i2cx->CR2 &= ~I2C_REG_CR2_BUF_INT_ENABLE;
}

/**
 * @brief  Enables/Disables error interrupt
 * @param  *i2cx  : Base address the I2C Peripheral
 * @param  enable : if this is set to 0, then error interrupts will be disabled
 * @retval None
 */
static void hal_i2c_configure_error_interrupt(I2C_TypeDef *i2cx, uint32_t enable)
{
    if(enable)
        i2cx->CR2 |= I2C_REG_CR2_ERR_INT_ENABLE;
    else
        i2cx->CR2 &= ~I2C_REG_CR2_ERR_INT_ENABLE;
}

/**
 * @brief  Enables/Disables I2C EVT interrupt
 * @param  *i2cx  : Base address the I2C Peripheral
 * @param  enable : if this is set to 0, then evt interrupts will be disabled
 * @retval None
 */
static void hal_i2c_configure_evt_interrupt(I2C_TypeDef *i2cx, uint32_t enable)
{
    if(enable)
        i2cx->CR2 |= I2C_REG_CR2_EVT_INT_ENABLE;
    else
        i2cx->CR2 &= ~I2C_REG_CR2_EVT_INT_ENABLE;
}

/**
 * @brief  Check whether bus is free or busy
 * @param  *i2cx : Base address the I2C Peripheral
 * @retval : return 1, if the bus is busy
 */
static uint8_t is_bus_busy(I2C_TypeDef *i2cx)
{
    if(i2cx->SR2 & I2C_REG_SR2_BUS_BUSY_FLAG )
        return 1; //yes
    else
        return 0;
}

/**
 * @brief  call this function to wait untill, SB(start byte ) flag is set.
 * it is set , when master has succesfully generated the start condition .
 * @param  *i2cx : Base address the I2C Peripheral
 * @retval returns 1 if SB is set
 */
static void i2c_wait_until_sb_set(I2C_TypeDef *i2cx)
{
    /*wait until SB is successfully generated by the master */
    while( ! (i2cx->SR1 & I2C_REG_SR1_SB_FLAG ));
}

/**
 * @brief  Call this function to wait untill, ADDR byte is sent.
 * it is set , when master has successfully finished sending
 * @param  *i2cx : Base address the I2C Peripheral
 * @retval None
 */
static void i2c_wait_until_addr_set(I2C_TypeDef *i2cx)
{
    /* Wait until ADDR flag is set, that means, address phase is succes */
    while( ! (i2cx->SR1 & I2C_REG_SR1_ADDR_SENT_FLAG ));
}

/**
 * @brief  Handle TXE Interrupt for master
 * @param  hi2c: pointer to a i2c_handle_t structure that contains
 * the configuration information for I2C module
 * @retval none
 */
static void hal_i2c_master_handle_TXE_interrupt(i2c_handle_t *hi2c)
{
    /* Write data to DR */
    hi2c->Instance->DR = (*hi2c->pBuffPtr++);
    hi2c->XferCount--;

    if(hi2c->XferCount == 0)
    {
        /* Disable BUF interrupt */
        hi2c->Instance->CR2 &= ~I2C_REG_CR2_BUF_INT_ENABLE;
    }
}

/**
 * @brief  Handle ADDR flag for Slave
 * @param  hi2c: pointer to a i2c_handle_t structure that contains
 * the configuration information for I2C module
 * @retval none
 */
static void hal_i2c_clear_addr_flag(i2c_handle_t *hi2c)
{
	volatile uint32_t tmpreg;
	tmpreg = hi2c->Instance->SR1;
	tmpreg = hi2c->Instance->SR2;
	(void)tmpreg;
}

/**
 * @brief  Sends the slave address on the bus
 * @param  *i2cx : Base address the I2C Peripheral
 * @param  slave_addr : The 7-bit address of the slave
 * @retval None
 */
static void hal_i2c_send_addr_first(I2C_TypeDef *i2cx, uint8_t slave_addr)
{
    /* Shift address to make room for R/W bit */
    slave_addr = (slave_addr << 1);
    i2cx->DR = slave_addr;
}

/**
 * @brief  Clears the STOPF flag for Slave
 * @param  hi2c: pointer to a i2c_handle_t structure
 * @retval none
 */
static void hal_i2c_clear_stop_flag(i2c_handle_t *hi2c)
{
	volatile uint32_t tmpreg;
	/* Clear STOP flag */
	tmpreg = hi2c->Instance->SR1;
	hi2c->Instance->CR1 |= I2C_REG_CR1_ACK;  // write to CR1
    (void)tmpreg;
}


static void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms * 4000; i++) {
        __asm__("nop");
    }
}

static void GPIO_setup(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;

    GPIOG->MODER &= ~((3 << 26) | (3 << 28));
    GPIOG->MODER |= ((1 << 26) | (1 << 28));
}

/**
 * @brief  Called when Master Tx completed .
 * @param  hi2c: pointer to a i2c_handle_t structure that contains
 * the configuration information for I2C module
 * @retval None
 */
static void hal_i2c_master_tx_complt(i2c_handle_t *hi2c)
{
  /* Blinking green led means transmission complete */
    GPIO_setup();
	while(1)
	{
        GPIOG->ODR |= (1 << 14); 
        delay_ms(100);
        GPIOG->ODR &= ~((1 << 14));
        delay_ms(100);
	}
	
}

/**
 * @brief  Called when Slave Tx completed .
 * @param  hi2c: pointer to a i2c_handle_t structure that contains
 * the configuration information for I2C module
 * @retval None
 */
static void hal_i2c_slave_tx_complt(i2c_handle_t *hi2c)
{
	/* Blinking green led means transmission complete */
    GPIO_setup();
	while(1)
	{
        GPIOG->ODR |= (1 << 14); 
        delay_ms(200);
        GPIOG->ODR &= ~((1 << 14));
        delay_ms(200);
	}
}

/**
 * @brief  Called when Slave Rx Transfer completed .
 * @param  hi2c: pointer to a i2c_handle_t structure that contains
 * the configuration information for I2C module
 * @retval None
 */
static void hal_i2c_slave_rx_complt(i2c_handle_t *hi2c)
{
  /* Blinking green led means transmission complete */
    GPIO_setup();
	while(1)
	{
        GPIOG->ODR |= (1 << 14); 
        delay_ms(250);
        GPIOG->ODR &= ~((1 << 14));
        delay_ms(250);
	}
}


/**
 * @brief  Handle BTF flag for Master transmitter
 * @param  hi2c: pointer to a i2c_handle_t structure that contains
 * the configuration information for I2C module
 * @retval none
 */
static void hal_i2c_master_tx_handle_btf(i2c_handle_t *hi2c)
{
    if(hi2c->XferCount != 0)
    {
        /* Write data to DR */
        hi2c->Instance->DR = (*hi2c->pBuffPtr++);
        hi2c->XferCount--;
    }
    else
    {
        /* Disable EVT, BUF and ERR interrupt */
        hi2c->Instance->CR2 &= ~ I2C_REG_CR2_EVT_INT_ENABLE;
        hi2c->Instance->CR2 &= ~ I2C_REG_CR2_BUF_INT_ENABLE;
        hi2c->Instance->CR2 &= ~ I2C_REG_CR2_ERR_INT_ENABLE;

        /* Generate Stop */
        hi2c->Instance->CR1 |= I2C_REG_CR1_STOP_GEN;

        //since all the bytes are sent, make state = ready
        hi2c->State = HAL_I2C_STATE_READY;

        //in this function , you can all application call back function
        hal_i2c_master_tx_complt(hi2c);
    }
}


/**
 * @brief  Handle STOPF flag for Slave
 * @param  hi2c: pointer to a i2c_handle_t structure that contains
 * the configuration information for I2C module
 * @retval none
 */
static void hal_i2c_slave_handle_stop_condition(i2c_handle_t *hi2c)
{
    /* Disable EVT, BUF and ERR interrupt */
    hi2c->Instance->CR2 &= ~ I2C_REG_CR2_EVT_INT_ENABLE;
    hi2c->Instance->CR2 &= ~ I2C_REG_CR2_BUF_INT_ENABLE;
    hi2c->Instance->CR2 &= ~ I2C_REG_CR2_ERR_INT_ENABLE;

    /* Clear STOPF flag */
    hal_i2c_clear_stop_flag(hi2c);

    /* Disable Acknowledge */
    hi2c->Instance->CR1 &= ~ I2C_REG_CR1_ACK;

    hi2c->State = HAL_I2C_STATE_READY;

    hal_i2c_slave_rx_complt(hi2c);
}

/**
 * @brief  Handle TXE flag for Slave
 * @param  hi2c: pointer to a i2c_handle_t structure that contains
 * the configuration information for I2C module
 * @retval none
 */
static void hal_i2c_slave_handle_TXE_interrupt(i2c_handle_t *hi2c)
{
    if(hi2c->XferCount != 0)
    {
        /* Write data to DR */
        hi2c->Instance->DR = (*hi2c->pBuffPtr++);
        hi2c->XferCount--;
    }
}

/**
 * @brief  Handle BTF flag for Slave transmitter
 * @param  hi2c: pointer to a i2c_handle_t structure that contains
 * the configuration information for I2C module
 * @retval none
 */
static void hal_i2c_slave_tx_handle_btf(i2c_handle_t *hi2c)
{
    if(hi2c->XferCount != 0)
    {
        /* Write data to DR */
        hi2c->Instance->DR = (*hi2c->pBuffPtr++);
        hi2c->XferCount--;
    }
}

/**
 * @brief  Handle BTF flag for Slave receiver
 * @param  hi2c: pointer to a i2c_handle_t structure that contains
 * the configuration information for I2C module
 * @retval none
 */
static void hal_i2c_slave_rx_handle_btf(i2c_handle_t *hi2c)
{
    if(hi2c->XferCount != 0)
    {
        /* Read data from DR */
        (*hi2c->pBuffPtr++) = hi2c->Instance->DR;
        hi2c->XferCount--;
    }
}

/**
 * @brief  Handle RXNE flag for Slave
 * @param  hi2c: pointer to a i2c_handle_t structure that contains
 * the configuration information for I2C module
 * @retval none
 */
static void hal_i2c_slave_handle_RXNE_interrupt(i2c_handle_t *hi2c)
{
    if(hi2c->XferCount != 0)
    {
        /* Read data from DR */
        (*hi2c->pBuffPtr++) = hi2c->Instance->DR;
        hi2c->XferCount--;
    }
}


/**
 * @brief  I2C error callbacks
 * @param  hi2c : I2C handle
 * @note   This example shows a simple way to report transfer error, and you can
 * add your own implementation.
 * @retval None
 */
void hal_i2c_error_cb(i2c_handle_t *hi2c)
{
    GPIO_setup();

    while(1) {
		/* LED RED Blinking*/
        GPIOG->ODR |= (1 << 14); 
        delay_ms(100);
        GPIOG->ODR &= ~((1 << 14));
        delay_ms(100);
    }
}


/**
 * @brief  Handle Ack Failure Condtion
 * @param  hi2c: pointer to a i2c_handle_t structure that contains
 * the configuration information for I2C module
 * @retval none
 */
static void hal_i2c_slave_handle_ack_failure(i2c_handle_t *hi2c)
{
    /* Disable EVT, BUF and ERR interrupts */
    hi2c->Instance->CR2 &= ~I2C_REG_CR2_EVT_INT_ENABLE;
    hi2c->Instance->CR2 &= ~I2C_REG_CR2_BUF_INT_ENABLE;
    hi2c->Instance->CR2 &= ~I2C_REG_CR2_ERR_INT_ENABLE;

    /* Clear AF flag */
    hi2c->Instance->SR1 &= ~I2C_REG_SR1_AF_FAILURE_FLAG;

    /* If STOP detected, clear it (prevents bus lock) */
    if(hi2c->Instance->SR1 & I2C_REG_SR1_STOP_DETECTION_FLAG)
    {
        hal_i2c_clear_stop_flag(hi2c);
    }

    /* Keep ACK enabled for next transaction */
    hi2c->Instance->CR1 |= I2C_REG_CR1_ACK;

    hi2c->State = HAL_I2C_STATE_READY;

    /* Notify application layer */
    hal_i2c_slave_tx_complt(hi2c);
}

/* ================================== I2C Functions ================================== */

/**
 * @brief  Initializes the give I2C device
 * @param  *hi2c : Handle to the I2C device, which the application wats to initialize
 * @retval None
 */
void hal_i2c_init(i2c_handle_t *hi2c)
{
    /* I2C clock related initializations */
    hal_i2c_clk_init(hi2c->Instance, hi2c->Init.ClockSpeed, hi2c->Init.DutyCycle);

    /* Set I2C addressing mode */
    hal_i2c_set_addressing_mode(hi2c->Instance, hi2c->Init.AddressingMode);

    /* Enable ACKing */
    if(hi2c->Init.ack_enable == I2C_ACK_ENABLE)
    {
        hi2c->Instance->CR1 |= I2C_REG_CR1_ACK;
    }
    else
    {
        hi2c->Instance->CR1 &= ~I2C_REG_CR1_ACK;
    }

    /* Enable/Disable clock stretching */
    hal_i2c_manage_clock_stretch(hi2c->Instance, hi2c->Init.NoStretchMode);

    /* 5. Configure the own address */
    hal_i2c_set_own_address1(hi2c->Instance, hi2c->Init.OwnAddress1);

    /* Finally, enable the I2C peripheral */
    hal_i2c_enable_peripheral(hi2c->Instance);
}


/**
 * @brief  API to do master data transmission
 * @param  *hi2c : pointer to the handle structure of the I2C Device
 * @param  slave_addr : address to which we want to tx
 * @param  *buffer : holds the pointer to the tx buffer
 * @param  len : len of the data to be TXed
 * @retval None
 */
void hal_i2c_master_tx(i2c_handle_t *hi2c, uint8_t slave_addr, uint8_t *buffer, uint32_t len)
{
    /* populate the handle with tx buffer pointer and lenght information */
    hi2c->pBuffPtr = buffer;
    hi2c->XferCount = len;
    hi2c->XferSize = len;
    hi2c->State = HAL_I2C_STATE_BUSY_TX;

    /* make sure that I2C is enabled */
    hal_i2c_enable_peripheral(hi2c->Instance);

    /* Check if Busy*/
    while(is_bus_busy(hi2c->Instance));

    /* first lets generate the start condition by using our helper function */
    hal_i2c_generate_start_condition(hi2c->Instance);

    /* wait till sb is set */
    i2c_wait_until_sb_set(hi2c->Instance);

 /* address phase: send the 8 bit slave address first (8th bit is r/w bit) */
    hal_i2c_send_addr_first(hi2c->Instance, slave_addr);

    i2c_wait_until_addr_set(hi2c->Instance);

    /* if you are here, that means, ADDR is set, and clock is stretched(wait state) ! */
    /* clearing the ADDR flag make I2C comes out of wait state */
    hal_i2c_clear_addr_flag(hi2c);

    /* enable the buff, err , event interrupts */
    hal_i2c_configure_buffer_interrupt(hi2c->Instance, 1);
    hal_i2c_configure_error_interrupt(hi2c->Instance, 1);
    hal_i2c_configure_evt_interrupt(hi2c->Instance, 1);
}


/**
 * @brief  API to do master data reception
 * @param  *hi2c      : pointer to the handle structure of the I2C Device
 * @param  slave_addr : address from which we want to receive
 * @param  *buffer    : holds the pointer to the rx buffer
 * @param  len        : len of the data to be received
 * @retval None
 */
void hal_i2c_master_rx(i2c_handle_t *hi2c, uint8_t slave_addr, uint8_t *buffer, uint32_t len)
{
    /* populate the handle with rx buffer pointer and lenght information */
	hi2c->pBuffPtr = buffer;
	hi2c->XferCount = len;
	hi2c->XferSize = len;

    /* make state is bus in RX */
	hi2c->State = HAL_I2C_STATE_BUSY_RX;

    /* enable the I2C peripheral */
    hal_i2c_enable_peripheral(hi2c->Instance);

    /* Check if Busy*/
    while(is_bus_busy(hi2c->Instance));

    /* make sure that POS bit is disabled */
    hi2c->Instance->CR1 &= ~I2C_CR1_POS;

    /* make sure that ACKing is enabled */
    hi2c->Instance->CR1 |= I2C_CR1_ACK;

    /* first lets generate the start condition by using our helper function */
    hal_i2c_generate_start_condition(hi2c->Instance);

    /* wait till sb is set */
    i2c_wait_until_sb_set(hi2c->Instance);

    // send the slave address
    hal_i2c_send_addr_first(hi2c->Instance, slave_addr);

    // wait until ADDR = 1, that means Address phase is complted successfully
    i2c_wait_until_addr_set(hi2c->Instance);

    // clear the ADDR flag which is set
    hal_i2c_clear_addr_flag(hi2c);

    /* Enable the buff, err, event interrupts */
    hal_i2c_configure_buffer_interrupt(hi2c->Instance, 1);
    hal_i2c_configure_error_interrupt(hi2c->Instance, 1);
    hal_i2c_configure_evt_interrupt(hi2c->Instance, 1);
}


/**
 * @brief  API to do Slave data transmission
 * @param  *hi2c   : pointer to the handle structure of the I2C Peripheral
 * @param  *buffer : holds the pointer to the tx buffer
 * @param  len     : len of the data to be TXed
 * @retval None
 */
void hal_i2c_slave_tx(i2c_handle_t *hi2c, uint8_t *buffer, uint32_t len)
{
    /* populate the handle with tx buffer pointer and length information */
	hi2c->pBuffPtr = buffer;
	hi2c->XferCount = len;
	hi2c->XferSize = len;

	hi2c->State = HAL_I2C_STATE_BUSY_TX;

    // make sure that POS bit is disabled
	hi2c->Instance->CR1 &= ~I2C_CR1_POS;

    hal_i2c_enable_peripheral(hi2c->Instance);

    /* Enable Address Acknowledge */
    hi2c->Instance->CR1 |= I2C_CR1_ACK;

    /* Enable the buff, err, event interrupts */
    hal_i2c_configure_buffer_interrupt(hi2c->Instance, 1);
    hal_i2c_configure_error_interrupt(hi2c->Instance, 1);
    hal_i2c_configure_evt_interrupt(hi2c->Instance, 1);
}

/**
 * @brief  API to do Slave data Reception
 * @param  *hi2c   : pointer to the handle structure of the I2C Peripheral
 * @param  *buffer : holds the pointer to the RX buffer
 * @param  len     : len of the data to be RXed
 * @retval None
 */
void hal_i2c_slave_rx(i2c_handle_t *hi2c, uint8_t *buffer, uint32_t len)
{
    /* populate the handle with rx buffer pointer and length information */
	hi2c->pBuffPtr = buffer;
    hi2c->XferCount = len;
    hi2c->XferSize = len;

    hi2c->State = HAL_I2C_STATE_BUSY_RX;

    // make sure that POS bit is disabled
    hi2c->Instance->CR1 &= ~I2C_CR1_POS;

    // enable the given I2C peripheral
    hal_i2c_enable_peripheral(hi2c->Instance);

    // enable the ACKing
    hi2c->Instance->CR1 |= I2C_CR1_ACK;

    /* enable the buff, err, event interrupts */
    hal_i2c_configure_buffer_interrupt(hi2c->Instance, 1);
    hal_i2c_configure_error_interrupt(hi2c->Instance, 1);
    hal_i2c_configure_evt_interrupt(hi2c->Instance, 1);
}

/**
 * @brief  This function handles I2C error interrupt request.
 * @param  *hi2c: pointer to a i2c_handle_t structure that contains the configuration information for I2C module
 * @retval none
 */
void hal_i2c_handle_evt_interrupt(i2c_handle_t *hi2c)
{
    uint32_t sr1 = hi2c->Instance->SR1;
    uint32_t sr2 = hi2c->Instance->SR2;
    uint32_t cr2 = hi2c->Instance->CR2;

    /* ================= MASTER MODE ================= */
    if(sr2 & I2C_REG_SR2_MSL_FLAG)
    {
        /* TXE */
        if((sr1 & I2C_REG_SR1_TXE_FLAG) && (cr2 & I2C_REG_CR2_BUF_INT_ENABLE))
        {
            if(hi2c->State == HAL_I2C_STATE_BUSY_TX)
                hal_i2c_master_handle_TXE_interrupt(hi2c);
        }

        /* BTF */
        if((sr1 & I2C_REG_SR1_BTF_FLAG) && (cr2 & I2C_REG_CR2_EVT_INT_ENABLE))
        {
            if(hi2c->State == HAL_I2C_STATE_BUSY_TX)
                hal_i2c_master_tx_handle_btf(hi2c);
        }
    }

    /* ================= SLAVE MODE ================= */
    else
    {
        /* ADDR */
        if((sr1 & I2C_REG_SR1_ADDR_MATCHED_FLAG) && (cr2 & I2C_REG_CR2_EVT_INT_ENABLE))
        {
            hal_i2c_clear_addr_flag(hi2c);
        }

        /* STOPF */
        if((sr1 & I2C_REG_SR1_STOP_DETECTION_FLAG) && (cr2 & I2C_REG_CR2_EVT_INT_ENABLE))
        {
        	hal_i2c_slave_handle_stop_condition(hi2c);
        }

        /* TX MODE */
        if(sr2 & I2C_REG_SR2_TRA_FLAG)
        {
            if((sr1 & I2C_REG_SR1_TXE_FLAG) && (cr2 & I2C_REG_CR2_BUF_INT_ENABLE))
                hal_i2c_slave_handle_TXE_interrupt(hi2c);

            if((sr1 & I2C_REG_SR1_BTF_FLAG) && (cr2 & I2C_REG_CR2_EVT_INT_ENABLE))
                hal_i2c_slave_tx_handle_btf(hi2c);
        }
        /* RX MODE */
        else
        {
            if((sr1 & I2C_REG_SR1_RXNE_FLAG) && (cr2 & I2C_REG_CR2_BUF_INT_ENABLE))
                hal_i2c_slave_handle_RXNE_interrupt(hi2c);

            if((sr1 & I2C_REG_SR1_BTF_FLAG) && (cr2 & I2C_REG_CR2_EVT_INT_ENABLE))
                hal_i2c_slave_rx_handle_btf(hi2c);
        }
    }
}


/**
 * @brief  This function handles I2C error interrupt request.
 * @param  hi2c: pointer to a i2c_handle_t structure that contains
 * the configuration information for I2C module
 * @retval none
 */
void hal_i2c_handle_error_interrupt(i2c_handle_t *hi2c)
{
    uint32_t tmp1 = 0, tmp2 = 0, tmp3 = 0;
    tmp1 = (hi2c->Instance->SR1 & I2C_REG_SR1_BUS_ERROR_FLAG);
    tmp2 = (hi2c->Instance->CR2 & I2C_REG_CR2_ERR_INT_ENABLE);

    /* I2C Bus error interrupt occurred ------------------------------------*/
    if((tmp1) && (tmp2))
    {
        hi2c->ErrorCode |= HAL_I2C_ERROR_BERR;
        /* Clear BERR flag */
        hi2c->Instance->SR1 &= ~ (I2C_REG_SR1_BUS_ERROR_FLAG);
    }

    tmp1 = (hi2c->Instance->SR1 & I2C_REG_SR1_ARLO_FLAG);
    tmp2 = (hi2c->Instance->CR2 & I2C_REG_CR2_ERR_INT_ENABLE);

    /* I2C Arbitration Loss error interrupt occurred -----------------------*/
    if((tmp1) && (tmp2))
    {
        hi2c->ErrorCode |= HAL_I2C_ERROR_ARLO;
        /* Clear ARLO flag */
        hi2c->Instance->SR1 &= ~ (I2C_REG_SR1_ARLO_FLAG);
    }

    tmp1 = (hi2c->Instance->SR1 & I2C_REG_SR1_AF_FAILURE_FLAG);
    tmp2 = (hi2c->Instance->CR2 & I2C_REG_CR2_ERR_INT_ENABLE);

    /* I2C Acknowledge failure error interrupt occurred --------------------*/
    if((tmp1) && (tmp2))
    {
        tmp1 = (hi2c->Instance->SR2 & I2C_REG_SR2_MSL_FLAG);
        tmp2 = hi2c->XferCount;
        tmp3 = hi2c->State;

        if((!tmp1) && (tmp3 == HAL_I2C_STATE_BUSY_TX))
        {
            /* if ack failure happens for slave,
               then slave should assume that, master doesnt want more data
               so, it should stop sending more data to the master */
            hal_i2c_slave_handle_ack_failure(hi2c);
        }
        else
        {
            /* if ack failure happens for master, then its an Error */
            hi2c->ErrorCode |= HAL_I2C_ERROR_AF;
            /* Clear AF flag */
            hi2c->Instance->SR1 &= ~ (I2C_REG_SR1_AF_FAILURE_FLAG);
        }
    }

    tmp1 = (hi2c->Instance->SR1 & I2C_REG_SR1_OVR_FLAG);
    tmp2 = (hi2c->Instance->CR2 & I2C_REG_CR2_ERR_INT_ENABLE);

    /* I2C Over-Run/Under-Run interrupt occurred ---------------------------*/
    if((tmp1) && (tmp2))
    {
        hi2c->ErrorCode |= HAL_I2C_ERROR_OVR;
        /* Clear OVR flag */
        hi2c->Instance->SR1 &= ~ (I2C_REG_SR1_OVR_FLAG);
    }

    if(hi2c->ErrorCode != HAL_I2C_ERROR_NONE)
    {
        hi2c->State = HAL_I2C_STATE_READY;

        /* Disable Pos bit in I2C CR1 when error occurred in Master/Mem Receive IT Process */
        hi2c->Instance->CR1 &= ~I2C_REG_CR1_POS;

        hal_i2c_error_cb(hi2c);
    }
}


