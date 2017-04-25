#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include "i2c.h"
#define LEDPORT GPIOE
#define LED GPIO15
#define PCA_9532 0xc0
void i2c1_setup(void)
{
	/* Enable clocks for I2C2 and AFIO. */
	rcc_periph_clock_enable(RCC_I2C1);
	rcc_periph_clock_enable(RCC_AFIO);

	/* Set alternate functions for the SCL and SDA pins of I2C2. */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
			GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
			GPIO_I2C1_SCL | GPIO_I2C1_SDA);

	/* Disable the I2C before changing any configuration. */
	i2c_peripheral_disable(I2C1);

	/* APB1 is running at 36MHz. */
	i2c_set_clock_frequency(I2C1, I2C_CR2_FREQ_36MHZ);

	/* 100KHz - I2C Standart Mode */
	i2c_set_standard_mode(I2C1);

	i2c_set_ccr(I2C1, 0xb4);

	i2c_set_trise(I2C1, 0x25);

	/* If everything is configured -> enable the peripheral. */
	i2c_peripheral_enable(I2C1);
}


uint8_t i2c1_read(uint32_t i2c, uint8_t address, uint8_t reg_addr)
{
	uint32_t reg32 __attribute__((unused));
	uint8_t result;

	/* Send START condition. */
	i2c_send_start(i2c);

	/* Waiting for START is send and switched to master mode. */
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
				& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	/* Say to what address we want to talk to. */

	i2c_send_data(i2c, address & 0xfe);
	/* Waiting for address is transferred. */
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));

	/* Cleaning ADDR condition sequence. */
	reg32 = I2C_SR2(i2c);

	i2c_send_data(i2c, reg_addr); 
	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)));

	/* Send START condition. */
	i2c_send_start(i2c);

	/* Waiting for START is send and switched to master mode. */
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
				& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	/* Say to what address we want to talk to. */
	i2c_send_data(i2c, address | 0x01);

	/* Waiting for address is transferred. */
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));

	/* Cleaning ADDR condition sequence. */
	reg32 = I2C_SR2(i2c);

	/* Cleaning I2C_SR1_ACK. */
	I2C_CR1(i2c) &= ~I2C_CR1_ACK;

	/* Now the slave should begin to send us the first byte. Await BTF. */
	while (!(I2C_SR1(i2c) & I2C_SR1_BTF));

	I2C_CR1(i2c) |= I2C_CR1_STOP;

	result |= I2C_DR(i2c); /* LSB */

	/* Original state. */
	I2C_CR1(i2c) &= ~I2C_CR1_POS;

	/* i2c_enable_interrupt(I2C1, I2C_CR2_ITEVTEN); */
	/* i2c_enable_interrupt(I2C1, I2C_CR2_ITERREN); */
	return result;
}

void i2c1_write(uint32_t i2c, uint8_t address, uint8_t reg_adr, uint8_t data )
{
	uint32_t reg32 __attribute__((unused));

	/* Send START condition. */
	i2c_send_start(i2c);

	/* Waiting for START is send and switched to master mode. */
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
				& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	/* Send destination address. */
	i2c_send_data(i2c, address & 0xfe);
	/* Waiting for address is transferred. */
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));

	/* Cleaning ADDR condition sequence. */
	reg32 = I2C_SR2(i2c);

	/* Sending the data. */
	i2c_send_data(i2c, reg_adr); /*  register */
	while (!(I2C_SR1(i2c) & I2C_SR1_BTF));
	i2c_send_data(i2c, data);
	/* After the last byte we have to wait for TxE too. */
	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)));

	/* Send STOP condition. */
	i2c_send_stop(i2c);
}
