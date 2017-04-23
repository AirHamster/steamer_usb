#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include "i2c.h"
#define LEDPORT GPIOE
#define LED GPIO15
#define PCA_9532 0xc0
struct iic i2c;
void i2c1_ev_isr(void)
{
	uint32_t reg32 __attribute__((unused));
	/* If writing proceed*/
	if (i2c.w== 1){
	/*Sending address*/
	if ((I2C_SR1(I2C1) & I2C_SR1_SB)){
		i2c_send_data(I2C1, i2c.address & 0xfe);
		return;
		}
	 /*When address sent, send register addr*/
	if ((I2C_SR1(I2C1) & I2C_SR1_ADDR)){
		reg32 = I2C_SR2(I2C1);
		i2c_send_data(I2C1, i2c.reg_addr);
		return;
		}
	/*Send data while lenth is valid*/
	if ((I2C_SR1(I2C1) & I2C_SR1_BTF)){
		i2c_send_data(I2C1, *i2c.data_pointer);
		i2c.data_pointer++;

	/* If all bytes have been sent */
		if (i2c.lenth-- == 0){
			i2c_send_stop(I2C1);
			i2c.busy = 0;
			return;
			}
		}
	}
	/*If reading needed*/
	else if (i2c.r == 1){
	if (i2c.rs == 0){
	/*Start send*/
	if ((I2C_SR1(I2C1) & I2C_SR1_SB)){
		i2c_send_data(I2C1, (i2c.address & 0xfe));
		return;
		}
	 
	 /*When address sent, send register addr*/
	if ((I2C_SR1(I2C1) & I2C_SR1_ADDR)){
		reg32 = I2C_SR2(I2C1);
		i2c_send_data(I2C1, i2c.reg_addr);
		return;
		}
	/*When reg addr send, repeat start	*/
	if ((I2C_SR1(I2C1) & I2C_SR1_BTF) != 0){
		i2c_send_start(I2C1);
		i2c.rs = 1;	//repeated start send
		return;
		}
	}
	/*We set i2c.rs, so now can read data from i2c*/
	else{
		if((I2C_SR1(I2C1) & I2C_SR1_SB)){
			i2c_send_data(I2C1, (i2c.address | 0x01));
			return;
			}
		if ((I2C_SR1(I2C1) & I2C_SR1_ADDR)){
			i2c_enable_ack(I2C1);
			return;
			}
		if ((I2C_SR1(I2C1) & I2C_SR1_BTF)){
			*i2c.data_pointer = i2c_get_data(I2C1);
			i2c.data_pointer++;
			
			if (--i2c.lenth == 1){
				i2c_nack_current(I2C1);
				return;
				}
			}	
		if ((I2C_SR1(I2C1) & I2C_SR1_RxNE) !=0) {
			i2c_send_stop(I2C1);
			i2c.busy = 0;
			}	
	}
}
}

void i2c1_er_isr(void)
{
	/*TODO errors handling*/

 	gpio_set(LEDPORT, LED);
	
}

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

	/*
	 * fclock for I2C is 36MHz APB2 -> cycle time 28ns, low time at 400kHz
	 * incl trise -> Thigh = 1600ns; CCR = tlow/tcycle = 0x1C,9;
	 * Datasheet suggests 0x1e.
	 */
	i2c_set_ccr(I2C1, 0xb4);

	/*
	 * fclock for I2C is 36MHz -> cycle time 28ns, rise time for
	 * 400kHz => 300ns and 100kHz => 1000ns; 1000ns/28ns = 36;
	 * Incremented by 1 -> 37.
	 */
	i2c_set_trise(I2C1, 0x25);

	/* nvic_enable_irq(NVIC_I2C1_EV_IRQ); */
	/* nvic_enable_irq(NVIC_I2C1_ER_IRQ); */
	/* i2c_enable_interrupt(I2C1, I2C_CR2_ITEVTEN); */
	/* i2c_enable_interrupt(I2C1, I2C_CR2_ITERREN); */
	/* If everything is configured -> enable the peripheral. */
	i2c_peripheral_enable(I2C1);
}


/* void i2c1_write(uint8_t address, uint8_t reg_addr, uint8_t *data, uint8_t lenth)
 * {
 *         i2c.busy = 1;
 *         i2c.r = 0;
 *         i2c.w = 1;
 *         i2c.reg_addr = reg_addr;
 *         i2c.address = address;
 *         i2c.data_pointer = data;
 *         i2c.lenth = lenth;
 *         i2c_send_start(I2C1);
 * }
 * 
 * void i2c1_read(uint8_t address, uint8_t reg_addr, uint8_t *data, uint8_t lenth)
 * {
 *         i2c.busy = 1;
 *         i2c.r = 1;
 *         i2c.w = 0;
 *         i2c.reg_addr = reg_addr;
 *         i2c.address = address;
 *         i2c.data_pointer = data;
 *         i2c.lenth = lenth;
 *         i2c_send_start(I2C1);
 * }  */

uint8_t i2c1_read(uint32_t i2c, uint8_t address, uint8_t reg_addr)
{
	i2c_disable_interrupt(I2C1, I2C_CR2_ITEVTEN);
	i2c_disable_interrupt(I2C1, I2C_CR2_ITERREN);
	uint32_t reg32 __attribute__((unused));
	uint8_t result;

	/* Send START condition. */
	i2c_send_start(i2c);

	/* Waiting for START is send and switched to master mode. */
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	/* Say to what address we want to talk to. */
	/* Yes, WRITE is correct - for selecting register in STTS75. */
	//i2c_send_7bit_address(i2c, address, I2C_WRITE);

	i2c_send_data(i2c, address & 0xfe);
	/* Waiting for address is transferred. */
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));

	/* Cleaning ADDR condition sequence. */
	reg32 = I2C_SR2(i2c);

	i2c_send_data(i2c, reg_addr); /* temperature register */
	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)));

	/*
	 * Now we transferred that we want to ACCESS the temperature register.
	 * Now we send another START condition (repeated START) and then
	 * transfer the destination but with flag READ.
	 */

	/* Send START condition. */
	i2c_send_start(i2c);

	/* Waiting for START is send and switched to master mode. */
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	/* Say to what address we want to talk to. */
	/* i2c_send_7bit_address(i2c, address, I2C_READ);  */

	i2c_send_data(i2c, address | 0x01);
	/* 2-byte receive is a special case. See datasheet POS bit. */
	//I2C_CR1(i2c) |= (I2C_CR1_POS | I2C_CR1_ACK);

	/* Waiting for address is transferred. */
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));

	/* Cleaning ADDR condition sequence. */
	reg32 = I2C_SR2(i2c);

	/* Cleaning I2C_SR1_ACK. */
	I2C_CR1(i2c) &= ~I2C_CR1_ACK;

	/* Now the slave should begin to send us the first byte. Await BTF. */
	while (!(I2C_SR1(i2c) & I2C_SR1_BTF));
	//temperature = (uint16_t)(I2C_DR(i2c) << 8); /* MSB */

	/*
	 * Yes they mean it: we have to generate the STOP condition before
	 * saving the 1st byte.
	 */
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
	//i2c_send_7bit_address(i2c, sensor, I2C_WRITE);
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
