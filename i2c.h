#include <stdint.h>
#ifndef I2C_
#define I2C_

//void i2c1_er_isr(void);
//void i2c1_ev_isr(void);
// void i2c1_read(uint8_t address, uint8_t reg_addr, uint8_t *data, uint8_t lenth);
uint8_t i2c1_read(uint32_t i2c, uint8_t address, uint8_t reg_addr);
void i2c1_write(uint32_t i2c, uint8_t address, uint8_t reg_adr, uint8_t data );
// void i2c1_write(uint8_t address, uint8_t reg_addr, uint8_t *data, uint8_t lenth);
void i2c1_setup(void);

struct iic
{
	uint8_t address;
	uint8_t reg_addr; 
	uint8_t lenth;
	uint8_t *data_pointer;
	uint8_t busy;
	unsigned int r:1;
	unsigned int w:1;
	unsigned int rs:1;
};
#endif
