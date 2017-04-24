
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/cm3/scb.h>
#include "i2c.h"

#define LEDPORT GPIOE
#define LED GPIO15
#define PCA_9532 0xc0
#define CONTROL_REG 0x12
#define ENDPOINT1 0x82
#define ENDPOINT2 0x83
static int tr_round = 0;

/*
 * stdlib _write callback
 */
int _write(int file, char *ptr, int len);

static const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0xFF,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x0483,
	.idProduct = 0x5740,
	.bcdDevice = 0x0200,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

/*
 * Bulk endpoint descriptions
 */
static const struct usb_endpoint_descriptor bulk_endp[] = {{
  .bLength = USB_DT_ENDPOINT_SIZE,
  .bDescriptorType = USB_DT_ENDPOINT,
  .bEndpointAddress = 0x1,
  .bmAttributes = USB_ENDPOINT_ATTR_BULK,
  .wMaxPacketSize = 64,
  .bInterval = 1,
}, {
  .bLength = USB_DT_ENDPOINT_SIZE,
  .bDescriptorType = USB_DT_ENDPOINT,
  .bEndpointAddress = ENDPOINT1,
  .bmAttributes = USB_ENDPOINT_ATTR_BULK,
  .wMaxPacketSize = 64,
  .bInterval = 1,
}, {
  .bLength = USB_DT_ENDPOINT_SIZE,
  .bDescriptorType = USB_DT_ENDPOINT,
  .bEndpointAddress = ENDPOINT2,
  .bmAttributes = USB_ENDPOINT_ATTR_BULK,
  .wMaxPacketSize = 64,
  .bInterval = 1,
}};  

/*
 * Interrupt endpoint description
 */
static const struct usb_endpoint_descriptor int_endp[] = {{
  .bLength = USB_DT_ENDPOINT_SIZE,
  .bDescriptorType = USB_DT_ENDPOINT,
  .bEndpointAddress = 0x3,
  .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
  .wMaxPacketSize = 1,
  .bInterval = 255,
}};  

/*
 * Bulk interface descriptions
 */
static const struct usb_interface_descriptor bulk_iface[] = {{
  .bLength = USB_DT_INTERFACE_SIZE,
  .bDescriptorType = USB_DT_INTERFACE,
  .bInterfaceNumber = 0,
  .bAlternateSetting = 0,
  .bNumEndpoints = 3,
  .bInterfaceClass = 0xFF,
  .bInterfaceSubClass = 0xFF,
  .bInterfaceProtocol = 0x0,
  .iInterface = 0,

  .endpoint = bulk_endp,
}};

/*
 * Interrupt interface description
 */
static const struct usb_interface_descriptor int_iface[] = {{
  .bLength = USB_DT_INTERFACE_SIZE,
  .bDescriptorType = USB_DT_INTERFACE,
  .bInterfaceNumber = 1,
  .bAlternateSetting = 0,
  .bNumEndpoints = 1,
  .bInterfaceClass = 0xFF,
  .bInterfaceSubClass = 0xFF,
  .bInterfaceProtocol = 0x0,
  .iInterface = 0,

  .endpoint = int_endp,
}};

static const struct usb_interface ifaces[] = {{
  .num_altsetting = 1,
  .altsetting = bulk_iface,
},{
  .num_altsetting = 1,
  .altsetting = int_iface,
}};


/*
 * Usb device configuration
 */
static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 2,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80,
	.bMaxPower = 0x32,

	.interface = ifaces,
};


static const char * usb_strings[] = {
	"STC Metrotek",
	"Vendor-specific demo",
	"DEMO",
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

static int vendorspec_control_request(usbd_device *usbd_dev,
	struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
	void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
  ( void )usbd_dev;
  ( void )req;
  ( void )buf;
  ( void )len;
  ( void )complete;
  return 0;
}

static void ep1_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
  (void)ep;
  int r;
  uint8_t temp;
  uint8_t buf[64];
  char *message;
  message = malloc(64);
  int len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);
  if (len == 2){
	switch (buf[1]){
		case 2:{
		temp = i2c1_read(I2C1, PCA_9532, 2);
  		r = usbd_ep_write_packet( usbd_dev,ENDPOINT1,&temp, 1);
		       }
		       break;
		case 3:{
  		temp = i2c1_read(I2C1, PCA_9532, 3); 
  		r = usbd_ep_write_packet( usbd_dev, ENDPOINT1, &temp, 1);
		       }
		       break;
		case 4:{
  		temp = i2c1_read(I2C1, PCA_9532, 4);
  		r = usbd_ep_write_packet( usbd_dev, ENDPOINT1, &temp, 1);
		       }
		       break;
		case 5:{
  		temp = i2c1_read(I2C1, PCA_9532, 5);
  		r = usbd_ep_write_packet( usbd_dev, ENDPOINT1, &temp, 1);
		       }
		       break;
	}
  }
  if (len == 3){
if (buf[1] >= 6)
	{
		if (buf[2] <= 4)
		{
  			temp = i2c1_read(I2C1, PCA_9532, 6);
			temp = temp >> ((buf[2]-1) * 2);
			temp &= 3;
			message = "LED1";
  usbd_ep_write_packet( usbd_dev, ENDPOINT2, message, 4 );
			  r = usbd_ep_write_packet( usbd_dev, ENDPOINT1, &temp , 1 );
		}else if((buf[2] > 4) & (buf[2] <= 8))
		{
  			temp = i2c1_read(I2C1, PCA_9532, 7);
			temp = temp >> ((buf[2]-5) * 2);
			temp &= 3;
			message = "LED2";
  usbd_ep_write_packet( usbd_dev, ENDPOINT2, message, 4 );
			  r = usbd_ep_write_packet( usbd_dev, ENDPOINT1, &temp , 1 );
		}else if((buf[2] > 8) & (buf[2] <= 12))
		{
  			temp = i2c1_read(I2C1, PCA_9532, 8);
			temp = temp >> ((buf[2]-9) * 2);
			temp &= 3;
			message = "LED3";
  usbd_ep_write_packet( usbd_dev, ENDPOINT2, message, 4 );
			  r = usbd_ep_write_packet( usbd_dev, ENDPOINT1, &temp , 1 );
		}else
		{

		}
		
	}else{
  i2c1_write(I2C1, PCA_9532, buf[1], buf[2]);
  gpio_toggle(LEDPORT, LED);
  temp = i2c1_read(I2C1, PCA_9532, buf[1]);
  r = usbd_ep_write_packet( usbd_dev, ENDPOINT1, &temp , 1);
	}
  }else if (len == 4)
  {
	  if(buf[3] == 0){
	
	buf[1] = buf[1] + (buf[2]-1)/4;
	if (buf[2] == 1 || buf[2] ==5 || buf[2] == 9){
  	buf[2] = i2c1_read(I2C1, PCA_9532, buf[1]);
		buf[2] &= 0xfc;
		buf[2] |= 2;	
	  }else if (buf[2] == 2 || buf[2] == 6 || buf[2] == 10){
  	buf[2] = i2c1_read(I2C1, PCA_9532, buf[1]);
		buf[2] &= 0xf3;
		buf[2] |= 2 << 2;
	}
	else if (buf[2] == 3 || buf[2] == 7 || buf[2] == 11){
  	buf[2] = i2c1_read(I2C1, PCA_9532, buf[1]);
		buf[2] &= 0xcf;
		buf[2] |= 2 << 4;
	}
  	else if (buf[2] == 4 || buf[2] == 8 || buf[2] == 12){
  	buf[2] = i2c1_read(I2C1, PCA_9532, buf[1]);
		buf[2] &= 0x3f;
		buf[2] |= 2 << 6;
	}
	  }else if(buf[3] == 1){

	buf[1] = buf[1] + (buf[2]-1)/4;
	if (buf[2] == 1 || buf[2] ==5 || buf[2] == 9){
  	buf[2] = i2c1_read(I2C1, PCA_9532, buf[1]);
		buf[2] &= 0xfc;
		buf[2] |= 3;	
	}else if (buf[2] == 2 || buf[2] == 6 || buf[2] == 10){
  	buf[2] = i2c1_read(I2C1, PCA_9532, buf[1]);
		buf[2] &= 0xf3;
		buf[2] |= 3 << 2;
	}
	else if (buf[2] == 3 || buf[2] == 7 || buf[2] == 11){
  	buf[2] = i2c1_read(I2C1, PCA_9532, buf[1]);
		buf[2] &= 0xcf;
		buf[2] |= 3 << 4;
	}
  	else if (buf[2] == 4 || buf[2] == 8 || buf[2] == 12){
  	buf[2] = i2c1_read(I2C1, PCA_9532, buf[1]);
		buf[2] &= 0x3f;
		buf[2] |= 3 << 6;
	}
	  }
  i2c1_write(I2C1, PCA_9532, buf[1], buf[2]);
  }
  r = usbd_ep_write_packet( usbd_dev, ENDPOINT2, &message, 2 );
  /* ZLP */
  if( len == 64 )
    usbd_ep_write_packet( usbd_dev, ENDPOINT1, NULL, 0 );

  tr_round++;
}

static void ep82_data_tx_cb(usbd_device *usbd_dev, uint8_t ep)
{
  ( void )ep;
  ( void )usbd_dev;
}

static void ep83_data_tx_cb(usbd_device *usbd_dev, uint8_t ep)
{
  ( void )ep;
  ( void )usbd_dev;
  gpio_toggle( GPIOD, GPIO13 );
  /*       uint8_t temp = 10; */
  /* usbd_ep_write_packet( usbd_dev, ENDPOINT1, &temp , 1 ); */
}


static void ep3_int_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
  ( void )ep;
  ( void )usbd_dev;

  unsigned char buf = 0;
  int len = usbd_ep_read_packet(usbd_dev, 0x03, &buf, sizeof( unsigned char ));
  printf( "int r %d, %c\n", len, buf );
}

static void vendorspec_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
  (void)wValue;
  
  usbd_ep_setup(usbd_dev, 0x1 , USB_ENDPOINT_ATTR_BULK, 64,
                                     ep1_data_rx_cb );
  usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64,
                                     ep82_data_tx_cb );
  usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_BULK, 64,
                                     ep83_data_tx_cb );
  usbd_ep_setup(usbd_dev, 0x3, USB_ENDPOINT_ATTR_INTERRUPT, 1,
                                     ep3_int_rx_cb );

  usbd_register_control_callback(
       usbd_dev,
       USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
       USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
       vendorspec_control_request);
}

static void gpio_setup(void)
{
	/* Enable GPIO clock. */
	rcc_periph_clock_enable(RCC_GPIOE);
	rcc_periph_clock_enable(RCC_GPIOB);

	/* Set GPIO12 (in GPIO port C) to 'output push-pull'. */
	gpio_set_mode(LEDPORT, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, LED);
}

static void rcc_setup(void)
{
	rcc_clock_setup_in_hse_25mhz_out_72mhz();
	/* rcc_set_ppre1(RCC_CFGR_PPRE1_HCLK_DIV2);	 */
}


int main(void)
{
	usbd_device *usbd_dev;
	gpio_setup();
	rcc_setup();
	i2c1_setup();
	
  /* i2c1_write(I2C1, PCA_9532, 7, 0xfe); */
	usbd_dev = usbd_init(&otgfs_usb_driver, &dev, &config,
			usb_strings, 3,usbd_control_buffer, sizeof(usbd_control_buffer));

	usbd_register_set_config_callback(usbd_dev, vendorspec_set_config);
    
    while (1) {
	usbd_poll(usbd_dev);
  }
	
	return 0;
}
