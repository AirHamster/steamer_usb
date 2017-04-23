
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
struct pca_regs
{
	uint8_t INPUT0;
	uint8_t INPUT1;
	uint8_t PSC0;
	uint8_t PWM0;
	uint8_t PSC1;
	uint8_t PWM1;
	uint8_t LS0;
	uint8_t LS1;
	uint8_t LS2;
	uint8_t LS3;
} led_driver;

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
  .bEndpointAddress = 0x82,
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
  .bNumEndpoints = 2,
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
  uint8_t temp;
  uint8_t buf[64];

  printf( "---------- Round %d ----------\n", tr_round );
  int len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);
  printf( "%d bytes received\n", len );
if (buf[2] >= 6)
	{
  		temp = i2c1_read(I2C1, PCA_9532, buf[1]);
		
	}
  i2c1_write(I2C1, PCA_9532, buf[1], buf[2]);
  gpio_toggle(LEDPORT, LED);
  buf[2] = i2c1_read(I2C1, PCA_9532, buf[1]);
  int r = usbd_ep_write_packet( usbd_dev, 0x82, buf, len );
  printf( "Send back: %d bytes\n", r );

  /* ZLP */
  if( len == 64 )
    usbd_ep_write_packet( usbd_dev, 0x82, NULL, 0 );

  tr_round++;
}

static void ep82_data_tx_cb(usbd_device *usbd_dev, uint8_t ep)
{
  ( void )ep;
  ( void )usbd_dev;
}


static void ep3_int_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
  ( void )ep;
  ( void )usbd_dev;

  unsigned char buf = 0;
  int len = usbd_ep_read_packet(usbd_dev, 0x03, &buf, sizeof( unsigned char ));
  gpio_toggle( GPIOD, GPIO13 );
  printf( "int r %d, %c\n", len, buf );
}

static void vendorspec_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
  (void)wValue;
  
  usbd_ep_setup(usbd_dev, 0x1 , USB_ENDPOINT_ATTR_BULK, 64,
                                     ep1_data_rx_cb );
  usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64,
                                     ep82_data_tx_cb );
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
	led_driver.PSC0 = 100; // 1.5Hz
	led_driver.PWM0 = 84;	// 33%
	led_driver.PSC1 = 100;
	led_driver.PWM1 = 64;	//25%
	led_driver.LS0 = 0x15;
	led_driver.LS1 = 0xa0;
	led_driver.LS2 = 0xfe;
	led_driver.LS3 = 0;
	
	gpio_setup();
	rcc_setup();
	i2c1_setup();
	
  i2c1_write(I2C1, PCA_9532, 7, 0xfe);
	/* i2c1_write(PCA_9532, CONTROL_REG, &led_driver.PSC0, 8); */
			
	usbd_dev = usbd_init(&otgfs_usb_driver, &dev, &config,
			usb_strings, 3,usbd_control_buffer, sizeof(usbd_control_buffer));

	usbd_register_set_config_callback(usbd_dev, vendorspec_set_config);
    
    while (1) {
	usbd_poll(usbd_dev);
  }
	
	return 0;
}
