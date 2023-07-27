/* #include <stdio.h> */
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "pico/time.h"
#include "tusb.h"
#include "device/usbd_pvt.h"

#define ICM20948_SCL 17
#define ICM20948_SDA 16
#define BUTTON_LAUNCH 6
#define BUTTON_LEFT_FLIPPER 10
#define BUTTON_RIGHT_FLIPPER 15

typedef union {
	uint8_t value;
	struct {
		unsigned up:1;
		unsigned down:1;
		unsigned left:1;
		unsigned right:1;
		unsigned start:1;
		unsigned back:1;
		unsigned left_thumb:1;
		unsigned right_thumb:1;
	} buttons;
} digital_buttons_0_t;

typedef union {
	uint8_t value;
	struct {
		unsigned left_shoulder:1;
		unsigned right_shoulder:1;
		unsigned unused:2;
		unsigned a:1;
		unsigned b:1;
		unsigned x:1;
		unsigned y:1;
	} buttons;
} digital_buttons_1_t;

typedef struct __attribute((packed, aligned(1))) {
	uint8_t report_id;
	uint8_t report_size;
	uint8_t buttons_0;
	uint8_t buttons_1;
	uint8_t lt;
	uint8_t rt;
	int16_t l_x;
	int16_t l_y;
	int16_t r_x;
	int16_t r_y;
	uint8_t reserved_1[6];
} xinput_report_data_t;

static xinput_report_data_t xinput_report_data;
static digital_buttons_0_t buttons_0;
static digital_buttons_1_t buttons_1;
static volatile bool left_tilt = false;
static volatile bool right_tilt = false;
static volatile bool up_tilt = false;
static uint8_t endpoint_in = 0;
static uint8_t endpoint_out = 0;
static const int icm20948_address = 0x69;

static inline uint32_t board_millis(void) {
	return to_ms_since_boot(get_absolute_time());
}

static void update_buttons()
{
	/* Reset all button states */
	buttons_0.value = 0;
	buttons_1.value = 0;

	/* Get the new states */
	buttons_1.buttons.a = !gpio_get(BUTTON_LAUNCH);
	buttons_1.buttons.left_shoulder = !gpio_get(BUTTON_LEFT_FLIPPER);
	buttons_1.buttons.right_shoulder = !gpio_get(BUTTON_RIGHT_FLIPPER);

	/* Copy states to report data */
	xinput_report_data.buttons_0 = buttons_0.value;
	xinput_report_data.buttons_1 = buttons_1.value;
}

static void update_left_stick()
{
	static uint32_t max_hold_count = 50000;
	static uint32_t hold_x_count = 0;
	static uint32_t hold_y_count = 0;

	if (!hold_x_count) {
		xinput_report_data.l_x = 0;
	}

	if (!hold_y_count) {
		xinput_report_data.l_y = 0;
	}

	if (left_tilt) {
		xinput_report_data.l_x = INT16_MIN;
		hold_x_count = max_hold_count;
		left_tilt = false;
	}

	if (right_tilt) {
		xinput_report_data.l_x = INT16_MAX;
		hold_x_count = max_hold_count;
		right_tilt = false;
	}

	if (up_tilt) {
		xinput_report_data.l_y = INT16_MAX;
		hold_y_count = max_hold_count;
		up_tilt = false;
	}

	if (hold_x_count > 0) {
		hold_x_count--;
	}

	if (hold_y_count > 0) {
		hold_y_count--;
	}
}

static void send_report_data(void) {
	/* Poll every 1ms */
	const uint32_t interval_ms = 1;
	static uint32_t start_ms = 0;

	if (board_millis() - start_ms < interval_ms) {
		return;  // Not enough time
	}

	start_ms += interval_ms;

	/* Remote wakeup */
	if (tud_suspended()) {
		tud_remote_wakeup();
	}

	xinput_report_data.report_size = 20;

	if ((tud_ready()) && ((endpoint_in != 0)) && (!usbd_edpt_busy(0, endpoint_in))) {
		usbd_edpt_claim(0, endpoint_in);
		usbd_edpt_xfer(0, endpoint_in, (uint8_t *)&xinput_report_data, 20);
		usbd_edpt_release(0, endpoint_in);
	}
}

static void xinput_init(void) {

}

static void xinput_reset(uint8_t __unused rhport) {

}

static uint16_t xinput_open(uint8_t __unused rhport, tusb_desc_interface_t const *itf_desc, uint16_t max_len) {
	/* +16 is for the unknown descriptor */
	uint16_t const drv_len = sizeof(tusb_desc_interface_t) + itf_desc->bNumEndpoints*sizeof(tusb_desc_endpoint_t) + 16;
	TU_VERIFY(max_len >= drv_len, 0);

	uint8_t const * p_desc = tu_desc_next(itf_desc);
	uint8_t found_endpoints = 0;

	while ((found_endpoints < itf_desc->bNumEndpoints) && (drv_len <= max_len)) {
		tusb_desc_endpoint_t const * desc_ep = (tusb_desc_endpoint_t const *) p_desc;

		if (TUSB_DESC_ENDPOINT == tu_desc_type(desc_ep)) {
			TU_ASSERT(usbd_edpt_open(rhport, desc_ep));

			if (tu_edpt_dir(desc_ep->bEndpointAddress) == TUSB_DIR_IN) {
				endpoint_in = desc_ep->bEndpointAddress;
			} else {
				endpoint_out = desc_ep->bEndpointAddress;
			}

			found_endpoints += 1;
		}

		p_desc = tu_desc_next(p_desc);
	}

	return drv_len;
}

/* Callback after xfer_transfer */
static bool xinput_xfer_cb(uint8_t __unused rhport, uint8_t __unused ep_addr, xfer_result_t __unused result, uint32_t __unused xferred_bytes) {
	return true;
}

static usbd_class_driver_t xinput_driver = {
#if CFG_TUSB_DEBUG >= 2
	.name = "XINPUT",
#endif
	.init             = xinput_init,
	.reset            = xinput_reset,
	.open             = xinput_open,
	.xfer_cb          = xinput_xfer_cb,
	.sof              = NULL
};

/* Implement callback to add our custom driver */
usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count) {
	*driver_count = 1;

	return &xinput_driver;
}

static void icm20948_reset() {
	uint8_t buf[] = {0x06, 0b11000001};

	i2c_write_blocking(i2c_default, icm20948_address, buf, 2, false);

	sleep_ms(100);

	buf[0] = 0x06;
	buf[1] = 0b00000001;
	i2c_write_blocking(i2c_default, icm20948_address, buf, 2, false);

	while (buf[0]) {
		i2c_read_blocking(i2c_default, icm20948_address, buf, 1, false);
	}

	/* Set bank 2 */
	buf[0] = 0x7F;
	buf[1] = 0b00100000;
	i2c_write_blocking(i2c_default, icm20948_address, buf, 2, false);

	/* Set accelerometer to +-16g */
	buf[0] = 0x14;
	buf[1] = 0b00001111;
	i2c_write_blocking(i2c_default, icm20948_address, buf, 2, false);

	/* Set bank 0 */
	buf[0] = 0x7F;
	buf[1] = 0b00000000;
	i2c_write_blocking(i2c_default, icm20948_address, buf, 2, false);
}

static void read_accelerometer(int16_t data[3])
{
	uint8_t buffer[6] = {0};
	uint8_t register_address = 0x2D;
	i2c_write_blocking(i2c_default, icm20948_address, &register_address, 1, true);
	i2c_read_blocking(i2c_default, icm20948_address, buffer, 6, false);

	data[0] = buffer[0] << 8 | buffer[1];
	data[1] = buffer[2] << 8 | buffer[3];
	data[2] = buffer[4] << 8 | buffer[5];
}

void core1_main(void)
{
	i2c_init(i2c_default, 400 * 1000);
	gpio_set_function(ICM20948_SDA, GPIO_FUNC_I2C);
	gpio_set_function(ICM20948_SCL, GPIO_FUNC_I2C);
	gpio_pull_up(ICM20948_SDA);
	gpio_pull_up(ICM20948_SCL);
	bi_decl(bi_2pins_with_func(ICM20948_SDA, ICM20948_SCL, GPIO_FUNC_I2C));

	icm20948_reset();

	/* Read WHO_AM_I register (addr 0x00) */
	uint8_t register_address = 0x00;
	uint8_t buffer[1];

	i2c_write_blocking(i2c_default, icm20948_address, &register_address, 1, true);
	i2c_read_blocking(i2c_default, icm20948_address, buffer, 1, false);

	printf("WHO_AM_I: 0x%02X\r\n", buffer[0]);

	int16_t acceleration[3];

	while (true) {
		left_tilt = false;
		right_tilt = false;
		up_tilt = false;

		read_accelerometer(acceleration);

		if (acceleration[0] < -8000 && !left_tilt) {
			left_tilt = true;
			sleep_ms(100);
		}

		if (acceleration[0] > 8000 && !right_tilt) {
			right_tilt = true;
			sleep_ms(100);
		}

		if ((acceleration[1] < -2500 || acceleration[1] > 2500) && !up_tilt) {
			up_tilt = true;
			sleep_ms(250);
		}
	}
}

int main() {
	stdio_init_all();

	gpio_init(BUTTON_LAUNCH);
	gpio_set_dir(BUTTON_LAUNCH, GPIO_IN);
	gpio_pull_up(BUTTON_LAUNCH);

	gpio_init(BUTTON_LEFT_FLIPPER);
	gpio_set_dir(BUTTON_LEFT_FLIPPER, GPIO_IN);
	gpio_pull_up(BUTTON_LEFT_FLIPPER);

	gpio_init(BUTTON_RIGHT_FLIPPER);
	gpio_set_dir(BUTTON_RIGHT_FLIPPER, GPIO_IN);
	gpio_pull_up(BUTTON_RIGHT_FLIPPER);

	multicore_launch_core1(&core1_main);

	tusb_init();

	while (true) {
		update_buttons();
		update_left_stick();
		tud_task();
		send_report_data();
	}

	return 0;
}
