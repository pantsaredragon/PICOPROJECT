#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <inttypes.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "hardware/structs/rosc.h"
#include "pico/cyw43_arch.h"
#include "lwip/apps/mqtt.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "lwip/dns.h"
#include "lwip/altcp_tcp.h"
#include "lwip/altcp_tls.h"
#include "lwip/apps/mqtt_priv.h"
#include "tusb.h"
#include "crypto_consts_example.h"
#include "pico/multicore.h"

/* Code to talk to a ADXL triple-axis accelerometer.
   
   This reads and writes to registers on the board. Sends the data through a LWIP MQTT client to a MQTT Brooker.

   Connections on Raspberry Pi Pico board, other boards may vary.

   GPIO PICO_DEFAULT_I2C_SDA_PIN (On Pico this is GP4 (physical pin 6)) -> SDA on ADXL355 board
   GPIO PICO_DEFAULT_I2C_SCK_PIN (On Pico this is GP5 (physcial pin 7)) -> SCL on ADXL355 board
   3v3 (physical pin 36) -> VDD on ADXL355 board
   GND (physical pin 38) -> GND on ADXl355 board

*/

#define FLAG_VAL 123
#define BUFFER_SIZE 8096
int data_ready = 0;
float payload[BUFFER_SIZE];
u16_t payload_len = 0;

/**------------------------ MQTT CLIENT ------------------------------------*/
#define MQTT_TLS 0
#define _OPEN_SYS_ITOA_EXT

typedef struct MQTT_CLIENT_T_ {
	ip_addr_t remote_addr;
	mqtt_client_t *mqtt_client;
} MQTT_CLIENT_T;

static MQTT_CLIENT_T* mqtt_client_init() {
	MQTT_CLIENT_T *state = calloc(1, sizeof(MQTT_CLIENT_T));
	if (!state) {
		printf("failed to allocate state\n");
		return NULL;
	}
	return state;
}

void dns_found(const char *name, const ip_addr_t *ipaddr, void *callback_arg) {
	MQTT_CLIENT_T *state = (MQTT_CLIENT_T*)callback_arg;
	printf("DNS query finished with resolved addr of %s.\n", ip4addr_ntoa(ipaddr));
	state->remote_addr = *ipaddr;
}

void dns_lookup(MQTT_CLIENT_T *state) {
	printf("Running DNS query for %s.\n", MQTT_SERVER_HOST);

	cyw43_arch_lwip_begin();
	err_t err = dns_gethostbyname(MQTT_SERVER_HOST, &(state->remote_addr), dns_found, state);
	cyw43_arch_lwip_end();

	if (err == ERR_ARG) {
		printf("failed to start DNS query\n");
		return;
	}

	if (err == ERR_OK) {
		printf("no lookup needed\n");
		return;
	}

	while (state->remote_addr.addr == 0) {
		cyw43_arch_poll();
		sleep_ms(1);
	}
}


static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
	MQTT_CLIENT_T *state = (MQTT_CLIENT_T *)arg;
	if (status != 0) {
		printf("Error during connection: err %d.\n", status);
	}
	else {
		printf("MQTT connected.\n");
	}
}

void mqtt_pub_request_cb(void *arg, err_t err) {
	MQTT_CLIENT_T *state = (MQTT_CLIENT_T *)arg;
}

err_t mqttPublish(MQTT_CLIENT_T *state, float buf[], u16_t len)
{
	err_t err;
	u8_t qos = 0; /* 0 1 or 2, see MQTT specification */
	u8_t retain = 0;
	cyw43_arch_lwip_begin();
	err = mqtt_publish(state->mqtt_client, "pico_w/adxl355", buf, len, qos, retain, NULL, state); 
	cyw43_arch_lwip_end();
	return err;
}

err_t mqttConnect(MQTT_CLIENT_T *state) {
	struct mqtt_connect_client_info_t ci;
	err_t err;

	memset(&ci, 0, sizeof(ci));

	ci.client_id = "pico_w";
	ci.client_user = NULL;
	ci.client_pass = NULL;
	ci.keep_alive = 60;
	ci.will_topic = NULL;
	ci.will_msg = NULL;
	ci.will_retain = 0;
	ci.will_qos = 0;

	printf("mqtt_client_connect\n");
	err = mqtt_client_connect(state->mqtt_client, &(state->remote_addr), MQTT_SERVER_PORT, mqtt_connection_cb, state, &ci);

	if (err != ERR_OK) {
		printf("mqtt_connect return %d\n", err);
	}
	printf("ERR_OK\n");
	return err;
}


/**----------------------- ADXL355 --------------------------*/
//Hardware address
const uint8_t ADXL_355_ADDRESS = 0x1D;

//Hardware registers
const u_int8_t REG_TEMP = 0x06;
const uint8_t REG_X = 0x08;
const uint8_t REG_Y = 0x0B;
const uint8_t REG_Z = 0x0E;
const uint8_t REG_RANGE = 0x2C;
const uint8_t REG_POWER_CTL = 0x2D;
const uint8_t REG_FILTER = 0x28;
const uint8_t REG_RESET = 0x2F;

//Measurement modes
const uint8_t STANDBY = 0x01;
const uint8_t MEASUREMENT_MODE = 0x00;

//Measurement ranges
const uint8_t _2g = 0x81;
const uint8_t _4g = 0x82;
const uint8_t _8g = 0x83;
 
float scalefactor_2g = 2.048 / 524288;
float scalefactor_4g = 4.096 / 524288;
float scalefactor_8g = 8.192 / 524288;

float scale_factor = 0;

//Buffer
uint8_t buf[11];
#define bias 1852.0
#define slope -9.05

void set_measuremnet_mode(uint8_t mode) {
	if (mode == 1) {
		buf[0] = REG_POWER_CTL;
		buf[1] = STANDBY;
		i2c_write_blocking(i2c0, ADXL_355_ADDRESS, buf, 2, false);
	}

	else {
		buf[0] = REG_POWER_CTL;
		buf[1] = MEASUREMENT_MODE;
		i2c_write_blocking(i2c0, ADXL_355_ADDRESS, buf, 2, false);
        
	}
}

void set_range(uint8_t range) {
	if (range == _2g) {
		buf[0] = REG_RANGE;
		buf[1] = _2g;
		i2c_write_blocking(i2c0, ADXL_355_ADDRESS, buf, 2, false);
		scale_factor = scalefactor_2g;

	}
	else if (range == _4g) {
		buf[0] = REG_RANGE;
		buf[1] = _4g;
		i2c_write_blocking(i2c0, ADXL_355_ADDRESS, buf, 2, false);
		scale_factor = scalefactor_4g;

	}
	else if (range == _8g) {
		buf[0] = REG_RANGE;
		buf[1] = _8g;
		i2c_write_blocking(i2c0, ADXL_355_ADDRESS, buf, 2, false);
		scale_factor = scalefactor_8g;
	}
}

void filter_off() {
	buf[0] = REG_FILTER;
	buf[1] = 0;
	i2c_write_blocking(i2c0, ADXL_355_ADDRESS, buf, 2, false);
}

float get_acc(int32_t value) {
	
	if (0x80000 & value) {
		return -(0x0100000 - value) * scale_factor;
	}
	else {
		return value*scale_factor;
	}
}

float get_temp(int32_t value)
{
	return (value - bias) / slope + 25.0 ;
}
/**----------------------HELP FUNCTIONS ----------------------*/

void wait_for_usb() {
	while (!tud_cdc_connected()) {
		printf(".");
		sleep_ms(500);
	}
	printf("usb host detected\n");
}
/**------------------------- THREAD -------------------------*/
void thread_entry()
{
	uint32_t g = multicore_fifo_pop_blocking();
	
	if (g != FLAG_VAL)
	{
		printf("Error in thread\n");
	}
	else
	{
		printf("All good in thread\n"); 
	}
	// This example will use I2C0 on the default SDA and SCL pins (4, 5 on a Pico)
	i2c_init(i2c0, 400 * 1000);
	gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
	gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
	gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
	gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
	// Make the I2C pins available to picotool
	bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

	// Reset ADXL355
	buf[0] = REG_RESET;   
	buf[1] = 0x52;
	i2c_write_blocking(i2c0, ADXL_355_ADDRESS, buf, 2, false);
	printf("Reset of adxl355 successful\n");
    
	// Enable standby mode
	set_measuremnet_mode(STANDBY);

	// Set range
	set_range(_2g);

	// Set filter off
	filter_off();

	// Enable active mode
	set_measuremnet_mode(MEASUREMENT_MODE);
    
	float data_buffer[BUFFER_SIZE];
	int sample = 0;
	u16_t bit_counter = 0;
	uint32_t start_time, duration;
	uint32_t period_time = 390; // (us)
	multicore_fifo_push_blocking(FLAG_VAL);
	printf("Reading sensor data\n");
	while (1) 
	{
		start_time = time_us_32();
		
		// Start reading acceleration registers
		i2c_write_blocking(i2c0, ADXL_355_ADDRESS, &REG_TEMP, 1, true);
		i2c_read_blocking(i2c0, ADXL_355_ADDRESS, buf, 11, false);
		data_buffer[sample] = get_temp((buf[0]&15) << 8 | buf[1]); //temp
		data_buffer[sample+1] = get_acc(buf[2] << 12 | buf[3] << 4 | buf[4] >> 4); //x-axis
		data_buffer[sample+2] = get_acc(buf[5] << 12 | buf[6] << 4 | buf[7] >> 4); //y-axis
		data_buffer[sample+3] = get_acc(buf[8] << 12 | buf[9] << 4 | buf[10] >> 4); //z-axis
		
		bit_counter += 16;
		sample+=4;
		
		duration = time_us_32() - start_time;
		
		if (duration < period_time)
		{
			sleep_us(period_time - duration);
			duration = period_time;
		}
		
		// Sampling time
		//data_buffer[sample - 1] = (float)(duration);
		
		if (sample >= 512 && !data_ready) {
		memcpy(payload, data_buffer, bit_counter);
		memset(data_buffer, 0, bit_counter);
		payload_len = bit_counter; 
		bit_counter = 0;
		data_ready = 1;
		sample = 0;
		}	
	}
}

/**---------------------------- MAIN --------------------------*/
int main() {
	stdio_init_all();
	
	wait_for_usb();
	
	//Connect to WIFI
	if (cyw43_arch_init()) {
		printf("failed to initialise\n");
		return 1;
	}
	cyw43_arch_enable_sta_mode();

	if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
		printf("failed to connect\n");
		return 1;
	}
	else {
		printf("Connected\n");
	}
	
	MQTT_CLIENT_T *state = mqtt_client_init();
	dns_lookup(state);
	state->mqtt_client = mqtt_client_new();
	
	if (state->mqtt_client == NULL) {
		printf("Failed to create new mqtt client\n");
		return 1;
	}
	err_t err = mqttConnect(state);
	if (err != ERR_OK)
	{
		printf("MQTT client failed to connect %d\n", err);
		return 1;
	}

	multicore_launch_core1(thread_entry);
	multicore_fifo_push_blocking(FLAG_VAL);
	uint32_t g = multicore_fifo_pop_blocking();
	if (g != FLAG_VAL)
	{
		printf("Error in main\n"); 
	}
	else
	{	
		printf("All good in main\n");
	}
	
	while (1)
	{
		cyw43_arch_poll();
		while (!data_ready) ;
		if (mqtt_client_is_connected(state->mqtt_client))
		{
			cyw43_arch_lwip_begin();
			mqttPublish(state, payload, payload_len);
			cyw43_arch_lwip_end();
			payload_len = 0;
			data_ready = 0;
		}	
	}
}
