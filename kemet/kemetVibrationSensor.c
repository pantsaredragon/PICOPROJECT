#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <inttypes.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/structs/rosc.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/uart.h"
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
/**-----------------------------------------------------------------------------------*/
/* Code to talk to the kemet accelerometer.
   
   This reads the analog signal from the accelerometer and sends the data through a LWIP MQTT client to a MQTT Brooker.

   Connections on Raspberry Pi Pico board, other boards may vary.

   GPIO PICO_DEFAULT_I2C_SDA_PIN (On Pico this is GP4 (physical pin 6)) -> SDA on ADXL355 board
   GPIO PICO_DEFAULT_I2C_SCK_PIN (On Pico this is GP5 (physcial pin 7)) -> SCL on ADXL355 board
   3v3 (physical pin 36) -> VDD on ADXL355 board
   GND (physical pin 38) -> GND on ADXl355 board

*/

#define BUFFER_SIZE 8096

#define FLAG_VAL 123
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
		printf("Connection callback: MQTT connected.\n");
	}
}

void mqtt_pub_request_cb(void *arg, err_t err) {
	MQTT_CLIENT_T *state = (MQTT_CLIENT_T *)arg;
}

err_t mqttPublish(MQTT_CLIENT_T *state, float buf[],u16_t len)
{ 
	err_t err;
	u8_t qos = 0; /* 0 1 or 2, see MQTT specification */
	u8_t retain = 0;
	cyw43_arch_lwip_begin();
	err = mqtt_publish(state->mqtt_client, "pico_w/kemet", buf, len, qos, retain, NULL, state); 
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
	return err;
}

/**----------------------HELP FUNCTIONS ----------------------*/

void wait_for_usb() {
	while (!tud_cdc_connected()) {
		printf(".");
		sleep_ms(500);
	}
	printf("usb host detected\n");
}

/**---------------------- Thread ----------------------------*/
void thread_entry()
{
	uint32_t g = multicore_fifo_pop_blocking();
	
	if (g != FLAG_VAL)
	{
		printf("Error in thread\n");
	}
	else
	{
		printf("Thread is active\n"); 
	}
	
	#define ADC_VREF 3.3
	#define ADC_RANGE (1 << 12)
	#define ADC_CONVERT (ADC_VREF / (ADC_RANGE - 1))
	#define HALF_ADC_VREF (ADC_VREF/2)
	
	#define ADC_NUM_ACC 0
	#define ADC_PIN_ACC (26 + ADC_NUM_ACC)
	#define SCALE_FACTOR (0.17*9.81)
	
	#define ADC_NUM_MIC 1
	#define ADC_PIN_MIC (26 + ADC_NUM_MIC)
	
	bi_decl(bi_program_description("Analog microphone example for Raspberry Pi Pico")); // for picotool
	bi_decl(bi_1pin_with_name(ADC_PIN_ACC, "ADC input pin"));

	adc_init();
	adc_gpio_init(ADC_PIN_ACC);
	adc_gpio_init(ADC_PIN_MIC); 
	
	float data_buffer[BUFFER_SIZE];
	uint32_t period_time = 22; // (us)
	int sample = 0;
	u16_t bit_counter = 0;
	uint32_t duration, start_time;
	
	multicore_fifo_push_blocking(FLAG_VAL);
	
	printf("Reading sensor data\n");
	
	while (1) 
	{
		start_time = time_us_32();
		// Read from accelerometer
		adc_select_input(ADC_NUM_ACC);
		data_buffer[sample] = (float)((adc_read()*ADC_CONVERT - HALF_ADC_VREF) / SCALE_FACTOR);
		
		// Read from microphone
		adc_select_input(ADC_NUM_MIC);
		data_buffer[sample+1] = (float)(adc_read()*ADC_CONVERT - HALF_ADC_VREF);
		
		bit_counter += 12;
		sample += 3;
		
		duration = time_us_32() - start_time;
	
		if (duration < period_time)
		{
			sleep_us(period_time - duration);
			duration = period_time;
		}
		
		//Timestamp
		data_buffer[sample - 1] = (float)(duration);
		
		if (sample >= 1536  && !data_ready) 
		{
			memcpy(payload, data_buffer, bit_counter); 
			memset(data_buffer, 0, bit_counter);
			payload_len = bit_counter;
			bit_counter = 0;
			sample = 0;
			data_ready = 1;
		}
	}
}

/**---------------------------- MAIN --------------------------*/
int main() {
	stdio_init_all();
	
	wait_for_usb();
	
	if (cyw43_arch_init()) 
	{
		printf("failed to initialise\n");
		return 1;
	}
	
	cyw43_arch_enable_sta_mode();

	if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) 
	{
		printf("failed to connect\n");
		return 1;
	}
	else 
	{
		printf("Connected\n");
	}
	
	MQTT_CLIENT_T *state = mqtt_client_init();
	dns_lookup(state);
	state->mqtt_client = mqtt_client_new();
	
	if (state->mqtt_client == NULL) 
	{
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
		printf("Main is active\n");
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
