/* C Standard Library headers */
#include <stdlib.h>
#include <stdio.h>

/* Board headers */
#include "init_mcu.h"
#include "init_board.h"
#include "init_app.h"
#include "ble-configuration.h"
#include "board_features.h"
#include "retargetserial.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"
#include <gecko_configuration.h>
#include "mesh_generic_model_capi_types.h"
#include "mesh_lib.h"
#include <mesh_sizes.h>

/* Libraries containing default Gecko configuration values */
#include "em_device.h"
#include "em_emu.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_crypto.h"
#include "mbedtls_config.h"
#include "aes_alt.h"

/* Device initialization header */
#include "hal-config.h"

#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif

/* LCD libraries */
#include "LCD/graphics.h"
#include "LCD/lcd_driver.h"

/* Mesh Data Struct */
#include "mesh_data.h"

/* Bluetooth stack heap */
#define MAX_CONNECTIONS 2

uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS)
		+ BTMESH_HEAP_SIZE + 1760];

/* Bluetooth advertisement set configuration
 ** At minimum the following is required:
 ** One advertisement set for Bluetooth LE stack (handle number 0)
 ** One advertisement set for Mesh data (handle number 1)
 ** One advertisement set for Mesh unprovisioned beacons (handle number 2)
 ** One advertisement set for Mesh unprovisioned URI (handle number 3)
 ** N advertisement sets for Mesh GATT service advertisements
 ** (one for each network key, handle numbers 4 .. N+3)
 **/

#define MAX_ADVERTISERS (4 + MESH_CFG_MAX_NETKEYS)

static gecko_bluetooth_ll_priorities linklayer_priorities =
		GECKO_BLUETOOTH_PRIORITIES_DEFAULT;

/* Bluetooth stack configuration */
extern const struct bg_gattdb_def bg_gattdb_data;

/* Flag for indicating DFU Reset must be performed */
uint8_t boot_to_dfu = 0;

const gecko_configuration_t config = { .sleep.flags = 0,
		.bluetooth.max_connections = MAX_CONNECTIONS,
		.bluetooth.max_advertisers = MAX_ADVERTISERS, .bluetooth.heap =
				bluetooth_stack_heap, .bluetooth.heap_size =
				sizeof(bluetooth_stack_heap) - BTMESH_HEAP_SIZE,
		.bluetooth.sleep_clock_accuracy = 100, .bluetooth.linklayer_priorities =
				&linklayer_priorities, .gattdb = &bg_gattdb_data,
		.btmesh_heap_size = BTMESH_HEAP_SIZE,
#if (HAL_PA_ENABLE)
		.pa.config_enable = 1, // Set this to be a valid PA config
#if defined(FEATURE_PA_INPUT_FROM_VBAT)
		.pa.input = GECKO_RADIO_PA_INPUT_VBAT, // Configure PA input to VBAT
#else
		.pa.input = GECKO_RADIO_PA_INPUT_DCDC,
#endif // defined(FEATURE_PA_INPUT_FROM_VBAT)
#endif // (HAL_PA_ENABLE)
		.max_timers = 16, };

/* Define header for graphic LCD */
#define LCD_HEADER 		             "\nFIRMESH TEAM\nBLE MESH THESIS\n******************\n"
#define LCD_HEADER_SIZE 	         (sizeof(LCD_HEADER))

/* System reset mode */
#define SYSTEM_NORMAL_RESET        0
#define SYSTEM_OTA_DFU_MODE        2

/* Define clock frequency */
#define TIMER_CLOCK_FREQ           (uint32) 32768

#define TIMER_REPEAT               0
#define TIMER_NON_REPEAT           1
#define TIMER_ID_REMOVE            0

#define TIME_RESTART               (uint32) (2 * TIMER_CLOCK_FREQ)
#define TIME_FACTORY_RESET         (uint32) (2 * TIMER_CLOCK_FREQ)
#define TIME_ADVERTISE_DATA        (uint32) (3 * TIMER_CLOCK_FREQ)
#define TIME_CHECK_HEALTH          (uint32) (30 * TIMER_CLOCK_FREQ)
#define TIME_BLINK_LED             (uint32) (TIMER_CLOCK_FREQ / 2)
#define TIME_CHANGE_DATA           (uint32) (30 * TIMER_CLOCK_FREQ)

/* Timer handlers defines */
#define TIMER_ID_RESTART          78
#define TIMER_ID_FACTORY_RESET    77
#define TIMER_ID_ADVERTISE_DATA   75
#define TIMER_ID_CHECK_HEALTH     70
#define TIMER_ID_BLINK_LED        10
#define TIMER_ID_CHANGE_DATA      20

/* Define for encryption and decryption */
#define ENCRYPT_MODE 1
#define DECRYPT_MODE 0

/* Data payload when advertising must be 16 byte */
#define ENCRYPT_DATA_LENGTH 16
#define ENCRYPT_KEY_LENGTH  16
#define ENCRYPT_KEY_BITS 128UL

/* Define advertising parameters */
#define ADVERTISING_HANDLE 0
#define ADVERTISING_PACKET 0

#define INTERVAL_MIN 160
#define INTERVAL_MAX 160
#define ADVERTISING_DURATION 30
#define ADVERTISING_MAX_EVENT 3

/* Define Reset Button Pin */
#define RESET_BUTTON_PORT          (gpioPortF)
#define RESET_BUTTON_PIN           (6U)

/* Define Led Pin */
#define LED_PORT                   (gpioPortF)
#define LED_PIN                    (4U)

/* Define Max Model in all elements */
#define MAX_MODEL                  4

#define MAX_LOST_PACKET            5

/* Define key */
#define NET_KEY_INDEX              0
#define APP_KEY_INDEX              0

/* Define bearer to advertise beacon data */
#define ADV_BEARER                 0x01
#define GATT_BEARER                0x02

#define RECEIVE_ARRAY 0
#define STATUS_ARRAY 1

/* Global Variable */
/* Transaction identifier */
static uint8 transaction_id = 0;
/* Number of active Bluetooth connections */
static uint8 num_connections = 0;
/* Handle of the last opened LE connection */
static uint8 connection_handle = 0xFF;
/* Primary element for temperature */
static uint16 primary_element = 0;

static uint16 mode = 0;
static uint16 num_lpn = 0;
static uint16 gateway_address = 1;
advertising_data_t advertising_data;
mesh_receive_array_t mesh_receive_array;
mesh_status_array_t mesh_status_array;

mesh_data_t* lpn_data_array;
mesh_status_t* lpn_status_array;

mesh_receive_data_t mesh_receive_data_default;
mesh_status_t mesh_status_default;

//Variable for AES Test
mbedtls_aes_context aes_context;
const unsigned char aes_default_key[ENCRYPT_KEY_LENGTH] = { 0xff, 0x00, 0xff,
		0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff,
		0x00 };

/* User function */
void gpio_init();
void lcd_init();
void stack_init();

void advertising_data_init();
void mesh_data_init();
void aes_encryption_init();

void push_data(mesh_receive_data_t receive_data, mesh_status_t status_data,
		uint8 array_type);
uint16 find_id_by_address(uint16 unicast_address, uint8 array_type);
void set_device_name(bd_addr *pAddr);
void factory_reset();
void gateway_node_init();

unsigned char * aes_encypt_data(unsigned char input_data[ENCRYPT_DATA_LENGTH]);
void parse_advertising_data();
void send_advertising_data();

void level_model_request(uint16_t model_id, uint16_t element_index,
		uint16_t client_addr, uint16_t server_addr, uint16_t appkey_index,
		const struct mesh_generic_request *request, uint32_t transition_ms,
		uint16_t delay_ms, uint8_t request_flags);
void level_model_change(uint16_t model_id, uint16_t element_index,
		const struct mesh_generic_state *current,
		const struct mesh_generic_state *target, uint32_t remaining_ms);

bool mesh_bgapi_listener(struct gecko_cmd_packet *evt);
static void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt);

int main() {
	/* Initialize device */
	initMcu();
	/* Initialize board */
	initBoard();
	/* Initialize application */
	initApp();
	/* Init Stack */
	stack_init();
	/* Init retarget serial to use printf function */
	RETARGET_SerialInit();
	/* Init led and button pin */
	gpio_init();
	/* Init LCD */
	lcd_init();
	/* Init advertising data packet */
	advertising_data_init();
	/* Init array use to store data received from Mesh Network */
	mesh_data_init();
	/* Init AES 128 bits */
	aes_encryption_init();

	while (1) {
		struct gecko_cmd_packet *evt = gecko_wait_event();
		bool pass = mesh_bgapi_listener(evt);
		if (pass) {
			handle_gecko_event(BGLIB_MSG_ID(evt->header), evt);
		}
	}
}

void gpio_init() {
	GPIO_PinModeSet(LED_PORT, LED_PIN, gpioModePushPull, 0);

	/* For Factory Reset Function */
	GPIO_PinModeSet(RESET_BUTTON_PORT, RESET_BUTTON_PIN, gpioModeInputPull, 1);
}

void lcd_init() {
	/* Print to console */
	printf("\r\n \r\n \r\n \r\n");
	printf("------------------------------\r\n");
	printf("Welcome to FIRMESH TEAM\r\n");
	printf("GATEWAY NODE\r\n");
	printf("------------------------------\r\n");

	/* Initialize header of graphics LCD */
	char header_buffer[LCD_HEADER_SIZE + 1];
	snprintf(header_buffer, LCD_HEADER_SIZE, LCD_HEADER);
	LCD_init(header_buffer);
}

void stack_init() {
	/* Minimize advertisement latency by allowing the advertiser to always interrupt the scanner. */
	linklayer_priorities.scan_max = linklayer_priorities.adv_min + 1;

	gecko_stack_init(&config);

	gecko_bgapi_class_dfu_init();
	gecko_bgapi_class_system_init();
	gecko_bgapi_class_le_gap_init();
	gecko_bgapi_class_le_connection_init();
	gecko_bgapi_class_gatt_server_init();
	gecko_bgapi_class_hardware_init();
	gecko_bgapi_class_flash_init();

	gecko_bgapi_class_mesh_node_init();
	gecko_bgapi_class_mesh_proxy_init();
	gecko_bgapi_class_mesh_proxy_server_init();
	gecko_bgapi_class_mesh_generic_server_init();
	gecko_bgapi_class_mesh_generic_client_init();
	gecko_bgapi_class_mesh_friend_init();
	gecko_initCoexHAL();
}

void advertising_data_init() {
	advertising_data.flags_len = 0x02;
	advertising_data.flags_type = 0x01;
	advertising_data.flags = 0x04 | 0x02;
	advertising_data.manu_data_len = 19;
	advertising_data.manu_data_type = 0xff;
	advertising_data.comp_id_low = 0xff;
	advertising_data.comp_id_high = 0x02;
	memset(advertising_data.data_payload, 0, 16);
}

void mesh_data_init() {
	mesh_receive_array.receive_data = (mesh_receive_data_t *) malloc(
			sizeof(mesh_receive_data_t) * DEFAULT_ARRAY_SIZE);
	if (mesh_receive_array.receive_data == NULL) {
		printf("Out of memory !!! \r\n");
		return;
	}
	mesh_receive_array.max_elements = DEFAULT_ARRAY_SIZE;
	mesh_receive_array.current_elements = 0;

	mesh_status_array.mesh_status = (mesh_status_t *) malloc(
			sizeof(mesh_status_t) * DEFAULT_ARRAY_SIZE);
	if (mesh_status_array.mesh_status == NULL) {
		printf("Out of memory !!! \r\n");
		return;
	}
	mesh_status_array.max_elements = DEFAULT_ARRAY_SIZE;
	mesh_status_array.current_elements = 0;
	//
	lpn_data_array = (mesh_data_t*) malloc(
			sizeof(mesh_data_t) * (MESH_CFG_MAX_FRIENDSHIPS + 1));
	lpn_data_array[0].unicast_address = gateway_address;
	//mesh_data_array[0].unicast_address = gateway_address;
	lpn_data_array[0].heart_beat = 1;

	lpn_status_array = (mesh_status_t *) malloc(
			sizeof(mesh_status_t) * (MESH_CFG_MAX_FRIENDSHIPS + 1));
	if (lpn_status_array == NULL) {
		printf("Out of memory !!! \r\n");
		return;
	}
	lpn_status_array[0].packet_count = 0;
	lpn_status_array[0].unicast_address = gateway_address;
}

void aes_encryption_init() {
	int result;
	mbedtls_aes_init(&aes_context);
	result = mbedtls_aes_setkey_enc(&aes_context, aes_default_key,
			ENCRYPT_KEY_BITS);
	if (result == 0) {
		printf("Set encrypt key successfully !!! \r\n");
	} else {
		printf("Invalid encrypt key !!! \r\n");
	}
}

void push_data(mesh_receive_data_t receive_data, mesh_status_t status_data,
		uint8 array_type) {
	if (array_type == RECEIVE_ARRAY) {
		if (mesh_receive_array.current_elements
				>= mesh_receive_array.max_elements) {
			mesh_receive_array.receive_data = (mesh_receive_data_t *) realloc(
					mesh_receive_array.receive_data,
					sizeof(mesh_receive_data_t)
							* (mesh_receive_array.max_elements
									+ DEFAULT_ARRAY_SIZE));
			if (mesh_receive_array.receive_data == NULL) {
				printf("Out of memory !!! \r\n");
				return;
			}
			mesh_receive_array.max_elements += DEFAULT_ARRAY_SIZE;
		}
		mesh_receive_array.receive_data[mesh_receive_array.current_elements] =
				receive_data;
		mesh_receive_array.current_elements++;
	} else if (array_type == STATUS_ARRAY) {
		if (mesh_status_array.current_elements
				>= mesh_status_array.max_elements) {
			mesh_status_array.mesh_status = (mesh_status_t *) realloc(
					mesh_status_array.mesh_status,
					sizeof(mesh_status_t)
							* (mesh_receive_array.max_elements
									+ DEFAULT_ARRAY_SIZE));
			if (mesh_status_array.mesh_status == NULL) {
				printf("Out of memory !!! \r\n");
				return;
			}
			mesh_status_array.max_elements += DEFAULT_ARRAY_SIZE;
		}
		mesh_status_array.mesh_status[mesh_status_array.current_elements] =
				status_data;
		mesh_status_array.current_elements++;
	}
}

uint16 find_id_by_address(uint16 unicast_address, uint8 array_type) {
	uint16 i;

	if (array_type == RECEIVE_ARRAY) {
		for (i = 0; i < mesh_receive_array.current_elements; i++) {
			if (mesh_receive_array.receive_data[i].unicast_address
					== unicast_address) {
				return i;
			}
		}
		/* If cannot find unicast address in mesh_receive_array */
		/* return max_elements, it is a null element in mesh_receive_array.receive_data */
		return mesh_receive_array.max_elements;
	} else if (array_type == STATUS_ARRAY) {
		for (i = 0; i < mesh_status_array.current_elements; i++) {
			if (mesh_status_array.mesh_status[i].unicast_address
					== unicast_address) {
				return i;
			}
		}
	}
	/* If cannot find unicast address in mesh_status_array */
	/* return max_elements, it is a null element in mesh_status_array.mesh_status */
	return mesh_status_array.max_elements;
}

void set_device_name(bd_addr *pAddr) {
	char name[20];
	sprintf(name, "Address: %02x:%02x", pAddr->addr[1], pAddr->addr[0]);
	printf("Bluetooth Mesh Device Address: %02x:%02x:%02x:%02x:%02x:%02x\r\n",
			pAddr->addr[5], pAddr->addr[4], pAddr->addr[3], pAddr->addr[2],
			pAddr->addr[1], pAddr->addr[0]);

	LCD_write(name, LCD_ROW_NAME);
}

void factory_reset() {
	LCD_write("*******", LCD_ROW_INFO);
	LCD_write("FACTORY RESET", LCD_ROW_INFO + 1);
	LCD_write("*******", LCD_ROW_INFO + 2);
	LCD_write("", LCD_ROW_INFO + 3);

	printf("***********************************************\r\n");
	printf("*************FACTORY RESET*********************\r\n");
	printf("***********************************************\r\n");

	/* Clear all data in PS store */
	gecko_cmd_flash_ps_erase_all();

	/* Start Reset after 2 seconds */
	gecko_cmd_hardware_set_soft_timer(TIME_FACTORY_RESET,
	TIMER_ID_FACTORY_RESET,
	TIMER_NON_REPEAT);
}

void gateway_node_init() {
	errorcode_t error_code;
	uint16 result;

	/* Up to 4 models in all elements !!! */
	mesh_lib_init(malloc, free, MAX_MODEL);

	primary_element = 0;

	/* Init data for Gateway Node */
	mesh_data_t gateway_data = { 0, 0, 1, 50 };

	/* Add gateway_data to first element in mesh_receive_array */
	mesh_receive_data_t gateway_init_data = { 0, 0 };

	printf("Initialize friend function !!! \r\n");

	result = gecko_cmd_mesh_friend_init()->result;
	if (result) {
		printf("Friend init failed !!! \r\n");
	}
	/* Get unicast address of primary element */
	struct gecko_msg_mesh_node_get_element_address_rsp_t *element_address;

	element_address = gecko_cmd_mesh_node_get_element_address(primary_element);
	if (element_address->result == 0) {
		gateway_data.unicast_address = element_address->address;
		printf("Primary Element, Unicast address = %d \r\n",
				gateway_data.unicast_address);
	} else {
		printf("Get Unicast address from Promary element failed !!! \r\n");
	}

	gateway_init_data.unicast_address = gateway_data.unicast_address;
	gateway_init_data.data = set_mesh_data(gateway_data);

	/* Add init data to mesh_receive_array */
	push_data(gateway_init_data, mesh_status_default, RECEIVE_ARRAY);

	error_code = mesh_lib_generic_server_register_handler(
			MESH_GENERIC_LEVEL_SERVER_MODEL_ID, primary_element,
			level_model_request, level_model_change);
	printf("Register handler for primary element !!! \r\n");
	if (error_code == bg_err_success) {
		printf("Register handler successfully !!! \r\n");
	} else {
		if (error_code == bg_err_wrong_state) {
			printf("Handler is already exist !!! \r\n");
		}
		if (error_code == bg_err_out_of_memory) {
			printf("Register handler failed: Out of memory !!! \r\n");
		}
	}

	/* Start timer use to check status of Friend Nodes */
	result = gecko_cmd_hardware_set_soft_timer(TIME_CHECK_HEALTH,
			TIMER_ID_CHECK_HEALTH, TIMER_REPEAT)->result;
	if (result) {
		printf("Start timer for check Friend Nodes status failed !!!\r\n");
	}

	/* Start timer user to send advertising data to Dongle and Smart Phone */
	result = gecko_cmd_hardware_set_soft_timer(TIME_ADVERTISE_DATA,
			TIMER_ID_ADVERTISE_DATA, TIMER_REPEAT)->result;
	if (result) {
		printf("Start timer for adveetise data failed !!! \r\n");
	}
}

unsigned char * aes_encypt_data(unsigned char input_data[ENCRYPT_DATA_LENGTH]) {
	int result;
	static unsigned char encrypted_data[ENCRYPT_DATA_LENGTH];
	result = mbedtls_aes_crypt_ecb(&aes_context, ENCRYPT_MODE, input_data,
			encrypted_data);
	if (result == 0) {
		printf("Encryption process is succesful !!! \r\n");
	} else {
		printf("Encryption process is fail !!! \r\n");
	}
	return encrypted_data;
}

void send_mesh_data(uint16 element_index, uint16 unicast_address) {
	uint16 resp;
	uint32_t transition_ms = 0;
	uint16_t delay_ms = 0;
	struct mesh_generic_request req;

	printf("***********************\r\n");
	printf("Send Mesh Data Function \r\n");

	req.kind = mesh_generic_request_level;
	req.level = 0xffff;

	/* Increase transaction_id after each packet sent with non - retransmition */
	transaction_id++;

	resp = mesh_lib_generic_client_set(MESH_GENERIC_LEVEL_CLIENT_MODEL_ID,
			element_index, unicast_address, 0, transaction_id, &req,
			transition_ms, delay_ms, 0x00);

	if (resp) {
		printf("Send Mesh data failed !!! \r\n");
	} else {
		printf("Mesh data sent !!! \r\n");
	}
	printf("***********************\r\n");
}

void parse_advertising_data() {
	uint16 i;
	if (mesh_receive_array.current_elements >= DEFAULT_ARRAY_SIZE) {
		printf("Not support for the number of node great than 8 !!! \r\n");
		printf("It will be update soon with application in Android !!! \r\n");
	} else {
		for (i = 0; i < DEFAULT_ARRAY_SIZE; i++) {
			if (i < mesh_receive_array.current_elements) {
				advertising_data.data_payload[2 * i] =
						(uint8) ((mesh_receive_array.receive_data[i].data) >> 8);
				advertising_data.data_payload[2 * i + 1] =
						(uint8) mesh_receive_array.receive_data[i].data;
			} else {
				advertising_data.data_payload[2 * i] = 0;
				advertising_data.data_payload[2 * i + 1] = 0;
			}
		}
	}

	/* Print data payload to check if it is true */
	printf("Data send to Dongle and Smart Phone !!! \r\n");
	for (i = 0; i < 16; i++) {
		printf("%x    ", advertising_data.data_payload[i]);
	}
}

void send_advertising_data() {
	/* Convert data to byte array */
	parse_advertising_data();

	uint8 data_len = sizeof(advertising_data);
	uint8 *data_send = (uint8 *) (&advertising_data);

	/* Set 0 dBm Transmit Power */
	gecko_cmd_system_set_tx_power(0);

	/* Set custom advertising data */
	gecko_cmd_le_gap_bt5_set_adv_data(ADVERTISING_HANDLE, ADVERTISING_PACKET,
			data_len, data_send);
	/**************************************/

	/* Set advertising parameters:
	 *   + 100ms advertisement interval
	 *   + Duration = 300 ms
	 *   + Max event = 3
	 */
	gecko_cmd_le_gap_set_advertise_timing(ADVERTISING_HANDLE,
	INTERVAL_MIN,
	INTERVAL_MAX,
	ADVERTISING_DURATION,
	ADVERTISING_MAX_EVENT);

	/* Start advertising in user mode and enable connections */
	gecko_cmd_le_gap_start_advertising(ADVERTISING_HANDLE, le_gap_user_data,
			le_gap_non_connectable);
}

void level_model_request(uint16_t model_id, uint16_t element_index,
		uint16_t client_addr, uint16_t server_addr, uint16_t appkey_index,
		const struct mesh_generic_request *request, uint32_t transition_ms,
		uint16_t delay_ms, uint8_t request_flags) {
	uint16 id_in_array = 0;
	uint16 level_model_data = 0;

	printf("Level Model Request Function !!! \r\n");

	if (request->kind != mesh_generic_request_level) {
		printf(
				"The type of received data is not belong to level model !!! \r\n");
		return;
	}

	/* Get data from client */
	level_model_data = request->level;
	if (mode == 0) {
		uint8 i = 0;
		for (; i <= num_lpn; i++) {
			if (lpn_status_array[i].unicast_address == client_addr) {
				lpn_status_array[i].packet_count = 0;
				if (i != 0) {
					struct mesh_generic_request req;
					req.kind = mesh_generic_request_level;
					req.level = level_model_data
							& ((lpn_data_array[i].heart_beat & 0x1) << 8);
					uint16 resp = mesh_lib_generic_client_set(
					MESH_GENERIC_LEVEL_CLIENT_MODEL_ID, element_index,
							gateway_address,
							APP_KEY_INDEX, transaction_id, &req, transition_ms,
							delay_ms, 0);

					if (resp) {
						printf("Send Mesh data failed !!! \r\n");
					} else {
						printf("Mesh data sent %x!!! \r\n", req.level);
					}
				}
			} else {
				gecko_cmd_mesh_friend_deinit();
				int8 index = 1;
				for (; index <= MESH_CFG_MAX_FRIENDSHIPS; index++) {
					lpn_status_array[index].packet_count = 0;
					lpn_status_array[index].unicast_address = 0;
				}
				num_lpn = 0;
				mode = 1;
			}
		}
	}
	if (mode == 1) {
		/* Just primary element of Friend Node request response */
		if (request_flags & MESH_REQUEST_FLAG_RESPONSE_REQUIRED) {
			printf("Receive data from Primary Element of Friend Node !!! \r\n");
			id_in_array = find_id_by_address(client_addr, STATUS_ARRAY);
			if (id_in_array != mesh_status_array.max_elements) {
				mesh_status_array.mesh_status[id_in_array].packet_count = 0;
			}
			/* New node */
			else {
				mesh_status_t new_friend_node = { client_addr, 0 };
				push_data(mesh_receive_data_default, new_friend_node,
						STATUS_ARRAY);

				/* Data of Friend Node do not almost change in life-time */
				/* Just have heart beat will be change, it will be update in other function */
				mesh_receive_data_t new_node = { client_addr, level_model_data };
				push_data(new_node, mesh_status_default, RECEIVE_ARRAY);
			}

			/* Response to client */
			send_mesh_data(element_index, client_addr);
		}
		/* Data is received from secondary elements of Friend Node */
		/* It is LPN data */
		else {
			id_in_array = find_id_by_address(client_addr, RECEIVE_ARRAY);
			/* New Node */
			if (id_in_array == mesh_receive_array.max_elements) {
				mesh_receive_data_t new_node = { client_addr, level_model_data };
				push_data(new_node, mesh_status_default, RECEIVE_ARRAY);
			} else {
				/* Update LPN data */
				mesh_receive_array.receive_data[id_in_array].data =
						level_model_data;
			}
		}
	}
}
void level_model_change(uint16_t model_id, uint16_t element_index,
		const struct mesh_generic_state *current,
		const struct mesh_generic_state *target, uint32_t remaining_ms) {
}

static void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt) {
	uint16 i;
	uint16 result;
	/* Buffer containt LCD data */
	char buffer[30];

	if (evt == NULL) {
		return;
	}

	switch (evt_id) {
	case gecko_evt_system_boot_id:
		if (GPIO_PinInGet(RESET_BUTTON_PORT, RESET_BUTTON_PIN) == 0) {
			factory_reset();
		} else {
			struct gecko_msg_system_get_bt_address_rsp_t *pAddr =
					gecko_cmd_system_get_bt_address();

			/* Print MAC address to LCD and console */
			set_device_name(&pAddr->address);

			/*Init Mesh Stack,  it will generate initialized event */
			result = gecko_cmd_mesh_node_init()->result;
			if (result) {
				sprintf(buffer, "Init Failed");
				printf("Bluetooth Mesh Node Init Failed !!!!!!\r\n");
				LCD_write(buffer, LCD_ROW_ERR);
			}
		}
		break;
	case gecko_evt_hardware_soft_timer_id:
		switch (evt->data.evt_hardware_soft_timer.handle) {
		case TIMER_ID_FACTORY_RESET:
			gecko_cmd_system_reset(SYSTEM_NORMAL_RESET);
			break;

		case TIMER_ID_RESTART:
			gecko_cmd_system_reset(SYSTEM_NORMAL_RESET);
			break;

		case TIMER_ID_BLINK_LED:
			GPIO_PinOutToggle(LED_PORT, LED_PIN);
			break;

		case TIMER_ID_CHECK_HEALTH:
			if (mode == 1) {
				for (i = 0; i < mesh_status_array.current_elements; i++) {
					mesh_status_array.mesh_status[i].packet_count++;
					/* Just change heart beat data one time when the Friend Node die !!! */
					/* LPN Node after 5 minutes find Friend Node will send data directly to Gateway Node */
					/* This data will be stored in array, when Friend Node reactive, this is not necessary !!! */
					uint16 id = find_id_by_address(
							mesh_status_array.mesh_status[i].unicast_address,
							RECEIVE_ARRAY);
					if (id == mesh_receive_array.max_elements) {
						printf("Error in push node process !!! \r\n");
					}
					if (mesh_status_array.mesh_status[i].packet_count
							>= MAX_LOST_PACKET) {
						/* Set heart beat to 0 */
						mesh_receive_array.receive_data[id].data &= 0xfeff;
					} else {
						/* Set heart beat to 1 */
						mesh_receive_array.receive_data[id].data |= 0x0100;
					}
				}
			}
			if (mode == 0) {
				printf("CHECK HEALTH\r\n");
				//check gateway
				lpn_status_array[0].packet_count++;
				if (lpn_status_array[0].packet_count >= MAX_LOST_PACKET) {
					//gateway_address = 2;
					gecko_cmd_mesh_friend_deinit();
					uint8 index = 1;
					for (; index <= MESH_CFG_MAX_FRIENDSHIPS; index++) {
						lpn_status_array[index].packet_count = 0;
						lpn_status_array[index].unicast_address = 0;

					}
					num_lpn = 0;
					mode = 1;
				}
				//send_mesh_data(FLAG_RESPONSE, FLAG_NON_RETRANS, 0);
				uint16 index = 1;
				for (; index <= num_lpn; index++) {
					lpn_status_array[index].packet_count++;
					if (lpn_status_array[index].packet_count >= MAX_LOST_PACKET) {
						lpn_data_array[index].heart_beat = 0;
					}
					//send_mesh_data(FLAG_NON_RESPONSE, FLAG_NON_RETRANS, index);
					struct mesh_generic_request req;
					req.kind = mesh_generic_request_level;
					req.level = set_mesh_data(lpn_data_array[index]);
					uint16 resp = mesh_lib_generic_client_set(
					MESH_GENERIC_LEVEL_CLIENT_MODEL_ID, index, gateway_address,
					APP_KEY_INDEX, transaction_id, &req, 0, 0, 0);

					if (resp) {
						printf("Send Mesh data failed !!! \r\n");
					} else {
						printf("Mesh data sent %x!!! \r\n", req.level);
					}
				}
			}
			break;

		case TIMER_ID_ADVERTISE_DATA:
			send_advertising_data();
			break;

		default:
			break;
		}
		break;

	case gecko_evt_mesh_node_initialized_id:
		printf("Node initialized !!! \r\n");
		result = gecko_cmd_mesh_generic_server_init()->result;
		if (result) {
			printf("Generic Server Init failed !!! \r\n");
		}
		result = gecko_cmd_mesh_generic_client_init()->result;
		if (result) {
			printf("Init Generic Client failed !!! \r\n");
		}

		if (!evt->data.evt_mesh_node_initialized.provisioned) {
			LCD_write("Unprovisioned !!!", LCD_ROW_INFO);
			printf("Device Unprovisioned !!!!!!\r\n");

			/* The Node is now initialized, start unprovisioned Beaconing using PB-ADV and PB-GATT Bearers */
			gecko_cmd_mesh_node_start_unprov_beaconing(
					ADV_BEARER | GATT_BEARER);
		} else {
			gateway_node_init();

			LCD_write("Provisioned !!!", LCD_ROW_INFO);
			printf("Device Provisioned !!!!!!\r\n");
		}
		break;

	case gecko_evt_mesh_friend_friendship_established_id:
		LCD_write("FRIEND", LCD_ROW_FRIEND_INFOR);
		printf("Event gecko_evt_mesh_friend_friendship_established !!! \r\n");
		num_lpn++;
		printf("num_lpn %d \r\n", num_lpn);
		uint16 new_friendship_address =
				evt->data.evt_mesh_friend_friendship_established.lpn_address;
		if (num_lpn <= MESH_CFG_MAX_FRIENDSHIPS) {
			//uint16 id_in_array = find_id_by_address(new_friendship_address, STATUS_ARRAY);
			lpn_status_array[num_lpn].unicast_address = new_friendship_address;
			lpn_status_array[num_lpn].packet_count = 0;
			lpn_data_array[num_lpn].unicast_address = new_friendship_address;
			lpn_data_array[num_lpn].heart_beat = 1;
			lpn_data_array[num_lpn].battery_percent = 100;
			lpn_data_array[num_lpn].alarm_signal = 0;
			printf("%d\t%d\r\n", lpn_status_array[num_lpn].unicast_address,
					lpn_status_array[num_lpn].packet_count);
		} else {
			printf("Max number of friendship was established");
		}
		//printf("LPN stats:%d\t %d\t%d\r\n", lpn_status_arr[num_lpn].address, lpn_status_arr[num_lpn].timeOut);
		break;

	case gecko_evt_mesh_friend_friendship_terminated_id:
		printf("Event gecko_evt_mesh_friend_friendship_terminated !!!\r\n");
		LCD_write("NO LPN", LCD_ROW_FRIEND_INFOR);
		gecko_cmd_mesh_friend_deinit();
		//clear_lpn_status_arr(lpn_status_arr, num_lpn);
		uint8 index = 1;
		for (; index <= MESH_CFG_MAX_FRIENDSHIPS; index++) {
			lpn_status_array[index].packet_count = 0;
			lpn_status_array[index].unicast_address = 0;
		}
		num_lpn = 0;
		gecko_cmd_mesh_friend_init();
		break;

	case gecko_evt_mesh_node_provisioning_started_id:
		LCD_write("Provisioning...", LCD_ROW_INFO);
		printf("Provisioning Process......... \r\n");

		gecko_cmd_hardware_set_soft_timer(TIME_BLINK_LED,
		TIMER_ID_BLINK_LED,
		TIMER_REPEAT);
		break;

	case gecko_evt_mesh_node_provisioned_id:
		gateway_node_init();

		LCD_write("Provisioned !!!", LCD_ROW_INFO);
		printf("Device Provisioned !!!!!!\r\n");

		/* Stop blink led */
		gecko_cmd_hardware_set_soft_timer(TIMER_ID_REMOVE,
		TIMER_ID_BLINK_LED,
		TIMER_NON_REPEAT);
		GPIO_PinOutClear(LED_PORT, LED_PIN);
		break;

	case gecko_evt_mesh_node_provisioning_failed_id:
		LCD_write("Prov Failed !!!", LCD_ROW_INFO);
		printf("Provisioning Process Failed !!!!!!\r\n");

		/* Restart after 2 seconds */
		gecko_cmd_hardware_set_soft_timer(TIME_RESTART,
		TIMER_ID_RESTART,
		TIMER_NON_REPEAT);
		break;

	case gecko_evt_mesh_generic_server_client_request_id:
		printf("***************************************** \r\n");
		printf("Receive request command from Friend Node !!! \r\n");
		printf("Model ID:: %d \r\n",
				evt->data.evt_mesh_generic_server_client_request.model_id);
		printf("Element Index:: %d \r\n",
				evt->data.evt_mesh_generic_server_client_request.elem_index);
		printf("Client Address:: %d \r\n",
				evt->data.evt_mesh_generic_server_client_request.client_address);

		mesh_lib_generic_server_event_handler(evt);
		break;

	case gecko_evt_mesh_generic_server_state_changed_id:
		printf("Server state changed !!! \r\n");
		printf("Do not have process function for this event !!! \r\n");
		mesh_lib_generic_server_event_handler(evt);
		break;

	case gecko_evt_mesh_node_key_added_id:
		printf("Key added event occurs !!! \r\n");
		break;

	case gecko_evt_mesh_node_model_config_changed_id:
		printf("Config change event occurs !!! \r\n");
		break;

	case gecko_evt_mesh_node_reset_id:
		printf("Event gecko_evt_mesh_node_reset_id !!! \r\n");
		factory_reset();
		break;

	case gecko_evt_le_connection_opened_id:
		printf("Open BLE connection !!! \r\n");
		num_connections++;
		connection_handle = evt->data.evt_le_connection_opened.connection;
		LCD_write("Connected !!!", LCD_ROW_CONNECTION);
		break;

	case gecko_evt_le_connection_closed_id:
		if (boot_to_dfu) {
			gecko_cmd_system_reset(SYSTEM_OTA_DFU_MODE);
		}

		printf("Close BLE connection !!! \r\n");
		connection_handle = 0xFF;
		if (num_connections > 0) {
			if (--num_connections == 0) {
				LCD_write("", LCD_ROW_CONNECTION);
			}
		}
		break;

	case gecko_evt_le_connection_parameters_id:
		printf("BLE connection parameter: interval %d, timeout %d \r\n",
				evt->data.evt_le_connection_parameters.interval,
				evt->data.evt_le_connection_parameters.timeout);
		break;

	case gecko_evt_gatt_server_user_write_request_id:
		if (evt->data.evt_gatt_server_user_write_request.characteristic
				== gattdb_ota_control) {
			boot_to_dfu = 1;

			gecko_cmd_gatt_server_send_user_write_response(
					evt->data.evt_gatt_server_user_write_request.connection,
					gattdb_ota_control, bg_err_success);

			gecko_cmd_le_connection_close(
					evt->data.evt_gatt_server_user_write_request.connection);
		}
		break;

	default:
		break;
	}
}
