/*
 * can_drv_esp32.h
 *
 *  Created on: May 4, 2022
 *      Author: refo
 */

#ifndef _DRIVERS_CAN_DRV_ESP32_H_
#define _DRIVERS_CAN_DRV_ESP32_H_

#define CANDRV_LOG_NONE 0
#define CANDRV_LOG_INFO 1
#define CANDRV_LOG_DEBUG 2
#define CANDRV_LOG_VERBOSE 3

#define CANDRV_LOG CANDRV_LOG_VERBOSE

#if CANDRV_LOG >= CANDRV_LOG_INFO
	#define candrv_logi(__fmt, ...) printf(__fmt, ## __VA_ARGS__)
#else
	#define candrv_logi(__fmt, ...)
#endif

#if CANDRV_LOG >= CANDRV_LOG_DEBUG
	#define candrv_logd(__fmt, ...) printf(__fmt, ## __VA_ARGS__)
#else
	#define candrv_logd(__fmt, ...)
#endif

#if CANDRV_LOG == CANDRV_LOG_VERBOSE
	#define candrv_logv(__fmt, ...) printf(__fmt, ## __VA_ARGS__)
#else
	#define candrv_logv(__fmt, ...)
#endif

typedef enum {
	CAN_ID_STANDARD = 0, // Standard ID 11 bit
	CAN_ID_EXTENDED = 1  // Extended ID 29 bit
} can_id_type_t;

typedef union {
	uint8_t u8[8];
	uint16_t u16[4];
	uint32_t u32[2];
	uint64_t u64;
} can_frame_data_t;

typedef struct __attribute__((packed)) {
	union {
		uint32_t fir;
		struct {
			uint32_t dlc:4;	//number of data bytes in message
			uint32_t :2;	//reserved
			uint32_t rtr:1;	//Remote Transmission Request
			uint32_t idt:1;	//ID type(Frame Format): 0: 11bit or 1: 29 bit
			uint32_t :24;
		};
	};
	uint32_t id;			//CAN ID
	union {
		uint8_t data_u8[8];
		can_frame_data_t data;
	};
} can_frame_esp32_t;

int can_drv_esp32_init(	const twai_general_config_t *g_config,
						const twai_timing_config_t *t_config,
						const twai_filter_config_t *f_config);
void can_drv_esp32_start(void);
void can_drv_esp32_stop(void);
esp_err_t can_drv_esp32_tx(can_frame_esp32_t *tx_frame);
esp_err_t can_drv_esp32_rx(can_frame_esp32_t *rx_frame, TickType_t ticks_to_wait);
esp_err_t can_drv_esp32_wait_tx_end(TickType_t ticks_to_wait);

#endif /* _DRIVERS_CAN_DRV_ESP32_H_ */
