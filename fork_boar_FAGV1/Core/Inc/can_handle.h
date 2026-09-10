/*
 * can_handle.h
 *
 *  Created on: Sep 8, 2026
 *      Author: syawxyz
 */

#ifndef INC_CAN_HANDLE_H_
#define INC_CAN_HANDLE_H_

#include "stm32f1xx_hal.h"
#include <stdint.h>

#define CAN_ID_CMD_FORK       0x101U
#define CAN_ID_TLM_FORK        0x200U


/* ---- Timing -------------------------------------------------------------- */
#define CAN_CMD_TIMEOUT_MS    200U
#define CAN_TLM_PERIOD_MS     20U

/* ---- Bit status pada byte 6 frame TLM_VEL -------------------------------- */
#define CAN_STATUS_LIMIT_NOT     	(1U << 0)
#define CAN_STATUS_LIMIT_LEFT  		(1U << 1)
#define CAN_STATUS_FORK_RIGHT    	(1U << 2)
#define CAN_STATUS_CMD_BOTH   		(1U << 3)
#define CAN_STATUS_FAULT         	(1U << 4)

/* ---- Perintah terakhir yang valid, sudah dalam satuan SI ----------------- */
typedef struct {
	uint8_t mode;
	uint32_t tick;
} dock_cmd;

/* ---- Data yang dikirim sebagai telemetri, diisi oleh main.c -------------- */
typedef struct {
	uint8_t state;
	float height;
	uint32_t tick;
} dock_telemetry;

/* ---- Statistik untuk debugging lewat debugger / Live Expressions --------- */
typedef struct {
	uint32_t rx_frames;        /* frame RX yang dikenali dan diproses        */
	uint32_t rx_unknown;       /* frame RX lolos filter tapi ID/DLC tak sesuai */
	uint32_t tx_frames;        /* frame TX yang berhasil masuk mailbox       */
	uint32_t tx_dropped;       /* frame TX dibuang karena mailbox penuh      */
	uint8_t  tec;              /* Transmit Error Counter dari register ESR   */
	uint8_t  rec;              /* Receive Error Counter dari register ESR    */
	uint8_t  bus_off;          /* 1 = controller sedang bus-off              */
} CanStats;

extern dock_cmd dock_receive;
extern dock_telemetry dock_transmit;

HAL_StatusTypeDef can_handle_init(void);
uint8_t can_handle_cmd_vel_timed_out(const dock_cmd *cmd);
void can_handle_send_telemetry(const dock_telemetry *t);
void can_handle_get_command(dock_cmd *out);

#endif /* INC_CAN_HANDLE_H_ */
