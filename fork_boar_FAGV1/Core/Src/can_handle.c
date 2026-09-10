/*
 * can_handle.c
 *
 *  Created on: Sep 8, 2026
 *      Author: syawxyz
 */

#include "can_handle.h"   /* API dan definisi protokol modul ini                      */
#include "can.h"          /* handle hcan1 yang dibuat CubeMX                           */
#include <string.h>       /* memset untuk mengosongkan struct  */

static volatile CanStats stats;

static uint8_t tlm_seq;

dock_cmd dock_receive;
dock_telemetry dock_transmit;

static volatile struct {
	uint8_t  mode_raw;     /* byte mode apa adanya                                */
	uint32_t tick_raw;
} rx_raw;


/* =========================================================================
 * Helper konversi byte <-> integer (little-endian eksplisit)
 * ========================================================================= */

/* Ambil int16 little-endian dari dua byte: byte[0] rendah, byte[1] tinggi. */

//static int16_t get_i16(const uint8_t *p)
//{
//	return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));   /* gabung lalu tafsirkan sebagai signed */
//}

/* Tulis int16 ke dua byte little-endian.                                   */
static void put_i16(uint8_t *p, int16_t v)
{
	p[0] = (uint8_t)((uint16_t)v & 0xFFU);          /* byte rendah                        */
	p[1] = (uint8_t)(((uint16_t)v >> 8) & 0xFFU);   /* byte tinggi                        */
}

/* Tulis int32 ke empat byte little-endian.                                 */
static void put_i32(uint8_t *p, int32_t v)
{
	uint32_t u = (uint32_t)v;                       /* geser bit pada unsigned, hindari UB */
	p[0] = (uint8_t)(u & 0xFFU);                    /* byte 0, paling rendah              */
	p[1] = (uint8_t)((u >> 8) & 0xFFU);             /* byte 1                             */
	p[2] = (uint8_t)((u >> 16) & 0xFFU);            /* byte 2                             */
	p[3] = (uint8_t)((u >> 24) & 0xFFU);            /* byte 3, paling tinggi              */
}

/* Konversi float ke int16 dengan saturasi, supaya nilai besar tidak wrap.  */
static int16_t f_to_i16_sat(float x)
{
	if (x >=  32767.0f) return  32767;              /* batas atas int16                   */
	if (x <= -32768.0f) return -32768;              /* batas bawah int16                  */
	return (int16_t)x;                              /* aman dikonversi                    */
}

/* Konversi float ke int32 dengan saturasi.                                 */
//static int32_t f_to_i32_sat(float x)
//{
//	if (x >=  2147483647.0f) return  INT32_MAX;     /* batas atas int32                   */
//	if (x <= -2147483648.0f) return  INT32_MIN;     /* batas bawah int32                  */
//	return (int32_t)x;                              /* aman dikonversi                    */
//}

/* =========================================================================
 * Inisialisasi
 * ========================================================================= */

HAL_StatusTypeDef can_handle_init(void)
{
	CAN_FilterTypeDef f;                                 /* struktur konfigurasi filter HAL      */
	HAL_StatusTypeDef st;                                /* status tiap panggilan HAL            */

	memset((void *)&dock_receive, 0, sizeof(dock_receive));          /* kosongkan buffer RX                  */
	memset((void *)&stats, 0, sizeof(stats));            /* kosongkan statistik                  */
	tlm_seq = 0U;                                        /* mulai nomor urut dari nol            */

	/* Filter bank 0, mode ID LIST, skala 16-bit: satu bank memuat 4 ID standar.
	 * Format register 16-bit: STDID[10:0] di bit 15..5, RTR bit 4, IDE bit 3.
	 * Hanya CMD_VEL dan CMD_FORK yang lolos; frame lain ditolak oleh hardware. */
	f.FilterBank           = 0;                                  /* bank 0 dari 0..13 milik CAN1  */
	f.FilterMode           = CAN_FILTERMODE_IDLIST;              /* cocokkan ID persis, bukan mask */
	f.FilterScale          = CAN_FILTERSCALE_16BIT;              /* empat slot ID 16-bit per bank  */
	f.FilterIdHigh         = 0;
	f.FilterIdLow          = 0;
	f.FilterMaskIdHigh     = 0;
	f.FilterMaskIdLow      = 0;
	f.FilterFIFOAssignment = CAN_RX_FIFO0;                       /* frame lolos masuk FIFO0        */
	f.FilterActivation     = ENABLE;                             /* aktifkan bank ini              */
	f.SlaveStartFilterBank = 14;                                 /* bank 14..27 untuk CAN2, tak dipakai */

	st = HAL_CAN_ConfigFilter(&hcan, &f);               /* tulis konfigurasi filter ke hardware */
	if (st != HAL_OK) return st;                         /* gagal: laporkan, jangan lanjut       */

	st = HAL_CAN_Start(&hcan);                          /* keluar dari init mode, sinkron ke bus */
	if (st != HAL_OK) return st;                         /* gagal (mis. tanpa transceiver)       */

	return HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef h;                               /* header frame: ID, DLC, IDE, RTR     */
	uint8_t d[8];                                        /* payload maksimum 8 byte             */

	if (hcan->Instance != CAN1) return;                  /* callback ini hanya untuk CAN1       */

	/* Baca satu frame; ini juga melepaskan slot FIFO. Jika gagal, keluar saja. */
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &h, d) != HAL_OK) return;

	if (h.IDE != CAN_ID_STD) {                           /* extended ID tidak dipakai protokol  */
		stats.rx_unknown++;                              /* catat untuk debugging               */
		return;                                          /* abaikan frame                       */
	}
	if (h.RTR != CAN_RTR_DATA) {                         /* remote frame tidak membawa data     */
		stats.rx_unknown++;                              /* catat                               */
		return;                                          /* abaikan                             */
	}
}

uint8_t can_handle_cmd_vel_timed_out(const dock_cmd *cmd)
{
	if (!cmd->mode) return 1U;                      /* belum pernah ada perintah = timeout */
	/* Pengurangan unsigned aman terhadap wrap HAL_GetTick setelah 49 hari.  */
	return ((HAL_GetTick() - cmd->tick) > CAN_CMD_TIMEOUT_MS) ? 1U : 0U;
}

void can_handle_get_command(dock_cmd *out)
{
	uint8_t  mode_raw;
	uint32_t tick_raw;

	/* Salin buffer ISR ke variabel lokal dengan interrupt dimatikan sebentar,
	 * supaya tidak membaca frame yang sedang ditulis separuh oleh ISR.
	 * PRIMASK disimpan lalu dipulihkan agar tidak mengaktifkan interrupt yang
	 * sebelumnya memang sengaja dimatikan pemanggil.                          */
	uint32_t primask = __get_PRIMASK();                  /* simpan status interrupt global      */
	__disable_irq();                                     /* masuk critical section              */
	mode_raw   	= rx_raw.mode_raw;                        /* salin mode                          */
	tick_raw 	= rx_raw.tick_raw;
	__set_PRIMASK(primask);                              /* keluar critical section             */

	/* Konversi ke satuan SI di luar critical section (operasi float di sini). */
	out->mode = mode_raw;
	out->tick = tick_raw;
}

static void tx_frame(uint32_t id, const uint8_t *data, uint8_t dlc)
{
	CAN_TxHeaderTypeDef h;                               /* header frame kirim                  */
	uint32_t mailbox;                                    /* nomor mailbox yang dipakai HAL      */

	if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0U) { /* ketiga mailbox masih terisi         */
		stats.tx_dropped++;                              /* catat frame dibuang                 */
		return;                                          /* jangan blocking                     */
	}

	h.StdId              = id;                           /* ID standar 11-bit                   */
	h.ExtId              = 0U;                           /* tidak dipakai                       */
	h.IDE                = CAN_ID_STD;                   /* format standar                      */
	h.RTR                = CAN_RTR_DATA;                 /* data frame, bukan remote            */
	h.DLC                = dlc;                          /* jumlah byte payload                 */
	h.TransmitGlobalTime = DISABLE;                      /* time-triggered mode tidak dipakai   */

	/* HAL menyalin data ke register mailbox, jadi buffer lokal boleh hilang setelah ini. */
	if (HAL_CAN_AddTxMessage(&hcan, &h, (uint8_t *)data, &mailbox) == HAL_OK)
		stats.tx_frames++;                               /* berhasil antre                      */
	else
		stats.tx_dropped++;                              /* gagal antre                         */
}

static void update_error_stats(void)
{
	uint32_t esr = hcan.Instance->ESR;                                  /* Error Status Register */
	stats.tec     = (uint8_t)((esr & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos);   /* Transmit Error Counter */
	stats.rec     = (uint8_t)((esr & CAN_ESR_REC) >> CAN_ESR_REC_Pos);   /* Receive Error Counter  */
	stats.bus_off = (esr & CAN_ESR_BOFF) ? 1U : 0U;                      /* flag bus-off           */
}

void can_handle_send_telemetry(const dock_telemetry *t)
{
	uint8_t d[8];                                        /* buffer payload dipakai bergantian   */

	update_error_stats();                                /* segarkan TEC/REC/bus-off            */
	d[0] = t->state;
	put_i16(&d[1], f_to_i16_sat(t->height));
	put_i32(&d[3],t->tick);
	tx_frame(CAN_ID_TLM_FORK, d, 6U);                                  /* kirim                  */
}

const CanStats* can_handle_stats(void)
{
	update_error_stats();                                /* segarkan sebelum dibaca             */
	return (const CanStats *)&stats;                     /* buang volatile untuk pembaca        */
}


