/*
 * can_drv_esp32.c
 *
 *  Created on: May 4, 2022
 *      Author: refo
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/twai.h"
#include "hal/twai_types.h"
#include "hal/twai_ll.h"
#include "soc/dport_reg.h"

#include "can_drv_esp32.h"

extern twai_dev_t TWAI;
static SemaphoreHandle_t sem_int_event;
static volatile uint32_t err_int;
static twai_mode_t can_drv_esp32_mode;
QueueHandle_t can_drv_rx_queue;
intr_handle_t can_drv_isr_ret_handle;

static void can_drv_esp32_read_phy(BaseType_t *higherPriorityTaskWoken)
{
	uint8_t i = 0;

	// frame read buffer
//	CAN_frame_t rx_frame;
	can_frame_esp32_t rx_frame;

	// check if we have a queue. If not, operation is aborted.
	if (can_drv_rx_queue == NULL) {
		TWAI.command_reg.rrb = 1;
		return;
	}

	rx_frame.fir = TWAI.tx_rx_buffer[i++].val;

	// check if this is a standard or extended CAN frame
	if (rx_frame.idt == CAN_ID_STANDARD) {
		// Get standard message ID
		rx_frame.id = (uint32_t)TWAI.tx_rx_buffer[i++].val << 3;
		rx_frame.id |= (uint32_t)TWAI.tx_rx_buffer[i++].val >> 5;
//		for (i = 0; i < rx_frame.FIR.B.DLC; i++) {
//			rx_frame.data.u8[i] = MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.STD.data[i];
//		}
	} else {
		// Get extended message ID
		rx_frame.id = (uint32_t)TWAI.tx_rx_buffer[i++].val << 21;
		rx_frame.id |= (uint32_t)TWAI.tx_rx_buffer[i++].val << 13;
		rx_frame.id |= (uint32_t)TWAI.tx_rx_buffer[i++].val << 5;
		rx_frame.id |= (uint32_t)TWAI.tx_rx_buffer[i++].val >> 3;


		// deep copy data bytes
//		for (i = 0; i < rx_frame.FIR.B.DLC; i++) {
//			rx_frame.data.u8[i] = MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.EXT.data[i];
//		}
	}
//	printf("\nRX: ");
	for (uint8_t j = 0; j < rx_frame.dlc; j++) {
//		printf("0x%02x ", rx_frame.can_data[j]);
		rx_frame.data_u8[j] = TWAI.tx_rx_buffer[i++].val;
	}
//	printf("\n");

	xQueueSendToBackFromISR(can_drv_rx_queue, &rx_frame, higherPriorityTaskWoken);

	// Let the hardware know the frame has been read.
//	MODULE_CAN->CMR.B.RRB = 1;
	TWAI.command_reg.rrb = 1;

}

static void can_drv_esp32_isr(void *arg_p)
{
	BaseType_t higherPriorityTaskWoken = pdFALSE;

	// Read interrupt status and clear flags
	uint32_t interrupt = TWAI.interrupt_reg.val;

	// Handle RX frame available interrupt
	if ((interrupt & TWAI_LL_INTR_RI) != 0) {
		err_int &= ~TWAI_LL_INTR_RI;
		can_drv_esp32_read_phy(&higherPriorityTaskWoken);
	}

	// Handle error interrupts.
	if ((interrupt & 	( TWAI_LL_INTR_EI
						| TWAI_LL_INTR_EPI
						| TWAI_LL_INTR_ALI
						| TWAI_LL_INTR_BEI )) != 0) {
		twai_ll_set_cmd_abort_tx(&TWAI);
		err_int = interrupt;
		xSemaphoreGive(sem_int_event);
	}

	// Handle TX complete interrupt
	if ((interrupt & TWAI_LL_INTR_TI) != 0) {
		err_int &= ~TWAI_LL_INTR_TI;
		xSemaphoreGive(sem_int_event);
	}

	// check if any higher priority task has been woken by any handler
	if (higherPriorityTaskWoken) {
		portYIELD_FROM_ISR();
	}

}

void can_drv_esp32_regs_print(void)
{
	printf("MODULE_CAN->MOD.B.RM = %d                 //MOD.0 Reset Mode\n",
												TWAI.mode_reg.rm);
	printf("MODULE_CAN->MOD.B.LOM = %d                //MOD.1 Listen Only Mode\n",
												TWAI.mode_reg.lom);
	printf("MODULE_CAN->MOD.B.STM = %d                //MOD.2 Self Test Mode\n",
												TWAI.mode_reg.stm);
	printf("MODULE_CAN->MOD.B.AFM = %d                //MOD.3 Acceptance Filter Mode\n",
												TWAI.mode_reg.afm);
	printf("MODULE_CAN->CDR.B.COD = %d                //CDR[2:0] CLKOUT frequency selector based of fOSC\n",
												TWAI.clock_divider_reg.cd);
	printf("MODULE_CAN->CDR.B.COFF = %d               //CDR.3 CLKOUT enable/disable\n",
												TWAI.clock_divider_reg.co);
	printf("MODULE_CAN->CDR.B.CAN_M = %d              //CDR.7 Register Layout. Basic:0 Extended:1 (PELICAN mode)\n",
												TWAI.clock_divider_reg.cm);
	printf("MODULE_CAN->BTR0.B.BRP = %d               //BTR0[5:0] Baud Rate Prescaler\n",
												TWAI.bus_timing_0_reg.brp);
	printf("MODULE_CAN->BTR0.B.SJW = %d               //BTR0[7:6] Synchronization Jump Width\n",
												TWAI.bus_timing_0_reg.sjw);
	printf("MODULE_CAN->BTR1.B.TSEG1 = %d             //BTR1[3:0] Timing Segment 1\n",
												TWAI.bus_timing_1_reg.tseg1);
	printf("MODULE_CAN->BTR1.B.TSEG2 = %d             //BTR1[6:4] Timing Segment 2\n",
												TWAI.bus_timing_1_reg.tseg2);
	printf("MODULE_CAN->BTR1.B.SAM = %d               //BTR1.7 Sampling\n",
												TWAI.bus_timing_1_reg.sam);
	printf("MODULE_CAN->IER.B.RIE = %d              //IER.0 Receive Interrupt Enable\n",
												TWAI.interrupt_enable_reg.rie);
	printf("MODULE_CAN->IER.B.TIE = %d              //IER.1 Transmit Interrupt Enable\n",
												TWAI.interrupt_enable_reg.tie);
	printf("MODULE_CAN->IER.B.EIE = %d              //IER.2 Error Interrupt Enable\n",
												TWAI.interrupt_enable_reg.eie);
	printf("MODULE_CAN->IER.B.DOIE = %d              //IER.3 Data Overrun Interrupt Enable\n",
												TWAI.interrupt_enable_reg.doie);
	printf("MODULE_CAN->IER.B.BPR_DIV = %d           //IER.4 prescale BRP by 2. ESP32 Rev2 and later\n",
												TWAI.interrupt_enable_reg.brp_div);
	printf("MODULE_CAN->IER.B.EPIE = %d              //IER.5 Error Passive Interrupt Enable\n",
												TWAI.interrupt_enable_reg.epie);
	printf("MODULE_CAN->IER.B.ALIE = %d              //IER.6 Arbitration Lost Interrupt Enable\n",
												TWAI.interrupt_enable_reg.alie);
	printf("MODULE_CAN->IER.B.BEIE = %d              //IER.7 Bus Error Interrupt Enable\n",
												TWAI.interrupt_enable_reg.beie);
	printf("MODULE_CAN->MBX_CTRL.ACC.CODE[0] = 0x%02x //Acceptance Code (Message ID)\n",
												TWAI.acceptance_filter.acr[0].byte);
	printf("MODULE_CAN->MBX_CTRL.ACC.CODE[1] = 0x%02x \n",
												TWAI.acceptance_filter.acr[1].byte);
	printf("MODULE_CAN->MBX_CTRL.ACC.CODE[2] = 0x%02x \n",
												TWAI.acceptance_filter.acr[2].byte);
	printf("MODULE_CAN->MBX_CTRL.ACC.CODE[3] = 0x%02x \n",
												TWAI.acceptance_filter.acr[3].byte);
	printf("MODULE_CAN->MBX_CTRL.ACC.MASK[0] = 0x%02x //Acceptance Mask\n",
												TWAI.acceptance_filter.amr[0].byte);
	printf("MODULE_CAN->MBX_CTRL.ACC.MASK[1] = 0x%02x \n",
												TWAI.acceptance_filter.amr[1].byte);
	printf("MODULE_CAN->MBX_CTRL.ACC.MASK[2] = 0x%02x \n",
												TWAI.acceptance_filter.amr[2].byte);
	printf("MODULE_CAN->MBX_CTRL.ACC.MASK[3] = 0x%02x \n",
												TWAI.acceptance_filter.amr[3].byte);

}

int can_drv_esp32_init(	const twai_general_config_t *g_config,
						const twai_timing_config_t *t_config,
						const twai_filter_config_t *f_config)
{
	// enable module
	DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_CAN_CLK_EN);
	DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_CAN_RST);

	twai_ll_enter_reset_mode(&TWAI);

	// configure TX pin
//	gpio_set_level(CAN_cfg.tx_pin_id, 1);
	gpio_set_level(g_config->tx_io, 1);
//	gpio_set_direction(CAN_cfg.tx_pin_id, GPIO_MODE_OUTPUT);
	gpio_set_direction(g_config->tx_io, GPIO_MODE_OUTPUT);
//	gpio_matrix_out(CAN_cfg.tx_pin_id, CAN_TX_IDX, 0, 0);
	gpio_matrix_out(g_config->tx_io, CAN_TX_IDX, 0, 0);
//	gpio_pad_select_gpio(CAN_cfg.tx_pin_id);
	gpio_pad_select_gpio(g_config->tx_io);

	// configure RX pin
//	gpio_set_direction(CAN_cfg.rx_pin_id, GPIO_MODE_INPUT);
	gpio_set_direction(g_config->rx_io, GPIO_MODE_INPUT);
//	gpio_matrix_in(CAN_cfg.rx_pin_id, CAN_RX_IDX, 0);
	gpio_matrix_in(g_config->rx_io, CAN_RX_IDX, 0);
//	gpio_pad_select_gpio(CAN_cfg.rx_pin_id);
	gpio_pad_select_gpio(g_config->rx_io);

//	MODULE_CAN->CDR.B.CAN_M = 0x1;
	TWAI.clock_divider_reg.cm = 1;	// Extended Register Layout

	// synchronization jump width is the same for all baud rates
//	MODULE_CAN->BTR0.B.SJW = 0x1;
	TWAI.bus_timing_0_reg.sjw = t_config->sjw -1;

//	MODULE_CAN->BTR1.B.TSEG1 = 14;
	TWAI.bus_timing_1_reg.tseg1 = t_config->tseg_1 - 1;
//	MODULE_CAN->BTR1.B.TSEG2 = 3;
	TWAI.bus_timing_1_reg.tseg2 = t_config->tseg_2 - 1;

	// set baud rate prescaler
//	MODULE_CAN->BTR0.B.BRP = (uint8_t) round((((APB_CLK_FREQ * __tq) / 2) - 1) / 1000000) - 1;
//	MODULE_CAN->BTR0.B.BRP = 3;
	TWAI.bus_timing_0_reg.brp = (t_config->brp / 2) - 1;

	/* Set sampling
	 * 1 -> triple; the bus is sampled three times; recommended for low/medium speed buses     (class A and B) where
	 * filtering spikes on the bus line is beneficial 0 -> single; the bus is sampled once; recommended for high speed
	 * buses (SAE class C)*/
//	MODULE_CAN->BTR1.B.SAM = 0x1;
	TWAI.bus_timing_1_reg.sam = t_config->triple_sampling;

//	MODULE_CAN->IER.B.WUIE = 0;	//This is not interrupt control bit
	TWAI.interrupt_enable_reg.brp_div = 0;

	// enable all interrupts
//	MODULE_CAN->IER.U = 0xff;
//	MODULE_CAN->IER.B.RIE = 1;
	TWAI.interrupt_enable_reg.rie = 1;
//	MODULE_CAN->IER.B.TIE = 1;
	TWAI.interrupt_enable_reg.tie = 1;
//	MODULE_CAN->IER.B.EIE = 1;
	TWAI.interrupt_enable_reg.eie = 1;
//	MODULE_CAN->IER.B.DOIE = 1;
	TWAI.interrupt_enable_reg.doie = 1;
//	MODULE_CAN->IER.B.EPIE = 1;
	TWAI.interrupt_enable_reg.epie = 1;
//	MODULE_CAN->IER.B.ALIE = 1;
	TWAI.interrupt_enable_reg.alie = 1;
//	MODULE_CAN->IER.B.BEIE = 1;
	TWAI.interrupt_enable_reg.beie = 1;

	// no acceptance filtering, as we want to fetch all messages
//	MODULE_CAN->MBX_CTRL.ACC.CODE[0] = 0;
//	MODULE_CAN->MBX_CTRL.ACC.CODE[1] = 0;
//	MODULE_CAN->MBX_CTRL.ACC.CODE[2] = 0;
//	MODULE_CAN->MBX_CTRL.ACC.CODE[3] = 0;
//	MODULE_CAN->MBX_CTRL.ACC.MASK[0] = 0xff;
//	MODULE_CAN->MBX_CTRL.ACC.MASK[1] = 0xff;
//	MODULE_CAN->MBX_CTRL.ACC.MASK[2] = 0xff;
//	MODULE_CAN->MBX_CTRL.ACC.MASK[3] = 0xff;
	twai_ll_set_acc_filter(&TWAI, f_config->acceptance_code,
							f_config->acceptance_mask, f_config->single_filter);
	// set to normal mode
//	MODULE_CAN->OCR.B.OCMODE = __CAN_OC_NOM;
// Not available in esp32

	// clear error counters
//	MODULE_CAN->TXERR.U = 0;
//	MODULE_CAN->RXERR.U = 0;
//	(void) MODULE_CAN->ECC;
	twai_ll_set_tec(&TWAI, 0);
	twai_ll_set_rec(&TWAI, 0);
	twai_ll_set_err_warn_lim(&TWAI, 96);

	// clear interrupt flags
//	(void) MODULE_CAN->IR.U;
//	(void) TWAI.interrupt_reg.val;
	(void) twai_ll_get_and_clear_intrs(&TWAI);

	can_drv_esp32_regs_print();

	// install CAN ISR
	esp_intr_alloc(ETS_CAN_INTR_SOURCE, 0, can_drv_esp32_isr, NULL, &can_drv_isr_ret_handle);

	// allocate the tx complete semaphore
	sem_int_event = xSemaphoreCreateBinary();

	// Showtime. Release Reset Mode.
//	MODULE_CAN->MOD.B.RM = 0;

	can_drv_esp32_mode = g_config->mode;

	return 0;
}

void can_drv_esp32_start(void)
{
	can_drv_rx_queue = xQueueCreate(10, sizeof(can_frame_esp32_t));
	twai_ll_set_mode(&TWAI, can_drv_esp32_mode);
	(void) twai_ll_get_and_clear_intrs(&TWAI);
	twai_ll_exit_reset_mode(&TWAI);
}

void can_drv_esp32_stop(void)
{
	twai_ll_enter_reset_mode(&TWAI);
	(void) twai_ll_get_and_clear_intrs(&TWAI);
	twai_ll_set_mode(&TWAI, TWAI_MODE_LISTEN_ONLY);
	esp_intr_disable(can_drv_isr_ret_handle);
	esp_intr_free(can_drv_isr_ret_handle);
	vQueueDelete(can_drv_rx_queue);
}

esp_err_t can_drv_esp32_tx(can_frame_esp32_t *tx_frame)
{
	uint8_t i = 0;

	candrv_logv("\t\tCANDRV TX: ID=0x%06x IDt=%d DLC=%d : ",
			tx_frame->id, tx_frame->idt, tx_frame->dlc);

#if CANDRV_LOG == CANDRV_LOG_VERBOSE
	for (uint8_t i=0; i< tx_frame->dlc; i++) {
		candrv_logv("0x%02x ", tx_frame->data_u8[i]);
	}
	uint32_t status = twai_ll_get_status(&TWAI);
#endif
	candrv_logv("\n");
	candrv_logv("\t\tCANDRV: Status %x\n", status);

	TWAI.tx_rx_buffer[i++].val = tx_frame->fir;

	if (tx_frame->idt == CAN_ID_STANDARD) {
		// Write standard message ID
		TWAI.tx_rx_buffer[i++].val = ((tx_frame->id) >> 3);
		TWAI.tx_rx_buffer[i++].val = ((tx_frame->id) << 5);
//		i+=2;
	} else {
		// Write extended message ID
		TWAI.tx_rx_buffer[i++].val = ((tx_frame->id) >> 21);
		TWAI.tx_rx_buffer[i++].val = ((tx_frame->id) >> 13);
		TWAI.tx_rx_buffer[i++].val = ((tx_frame->id) >> 5);
		TWAI.tx_rx_buffer[i++].val = ((tx_frame->id) >> 3);
	}

	// Copy the frame data to the hardware
	for (uint8_t j = 0; j < tx_frame->dlc; j++) {
		TWAI.tx_rx_buffer[i++].val = tx_frame->data_u8[j];
	}
	// Transmit
//	TWAI.command_reg.tr = 1;
//	TWAI.command_reg.tr = 3; //TR + AT
	twai_ll_set_cmd_tx_single_shot(&TWAI);
	return ESP_OK;
}

esp_err_t can_drv_esp32_rx(can_frame_esp32_t *rx_frame, TickType_t ticks_to_wait)
{
	if (xQueueReceive(can_drv_rx_queue, rx_frame, ticks_to_wait) == pdTRUE) {
		return ESP_OK;
	}
	return ESP_ERR_TIMEOUT;
}

esp_err_t can_drv_esp32_wait_tx_end(TickType_t ticks_to_wait)
{
	candrv_logv("\t\tCANDRV: Waiting to TX...\n");
	BaseType_t res = xSemaphoreTake(sem_int_event, ticks_to_wait);
#if CANDRV_LOG == CANDRV_LOG_VERBOSE
	uint32_t status = twai_ll_get_status(&TWAI);
#endif
	candrv_logv("\t\t\CANDRV: Int %x status %x\n", err_int, status);
	if (res == pdTRUE) {
		if (err_int != 0) {
			return ESP_FAIL;
		}
		candrv_logv("\t\tCANDRV: TX Done\n");
		return ESP_OK;
	}
	candrv_logv("\t\tERROR: TX TIMEOUT\n");
	return ESP_ERR_TIMEOUT;
}
