#include "app.h"
#include "main.h"
#include <stdint.h>
#include "uds.h"
#include "isotp.h"

#define UDS_REQ_ID (0x123)
#define UDS_RESP_ID (0x456)

void uds_isotp_send(uint8_t *data_ptr, uint16_t data_size);

extern FDCAN_HandleTypeDef hfdcan1;

static IsoTpLink _isotp_link;
static uint8_t _isotp_rx_buf[128];
static uint8_t _isotp_tx_buf[128];

static uint8_t _uds_rx_buf[128];
static uint8_t _uds_tx_buf[128];

static uds_cfg_s _uds_cfg = {
	.iso_tp_send_func_ptr = uds_isotp_send,
	.get_timestamp_func_ptr = HAL_GetTick,

	.rx_buf_ptr = _uds_rx_buf,
	.rx_buf_size = sizeof(_uds_rx_buf),

	.tx_buf_ptr = _uds_tx_buf,
	.tx_buf_size = sizeof(_uds_tx_buf),

	.tester_present_timeout_ms = 2000,
	.tester_present_st_change_cbk_ptr = NULL
};

uds_handle_s _uds_handle;

void uds_isotp_send(uint8_t *data_ptr, uint16_t data_size)
{
	isotp_send(&_isotp_link, data_ptr, data_size);
	printf("> ");
	for(int i = 0; i < data_size; i++) {
		printf("%02X ", data_ptr[i]);
	}
	printf("\n");
}

/* required, this must send a single CAN message with the given arbitration
* ID (i.e. the CAN message ID) and data. The size will never be more than 8
* bytes. */
int  isotp_user_send_can(const uint32_t arbitration_id,
			const uint8_t* data, const uint8_t size) {
	static FDCAN_TxHeaderTypeDef tx_header = {
		.Identifier = UDS_REQ_ID,
		.IdType = FDCAN_STANDARD_ID,
		.TxFrameType = FDCAN_DATA_FRAME,
		.DataLength = 8,
		.ErrorStateIndicator = FDCAN_ESI_ACTIVE,
		.BitRateSwitch = FDCAN_BRS_OFF,
		.FDFormat = FDCAN_CLASSIC_CAN,
		.TxEventFifoControl = FDCAN_NO_TX_EVENTS,
		.MessageMarker = 0u
	};
	tx_header.Identifier = arbitration_id;
	tx_header.DataLength = size;
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, data);
	return ISOTP_RET_OK;
}

/* required, return system tick, unit is millisecond */
uint32_t isotp_user_get_ms(void) {
	return HAL_GetTick();
}

/* optional, provide to receive debugging log messages */
void isotp_user_debug(const char* message, ...) {

}

void app(void)
{
	uds_err_e err = UDS_ERR_OK;
	int isotp_ret = ISOTP_RET_OK;
	uint8_t payload_buf[32];
	uint16_t payload_size = 0;

	isotp_init_link(&_isotp_link,
			UDS_RESP_ID,
			_isotp_tx_buf, sizeof(_isotp_tx_buf),
			_isotp_rx_buf, sizeof(_isotp_rx_buf));

	uds_init(&_uds_handle, &_uds_cfg, &err);
	uint32_t timestamp = HAL_GetTick();
	while(1) {
		isotp_poll(&_isotp_link);

		payload_size = 0;
		isotp_ret = isotp_receive(&_isotp_link,
					  payload_buf,
					  sizeof(payload_buf),
					  &payload_size);

		if(isotp_ret == ISOTP_RET_OK) {
			err = UDS_ERR_OK;
			uds_put(&_uds_handle, payload_buf, payload_size, &err);
			printf("< ");
			for(int i = 0; i < payload_size; i++) {
				printf("%02X ", payload_buf[i]);
			}
			printf("\n");
		}

		err = UDS_ERR_OK;
		uds_handler(&_uds_handle, &err);

		if (HAL_GetTick() - timestamp > 100) {
			timestamp = HAL_GetTick();
			BSP_LED_Toggle(LED_GREEN);
		}
	}
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	uint8_t rxData[8u];
	FDCAN_RxHeaderTypeDef rxHeader;
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U) {
		/* Retrieve Rx messages from RX FIFO0 */
		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
			if(rxHeader.Identifier == UDS_REQ_ID) {
				isotp_on_can_message(&_isotp_link, rxData, rxHeader.DataLength);
			}
		}

	}
}