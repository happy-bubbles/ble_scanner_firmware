#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "app_uart.h"
#include "app_timer.h"
#include "app_util.h"
#include "boards.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "softdevice_handler.h"

#define MAX_UART_DATA_LENGTH 300

#define CENTRAL_LINK_COUNT      0                               /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT   0                               /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE    GATT_MTU_SIZE_DEFAULT           /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define UART_TX_BUF_SIZE        256                             /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE        256                             /**< UART RX buffer size. */

#define APP_TIMER_PRESCALER     0                               /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE 2                               /**< Size of timer operation queues. */

#define SCAN_INTERVAL           0x00A0                          /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW             0x0050                          /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_ACTIVE             1                               /**< If 1, performe active scanning (scan requests). */
#define SCAN_SELECTIVE          0                               /**< If 1, ignore unknown devices (non whitelisted). */
#define SCAN_TIMEOUT            0x0000                          /**< Timout when scanning. 0x0000 disables timeout. */

#define UUID16_SIZE             2                               /**< Size of 16 bit UUID */
#define UUID32_SIZE             4                               /**< Size of 32 bit UUID */
#define UUID128_SIZE            16                              /**< Size of 128 bit UUID */

#define LED1_PIN 26

/**
 * @brief Parameters used when scanning.
 */
static const ble_gap_scan_params_t m_scan_params =
{
    .active   = 1,
    .interval = SCAN_INTERVAL,
    .window   = SCAN_WINDOW,
    .timeout  = SCAN_TIMEOUT,
    #if (NRF_SD_BLE_API_VERSION == 2)
        .selective   = 0,
        .p_whitelist = NULL,
    #endif
    #if (NRF_SD_BLE_API_VERSION == 3)
        .use_whitelist = 0,
    #endif
};

/**@brief Function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

void simple_uart_putstring(const uint8_t * str)
{
	uint_fast8_t i  = 0;
	uint8_t      ch = str[i++];

	while (ch != '\0')
	{
		app_uart_put(ch);
		//app_uart_flush();
		ch = str[i++];
	}
	app_uart_flush();
}

/**@brief Function to start scanning.
 */
static void scan_start(void)
{
    ret_code_t ret;

    ret = sd_ble_gap_scan_start(&m_scan_params);
    APP_ERROR_CHECK(ret);
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be processed when the last character received was a
 *          'new line' i.e '\r\n' (hex 0x0D) or if the string has reached a length of
 *          @ref MAX_UART_DATA_LENGTH.
 */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[MAX_UART_DATA_LENGTH];
    static uint8_t index = 0;

    switch (p_event->evt_type)
    {

        /*
				case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') || (index >= (MAX_UART_DATA_LENGTH)))
            {
							  //TODO: process the string here
								// maybe check if it's some command from the nRF, like to put into bootloader mode?

                index = 0;
            }
            break;
        */

				case APP_UART_COMMUNICATION_ERROR:
            //APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            //APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}

/**@brief Reads an advertising report and sends it out over UART
 *
 * @details The function is able to search for 16-bit, 32-bit and 128-bit service uuids.
 *          To see the format of a advertisement packet, see
 *          https://www.bluetooth.org/Technical/AssignedNumbers/generic_access_profile.htm
 *
 * @param[in]   p_adv_report  Pointer to the advertisement report.
 *
 * @retval     none 
 */
static void send_adv_uart(const ble_gap_evt_adv_report_t *p_adv_report)
{
    //uint32_t err_code;
    uint32_t index = 0;
    //uint8_t *p_data = (uint8_t *)p_adv_report->data;
    //ble_uuid_t extracted_uuid;

    while (index < p_adv_report->dlen)
    {
        //uint8_t field_length = p_data[index];
        //uint8_t field_type   = p_data[index + 1];

				// TODO: structure the output here and send out over UART
				//app_uart_put("got a packet\n");
				simple_uart_putstring((const uint8_t *)"got a packet\n\0");

		}
    return;
}

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    //uint32_t              err_code;
    const ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_ADV_REPORT:
        {
            const ble_gap_evt_adv_report_t * p_adv_report = &p_gap_evt->params.adv_report;
						send_adv_uart(p_adv_report);
        }break; // BLE_GAP_EVT_ADV_REPORT

        case BLE_GAP_EVT_TIMEOUT:
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
            {
                //NRF_LOG_DEBUG("Scan timed out.\r\n");
                scan_start();
            }
            else if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                //simple_uart_putstring((const uint8_t *)"Connection Request timed out.\r\n");
            }
            break; // BLE_GAP_EVT_TIMEOUT

        default:
            break;
    }
}

/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack event has
 *          been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    on_ble_evt(p_ble_evt);
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
	
		nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the UART.
 */
static void uart_init(void)
{
    uint32_t err_code;
		const app_uart_comm_params_t comm_params =
		{
			RX_PIN_NUMBER,
			TX_PIN_NUMBER,
			UART_PIN_DISCONNECTED,
			UART_PIN_DISCONNECTED,
			APP_UART_FLOW_CONTROL_DISABLED,
			false,
			UART_BAUDRATE_BAUDRATE_Baud115200
		};


		APP_UART_FIFO_INIT(&comm_params,
				UART_RX_BUF_SIZE,
				UART_TX_BUF_SIZE,
				uart_event_handle,
				APP_IRQ_PRIORITY_LOWEST,
				err_code);
/*
		APP_UART_INIT(&comm_params,
				uart_event_handle,
				APP_IRQ_PRIORITY_LOWEST,
				err_code);
		app_uart_init(&comm_params,
				uart_event_handle,
				APP_IRQ_PRIORITY_LOWEST,
				err_code);
*/
		
		APP_ERROR_CHECK(err_code);
}

/** @brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


int main(void)
{
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, NULL);

    uart_init();
    //buttons_leds_init();
    ble_stack_init();

		//nrf_gpio_cfg_output(LED1_PIN);
		//nrf_gpio_pin_clear(LED1_PIN);

    // Start scanning for peripherals 
    simple_uart_putstring((const uint8_t *)"Uart_c Scan started\r\n\0");
		scan_start();

		/*
		while(1)
		{
			nrf_gpio_pin_toggle(LED1_PIN);
			nrf_delay_ms(1000);
			simple_uart_putstring((const uint8_t *)"Uart_c Scan started\r\n\0");
		}
		*/
    for (;;)
    {
        //power_manage();
    }
}
