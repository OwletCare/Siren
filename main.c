/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/**
  @defgroup dtm_standalone main.c
  @{
  @ingroup ble_sdk_app_dtm_serial
  @brief Stand-alone DTM application for UART interface.
  
 */

#include <stdint.h>
#include <stdbool.h>
#include "nrf.h"
#include "ble_dtm.h"
#include "bsp.h"

// Configuration parameters.
#define BITRATE  UART_BAUDRATE_BAUDRATE_Baud115200  /**< Serial bitrate on the UART */

// @note: The BLE DTM 2-wire UART standard specifies 8 data bits, 1 stop bit, no flow control.
//        These parameters are not configurable in the BLE standard.

/**
 * @details Maximum iterations needed in the main loop between stop bit 1st byte and start bit 2nd
 * byte. DTM standard allows 5000us delay between stop bit 1st byte and start bit 2nd byte. 
 * As the time is only known when a byte is received, then the time between between stop bit 1st 
 * byte and stop bit 2nd byte becomes: 
 *      5000us + transmission time of 2nd byte.
 *
 * Byte transmission time is (Baud rate of 19200):
 *      10bits * 1/19200 = approx. 520 us/byte (8 data bits + start & stop bit).
 *
 * Loop time on polling UART register for received byte is defined in ble_dtm.c as:
 *   UART_POLL_CYCLE = 260 us
 *
 * The max time between two bytes thus becomes (loop time: 260us / iteration): 
 *      (5000us + 520us) / 260us / iteration = 21.2 iterations. 
 *
 * This is rounded down to 21. 
 *
 * @note If UART bit rate is changed, this value should be recalculated as well.
 */
#define MAX_ITERATIONS_NEEDED_FOR_NEXT_BYTE 21

/**
 * @brief Function for UART initialization.
 */
static void uart_init(void)
{
    // Configure UART0 pins.
    nrf_gpio_cfg_output(TX_PIN_NUMBER);
    nrf_gpio_cfg_input(RX_PIN_NUMBER, NRF_GPIO_PIN_NOPULL);

    NRF_UART0->PSELTXD       = TX_PIN_NUMBER;
    NRF_UART0->PSELRXD       = RX_PIN_NUMBER;
    NRF_UART0->BAUDRATE      = BITRATE;

    // Clean out possible events from earlier operations
    NRF_UART0->EVENTS_RXDRDY = 0;
    NRF_UART0->EVENTS_TXDRDY = 0;
    NRF_UART0->EVENTS_ERROR  = 0;

    // Activate UART.
    NRF_UART0->ENABLE        = UART_ENABLE_ENABLE_Enabled;
    NRF_UART0->INTENSET      = 0;
    NRF_UART0->TASKS_STARTTX = 1;
    NRF_UART0->TASKS_STARTRX = 1;
}


/**
 * @brief Function for application main entry.
 *
 * @details This function serves as an adaptation layer between a 2-wire UART interface and the
 *          dtmlib. After initialization, DTM commands submitted through the UART are forwarded to
 *          dtmlib and events (i.e. results from the command) is reported back through the UART.
 */
int main(void)
{
    uint32_t    dtm_error_code;
    dtm_event_t result;                    // Result of a DTM operation.

    uart_init();

    dtm_error_code = dtm_init();
    if (dtm_error_code != DTM_SUCCESS)
    {
        // If DTM cannot be correctly initialized, then we just return.
        return -1;
    }

    if (dtm_cmd(LE_TRANSMITTER_TEST, 10, CARRIER_TEST, DTM_PKT_VENDORSPECIFIC) != DTM_SUCCESS)
    {
        // Extended error handling may be put here. 
        // Default behavior is to return the event on the UART (see below);
        // the event report will reflect any lack of success.
    }

    // Retrieve result of the operation. This implementation will busy-loop
    // for the duration of the byte transmissions on the UART.
    if (dtm_event_get(&result))
    {
        // Report command status on the UART.
        // Transmit MSB of the result.
        NRF_UART0->TXD = (result >> 8) & 0xFF;
        // Wait until MSB is sent.
        while (NRF_UART0->EVENTS_TXDRDY != 1)
        {
            // Do nothing.
        }
        NRF_UART0->EVENTS_TXDRDY = 0;

        // Transmit LSB of the result.
        NRF_UART0->TXD = result & 0xFF;
        // Wait until LSB is sent.
        while (NRF_UART0->EVENTS_TXDRDY != 1)
        {
            // Do nothing.
        }
        NRF_UART0->EVENTS_TXDRDY = 0;
    }

    while (true);
}

/// @}
