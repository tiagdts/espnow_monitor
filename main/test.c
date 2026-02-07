/*
 * test.c
 *
 *  Created on: Dec 19, 2025
 *      Author: tiagd
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"

#define UART_TXD (GPIO_NUM_21)
#define UART_RXD (GPIO_NUM_20)
#define UART_NUM (UART_NUM_1)
#define BUF_SIZE (1024)

static QueueHandle_t tx_queue;

void tx_task(void *arg) {
    char *tx_data;
    while (1) {
        // Wait indefinitely for data to arrive in the queue
        if (xQueueReceive(tx_queue, &tx_data, portMAX_DELAY) == pdPASS) {
            // Once data is received, transmit it via UART
            uart_write_bytes(UART_NUM, tx_data, strlen(tx_data));
            // Free the allocated memory after transmission
            free(tx_data);
        }
    }
}

void app_main(void) {
    // 1. Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));

    // 2. Set UART pins
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TXD, UART_RXD, UART_PIN_NO_FLOW_CTRL, UART_PIN_NO_FLOW_CTRL));

    // 3. Install UART driver, but without an internal TX buffer (handled by our queue/task)
    // The TX buffer size is set to 0, so uart_write_bytes blocks until data is sent to the FIFO
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));

    // 4. Create the custom queue that holds pointers to strings
    tx_queue = xQueueCreate(10, sizeof(char *));

    // 5. Create the transmission task
    xTaskCreate(tx_task, "uart_tx_task", 2048, NULL, 10, NULL);

    // 6. Main task loop to send data to the queue
    while (1) {
        // Allocate memory for the string and copy data
        char *str_to_send = malloc(50);
        if (str_to_send) {
            sprintf(str_to_send, "Hello World: %d\r\n", xTaskGetTickCount());
            // Send the pointer to the tx_task
            if (xQueueSend(tx_queue, &str_to_send, portMAX_DELAY) != pdPASS) {
                // Handle error if queue is full
                free(str_to_send);
                ESP_LOGE("MAIN", "Failed to send to TX queue");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}



