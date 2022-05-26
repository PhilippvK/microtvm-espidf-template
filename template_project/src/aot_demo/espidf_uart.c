/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */
#include "espidf_uart.h"

#include <driver/uart.h>

#include "crt_config.h"

#define RING_BUF_SIZE_BYTES (TVM_CRT_MAX_PACKET_SIZE_BYTES + 100)

// Ring buffer used to store data read from the UART on rx interrupt.
static RingbufHandle_t buf_handle;

#define EX_UART_NUM UART_NUM_0

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
static QueueHandle_t uart0_queue;

static void uart_event_task(void* pvParameters) {
  uart_event_t event;
  uint8_t* data;
  for (;;) {
    // Waiting for UART event.
    if (xQueueReceive(uart0_queue, (void*)&event,
                      (portTickType)portMAX_DELAY)) {
      ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);
      switch (event.type) {
        // Event of UART receving data
        /* We'd better handle data event fast, there would be much more data
        events than other types of events. If we take too much time on data
        event, the queue might be full. */
        case UART_DATA:
          ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
          UBaseType_t res = xRingbufferSendAcquire(
              buf_handle, (void**)&data, event.size, pdMS_TO_TICKS(1000));
          uart_read_bytes(EX_UART_NUM, data, event.size, portMAX_DELAY);
          if (res != pdTRUE) {
            ESP_LOGI(TAG, "Failed to acquire memory for data\n");
            TVMPlatformAbort((tvm_crt_error_t)0xbeef4);
            break;
          }
          // Write it into the ring buffer.
          g_num_bytes_in_rx_buffer += event.size;

          if (g_num_bytes_in_rx_buffer > RING_BUF_SIZE_BYTES) {
            TVMPlatformAbort((tvm_crt_error_t)0xbeef3);
          }

          res = xRingbufferSendComplete(buf_handle, (void**)&data);
          if (res != pdTRUE) {
            ESP_LOGI(TAG, "Failed to send item\n");
            TVMPlatformAbort((tvm_crt_error_t)0xbeef4);
          }
          CHECK_EQ(bytes_read, bytes_written,
                   "bytes_read: %d; bytes_written: %d", bytes_read,
                   bytes_written);
          break;
        // Event of HW FIFO overflow detected
        case UART_FIFO_OVF:
          ESP_LOGI(TAG, "hw fifo overflow");
          // If fifo overflow happened, you should consider adding flow control
          // for your application. The ISR has already reset the rx FIFO, As an
          // example, we directly flush the rx buffer here in order to read more
          // data.
          uart_flush_input(EX_UART_NUM);
          xQueueReset(uart0_queue);
          break;
        // Event of UART ring buffer full
        case UART_BUFFER_FULL:
          ESP_LOGI(TAG, "ring buffer full");
          // If buffer full happened, you should consider encreasing your buffer
          // size As an example, we directly flush the rx buffer here in order
          // to read more data.
          uart_flush_input(EX_UART_NUM);
          xQueueReset(uart0_queue);
          break;
        // Event of UART RX break detected
        case UART_BREAK:
          ESP_LOGI(TAG, "uart rx break");
          break;
        // Event of UART parity check error
        case UART_PARITY_ERR:
          ESP_LOGI(TAG, "uart parity error");
          break;
        // Event of UART frame error
        case UART_FRAME_ERR:
          ESP_LOGI(TAG, "uart frame error");
          break;
        // UART_PATTERN_DET
        case UART_PATTERN_DET:
          ESP_LOGI(TAG, "uart pattern detect");
          break;
        // Others
        default:
          ESP_LOGI(TAG, "uart event type: %d", event.type);
          break;
      }
    }
  }
  vTaskDelete(NULL);
}

// Used to initialize the UART receiver.
// void uart_rx_init(struct ring_buf* rbuf, const struct device* dev) {
void uart_rx_init() {
  // Configure parameters of an UART driver, communication pins and install the
  // driver
  uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_APB,
  };
  // Install UART driver, and get the queue.
  uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 200,
                      &uart0_queue, 0);
  uart_param_config(EX_UART_NUM, &uart_config);

  // Set UART log level
  esp_log_level_set(TAG, ESP_LOG_INFO);
  // Set UART pins (using UART0 default pins ie no changes.)
  uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  // Create a task to handler UART event from ISR
  xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
}

uint32_t TVMPlatformUartRxRead(uint8_t* data, uint32_t data_size_bytes) {
  // unsigned int key = irq_lock();  // TODO
  uint32_t bytes_read = ring_buf_get(&uart_rx_rbuf, data, data_size_bytes);
  // irq_unlock(key);  // TODO
  return bytes_read;
}

uint32_t TVMPlatformWriteSerial(const char* data, uint32_t size) {
  uart_write_bytes(EX_UART_NUM, data, size);
  return size;
}

// Initialize UART
void TVMPlatformUARTInit() {
  // Claim console device.
  uart_rx_init();
}
