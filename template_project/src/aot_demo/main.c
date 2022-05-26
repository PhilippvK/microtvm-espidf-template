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

#include <assert.h>
// #include <float.h>
// #include <kernel.h>
// #include <power/reboot.h>
#include <stdio.h>
#include <string.h>
#include <tvm/runtime/c_runtime_api.h>
#include <tvm/runtime/crt/logging.h>
#include <tvm/runtime/crt/stack_allocator.h>
// #include <unistd.h>
// #include <zephyr.h>

#include "input_data.h"
#include "output_data.h"
#include "tvmgen_default.h"
// #include "zephyr_uart.h"

#if CONFIG_IDF_TARGET_ESP32C3

#define CONFIG_LED_PIN_RED ((gpio_num_t)3)
#define CONFIG_LED_PIN_GREEN ((gpio_num_t)4)
#define CONFIG_LED_PIN_BLUE ((gpio_num_t)5)
#define CONFIG_LED_PIN CONFIG_LED_PIN_RED

#endif

// WORKSPACE_SIZE defined in Project API Makefile
// #define WORKSPACE_SIZE (1 << 17)

static uint8_t g_aot_memory[WORKSPACE_SIZE];
tvm_workspace_t app_workspace;

// Transport Commands.
// Commands on host end with `\n`
// Commands on microTVM device end with `%`
const unsigned char CMD_WAKEUP[] = "wakeup\n";
const unsigned char CMD_READY[] = "ready\n";
const unsigned char CMD_INIT[] = "init";
const unsigned char CMD_INFER[] = "infer";

#define CMD_SIZE 80u
#define CMD_TERMINATOR '%'

size_t TVMPlatformFormatMessage(char* out_buf, size_t out_buf_size_bytes,
                                const char* fmt, va_list args) {
  return vsnprintf(out_buf, out_buf_size_bytes, fmt, args);
}

void TVMLogf(const char* msg, ...) {
  char buffer[256];
  int size;
  va_list args;
  va_start(args, msg);
  size = vsprintf(buffer, msg, args);
  va_end(args);
  TVMPlatformWriteSerial(buffer, (uint32_t)size);
}

void TVMPlatformAbort(tvm_crt_error_t error) {
  TVMLogf("TVMPlatformAbort: %08x\n", error);
  esp_restart();
  for (;;)
    ;
}

tvm_crt_error_t TVMPlatformMemoryAllocate(size_t num_bytes, DLDevice dev,
                                          void** out_ptr) {
  return StackMemoryManager_Allocate(&app_workspace, num_bytes, out_ptr);
}

tvm_crt_error_t TVMPlatformMemoryFree(void* ptr, DLDevice dev) {
  return StackMemoryManager_Free(&app_workspace, ptr);
}

int64_t g_microtvm_start_time;
int g_microtvm_timer_running = 0;

// Called to start system timer.
tvm_crt_error_t TVMPlatformTimerStart() {
  if (g_microtvm_timer_running) {
    TVMLogf("timer already running");
    return kTvmErrorPlatformTimerBadState;
  }

#ifdef CONFIG_LED_PIN
  // gpio_set_level(CONFIG_LED_PIN, 1);
#endif
  g_microtvm_start_time = esp_timer_get_time();
  g_microtvm_timer_running = 1;
  return kTvmErrorNoError;
}

// Called to stop system timer.
tvm_crt_error_t TVMPlatformTimerStop(double* elapsed_time_seconds) {
  if (!g_microtvm_timer_running) {
    TVMLogf("timer not running");
    return kTvmErrorSystemErrorMask | 2;
  }

  int64_t stop_time = esp_timer_get_time();
#ifdef CONFIG_LED_PIN
  // gpio_set_level(CONFIG_LED_PIN, 0);
#endif

  // compute how long the work took
  int64_t us_spent = stop_time - g_microtvm_start_time;

  // we do not expect a rollover because of using a 64 bit data type

  *elapsed_time_seconds = us_spent / 1e6;

  g_microtvm_timer_running = 0;
  return kTvmErrorNoError;
}

void* TVMBackendAllocWorkspace(int device_type, int device_id, uint64_t nbytes,
                               int dtype_code_hint, int dtype_bits_hint) {
  tvm_crt_error_t err = kTvmErrorNoError;
  void* ptr = 0;
  DLDevice dev = {device_type, device_id};
  assert(nbytes > 0);
  err = TVMPlatformMemoryAllocate(nbytes, dev, &ptr);
  CHECK_EQ(err, kTvmErrorNoError,
           "TVMBackendAllocWorkspace(%d, %d, %" PRIu64 ", %d, %d) -> %" PRId32,
           device_type, device_id, nbytes, dtype_code_hint, dtype_bits_hint,
           err);
  return ptr;
}

int TVMBackendFreeWorkspace(int device_type, int device_id, void* ptr) {
  tvm_crt_error_t err = kTvmErrorNoError;
  DLDevice dev = {device_type, device_id};
  err = TVMPlatformMemoryFree(ptr, dev);
  return err;
}

static uint8_t main_rx_buf[128];
static uint8_t g_cmd_buf[128];
static size_t g_cmd_buf_ind;

void TVMInfer() {
  struct tvmgen_default_inputs inputs = {
      .input_1 = input_data,
  };
  struct tvmgen_default_outputs outputs = {
      .Identity = output_data,
  };

  StackMemoryManager_Init(&app_workspace, g_aot_memory, WORKSPACE_SIZE);

  double elapsed_time = 0;
  TVMPlatformTimerStart();
  int ret_val = tvmgen_default_run(&inputs, &outputs);
  TVMPlatformTimerStop(&elapsed_time);

  if (ret_val != 0) {
    TVMLogf("Error: %d\n", ret_val);
    TVMPlatformAbort(kTvmErrorPlatformCheckFailure);
  }

  size_t max_ind = -1;
  float max_val = -FLT_MAX;
  for (size_t i = 0; i < output_data_len; i++) {
    if (output_data[i] >= max_val) {
      max_ind = i;
      max_val = output_data[i];
    }
  }
  TVMLogf("result:%d:%d\n", max_ind, (uint32_t)(elapsed_time * 1000));
}

// Execute functions based on received command
void command_ready(char* command) {
  if (strncmp(command, CMD_INIT, CMD_SIZE) == 0) {
    TVMPlatformWriteSerial(CMD_WAKEUP, sizeof(CMD_WAKEUP));
  } else if (strncmp(command, CMD_INFER, CMD_SIZE) == 0) {
    TVMInfer();
  } else {
    TVMPlatformWriteSerial(CMD_READY, sizeof(CMD_READY));
  }
}

// Append received characters to buffer and check for termination character.
void serial_callback(char* message, int len_bytes) {
  for (int i = 0; i < len_bytes; i++) {
    if (message[i] == CMD_TERMINATOR) {
      g_cmd_buf[g_cmd_buf_ind] = (char)0;
      command_ready(g_cmd_buf);
      g_cmd_buf_ind = 0;
    } else {
      g_cmd_buf[g_cmd_buf_ind] = message[i];
      g_cmd_buf_ind += 1;
    }
  }
}

void main(void) {
  g_cmd_buf_ind = 0;
  memset((char*)g_cmd_buf, 0, sizeof(g_cmd_buf));
  TVMPlatformUARTInit();
  // k_timer_init(&g_microtvm_timer, NULL, NULL);

  while (true) {
    int bytes_read = TVMPlatformUartRxRead(main_rx_buf, sizeof(main_rx_buf));
    if (bytes_read > 0) {
      serial_callback(main_rx_buf, bytes_read);
    }
  }
}
