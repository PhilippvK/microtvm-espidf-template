#pragma once

#include <stddef.h>
#include <unistd.h>

#include <stdint.h>

#include "freertos/ringbuf.h"

#ifdef __cplusplus

extern "C" {

#endif

void transport_wifi_init(RingbufHandle_t ringbuf);

size_t transport_wifi_read(

    uint8_t* buffer,

    size_t max_len);

ssize_t transport_wifi_write(

    const uint8_t* data,

    size_t len);

#ifdef __cplusplus

}

#endif
