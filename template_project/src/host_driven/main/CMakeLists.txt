set(MLF_DIR ${CMAKE_SOURCE_DIR}/model/)
# set(CRT_DIR ${CMAKE_SOURCE_DIR}/crt/)
set(CRT_DIR ${MLF_DIR}/runtime)

file(GLOB_RECURSE TVM_KERNEL_SRCS ${MLF_DIR}/codegen/host/src/*.c)

set(TVM_SRCS ${CRT_DIR}/src/runtime/crt/microtvm_rpc_server/rpc_server.cc ${CRT_DIR}/src/runtime/crt/memory/page_allocator.c ${CRT_DIR}/src/runtime/crt/common/crt_runtime_api.c ${CRT_DIR}/src/runtime/crt/common/func_registry.c ${CRT_DIR}/src/runtime/crt/common/ndarray.c ${CRT_DIR}/src/runtime/crt/microtvm_rpc_common/session.cc ${CRT_DIR}/src/runtime/crt/microtvm_rpc_common/frame_buffer.cc ${CRT_DIR}/src/runtime/crt/microtvm_rpc_common/framing.cc ${CRT_DIR}/src/runtime/crt/microtvm_rpc_common/write_stream.cc ${CRT_DIR}/src/runtime/crt/microtvm_rpc_common/crcccitt.c ${CRT_DIR}//src/runtime/crt/common/crt_backend_api.c ${CRT_DIR}/src/runtime/crt/aot_executor_module/aot_executor_module.c ${CRT_DIR}/src/runtime/crt/aot_executor/aot_executor.c ${CRT_DIR}/src/runtime/crt/common/packed_func.c ${CRT_DIR}/src/runtime/crt/graph_executor_module/graph_executor_module.c ${CRT_DIR}/src/runtime/crt/graph_executor/graph_executor.c ${CRT_DIR}/src/runtime/crt/graph_executor/load_json.c)

set(TVM_INCS ${CRT_DIR}/include/ ${CMAKE_SOURCE_DIR}/crt_config/)

if(EXISTS ${MLF_DIR}/codegen/host/include/)
    list(APPEND TVM_INCS ${MLF_DIR}/codegen/host/include/)
endif()


idf_component_register(
    SRCS
    main.c
    ${TVM_SRCS}
    ${TVM_KERNEL_SRCS}
    INCLUDE_DIRS
    .
    ${TVM_INCS}
    REQUIRES
    spi_flash
    driver
    # esp_driver_i2s
    esp_driver_gpio
    esp_driver_uart
    esp_driver_ledc
    esp_ringbuf
    esp_timer
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-error=unused-but-set-parameter
    -Wno-error=discarded-qualifiers
    -Wno-error=stringop-overread
    -Wno-error=format
    -Wno-error=strict-aliasing
    -Wno-error=format-extra-args
    $<$<COMPILE_LANGUAGE:C>:-Wno-incompatible-pointer-types>
    )
