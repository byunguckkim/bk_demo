// Copyright (C) 2020 Applied Intuition, Inc. All rights reserved.
// This source code file is distributed under and subject to the LICENSE in
// license.txt

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
#pragma once
extern "C" {
#endif

// The SimianPluginVersion enum is used at runtime to discover the interface
// used by this particular plugin. The discovery mechanism was introduced in
// Simian v1.4. Simian releases can support more than one plugin version,
// typically to release experimental new features early with an unstable
// interface while maintaining full support for stable features.
enum SimianPluginVersion {
  SIMIAN_PLUGIN_VERSION_INVALID = -1,
  // Current stable interface version.
  SIMIAN_PLUGIN_VERSION_1 = 0,
  // V1.1 API swaps C struct protobufs for serialized protobufs.
  SIMIAN_PLUGIN_VERSION_1_1 = 1,
};

//////////////////////////////////////////////////
// Simian plugin version. Should correspond with the customer API version.
// If unimplemented or returns nonzero, will be treated as
// SIMIAN_PLUGIN_VERSION_1
SimianPluginVersion simian_plugin__get_version();

// Lifecycle functions.
//
// Returning NULL or nullptr here will report as an error.
void* customer_interface_v1_1__init();
void customer_interface_v1_1__free(void* self);

// return != 0 to indicate an error
int32_t customer_interface_v1_1__set_startup_options(
    void* self,
    // startup_options is a serialized
    // simian_public::simulator::CustomerStartupOptions protobuf
    const uint8_t* startup_options_buf, size_t startup_options_size);

//////////////////////////////////////////////////
// Required interface functions

// return <= 0 to indicate an error
int32_t customer_interface_v1_1__get_clock_hz(void* self);

// return <= 0 to indicate an error
int32_t customer_interface_v1_1__get_simulation_hz(void* self);

// return != 0 to indicate an error
int32_t customer_interface_v1_1__initialize(void* self, const uint8_t* simulator_output_buf,
                                            size_t simulator_output_size, int64_t sim_time_millis);

// return != 0 to indicate an error
int32_t customer_interface_v1_1__convert_and_publish_sim_outputs(
    void* self, const uint8_t* simulator_output_buf, size_t simulator_output_size,
    int64_t sim_time_millis);

// return != 0 to indicate an error
int32_t customer_interface_v1_1__convert_and_return_listened_sim_inputs(
    void* self, uint8_t** simulator_input_packed, uint64_t* simulator_input_packed_size);

//////////////////////////////////////////////////
// Optional interface functions. For the ones returning an int, a
// non-zero value gets treated as an error.
const char* customer_interface_v1_1__get_stack_version(void* self);
int32_t customer_interface_v1_1__publish_time(void* self, int64_t sim_time_millis);
int32_t customer_interface_v1_1__setup_middleware(void* self);
int32_t customer_interface_v1_1__teardown_middleware(void* self);
int32_t customer_interface_v1_1__setup_stack(void* self);
int32_t customer_interface_v1_1__teardown_stack(void* self);
int32_t customer_interface_v1_1__start_recording(void* self, const char* recording_path);
int32_t customer_interface_v1_1__stop_recording(void* self);
int32_t customer_interface_v1_1__start_visualization(void* self);
int32_t customer_interface_v1_1__stop_visualization(void* self);
int32_t customer_interface_v1_1__finalize(void* self);

int32_t customer_interface_v1_1__get_final_observers(void* self, uint8_t** custom_observers_packed,
                                                     uint64_t* custom_observers_packed_size);

#ifdef __cplusplus
}  // extern "C"
#endif
