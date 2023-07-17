// Copyright (C) 2019 Applied Intuition, Inc. All rights reserved.
// This source code file is distributed under and subject to the LICENSE in
// license.txt
#pragma once

#include "applied/simian/public/api_def.h"

#ifdef __cplusplus
#include <cstddef>
#include <cstdint>

extern "C" {
#endif

typedef int64_t TimepointNanos;

typedef struct SimianChannel {
  const char* name;
  // Maps to the simian_public::v2::ChannelType enum, but it's not pulled in
  // here to avoid depending on a particular protobuf library.
  int32_t type;
} SimianChannel, ADPChannel;

typedef struct ADPSendFunctions {
  // Sends simian_public.stack_logs.StackLogLine to Simian.
  int32_t (*send_stack_log)(const void*, const uint8_t*, uint64_t);

  // Sends simian_public.drawing.Drawing to Simian.
  int32_t (*send_drawing)(const void*, const uint8_t*, uint64_t);

  // Sends simian_public.common.DataPoint to Simian
  int32_t (*send_data_point)(const void*, const uint8_t*, uint64_t);

  // Sends public_triage.TriageEvent to Strada
  int32_t (*send_triage_event)(const void*, const uint8_t*, uint64_t);

  // Sends a simian_public::common::CustomField containing log data to ADP
  int32_t (*send_log_custom_field)(const void*, const uint8_t*, size_t);

  // Sends a simian_public::common::CustomField containing sim data to ADP
  int32_t (*send_simulation_custom_field)(const void*, const uint8_t*, size_t);

  // Sends simian_public.sim_command.SimCommand to Simian
  int32_t (*send_sim_command)(const void*, const uint8_t*, uint64_t);

  // Sends simian_public.common.ObserverEvent
  int32_t (*send_observer_event)(const void*, const uint8_t*, uint64_t);

  // Sends simian_public.common.Message
  int32_t (*send_message)(const void*, const uint8_t*, uint64_t);

  // Sends simian_public.common.CustomDataPointMetadata
  int32_t (*send_custom_data_point_metadata)(const void*, const uint8_t*, uint64_t);

  // Sends simian_public.common.TimestampedDataPoint to Simian
  int32_t (*send_timestamped_data_point)(const void*, const uint8_t*, uint64_t);

  // Sends simian_public.common.TimestampedStruct to Simian
  int32_t (*send_timestamped_struct)(const void*, const uint8_t*, uint64_t);
} ADPSendFunctions;

//////////////////////////////////////////////////
// Plugin generic functions
//////////////////////////////////////////////////

// From stack_interface_c.h, should this be copied (has its pros/cons) or moved
// or just modified?
enum CustomerInterfaceVersion {
  CUSTOMER_INTERFACE_VERSION_INVALID = -1,
  // Current stable interface version.
  CUSTOMER_INTERFACE_VERSION_1 = 0,
  CUSTOMER_INTERFACE_VERSION_1_1 = 1,
  // This file describes the V2 API
  CUSTOMER_INTERFACE_VERSION_2 = 2,
  // To support the updated V2.1 API:
  //   Changelog:
  //     * log_read() outputs the updated channels. This allows
  //       non-resampled log reading
  CUSTOMER_INTERFACE_VERSION_2_1 = 3,
  // To support the updated V2.2 API:
  //   Changelog:
  //     * log_open() accepts an output which allows log metadata
  //       to be returned
  CUSTOMER_INTERFACE_VERSION_2_2 = 4,

  // To support the updated V2.3 API:
  //  Changelog:
  //    * Moves the send_* functions to an initialization-time struct to support Windows
  //
  CUSTOMER_INTERFACE_VERSION_2_3 = 5,

  // To support the updated V2.4 API:
  //  Changelog:
  //    * New log_rewind() API that updates the log reading cursor to an earlier timestamp
  //
  CUSTOMER_INTERFACE_VERSION_2_4 = 6,
};

APPLIED_PUBLIC_API CustomerInterfaceVersion customer_interface_v2__get_version();

APPLIED_PUBLIC_API void* customer_interface_v2__create(const char* name);
APPLIED_PUBLIC_API void* customer_interface_v2_3__create(const char* name,
                                                         const ADPSendFunctions* func_ptrs);
APPLIED_PUBLIC_API void customer_interface_v2__destroy(void* self);

//////////////////////////////////////////////////
// Stack running-related functions
//////////////////////////////////////////////////

APPLIED_PUBLIC_API int32_t customer_interface_v2__set_startup_options(
    // This is CustomerStartupOptions, same as V1.0
    void* self, uint8_t* startup_options, uint64_t startup_options_size);
APPLIED_PUBLIC_API int32_t customer_interface_v2_1__set_startup_options(
    // This is simian_public.simulator.v2.InterfaceStartupOptions
    void* self, uint8_t* startup_options, uint64_t startup_options_size);
APPLIED_PUBLIC_API const char* customer_interface_v2__get_stack_version(void* self);

// Optional:
APPLIED_PUBLIC_API int32_t customer_interface_v2__middleware_setup(void* self);
APPLIED_PUBLIC_API int32_t customer_interface_v2__middleware_teardown(void* self);
APPLIED_PUBLIC_API int32_t customer_interface_v2__stack_setup(void* self);
APPLIED_PUBLIC_API int32_t customer_interface_v2__stack_teardown(void* self);
APPLIED_PUBLIC_API int32_t customer_interface_v2__recording_setup(void* self,
                                                                  const char* recording_path);
APPLIED_PUBLIC_API int32_t customer_interface_v2__recording_teardown(void* self);
APPLIED_PUBLIC_API int32_t customer_interface_v2__visualization_setup(void* self);
APPLIED_PUBLIC_API int32_t customer_interface_v2__visualization_teardown(void* self);
APPLIED_PUBLIC_API int32_t customer_interface_v2__initialize(void* self);
APPLIED_PUBLIC_API int32_t customer_interface_v2__finalize(void* self);
APPLIED_PUBLIC_API int32_t customer_interface_v2__simulation_summary(void* self,
                                                                     uint8_t* sim_summary,
                                                                     uint64_t sim_summary_size);
// One of these get_default_* functions must be implemented.
APPLIED_PUBLIC_API int32_t customer_interface_v2__get_default_rate(void* self,
                                                                   const SimianChannel* channel);
// This is the period in nanoseconds, return 1e9 for once per second.
APPLIED_PUBLIC_API int64_t
customer_interface_v2__get_default_period_ns(void* self, const SimianChannel* channel);

//////////////////////////////////////////////////
// Stack data-related functions
//////////////////////////////////////////////////

// Create any listeners needed for the given Channel.
APPLIED_PUBLIC_API int32_t customer_interface_v2__listen_setup(void* self,
                                                               const SimianChannel* channel);
// Cleanup any listeners created for the given Channel.
APPLIED_PUBLIC_API int32_t customer_interface_v2__listen_teardown(void* self,
                                                                  const SimianChannel* channel);
// Create any publishers needed for the given Channel.
APPLIED_PUBLIC_API int32_t customer_interface_v2__publish_setup(void* self,
                                                                const SimianChannel* channel);
// Cleanup any publishers created for the given Channel.
APPLIED_PUBLIC_API int32_t customer_interface_v2__publish_teardown(void* self,
                                                                   const SimianChannel* channel);
// Send currently-held data for the given Channel to the stack.
APPLIED_PUBLIC_API int32_t customer_interface_v2__publish_send(void* self,
                                                               const SimianChannel* channel);

//////////////////////////////////////////////////
// Simian-related functions
// Note that you are expected to reserve memory
// for the output. Ownership of that memory will
// move to Simian will be held until the free()
// method gets called.
//////////////////////////////////////////////////

// Convert (and hold) data for the given Channel.
APPLIED_PUBLIC_API int32_t customer_interface_v2__convert__to_simian(void* self,
                                                                     const SimianChannel* channel,
                                                                     uint8_t** output,
                                                                     uint64_t* output_size);
// Free the data returned by the previous convert__to_simian() call.
APPLIED_PUBLIC_API int32_t customer_interface_v2__convert__to_simian_free(void* self,
                                                                          uint8_t* output,
                                                                          uint64_t output_size);
// Convert and return currently-held data for the given Channel.
APPLIED_PUBLIC_API int32_t customer_interface_v2__convert__from_simian(void* self,
                                                                       const SimianChannel* channel,
                                                                       const uint8_t* input,
                                                                       uint64_t input_size);

////////////////////////////////////////////////////
// Output-to-Simian functions
// These don't need to be implemented, just called.
// Ownership of the input pointer will stay with the
// caller, it will not be accessed after the
// function completed.
////////////////////////////////////////////////////

#ifndef _WIN32

// Sends simian_public.stack_logs.StackLogLine to Simian.
APPLIED_PUBLIC_API int32_t customer_interface_v2__send_stack_log(const void* self,
                                                                 const uint8_t* input,
                                                                 uint64_t input_size);
// Sends simian_public.drawing.Drawing to Simian.
APPLIED_PUBLIC_API int32_t customer_interface_v2__send_drawing(const void* self,
                                                               const uint8_t* input,
                                                               uint64_t input_size);
// Sends simian_public.common.DataPoint to Simian
APPLIED_PUBLIC_API int32_t customer_interface_v2__send_data_point(const void* self,
                                                                  const uint8_t* input,
                                                                  uint64_t input_size);
// Sends public_triage.TriageEvent to Strada
APPLIED_PUBLIC_API int32_t customer_interface_v2__send_triage_event(const void* self,
                                                                    const uint8_t* input,
                                                                    uint64_t input_size);

// Sends a simian_public::common::CustomField containing log data to ADP
APPLIED_PUBLIC_API int32_t customer_interface_v2__send_log_custom_field(const void* self,
                                                                        const uint8_t* input,
                                                                        size_t input_size);

// Sends a simian_public::common::CustomField containing sim data to ADP
APPLIED_PUBLIC_API int32_t customer_interface_v2__send_simulation_custom_field(const void* self,
                                                                               const uint8_t* input,
                                                                               size_t input_size);

// Sends simian_public.sim_command.SimCommand to Simian
APPLIED_PUBLIC_API int32_t customer_interface_v2__send_sim_command(const void* self,
                                                                   const uint8_t* input,
                                                                   uint64_t input_size);
// Sends simian_public.common.ObserverEvent
APPLIED_PUBLIC_API int32_t customer_interface_v2__send_observer_event(const void* self,
                                                                      const uint8_t* input,
                                                                      uint64_t input_size);
// Sends simian_public.common.Message
APPLIED_PUBLIC_API int32_t customer_interface_v2__send_message(const void* self,
                                                               const uint8_t* input,
                                                               uint64_t input_size);

// Sends simian_public.common.CustomDataPointMetadata
APPLIED_PUBLIC_API int32_t customer_interface_v2__send_custom_data_point_metadata(
    const void* self, const uint8_t* input, uint64_t input_size);
// Sends simian_public.common.TimestampedDataPoint to Simian
APPLIED_PUBLIC_API int32_t customer_interface_v2__send_timestamped_data_point(const void* self,
                                                                              const uint8_t* input,
                                                                              uint64_t input_size);

// Send simian_public.common.StructData to simian
APPLIED_PUBLIC_API int32_t customer_interface_v2__send_struct_data(const void* self,
                                                                   const uint8_t* input,
                                                                   size_t input_size);

// Send simian_public.common.TimestampedStruct to simian
APPLIED_PUBLIC_API int32_t customer_interface_v2__send_timestamped_struct(const void* self,
                                                                          const uint8_t* input,
                                                                          size_t input_size);
#endif /* _WIN32 */

//////////////////////////////////////////////////
// Log-related functions
//////////////////////////////////////////////////

// V2.0: Open a drive log with the given path, to the given slot, listening to
// the given channels, which should be held whenever a log_read() call is made.
// `input` will be a serialized LogOpenOptions containing the path, slot, and
// channels.
// It is recommended that you implement the v2.2 (or newer) API and and implement
// `customer_interface_v2_2__log__open` instead of this function.
APPLIED_PUBLIC_API int32_t customer_interface_v2__log__open(void* self, const uint8_t* input,
                                                            size_t input_size);

// Close and cleanup the log in the given slot.
// `input` will be a serialized LogCloseOptions.
APPLIED_PUBLIC_API int32_t customer_interface_v2__log__close(void* self, const uint8_t* input,
                                                             size_t input_size);

// V2.1: Locate and download a log with the given path.
// Optionally cache the log for quicker retrieval in subsequent simulation runs.
// Note that the output pointer is owned by Simian and will be freed with the
// call to log__fetch_free.
APPLIED_PUBLIC_API int32_t customer_interface_v2_1__log__fetch(void* self, const uint8_t* input,
                                                               size_t input_size, uint8_t** output,
                                                               uint64_t* output_size);

// V2.1: Frees the buffer allocated in the response to
// customer_interface_v2_1_log__fetch(). If the buffer is allocated on the `self`
// object, then this function does not need to be implemented.
APPLIED_PUBLIC_API int32_t customer_interface_v2_1__log__fetch_free(void* self, uint8_t* output,
                                                                    uint64_t output_size);

// V2.1: Read up until the provided offset into the bag, optionally returning
// early when a channel is seen. Any data seen meant for the stack should be sent
// directly, preferably after updating the stack's time. This is equivalent to
// updating the TIME channel, calling publish_send() on it and the channel
// received from the log, without the Simian version of those channels.
// `input` will be a serialized LogReadOptions, `output` must be a
// serialized LogReadOutput.
// Note that the output pointer is owned by Simian and will be freed with the
// call to log__read_free.
APPLIED_PUBLIC_API int32_t customer_interface_v2_1__log__read(void* self, const uint8_t* input,
                                                              size_t input_size, uint8_t** output,
                                                              uint64_t* output_size);

// V2.1: Frees the buffer allocated in the response to
// customer_interface_v2_1__log__read(). If the buffer is allocated on the `self`
// object, then this function does not need to be implemented.
APPLIED_PUBLIC_API int32_t customer_interface_v2_1__log__read_free(void* self, uint8_t* output,
                                                                   uint64_t output_size);

// V2.4: Prepare the log readers to read from a previous sim-time, and clear log-channel data.
APPLIED_PUBLIC_API int32_t customer_interface_v2_4__log__rewind(void* self, const uint8_t* input,
                                                                size_t input_size, uint8_t** output,
                                                                uint64_t* output_size);

// V2.4: Frees the buffer allocated in the response to
// customer_interface_v2_4__log__rewind(). If the buffer is allocated on the `self`
// object, then this function does not need to be implemented.
APPLIED_PUBLIC_API int32_t customer_interface_v2_4__log__rewind_free(void* self, uint8_t* output,
                                                                     uint64_t output_size);

// V2.1: When a message in a channel needs to be patched, after the relevant
// log_read() and/or convert_to_simian() calls, patch() will be called with the
// changes necessary for the channel's type.
// `input` will be a serialized PatchOptions.
APPLIED_PUBLIC_API int32_t customer_interface_v2_1__patch(void* self, const SimianChannel* channel,
                                                          const uint8_t* input,
                                                          uint64_t input_size);

// V2.2: Open a drive log with the given path, to the given slot, listening to
// the given channels, which should be held whenever a log_read() call is made.
// `input` will be a serialized LogOpenOptions containing the path, slot, and
// channels. `output` must be a serialized LogOpenOutput, which contains the
// start timestamp of the log and information about where the drive in the log
// takes place.
// Note that the output pointer is owned by Simian and will be freed with the
// call to log__open_free.
APPLIED_PUBLIC_API int32_t customer_interface_v2_2__log__open(void* self, const uint8_t* input,
                                                              size_t input_size, uint8_t** output,
                                                              uint64_t* output_size);

// V2.2: Frees the buffer allocated in the response to
// customer_interface_v2_2__log__open(). If the buffer is allocated on the `self`
// object, then this function does not need to be implemented.
APPLIED_PUBLIC_API int32_t customer_interface_v2_2__log__open_free(void* self, uint8_t* output,
                                                                   uint64_t output_size);

#ifdef __cplusplus
}  // extern "C"
#endif
