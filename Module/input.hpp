#pragma once

#include <errno.h>

#include <zephyr/kernel.h>

#include "input_parser.hpp"
#include "remote_port.hpp"

struct remote_rx_chunk;

void input_process_chunk(const remote_rx_chunk &chunk, input_stream_parser::sample_callback on_sample, void *context);

void input_expire_partial_frames(uint32_t now_ms);

int input_get_sample(remote_protocol protocol, input_sample &sample);

uint32_t input_feedback_count();
