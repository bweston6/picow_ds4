#ifndef QUEUE_CONFIG_H
#define QUEUE_CONFIG_H

#include "pico/util/queue.h"

// A data packet to send from Core 0 to Core 1
struct bt_hid_state {
	uint16_t buttons;
	uint8_t lx;
	uint8_t ly;
	uint8_t rx;
	uint8_t ry;
	uint8_t l2;
	uint8_t r2;
	uint8_t hat;
	uint8_t pad;
};

// Declare the queue as 'extern'.
// It will be *defined* in main.c
extern queue_t hid_state_queue;

// Define queue parameters for easy changing
#define QUEUE_ELEMENT_SIZE sizeof(struct bt_hid_state)
#define QUEUE_DEPTH 8 // A small queue to show "full" state

#endif
