//
// Created by thejackimonster on 29.03.23.
//
// Copyright (c) 2023 thejackimonster. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//

#include "device3.h"
#include "device4.h"

#include <stdio.h>
#include <sys/wait.h>
#include <unistd.h>

#include <math.h>

#include <stdio.h>
#include <time.h>

struct timespec last_call_time = {0, 0};

float target_function() {
    struct timespec current_time;
    clock_gettime(CLOCK_MONOTONIC_RAW, &current_time);
    
	float elapsed = 0.0f;

    if (last_call_time.tv_sec != 0 || last_call_time.tv_nsec != 0) {
        long elapsed_sec = current_time.tv_sec - last_call_time.tv_sec;
        long elapsed_nsec = current_time.tv_nsec - last_call_time.tv_nsec;
        if (elapsed_nsec < 0) {
            elapsed_sec--;
            elapsed_nsec += 1000000000; // 1 second in nanoseconds
        }
        long elapsed_us = elapsed_sec * 1000000L + elapsed_nsec / 1000L;
		long elapsed_ms = elapsed_sec * 1000L + elapsed_nsec / 1000000L;
        elapsed = elapsed_ms / 1000.0f;
    }
    
    last_call_time = current_time;

	return elapsed;
}

void test3(uint64_t timestamp,
		   device3_event_type event,
		   const device3_ahrs_type* ahrs) {
	static device3_quat_type old;
	static device3_euler_type old_e;
	static float dmax = -1.0f;
	
	float dt = target_function();

	if (event != DEVICE3_EVENT_UPDATE) {
		return;
	}

	device3_quat_type q = device3_get_orientation(ahrs);
	
	const float dx = (old.x - q.x) * (old.x - q.x);
	const float dy = (old.y - q.y) * (old.y - q.y);
	const float dz = (old.z - q.z) * (old.z - q.z);
	const float dw = (old.w - q.w) * (old.w - q.w);
	
	const float d = sqrtf(dx*dx + dy*dy + dz*dz + dw*dw);
	
	device3_euler_type e = device3_get_euler(q);
	printf("{"
			"\"roll\": %.4f, "
			"\"pitch\": %.4f, "
			"\"yaw\": %.4f, "
			"\"dRoll\": %.4f, "
			"\"dPitch\": %.4f, "
			"\"dYaw\": %.4f, "
			"\"dt\": %.4f "
		"}\n", e.roll, e.pitch, e.yaw, e.roll - old_e.roll, e.pitch - old_e.pitch, e.yaw - old_e.yaw, dt);

	// printf("{"
	// 		"\"x\": %.4f, "
	// 		"\"y\": %.4f, "
	// 		"\"z\": %.4f, "
	// 		"\"w\": %.4f, "
	// 		"\"dx\": %f, "
	// 		"\"dy\": %f, "
	// 		"\"dz\": %f, "
	// 		"\"dw\": %f"
	// 	"}\n", q.x, q.y, q.z, q.w, dx, dy, dz, dw);
	
	old = q;
	old_e = e;
}

void test4(uint64_t timestamp,
		   device4_event_type event,
		   uint8_t brightness,
		   const char* msg) {
	switch (event) {
		case DEVICE4_EVENT_MESSAGE:
			printf("Message: `%s`\n", msg);
			break;
		case DEVICE4_EVENT_BRIGHTNESS_UP:
			printf("Increase Brightness: %u\n", brightness);
			break;
		case DEVICE4_EVENT_BRIGHTNESS_DOWN:
			printf("Decrease Brightness: %u\n", brightness);
			break;
		default:
			break;
	}
}

#define POLL_FREQUENCY_MS 10  // Desired polling frequency in milliseconds

// Function to perform rate-limited polling
void rate_limited_polling(device3_type* dev3) {
    struct timespec start, end;
    long time_diff_ns;
    long sleep_time_ms;

    while (DEVICE3_ERROR_NO_ERROR == device3_read(dev3, -1)) {
        usleep(POLL_FREQUENCY_MS * 1000);
    }
}

int main(int argc, const char** argv) {
	pid_t pid = fork();
	
	if (pid == -1) {
		perror("Could not fork!\n");
		return 1;
	}
	
	if (pid == 0) {
		device3_type dev3;
		if (DEVICE3_ERROR_NO_ERROR != device3_open(&dev3, test3)) {
			printf("It's not type 3.\n");
			return 1;
		}

		device3_clear(&dev3);
		device3_calibrate(&dev3, 1000, true, true, false);
		//while (DEVICE3_ERROR_NO_ERROR == device3_read(&dev3, -1));
		rate_limited_polling(&dev3);
		device3_close(&dev3);
		return 0;
	} else {
		int status = 0;

		device4_type dev4;
		if (DEVICE4_ERROR_NO_ERROR != device4_open(&dev4, test4)) {
			status = 1;
			printf("It's not type 4.\n");
			goto exit;
		}

		device4_clear(&dev4);
		while (DEVICE4_ERROR_NO_ERROR == device4_read(&dev4, -1));
		device4_close(&dev4);
		
	exit:
		if (pid != waitpid(pid, &status, 0)) {
			return 1;
		}
		
		return status;
	}
}
