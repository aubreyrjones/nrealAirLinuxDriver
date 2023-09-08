//
// Created by thejackimonster on 05.04.23.
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

#include "device4.h"

#include <stdio.h>

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

int main(int argc, const char** argv) {
	device4_type dev4;
	if (DEVICE4_ERROR_NO_ERROR != device4_open(&dev4, test4)) {
		return 1;
	}
	
	device4_clear(&dev4);
	while (DEVICE4_ERROR_NO_ERROR == device4_read(&dev4, -1));
	device4_close(&dev4);
	return 0;
}
