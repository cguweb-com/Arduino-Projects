/* This library code is placed under the MIT license
 * Copyright (c) 2020 Frank Bösing
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#if !defined(ARDUINO_TEENSY40) && !defined(ARDUINO_TEENSY41) && !defined(ARDUINO_TEENSYMM)
#error "This board is not supported."
#endif

#if !defined(T4PowerButton)
#define T4PowerButton

#include <Arduino.h>

#define SHOW_HARDFAULTS	 	// undef this, if not needed.
#define HARDFAULTSOUT Serial	// Output device for Hardfaults

enum arm_power_button_debounce {
	arm_power_button_debounce_0ms = 3,	//No debounce
	arm_power_button_debounce_50ms = 0,	//50ms debounce (default)
	arm_power_button_debounce_100ms = 1,	//100ms debounce
	arm_power_button_debounce_500ms = 2	//500ms debounce
};

enum arm_power_button_press_time_emergency {
	arm_power_button_press_time_emergency_5sec = 0,
	arm_power_button_press_time_emergency_10sec = 1,
	arm_power_button_press_time_emergency_15sec = 2,
	arm_power_button_press_time_emergency_off = 3
};

enum arm_power_button_press_on_time {		//Time to switch on
	arm_power_button_press_on_time_0ms = 3,
	arm_power_button_press_on_time_50ms = 1,
	arm_power_button_press_on_time_100ms = 2,
	arm_power_button_press_on_time_500ms = 0
};

enum callback_ex_action {
	callback_ex_action_poweroff = 1,
	callback_ex_action_poweroff_cancel = 0,
	callback_ex_action_poweroff_keeparmed = 2
};

void arm_reset(void); // reset
void arm_power_down(void); //switch off
void set_arm_power_button_callback(void (*fun_ptr)(void));
void set_arm_power_button_callback_ex(callback_ex_action (*fun_ptr)(void));
void set_arm_power_button_debounce(arm_power_button_debounce debounce);
void set_arm_power_button_press_time_emergency(arm_power_button_press_time_emergency emg);
void set_arm_power_button_press_on_time(arm_power_button_press_on_time ontime);
void rearm_power_button_callback(void);
void arm_enable_nvram(void);

bool arm_power_button_pressed(void);

unsigned memfree(void); //free stack / global variable space
unsigned heapfree(void); //heap heap space
void progInfo(void); //display file + vesion info
void flexRamInfo(void);
unsigned long maxstack(void); //maximal stack usage

#endif
