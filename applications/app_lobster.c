/*
	Copyright 2019 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "app.h"
#include "ch.h"
#include "hal.h"

// Some useful includes
#include "mc_interface.h"
#include "mcpwm_foc.h"
#include "utils_math.h"
#include "encoder/encoder.h"
#include "terminal.h"
#include "comm_can.h"
#include "hw.h"
#include "commands.h"
#include "timeout.h"
#include "buffer.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

// Threads
static THD_FUNCTION(my_thread, arg);
static THD_WORKING_AREA(my_thread_wa, 1024);

// Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;


// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_custom_start(void) {

	stop_now = false;
	chThdCreateStatic(my_thread_wa, sizeof(my_thread_wa),
			NORMALPRIO, my_thread, NULL);


}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_custom_stop(void) {

	stop_now = true;
	while (is_running) {
		chThdSleepMilliseconds(1);
	}
}

void app_custom_configure(app_configuration *conf) {
	(void)conf;
}



static THD_FUNCTION(my_thread, arg) {
	(void)arg;

	chRegSetThreadName("Lobster state");

	is_running = true;

    uint32_t controller_id = app_get_configuration()->controller_id;

    for(;;) {
		// Check if it is time to stop.
		if (stop_now) {
			is_running = false;
			return;
		}

//		timeout_reset(); // Reset timeout if everything is OK.

        // From data:
        {
            uint8_t msg_data[8];
            int32_t index = 0;

            float volt_d = mcpwm_foc_get_vd();
            utils_truncate_number(&volt_d, -100.0, 100.0);
            buffer_append_float16(msg_data, volt_d, 300.0, &index);

            float volt_q = mcpwm_foc_get_vq();
            utils_truncate_number(&volt_q, -100.0, 100.0);
            buffer_append_float16(msg_data, volt_q, 300.0, &index);

            float curr_d = mcpwm_foc_get_id();
            utils_truncate_number(&curr_d, -100.0, 100.0);
            buffer_append_float16(msg_data, curr_d, 300.0, &index);

            float curr_q = mcpwm_foc_get_iq();
            utils_truncate_number(&curr_q, -100.0, 100.0);
            buffer_append_float16(msg_data, curr_q, 300.0, &index);

            comm_can_transmit_sid(controller_id | 0x0400, msg_data, index);
        }
        {
            uint8_t msg_data[8];
            int32_t index = 0;

            float volt_in = mc_interface_get_input_voltage_filtered();
            utils_truncate_number(&volt_in, -100.0, 100.0);
            buffer_append_float16(msg_data, volt_in, 300.0, &index);

            float curr_motor = mc_interface_get_tot_current_filtered();
            utils_truncate_number(&curr_motor, -100.0, 100.0);
            buffer_append_float16(msg_data, curr_motor, 300.0, &index);

            float erpm = mc_interface_get_rpm();
            utils_truncate_number(&erpm, -325000.0, 325000.0);
            buffer_append_float16(msg_data, erpm / 10, 1.0, &index);

            float duty = mc_interface_get_duty_cycle_now();
            utils_truncate_number(&duty, -100.0, 100.0);
            buffer_append_float16(msg_data, duty, 300.0, &index);

            comm_can_transmit_sid(controller_id | 0x0500, msg_data, index);
        }
        {
            uint8_t msg_data[8];
            int32_t index = 0;

            float ah_used = mc_interface_get_amp_hours(false);
            utils_truncate_number(&ah_used, 0.0, 325.0);
            buffer_append_uint16(msg_data, (uint16_t)(ah_used * 200.0), &index); //bc unsigned float

            mc_fault_code fault = mc_interface_get_fault();
            msg_data[index++] = fault;

            float temp_mosfet = mc_interface_temp_fet_filtered();
            msg_data[index++] = (uint8_t)(temp_mosfet + 51.0);

            comm_can_transmit_sid(controller_id | 0x0600, msg_data, index);
        }

//        data.ah_used = mc_interface_get_amp_hours(false);
//        data.fault_code = mc_interface_get_fault();
//        data.temp_mos_max = mc_interface_temp_fet_filtered();


        // Run your logic here. A lot of functionality is available in mc_interface.h.
//        mc_fault_code fault = mc_interface_get_fault();  //29 faults -> 5 bit min
//        float motor_current =  mcpwm_foc_get_tot_current_directional_filtered(); // +-100A => 200A range @0.1A resolution => 11bit min
//        float rpm = mcpwm_foc_get_rpm(); //+-300k => 600k @ 100rpm range => 13 bits min
//        float temp_mosfet = mc_interface_temp_fet_filtered(); //-50 +150 => 200degr @ 1deg => 8 bit
//        float amp_hours = mc_interface_get_amp_hours(false); // 0 to 100 @ .1 => 1000 => 10 bits
//
//
//        uint8_t msg_data[8];
//        int32_t index = 0;
//
//        buffer_append_float16(msg_data, motor_current, 200.0, &index);
//        buffer_append_float16(msg_data, rpm / 10.0, 1, &index);
//        buffer_append_uint16(msg_data, (uint16_t)(amp_hours * 200.0), &index); //bc unsigned float
//        msg_data[index++] = (uint8_t)(temp_mosfet + 51.0);
//        msg_data[index++] = fault;
//
//
//        comm_can_transmit_sid(controller_id | 0x0300, msg_data, 8);

		chThdSleepMilliseconds(9); //111 hz -> backbone will be 100hz
	}
}


