/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "pioneer_flow.hpp"

static constexpr uint32_t TIME_us_TSWW = 11; //  - actually 10.5us
static const uint32_t PIONEERFLOW_US = 1000;  // delay 1 ms
static const uint32_t PIONEERFLOW_SAMPLE_INTERVAL = 20000;  // interval 20ms (50 Hz)

PioneerFlow::PioneerFlow(I2CSPIBusOption bus_option, int bus, enum Rotation yaw_rotation, int bus_frequency,
		 int address) :
	I2C(DRV_FLOW_DEVTYPE_PIONEERFLOW, MODULE_NAME, bus, address, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	_sample_perf(perf_alloc(PC_ELAPSED, "pflow: read")),
	_comms_errors(perf_alloc(PC_COUNT, "pflow: com err")),
	_yaw_rotation(yaw_rotation)
{
}

PioneerFlow::~PioneerFlow()
{
	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int PioneerFlow::measure(int8_t* dx, int8_t* dy, uint8_t* quality, int* height)
{
        perf_begin(_sample_perf);

        int ret = PX4_ERROR;
        uint8_t buffer[PIONEER_OPTFLOW_SIZE];

        ret = transfer(nullptr, 0, &buffer[0], sizeof(buffer));
        if (OK != ret) {
                perf_count(_comms_errors);
                PX4_DEBUG("i2c::transfer returned %d", ret);
                return ret;
        }

        perf_end(_sample_perf);

        uint8_t i_quality = buffer[1];

        int8_t i_dx = (int8_t)buffer[2];
        int8_t i_dy = (int8_t)buffer[3];

        int16_t i_height_mm = (int16_t)(buffer[4] | buffer[5] << 8);

        if (dx != nullptr) {
                *dx = i_dx;
                *dy = i_dy;
                *quality = i_quality;
                *height = i_height_mm;
        }

        return ret;
}

int
PioneerFlow::init()
{
	// get yaw rotation from sensor frame to body frame
	param_t rot = param_find("SENS_FLOW_ROT");

	if (rot != PARAM_INVALID) {
		int32_t val = 0;
		param_get(rot, &val);

		_yaw_rotation = (enum Rotation)val;
	}

        if (I2C::init() != OK) {
                return PX4_ERROR;
        }

        _class_instance = register_class_devname(RANGE_FINDER_BASE_DEVICE_PATH);

	_previous_collect_timestamp = hrt_absolute_time();

	start();

	return PX4_OK;
}

int
PioneerFlow::probe()
{
        return measure();
}

void
PioneerFlow::RunImpl()
{
	perf_begin(_sample_perf);

	int8_t delta_x_raw = 0;
	int8_t delta_y_raw = 0;
	uint8_t qual = 0;
	float delta_x = 0.0f;
	float delta_y = 0.0f;
        int height_mm = 0;

	uint64_t timestamp = hrt_absolute_time();
	uint64_t dt_flow = timestamp - _previous_collect_timestamp;
	_previous_collect_timestamp = timestamp;

	_flow_dt_sum_usec += dt_flow;

        int res = measure(&delta_x_raw, &delta_y_raw, &qual, &height_mm);

        if (OK != res) {
                PX4_DEBUG("measure failed");
                perf_end(_sample_perf);
                return;
        }

	if (qual > 0) {
		_flow_sum_x += delta_x_raw;
		_flow_sum_y += delta_y_raw;
		_flow_sample_counter ++;
		_flow_quality_sum += qual;
	}

	// returns if the collect time has not been reached
	if (_flow_dt_sum_usec < _collect_time) {
		return;
	}

        // TODO: check factors
	delta_x = (float)_flow_sum_x / 500.0f;		// proportional factor + convert from pixels to radians
	delta_y = (float)_flow_sum_y / 500.0f;		// proportional factor + convert from pixels to radians

	optical_flow_s report{};
	report.timestamp = timestamp;

	report.pixel_flow_x_integral = static_cast<float>(delta_x);
	report.pixel_flow_y_integral = static_cast<float>(delta_y);

	// rotate measurements in yaw from sensor frame to body frame according to parameter SENS_FLOW_ROT
	float zeroval = 0.0f;
	rotate_3f(_yaw_rotation, report.pixel_flow_x_integral, report.pixel_flow_y_integral, zeroval);
	rotate_3f(_yaw_rotation, report.gyro_x_rate_integral, report.gyro_y_rate_integral, report.gyro_z_rate_integral);

	report.frame_count_since_last_readout = _flow_sample_counter;	// number of frames
	report.integration_timespan = _flow_dt_sum_usec; 	// microseconds

	report.sensor_id = 0;
	report.quality = _flow_sample_counter > 0 ? _flow_quality_sum / _flow_sample_counter : 0;

        // height
        report.ground_distance_m = static_cast<float>(height_mm) / 1000.0f;  // convert to meters

	/* No gyro on this board */
	report.gyro_x_rate_integral = NAN;
	report.gyro_y_rate_integral = NAN;
	report.gyro_z_rate_integral = NAN;

	// set (conservative) specs according to datasheet
	report.max_flow_rate = 5.0f;       // Datasheet: 7.4 rad/s
	report.min_ground_distance = 0.1f; // Datasheet: 80mm
	report.max_ground_distance = 1.5f; // Datasheet: infinity

	_flow_dt_sum_usec = 0;
	_flow_sum_x = 0;
	_flow_sum_y = 0;
	_flow_sample_counter = 0;
	_flow_quality_sum = 0;

	_optical_flow_pub.publish(report);

        if (_class_instance == CLASS_DEVICE_PRIMARY) {
	        distance_sensor_s distance_report{};
		distance_report.timestamp = report.timestamp;
		distance_report.min_distance = 0.0f;
		distance_report.max_distance = 1.5f;
		distance_report.current_distance = report.ground_distance_m;
		distance_report.variance = 0.0f;
		distance_report.signal_quality = -1;
		distance_report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
		/* TODO: the ID needs to be properly set */
		distance_report.id = 0;
		distance_report.orientation = _yaw_rotation;

		_distance_sensor_pub.publish(distance_report);
        }

	perf_end(_sample_perf);
}

void
PioneerFlow::start()
{
	// schedule a cycle to start things
        PX4_ERR("start required");
	ScheduleOnInterval(PIONEERFLOW_SAMPLE_INTERVAL, PIONEERFLOW_US);
}

void
PioneerFlow::stop()
{
	ScheduleClear();
}

void
PioneerFlow::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
