/****************************************************************************
 *
 *   Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
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

/**
 * @file pioneer_flow.hpp
 * @author Nikita Maslov <wkernelteam@gmail.com>
 *
 * Driver for the Geoscan Pioneer optical flow sensor connected via I2C.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/device/i2c.h>
#include <conversion/rotation.h>
#include <lib/perf/perf_counter.h>
#include <lib/parameters/param.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/distance_sensor.h>
#include <px4_platform_common/i2c_spi_buses.h>

/* Configuration Constants */

#define PIONEER_OPTFLOW_BASEADDR 0x09
#define PIONEER_OPTFLOW_SIZE 9

class PioneerFlow : public device::I2C, public I2CSPIDriver<PioneerFlow>
{
public:
	PioneerFlow(I2CSPIBusOption bus_option, int bus, enum Rotation yaw_rotation, int bus_frequency,
	        int address = PIONEER_OPTFLOW_BASEADDR);

	virtual ~PioneerFlow();

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	virtual int init();

	void print_status();

	void RunImpl();

protected:
	virtual int probe();

private:

	const uint64_t _collect_time{15000}; // usecs, ensures flow data is published every second iteration of Run() (100Hz -> 50Hz)

	uORB::PublicationMulti<optical_flow_s> _optical_flow_pub{ORB_ID(optical_flow)};
	uORB::PublicationMulti<distance_sensor_s> _distance_sensor_pub{ORB_ID(distance_sensor)};

        int _class_instance{-1};

	perf_counter_t _sample_perf;
	perf_counter_t _comms_errors;

	uint64_t _previous_collect_timestamp{0};

	enum Rotation _yaw_rotation;

	int _flow_sum_x{0};
	int _flow_sum_y{0};
	uint64_t _flow_dt_sum_usec{0};
	uint16_t _flow_quality_sum{0};
	uint8_t _flow_sample_counter{0};

	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	void start();

	/**
	* Stop the automatic measurement state machine.
	*/
	void stop();

        int measure(int8_t* dx = nullptr, int8_t* dy = nullptr, uint8_t* quality = nullptr, int* height = nullptr);
};
