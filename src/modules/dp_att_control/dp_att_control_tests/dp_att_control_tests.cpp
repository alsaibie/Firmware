/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Simon Wilks <sjwilks@gmail.com>
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
 * @file dp_att_control_tests.cpp
 * Commander unit tests. Run the tests as follows:
 *   nsh> dp_att_control_tests
 *
 */

#include <systemlib/err.h>
#include <unit_test/unit_test.h>
#include <mathlib/mathlib.h>
#include <fstream>
#define PI 3.1415926535
#define TEST_DATA_PATH "./test_data/"
extern "C" __EXPORT int dp_att_control_tests_main(int argc, char *argv[]);

bool dpAttControlTests();

#include "../dp_att_control.hpp"

class DpAttControlTests : public UnitTest
{
public:
	DpAttControlTests();
	virtual ~DpAttControlTests();

	virtual bool run_tests();

private:
	bool dp_mixer_test();
  DolphinAttitudeControl *att_control;
};

DpAttControlTests::DpAttControlTests()
{
  att_control = new DolphinAttitudeControl();
}

DpAttControlTests::~DpAttControlTests()
{
}

bool DpAttControlTests::dp_mixer_test() {
  PX4_INFO("DP Mixer Test");
  // Arrange
  /* Generate dummy data */
  FILE *fp, *fp_out;
  fp = fopen(TEST_DATA_PATH "dp_att_control/mixer_input.txt", "rt");
  ut_test(fp);

  fp_out = fopen(TEST_DATA_PATH "dp_att_control/mixer_output.txt", "wt");
  ut_test(fp_out);
  float roll, pitch, yaw, thrust;
  int ret;

  while (EOF != (ret = fscanf(fp, "%f,%f,%f,%f", &roll, &pitch, &yaw, &thrust))) {

  // Act
  math::Vector<3> att_ctrl(roll, pitch, yaw);
  float thrust_sp = thrust;
  math::Vector<4> mixed_att_ctrl(0.0f, 0.0f, 0.0f, 0.0f);
  att_control->mix_control_output(att_ctrl, thrust_sp, mixed_att_ctrl);
//  PX4_INFO("Mixed Output %.6f, %.6f, %.6f, %.6f", (double) mixed_att_ctrl(0), (double) mixed_att_ctrl(1),
//           (double) mixed_att_ctrl(2), (double) mixed_att_ctrl(3));
  fprintf(fp_out, "%f,%f,%f,%f\n",(double) mixed_att_ctrl(0), (double) mixed_att_ctrl(1),
          (double) mixed_att_ctrl(2), (double) mixed_att_ctrl(3));
}
  fclose(fp);
  fclose(fp_out);
  ut_test(ret == EOF);
  // Assert



	return true;
}

bool DpAttControlTests::run_tests()
{
	ut_run_test(dp_mixer_test);

	return (_tests_failed == 0);
}

ut_declare_test(dpAttControlTests, DpAttControlTests);

int dp_att_control_tests_main(int argc, char *argv[])
{
	return dpAttControlTests() ? 0 : -1;
}
