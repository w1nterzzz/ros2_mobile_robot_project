// Copyright 2026 Yang Yuexiang
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <limits>

#include "mobile_robot_localization/measurement_freshness.hpp"

namespace
{

using mobile_robot_localization::is_measurement_fresh;

TEST(MeasurementFreshnessTest, AcceptsRecentMeasurement)
{
  EXPECT_TRUE(
    is_measurement_fresh(9.9, 10.0, 0.2));
}

TEST(MeasurementFreshnessTest, AcceptsTimeoutBoundary)
{
  EXPECT_TRUE(
    is_measurement_fresh(9.8, 10.0, 0.2));
}

TEST(MeasurementFreshnessTest, RejectsStaleMeasurement)
{
  EXPECT_FALSE(
    is_measurement_fresh(9.7, 10.0, 0.2));
}

TEST(MeasurementFreshnessTest, RejectsFutureMeasurement)
{
  EXPECT_FALSE(
    is_measurement_fresh(10.1, 10.0, 0.2));
}

TEST(MeasurementFreshnessTest, RejectsNonFiniteInputs)
{
  const double nan =
    std::numeric_limits<double>::quiet_NaN();

  EXPECT_FALSE(
    is_measurement_fresh(nan, 10.0, 0.2));
  EXPECT_FALSE(
    is_measurement_fresh(9.9, nan, 0.2));
  EXPECT_FALSE(
    is_measurement_fresh(9.9, 10.0, nan));
}

TEST(MeasurementFreshnessTest, RejectsNonPositiveTimeout)
{
  EXPECT_FALSE(
    is_measurement_fresh(9.9, 10.0, 0.0));
  EXPECT_FALSE(
    is_measurement_fresh(9.9, 10.0, -0.2));
}

}  // namespace
