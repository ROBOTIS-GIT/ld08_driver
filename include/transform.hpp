// Copyright 2021 ROBOTIS CO., LTD.
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
//
// Author: LD Robot, Will Son

#ifndef TRANSFORM_HPP_
#define TRANSFORM_HPP_

#include <math.h>
#include <vector>
#include <algorithm>
#include "../include/lipkg.hpp"

enum class LDVersion
{
  LD_ZERO,  /*Zero  generation lidar*/
  LD_THREE, /*Third generation lidar*/
  LD_EIGHT, /*Eight generation radar*/
  LD_NINE,  /*Nine  generation radar*/
};

class SlTransform
{
private:
  bool to_right_hand = true;
  double offset_x;
  double offset_y;

public:
  explicit SlTransform(LDVersion version);
  Points2D Transform(const Points2D & data);
  ~SlTransform();
};


#endif  // TRANSFORM_HPP_
