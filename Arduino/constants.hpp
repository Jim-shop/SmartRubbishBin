#ifndef _CONSTANTS_HPP
#define _CONSTANTS_HPP

#include "config.hpp"

const float T = CONTROL_PERIOD;
const float q0 = Kp * (1 + T / Ti + Td / T);
const float q1 = -Kp * (1 + 2 * Td / T);
const float q2 = Kp * Td / T;
#endif