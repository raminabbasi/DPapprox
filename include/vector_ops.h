/* This file is part of DPapprox
 * Copyright (C) 2025 Ramin Abbasi Esfeden.
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * This software is based on research described in:
 * "A Dynamic Programming-Inspired Approach for Mixed Integer Optimal Control Problems with Dwell Time Constraints,"
 * by Ramin Abbasi Esfeden, Christoph Plate, Sebastian Sager, Jan Swevers, 2025.
 */

#ifndef DPAPPROX_VECTOR_OPS_H
#define DPAPPROX_VECTOR_OPS_H

#include <vector>
#include "config.h"

/*
 * overloading +, -, * for vectors.
 */

namespace DPapprox {

std::vector<double> operator+(const std::vector<double>&, const std::vector<double>&);
std::vector<double> operator-(const ProblemConfig::disc_vector&, const std::vector<double>&);
std::vector<double> operator*(const std::vector<double>&, double);

}
#endif 
