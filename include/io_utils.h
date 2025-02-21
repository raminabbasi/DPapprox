/* This file is part of DPapprox
 * Copyright (C) 2025 Ramin Abbasi Esfeden.
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * This software is based on research described in:
 * "A Dynamic Programming-Inspired Approach for Mixed Integer Optimal Control Problems with Dwell Time Constraints,"
 * by Ramin Abbasi Esfeden, Christoph Plate, Sebastian Sager, Jan Swevers, 2025.
 */

#ifndef IO_UTILS_H
#define IO_UTILS_H

#include <vector>
#include <string>
#include "config.h"

/*
 * utilities for reading and writing csv files.
 */
namespace DPapprox {


std::vector<std::vector<double>> read_csv(const std::string &filename);


void write_csv(const std::string &filename, const std::vector<std::vector<double>> &data);

/*
 * get_column : a utility function to get a column slice out of vectors.
 */

std::vector<double> get_column(const std::vector<std::vector<double>> &v, size_t col_index);

}
#endif
