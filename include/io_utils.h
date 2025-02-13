#ifndef IO_UTILS_H
#define IO_UTILS_H

#include <vector>
#include <string>
#include "config.h"

// Function to read a CSV file into a 2D vector
std::vector<std::vector<double>> read_csv(const std::string& filename);

// Function to write a 2D vector to a CSV file
void write_csv(const std::string& filename, const std::vector<ProblemConfig::vtype>& data);

// Get a column
std::vector<double> get_column(const std::vector<std::vector<double>>& v, size_t col_index);
#endif // IO_UTILS_H