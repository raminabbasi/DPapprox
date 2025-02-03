#ifndef IO_UTILS_H
#define IO_UTILS_H

#include <vector>
#include <string>

// Function to read a CSV file into a 2D vector
std::vector<std::vector<double>> read_csv(const std::string& filename);

// Function to write a 2D vector to a CSV file
void write_csv(const std::string& filename, std::vector<int> data);
#endif // IO_UTILS_H