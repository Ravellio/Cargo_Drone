#ifndef FILEWRITER_HPP
#define FILEWRITER_HPP

#include <string>
#include <vector>
#include <iostream>
#include <fstream>

class FileWriter {
public:
	template <typename T> // Template is used since more than one type can be written to a csv file
	static void write(const std::string& path, const std::vector<std::string>& headers, const std::vector<std::vector<T>>& data) { // Writing to a csv file
		std::ofstream file(path); // File manager object

		if (!file) { // If file path is wrong, nothing is written to a file
			std::cout << "Wrong write path name: Dynamics log file is NOT saved";
			return;
		}

		for (size_t i = 0; i < headers.size(); i++) { // First write the headers
			if (i != 0) {
				file << ","; // Insert a comma since we are writing to a csv file
			}
			file << headers[i];
		}

		file << "\n";

		for (auto row : data) { // For every data row
			for (size_t i = 0; i < row.size(); i++) { // For every element in a row
				if (i != 0) {
					file << ","; // Insert a comma since we are writing to a csv file
				}
				file << row[i];
			}
			file << "\n";
		}
		file.close();
		std::cout << "\nDynamics log file saved!\n"; // Write success
	}
};

#endif

