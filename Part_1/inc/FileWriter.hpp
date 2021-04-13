#ifndef FILEWRITER_HPP
#define FILEWRITER_HPP

#include <string>
#include <vector>
#include <iostream>
#include <fstream>

class FileWriter {
public:
	template <typename T>
	static void write(const std::string& path, const std::vector<std::string>& headers, const std::vector<std::vector<T>>& data) {
		std::ofstream file(path);

		if (!file) {
			std::cout << "Wrong write path name: Dynamics log file is NOT saved";
			return;
		}

		for (int i = 0; i < headers.size(); i++) {
			if (i != 0) {
				file << ",";
			}
			file << headers[i];
		}

		file << "\n";

		for (auto row : data) {
			for (int i = 0; i < row.size(); i++) {
				if (i != 0) {
					file << ",";
				}
				file << row[i];
			}
			file << "\n";
		}
		file.close();
		std::cout << "\nDynamics log file saved!\n";
	}
};

#endif

