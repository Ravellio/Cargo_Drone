#ifndef FILEREADER_HPP
#define FILEREADER_HPP

#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <map>

class FileReader {
private:
    std::map<float, std::vector<float>> csv;
    std::map<float, std::vector<float>> read(std::string path);
    
public:
    FileReader(void);
    FileReader(std::string path);
    std::vector<float> get(float t) const;
};

#endif
