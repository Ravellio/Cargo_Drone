#ifndef FILEREADER_HPP
#define FILEREADER_HPP

#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <map>

class FileReader { 
private:
    std::map<float, std::vector<float>> csv; // Declaring a variable that stores csv file valued
    std::map<float, std::vector<float>> read(std::string path); // Reading a csv file
    
public:
    FileReader(void); // Default contructor
    FileReader(std::string path); // Constructor
    std::vector<float> get(float t) const; // Getting a vector of values from the csv variable given the time entry
};

#endif
