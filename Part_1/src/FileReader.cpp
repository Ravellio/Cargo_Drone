#include "inc/FileReader.hpp"

FileReader::FileReader(void) { // Default contructor
    csv = {};
}

FileReader::FileReader(std::string path) { // Contructor
    csv = read(path); // Intializing the csv variable
}

std::map<float, std::vector<float>> FileReader::read(std::string path){ // Reading the csv file
    std::map<float, std::vector<float>> result; // Declaring the result of the read function
    std::string line; // Memory allocation for reading every csv file line
    std::ifstream file(path); // File manager object

    if (!file) { // Check if the provided directory exists 
        throw "Dynamics input file is NOT read";
    }

    float value; // Memory allocation for reading every csv file value
    
    while (std::getline(file, line)) { // Reading the csv file line by line
        std::stringstream stream(line); // Declaring a stream object
        
        bool isFirstItem = true; // Flag for first csv file row (used for headers)
        float t; // Time entry
        std::vector<float> u; // Vector of values corresponding to that time
        
        while (stream >> value) { // Reading elements from one line
            if (isFirstItem) {
                t = value;
                isFirstItem = false;
            }
            else {
                u.push_back(value);
            }
    
            if (stream.peek() == ',') { 
                stream.ignore(); // Ignore the commas
            }
        }
        result.insert(std::make_pair(t, u)); // Creating a pair of time entry and the vector
    }
    
    file.close();
    return result;
}

std::vector<float> FileReader::get(float t) const { // Getting a vector based on the time entry: Note that for t1 < t <= t2, u(t) = u(t1)
    if (csv.size() == 0) { // Check if the csv file is empty
        std::cout << "File is empty!\n";
        return {};
    }
    auto itr1 = csv.find(t); // Get the vector based on the provided time entry
    if (itr1 == csv.end()) { // If the time entry does not exist in the csv file
        auto itr2 = csv.upper_bound(t); // Get the vector based on the next time entry
        if (itr2 == csv.begin()) { // Check the the time value is smaller than the smallest time value in the csv file
            return itr2->second; // Return the vector corresponding to the first time
        }
        return (--itr2)->second; // Since upper_bound is used, we need to subtract one time unit to achive the required interpolation
    }
    return itr1->second; // Return the vector corresponding to the exact time
}