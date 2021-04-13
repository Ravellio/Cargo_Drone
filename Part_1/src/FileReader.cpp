#include "inc/FileReader.hpp"

FileReader::FileReader(void) {
    csv = {};
}

FileReader::FileReader(std::string path) {
    csv = read(path);
}

std::map<float, std::vector<float>> FileReader::read(std::string path){
    std::map<float, std::vector<float>> result;
    std::string line;
    std::ifstream file(path);

    if (!file) {
        throw "Dynamics input file is NOT read";
    }

    float value;
    
    while (std::getline(file, line)) {
        std::stringstream stream(line);
        
        bool isFirstItem = true;
        float t;
        std::vector<float> u;
        
        while (stream >> value) {
            if (isFirstItem) {
                t = value;
                isFirstItem = false;
            }
            else {
                u.push_back(value);
            }
    
            if (stream.peek() == ',') {
                stream.ignore();
            }
        }
        result.insert(std::make_pair(t, u));
    }
    
    file.close();
    return result;
}

std::vector<float> FileReader::get(float t) {
    if (csv.size() == 0) {
        std::cout << "File is empty!\n";
        return {};
    }
    auto itr1 = csv.find(t);
    if (itr1 == csv.end()) {
        auto itr2 = csv.upper_bound(t);
        if (itr2 == csv.begin()) {
            return itr2->second;
        }
        return (--itr2)->second;
    }
    return itr1->second;
}
