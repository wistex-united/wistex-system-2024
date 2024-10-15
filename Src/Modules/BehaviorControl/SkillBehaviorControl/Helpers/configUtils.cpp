#include "Modules/BehaviorControl/SkillBehaviorControl/Helpers/configUtils.h"
#include <iostream>
#include <string> 
#include <stdio.h>
#include <fstream>
#include <regex>


bool stringToBool(const std::string& str) {
    std::string lowerStr;
    for (char ch : str) {
        lowerStr += std::tolower(ch);
    }
    if (lowerStr == "true" || lowerStr == "yes" || lowerStr == "1" || lowerStr == "t") {
        return true;
    } else if (lowerStr == "false" || lowerStr == "no" || lowerStr == "0" || lowerStr == "f") {
        return false;
    } else {
        throw std::invalid_argument("Invalid boolean string: " + str);
    }
}

int display_map(std::map<std::string, std::string> m)
{
    for (auto it = m.begin(); it != m.end(); ++it)
    {
        std::cout << it->first << " :: " << it->second << std::endl;
    }
    return 0;
}

std::map<std::string, std::string> parse_cfg_file(const std::string& filename)
{
    std::map<std::string, std::string> config;

    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Error opening file: " << filename << std::endl;
        return config;
    }

    std::string line;
    while (std::getline(file, line))
    {
        // Skip empty lines and comment lines
        if (line.empty() || line[0] == '#')
            continue;

        // Position of the delimiter '='
        size_t delimiter_pos = line.find('=');
        if (delimiter_pos == std::string::npos)
        {
            std::cerr << "Invalid line in config file: " << line << std::endl;
            continue;
        }

        std::string key = line.substr(0, delimiter_pos);
        std::string value = line.substr(delimiter_pos + 1);

        key.erase(0, key.find_first_not_of(" \t"));
        key.erase(key.find_last_not_of(" \t") + 1);
        value.erase(0, value.find_first_not_of(" \t"));
        value.erase(value.find_last_not_of(" \t") + 1);

        
        config[key] = value;
    }

    file.close();
    return config;
}

std::map<int, std::vector<float>> parse_strategy_file(const std::string& filename) {
    std::ifstream file(filename);
    std::string line;
    int strategyType = 0;
    std::map<int, std::vector<float>> robotPositions;

    if (!file.is_open()) {
        std::cerr << "Unable to open file: " << filename << std::endl;
        return robotPositions;
    }

    // Determine strategy type
    std::regex typeRegex("strategy_type=(\\d)");
    std::regex posRegex("robot(\\d+)_([a-z]+)_(x|y)=(\\-?\\d+\\.?\\d*)");
    std::smatch matches;
    std::string strategy;

    while (getline(file, line)) {
        if (std::regex_search(line, matches, typeRegex)) {
            strategyType = std::stoi(matches[1]);
            switch (strategyType) {
                case 1: strategy = "offense"; break;
                case 2: strategy = "neutral"; break;
                case 3: strategy = "defense"; break;
                default: std::cerr << "Invalid strategy type found." << std::endl; return robotPositions;
            }
        }
    }

    // Reset file to start to parse positions
    file.clear();
    file.seekg(0);

    while (getline(file, line)) {
        if (std::regex_search(line, matches, posRegex) && matches[2] == strategy) {
            int robotNum = std::stoi(matches[1]);
            float value = std::stof(matches[4]);
            if (robotPositions.find(robotNum) == robotPositions.end()) {
                robotPositions[robotNum] = std::vector<float>(2, 0.0f); // Initialize with 0.0 for both x and y
            }
            if (matches[3] == "x") {
                robotPositions[robotNum][0] = value;
            } else { // "y"
                robotPositions[robotNum][1] = value;
            }
        }
    }

    file.close();
    return robotPositions;
}