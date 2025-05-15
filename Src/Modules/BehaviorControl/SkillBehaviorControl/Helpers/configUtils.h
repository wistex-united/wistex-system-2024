#include <string> 
#include <map>

bool stringToBool(const std::string& str);
int display_map(std::map<std::string, std::string> m);
std::map<std::string, std::string> parse_cfg_file(const std::string& filename);
std::map<int, std::vector<float>> parse_strategy_file(const std::string& filename);
