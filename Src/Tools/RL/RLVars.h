#ifndef RL_VARS_H
#define RL_VARS_H
// File to store RL inference variables

#include <map>
#include <string>
#include <vector>

// Declare the function
std::map<std::string, int> get_policy_vars(std::string policy_type);

#endif // RL_VARS_H
