#include "RLVars.h"

#include <map>
#include <string>
#include <iostream>

// Get policy variables
std::map<std::string, int> get_policy_vars(std::string policy_type) {
    // TODO: This can be optimized but for some reason I can't get these defined OUTSIDE the function,
    // though cpp might not make a difference performance wise

    // Define policy variables
    std::map<std::string, int> on_ball_vars = {
        {"history_length", 3},
        {"num_teammates", 1},
        {"num_opponents", 0}
    };

    std::map<std::string, int> goalie_vars = {
         {"history_length", 0},
         {"individual_observation_length", 12},
     };

    std::map<std::string, int> off_ball_vars = {
        {"history_length", 3},
        {"num_teammates", 2},
        {"num_opponents", 0}
    };

    std::map<std::string, int> walk_to_ball_vars = {
        {"history_length", 3},
        {"num_teammates", 0},
        {"num_opponents", 0}
    };

    std::map<std::string, int> near_opp_goal_vars = {
        {"history_length", 0},
        {"num_teammates", 0},
        {"num_opponents", 0}
    };

    // Define policy map
    std::map<std::string, std::map<std::string, int>> policy_vars = {
        {"walk_to_ball", walk_to_ball_vars},
        {"on_ball", on_ball_vars},
        {"off_ball", off_ball_vars},
        {"goalie", goalie_vars},
        {"near_opp_goal", near_opp_goal_vars}
    };

    // print policy)type
    // std::cout << "Policy type: " << policy_type << std::endl;

    return policy_vars.at(policy_type);
}
