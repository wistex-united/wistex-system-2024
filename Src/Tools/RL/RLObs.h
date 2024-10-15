#ifndef RL_OBS
#define RL_OBS

#include <vector>
#include <string>
#include <map>

class Observation {
private:
    int history_length;
    std::map<int, std::vector<float>> strategy_positions;
    std::map<int, std::vector<std::vector<float>>> history;
    bool can_kick(std::vector<float> self, std::vector<float> ball);
    std::vector<float> get_default_observation(
        std::vector<float> self,
        std::vector<float> ball,
        int robot_num
    );
    std::vector<float> get_on_ball_observation(
        std::vector<float> self,
        std::vector<float> ball,
        std::vector<float> target,
        int robot_num
    );
    std::vector<float> get_goalie_observation(
        std::vector<float> self,
        std::vector<float> ball,
        int robot_num
    );
    std::vector<float> get_off_ball_observation(
        std::vector<float> self,
        std::vector<float> ball,
        std::vector<std::vector<float>> teammates,
        std::vector<std::vector<float>> opponents,
        int robot_num,
        int on_ball_robot
    );
    std::vector<float> get_near_opp_goal_observation(
        std::vector<float> self,
        std::vector<float> ball,
        std::vector<std::vector<float>> opponents,
        int robot_num
    );
    std::vector<float> get_on_ball_dribble_observation(std::vector<float> agent_loc,
        std::vector<float> ball_loc,
        std::vector<std::vector<float>> teammate_loc,
        std::vector<std::vector<float>> opponent_loc,
        int robotNum
    );

public:
    Observation();
    std::vector<float> get_relative_observation(std::vector<float> self, std::vector<float> object_loc);
    std::vector<float> get_observation(
        std::vector<float> self,
        std::vector<float> ball,
        std::vector<float> target,
        std::vector<std::vector<float>> teammates,
        std::vector<std::vector<float>> opponents,
        int robot_num,
        std::string policy_type,
        int on_ball_robot,
        std::vector<float> agent_id_vector,
        std::vector<float> home_location,
        std::vector<float> closest_input_vector,
        bool use_closest_input_vector,
        bool use_agent_id_vector,
        bool use_home_location,
        bool opponentBlocking
    );
    std::vector<float> get_training_observation(
        std::vector<float> self,
        std::vector<float> ball
    );
    void step_history(
        std::vector<float> self,
        std::vector<float> ball,
        int robot_num
    );
    void init_strategy(std::map<int, std::vector<float>> positions);
    std::map<std::string, int> policy_vars;
};

float dist(const std::vector<float> a, const std::vector<float> b);
float vector_norm(const std::vector<float> a);
float vector_norm(float x1, float x2);
std::vector<float> get_target(std::vector<float> ball, float target_angle, float radius);

#endif /* RL_OBS */
