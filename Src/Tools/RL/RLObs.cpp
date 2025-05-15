#include "RLObs.h"
#include "RLVars.h"

#include "Representations/Modeling/RobotPose.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Modules/BehaviorControl/SkillBehaviorControl/Helpers/configUtils.h"

#include <cmath>
#include <vector>
#include <iostream>
#include <string>

Observation::Observation() {}

std::vector<float> get_target(std::vector<float> ball, float target_angle, float radius) {
    float x = ball[0] + radius * std::cos(target_angle);
    float y = ball[1] + radius * std::sin(target_angle);
    float angle = std::atan2(ball[1] - y, ball[0] - x);
    return std::vector<float>{x, y, angle};
}

float dist(const std::vector<float> a, const std::vector<float> b) {
    return std::sqrt(std::pow(a[0] - b[0], 2) + std::pow(a[1] - b[1], 2));
}

float vector_norm(const std::vector<float> a) {
    return std::sqrt(std::pow(a[0], 2) + std::pow(a[1], 2));
}

float vector_norm(float x1, float x2) {
    const std::vector a = {x1, x2};
    return vector_norm(a);
}

std::vector<float> get_opp_closest_to_opp_goal(std::vector<std::vector<float>> opponents) {
    // return the location of the opponent (in global coordinates) who is closest
    // to the opponent goal, or {0, 0} if there are no opponents closer than {0, 0}
    std::vector<float> opp_goal = {4500, 0};
    std::vector<float> opp_goalie_loc = {0, 0};
    float min_dist = 1000000;
    for (const auto& opp_loc : opponents) {
        float dist_to_goal = dist(opp_loc, opp_goal);
        if (dist_to_goal < min_dist) {
            min_dist = dist_to_goal;
            opp_goalie_loc = opp_loc;
        }
    }
    return opp_goalie_loc;
}

void Observation::init_strategy(std::map<int, std::vector<float>> positions) {
    strategy_positions = positions;}

std::vector<float> Observation::get_relative_observation(std::vector<float> self, std::vector<float> object_loc) {
    // Get relative position of object to agent
    float x = object_loc[0] - self[0];
    float y = object_loc[1] - self[1];

    float angle = std::atan2(y, x) - self[2];

    // Rotate x, y by -agent angle
    float xprime = x * std::cos(-self[2]) - y * std::sin(-self[2]);
    float yprime = x * std::sin(-self[2]) + y * std::cos(-self[2]);

    return {xprime/10000, yprime/10000, std::sin(angle), std::cos(angle)};
}


std::vector<float> Observation::get_observation(
    std::vector<float> self,
    std::vector<float> ball,
    std::vector<float> target,
    std::vector<std::vector<float>> teammates,
    std::vector<std::vector<float>> opponents,
    int robot_num,
    std::string policy_type,
    int on_ball_robot,
    std::vector<float> agent_id_vector,
    std::vector<float> closest_input_vector,
    std::vector<float> home_location,
    bool use_closest_input_vector,
    bool use_agent_id_vector,
    bool use_home_location,
    bool opponentBlocking
)
{
    std::vector<float> observation;
    policy_vars = get_policy_vars(policy_type);
    history_length = policy_vars.at("history_length");

    if (policy_type == "on_ball"){
        if (!opponentBlocking)
            observation = get_on_ball_observation(self, ball, target, robot_num);
        else
            observation = get_on_ball_dribble_observation(self, ball, teammates, opponents, robot_num);
    }
    else if (policy_type == "near_opp_goal")
        observation = get_near_opp_goal_observation(self, ball, opponents, robot_num);
    else if (policy_type == "goalie")
        observation = get_goalie_observation(self, ball, robot_num);
    else if (policy_type == "off_ball")
        observation = get_off_ball_observation(self, ball, teammates, opponents, robot_num, on_ball_robot);
    else
        observation = get_default_observation(self, ball, robot_num);

    return observation;
}

std::vector<float> Observation::get_default_observation(std::vector<float> self, std::vector<float> ball, int robot_num) {
    std::vector<float> observation;

    // Get relative position of ball to agent
    std::vector<float> relative_observation = get_relative_observation(self, ball);
    observation.insert(observation.end(), relative_observation.begin(), relative_observation.end());

    // Get relative position to [4800, 0]
    std::vector<float> relative_observation_2 = get_relative_observation(self, {4800, 0});
    observation.insert(observation.end(), relative_observation_2.begin(), relative_observation_2.end());

    // Get relative position to [-4800, 0]
    std::vector<float> relative_observation_3 = get_relative_observation(self, {-4800, 0});
    observation.insert(observation.end(), relative_observation_3.begin(), relative_observation_3.end());

    // Add history
   for (int i = 0; i < history_length; i++) {
       if ((int)history.size() > i) {
           observation.insert(observation.end(), history[robot_num][i].begin(), history[robot_num][i].end());
       }
       else {
           observation.insert(observation.end(), 8, 0);
       }
   }

    return observation;
}

std::vector<float> Observation::get_on_ball_observation(
    std::vector<float> agent,
    std::vector<float> ball,
    std::vector<float> target,
    int player_number
)
{
    std::vector<float> observation;
    history_length = 3;

    // Init history for robot if it hasn't been initialized
    if (history.find(player_number) == history.end()) {
        history[player_number] = std::vector<std::vector<float>>();
    }
    while ((int) history[player_number].size() < history_length) {
        history[player_number].push_back({0, 0, 0, 0});
    }

    observation.push_back(can_kick(agent, ball) ? 1 : 0);

    std::vector<float> goal = get_relative_observation(target, {4500, 0});
    observation.insert(observation.end(), goal.begin(), goal.end());

    std::vector<float> agent_to_target = get_relative_observation(target, agent);
    observation.insert(observation.end(), agent_to_target.begin(), agent_to_target.end());

    std::vector<float> opp_post_1 = get_relative_observation(target, {4500, 500});
    observation.insert(observation.end(), opp_post_1.begin(), opp_post_1.end());

    std::vector<float> opp_post_2 = get_relative_observation(target, {4500, -500});
    observation.insert(observation.end(), opp_post_2.begin(), opp_post_2.end());

    std::vector<float> team_post_1 = get_relative_observation(target, {-4500, 500});
    observation.insert(observation.end(), team_post_1.begin(), team_post_1.end());

    std::vector<float> team_post_2 = get_relative_observation(target, {-4500, -500});
    observation.insert(observation.end(), team_post_2.begin(), team_post_2.end());

    std::vector<float> side_1 = get_relative_observation(target, {0, 3000});
    observation.insert(observation.end(), side_1.begin(), side_1.end());

    std::vector<float> side_2 = get_relative_observation(target, {0, -3000});
    observation.insert(observation.end(), side_2.begin(), side_2.end());

    // Add history
    int hsize = history[player_number].size();
    for (int i = hsize - history_length; i < hsize; i++) {
        observation.insert(observation.end(), history[player_number][i].begin(), history[player_number][i].end());
    }

     // step observation history
    step_history(agent, ball, player_number);

    return observation;



}

std::vector<float> Observation::get_off_ball_observation(std::vector<float> self, std::vector<float> ball, std::vector<std::vector<float>> teammates, std::vector<std::vector<float>> opponents, int robot_num, int on_ball_robot) {
    // Init history for robot if it hasn't been initialized
    if (history.find(robot_num) == history.end()) {
        history[robot_num] = std::vector<std::vector<float>>();
    }
    while ((int) history[robot_num].size() < history_length) {
        history[robot_num].push_back({0, 0, 0, 0});
    }

    std::vector<float> observation = std::vector<float>();

    // ball
    std::vector<float> ball_obs = get_relative_observation(self, ball);
    observation.insert(observation.end(), ball_obs.begin(), ball_obs.end());

    // get base position from strategy_positions
    // get number of teammates with lower number
    // int num_teammates_lower = 0;
    // for (int i = 0; i < (int)teammates.size(); i++) {
    //     if (teammates[i][3] < robot_num) {
    //         num_teammates_lower++;
    //     }
    // }

    // std::cout << "num_teammates_lower: " << num_teammates_lower << std::endl;

    std::vector<float> target_position = strategy_positions[robot_num];

    // maximum "drift" from base position due to ball. TODO: move into config
    float driftx = 1500.f;
    float drifty = 1500.f;

    // calculate drift based on ball position
    driftx = driftx * (ball[0])/4500;
    drifty = drifty * (ball[1])/3000;

    target_position = {target_position[0] + driftx, target_position[1] + drifty};

    // get relative position to target position
    std::vector<float> target_obs = get_relative_observation(self, target_position);
    observation.insert(observation.end(), target_obs.begin(), target_obs.end());

    // Add defenders
    for (int i = 0; i < 5; i++){
        // if not greater than the length of opponents
        if (i < (int)opponents.size()) {
            std::vector<float> opponent_obs = get_relative_observation(self, opponents[i]);
            observation.insert(observation.end(), opponent_obs.begin(), opponent_obs.end());
        } else {
            std::vector<float> opponent_obs = get_relative_observation(self, {4500, 0});
            observation.insert(observation.end(), opponent_obs.begin(), opponent_obs.end());
        }
    }

    // get relative position to 4500, 1100
    std::vector<float> goalPost1 = get_relative_observation(self, {4500, 500});
    observation.insert(observation.end(), goalPost1.begin(), goalPost1.end());

    // get relative position to 4500, -1100
    std::vector<float> goalPost2 = get_relative_observation(self, {4500, -500});
    observation.insert(observation.end(), goalPost2.begin(), goalPost2.end());

    // get relative position to -4500, 1100
    std::vector<float> goalPost3 = get_relative_observation(self, {-4500, 500});
    observation.insert(observation.end(), goalPost3.begin(), goalPost3.end());

    // get relative position to -4500, -1100
    std::vector<float> goalPost4 = get_relative_observation(self, {-4500, -500});
    observation.insert(observation.end(), goalPost4.begin(), goalPost4.end());

    std::vector<float> side1 = get_relative_observation(self, {0, 3000});
    observation.insert(observation.end(), side1.begin(), side1.end());

    std::vector<float> side2 = get_relative_observation(self, {0, -3000});
    observation.insert(observation.end(), side2.begin(), side2.end());

    // Add history
    int hsize = history[robot_num].size();
    for (int i = hsize - history_length; i < hsize; i++) {
        observation.insert(observation.end(), history[robot_num][i].begin(), history[robot_num][i].end());
    }

    // step observation history
    step_history(self, ball, robot_num);

    return observation;
}

std::vector<float> Observation::get_goalie_observation(std::vector<float> self, std::vector<float> ball, int robot_num) {
    // Init history for robot if it hasn't been initialized
    if (history.find(robot_num) == history.end()) {
        history[robot_num] = std::vector<std::vector<float>>();
        for (int i = 0; i < history_length; i++) {
            history[robot_num].push_back({0, 0, 0, 0});
        }
    }

    std::vector<float> observation;

    // Get relative position of ball to agent
    std::vector<float> relative_observation = get_relative_observation(self, ball);
    observation.insert(observation.end(), relative_observation.begin(), relative_observation.end());

    // Get relative position to [-4800, 0]
    std::vector<float> relative_observation_2 = get_relative_observation(self, {-4800, 0});
    observation.insert(observation.end(), relative_observation_2.begin(), relative_observation_2.end());

    // Get relative position of ball to [-4800, 0]
    std::vector<float> relative_observation_3 = get_relative_observation(ball, {-4800, 0});
    observation.insert(observation.end(), relative_observation_3.begin(), relative_observation_3.end());

    // Add history
    for (int i = 0; i < history_length; i++) {
        observation.insert(observation.end(), history[robot_num][i].begin(), history[robot_num][i].end());
    }

    return observation;
}

std::vector<float> Observation::get_near_opp_goal_observation(std::vector<float> self, std::vector<float> ball, std::vector<std::vector<float>> opponents, int robot_num) {
    // Init history for robot if it hasn't been initialized
    if (history.find(robot_num) == history.end()) {
        history[robot_num] = std::vector<std::vector<float>>();
    }
    while ((int) history[robot_num].size() < history_length) {
        history[robot_num].push_back({0, 0, 0, 0});
    }

    std::vector<float> observation;

    // Get relative position of ball to agent
    std::vector<float> relative_observation = get_relative_observation(self, ball);
    observation.insert(observation.end(), relative_observation.begin(), relative_observation.end());

    // Get relative position to right goal post
    std::vector<float> relative_observation_2 = get_relative_observation(self, {4500, -800});
    observation.insert(observation.end(), relative_observation_2.begin(), relative_observation_2.end());

    // Get relative position to left goal post
    std::vector<float> relative_observation_3 = get_relative_observation(self, {4500, 800});
    observation.insert(observation.end(), relative_observation_3.begin(), relative_observation_3.end());

    // Get relative position to opponent goalie
    // NOTE: Some NearOppGoal policies use this, some done. Don't delete it yet even if it's
    // commented out!
    // std::vector<float> opp_goalie_loc = get_opp_closest_to_opp_goal(opponents);
    // std::vector<float> relative_observation_4 = get_relative_observation(self, opp_goalie_loc);
    // observation.insert(observation.end(), relative_observation_4.begin(), relative_observation_4.end());

    // Add history
    for (int i = 0; i < history_length; i++) {
        observation.insert(observation.end(), history[robot_num][i].begin(), history[robot_num][i].end());
    }

    // step observation history
    step_history(self, ball, robot_num);

    return observation;
}

std::vector<float> Observation::get_on_ball_dribble_observation(std::vector<float> agent_loc, std::vector<float> ball_loc, std::vector<std::vector<float>> teammate_loc, std::vector<std::vector<float>> opponent_loc, int robotNum) {
    // Init history for robot if it hasn't been initialized
    if (history.find(robotNum) == history.end()) {
        history[robotNum] = std::vector<std::vector<float>>();
    }
    while ((int) history[robotNum].size() < history_length) {
        history[robotNum].push_back({0, 0, 0, 0});
    }

    std::vector<float> observation;

    // Get relative position of ball to agent
    std::vector<float> relative_observation = get_relative_observation(agent_loc, ball_loc);
    observation.insert(observation.end(), relative_observation.begin(), relative_observation.end());

    //    1 hot can kick (distance to ball < 300)
    if (can_kick(agent_loc, ball_loc)) {
        observation.push_back(1);
    } else {
        observation.push_back(0);
    }

    // Get teammate closest to goal
    std::vector<float> closest_teammate = {0, 0, 0};
    float min_distance = 1000000;
    for (int i = 0; i < (int)teammate_loc.size(); i++) {
        float distance = std::sqrt(std::pow(teammate_loc[i][0] - 4800, 2) + std::pow(teammate_loc[i][1], 2));
        if (distance < min_distance) {
            min_distance = distance;
            closest_teammate = teammate_loc[i];
        }
    }

    // Get relative position of closest teammate to agent
    std::vector<float> relative_teammate = get_relative_observation(agent_loc, closest_teammate);
    observation.insert(observation.end(), relative_teammate.begin(), relative_teammate.end());

    // get relative position to 4500, 1100
    std::vector<float> goalPost1 = get_relative_observation(agent_loc, {4500, 500});
    observation.insert(observation.end(), goalPost1.begin(), goalPost1.end());

    // get relative position to 4500, -1100
    std::vector<float> goalPost2 = get_relative_observation(agent_loc, {4500, -500});
    observation.insert(observation.end(), goalPost2.begin(), goalPost2.end());

    // get relative position to -4500, 1100
    std::vector<float> goalPost3 = get_relative_observation(agent_loc, {-4500, 500});
    observation.insert(observation.end(), goalPost3.begin(), goalPost3.end());

    // get relative position to -4500, -1100
    std::vector<float> goalPost4 = get_relative_observation(agent_loc, {-4500, -500});
    observation.insert(observation.end(), goalPost4.begin(), goalPost4.end());

    std::vector<float> side1 = get_relative_observation(agent_loc, {0, 3000});
    observation.insert(observation.end(), side1.begin(), side1.end());

    std::vector<float> side2 = get_relative_observation(agent_loc, {0, -3000});
    observation.insert(observation.end(), side2.begin(), side2.end());

    // Add history
    int hsize = history[robotNum].size();
    for (int i = hsize - history_length; i < hsize; i++) {
        observation.insert(observation.end(), history[robotNum][i].begin(), history[robotNum][i].end());
    }

     // step observation history
    step_history(agent_loc, ball_loc, robotNum);

    return observation;
}

void Observation::step_history(std::vector<float> self, std::vector<float> ball, int robot_num) {
    // Convert to relative observation
    std::vector<float> relative_observation = get_relative_observation(self, ball);
    history[robot_num].push_back(relative_observation);

    // using 3 here because that's the max history length of our policies right now
    while ((int) history[robot_num].size() > 3) {
        history[robot_num].erase(history[robot_num].begin());
    }
}

bool check_facing_ball(std::vector<float> self, std::vector<float> ball) {
    // Define the required angle
    float req_angle = 18;  // Adjust this value as needed

    // Convert from radians to degrees
    float robot_angle = std::fmod(self[2] * 180 / M_PI, 360);

    // Find the angle between the robot and the ball
    float angle_to_ball = std::atan2(ball[1] - self[1], ball[0] - self[0]) * 180 / M_PI;

    // Check if the robot is facing the ball
    float angle = std::fmod(robot_angle - angle_to_ball + 360, 360);

    if (angle < req_angle || angle > 360 - req_angle) {
        return true;
    } else {
        return false;
    }
}

bool Observation::can_kick(std::vector<float> self, std::vector<float> ball) {
    // Get distance
    float distance = std::sqrt(std::pow(self[0] - ball[0], 2) + std::pow(self[1] - ball[1], 2));

    // Get angle (in degrees)
    float angle = std::atan2(ball[1] - self[1], ball[0] - self[0]) * 180 / M_PI;

    // Check if distance is less than 300 and angle (either way) is less than 30
    return distance < 500 && check_facing_ball(self, ball);
}

std::vector<float> Observation::get_training_observation(
    std::vector<float> agent_loc,
    std::vector<float> ball_loc
) {
    std::vector<float> observation;

    // Get relative position of ball to agent
    std::vector<float> relative_observation = get_relative_observation(agent_loc, ball_loc);
    observation.insert(observation.end(), relative_observation.begin(), relative_observation.end());

    // Get relative position to right goal post
    std::vector<float> relative_observation_2 = get_relative_observation(agent_loc, {4500, -800});
    observation.insert(observation.end(), relative_observation_2.begin(), relative_observation_2.end());

    // Get relative position to left goal post
    std::vector<float> relative_observation_3 = get_relative_observation(agent_loc, {4500, 800});
    observation.insert(observation.end(), relative_observation_3.begin(), relative_observation_3.end());

    return observation;
}
