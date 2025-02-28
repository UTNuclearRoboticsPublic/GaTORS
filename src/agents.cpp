#include "gators/agents.hpp"

namespace agents
{

Robot::Robot(std::string handle, int type_enum, bool reach_upper, int max_movement, int max_coverage, int battery_life, int recharge_duration)
: id(handle), type(type_enum), can_reach_upper(reach_upper), max_turn_movement(max_movement), max_turn_coverage(max_coverage), max_battery_life(battery_life), recharge_time(recharge_duration), score{0}
{
    remaining_movement = max_movement;
    remaining_coverage = max_coverage;
    remaining_battery = battery_life;
    remaining_charge_time = recharge_time;
}

int Robot::get_max_turn_movement() {return max_turn_movement;}
bool Robot::get_can_reach_upper() {return can_reach_upper;}
int Robot::get_max_battery() {return max_battery_life;}
int Robot::get_recharge_time () {return recharge_time;}
int Robot::get_max_turn_coverage () {return max_turn_coverage;}
std::string Robot::get_id () {return id;}
int Robot::get_type () {return type;}
int Robot::get_location () {return location;}
int Robot::get_score () {return score;}
void Robot::reset_remaining_movement () {remaining_movement = max_turn_movement;}
void Robot::reset_remaining_battery () {remaining_battery = max_battery_life;}
void Robot::reset_remaining_charge_time () {remaining_charge_time = recharge_time;}
void Robot::reset_remaining_coverage () {remaining_coverage = max_turn_coverage;}
void Robot::reset_score () {score = 0;}
void Robot::update_location (int index) {location = index;}
void Robot::update_score (int update) {score += update;}

Drone::Drone(std::string handle) : Robot(handle, 0, true, 24, 2, 1, 18) {}
Quadruped::Quadruped(std::string handle) : Robot(handle, 1, false, 6, 3, 9, 12) {}
Gantry::Gantry(std::string handle) : Robot(handle, 2, true, 33, 50, 1, 12) {}

void randomShufflePlayingOrder (agents::Party &party)
{
    std::shuffle(std::begin(party.playing_order), std::end(party.playing_order), std::random_device());
    std::cout << "[Agents] Player turn order randomly shuffled as: \n";
    for (std::size_t i = 0; i < party.playing_order.size(); i++) {
        std::cout << "\t" << static_cast<int>(i) << ": " << party.playing_order[i] << std::endl;;
    }
}

Party instantiatePlayers (const unsigned int num_drones, const unsigned int num_quadrupeds, const unsigned int num_gantries, bool random_shuffle)
{
    Party party;
    for (std::size_t i = 0; i < num_drones; i++) {
        std::string agent_name {"drone_" + std::to_string(static_cast<int>(i))};
        party.players.insert({agent_name, Drone(agent_name)});
        party.playing_order.push_back(agent_name);
    }
    for (std::size_t i = 0; i < num_quadrupeds; i++) {
        std::string agent_name {"quadruped_" + std::to_string(static_cast<int>(i))};
        party.players.insert({agent_name, Quadruped(agent_name)});
        party.playing_order.push_back(agent_name);
    }
    for (std::size_t i = 0; i < num_gantries; i++) {
        std::string agent_name {"gantry_" + std::to_string(static_cast<int>(i))};
        party.players.insert({agent_name, Gantry(agent_name)});
        party.playing_order.push_back(agent_name);
    }
    if (random_shuffle) {
        randomShufflePlayingOrder(party);
    }
    return party;
}

} // namespace agents
