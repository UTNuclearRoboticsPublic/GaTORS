#pragma once 

#include "gators/common.h"
#include "gators/point.h"
#include <algorithm>
#include <random>
#include <unordered_map>

namespace agents
{
/** @class Robot
 * @brief contains and tracks static and dynamic robot state information and attributes
 */
// . Base class
class Robot
{
public:
    Robot (std::string handle, int type_enum, bool reach_upper, int max_movement, int max_coverage, int battery_life, int recharge_duration);
    // . Info
    int get_max_turn_movement();
    bool get_can_reach_upper ();
    int get_max_battery();
    int get_recharge_time ();
    int get_max_turn_coverage ();
    std::string get_id ();
    int get_type ();
    int get_location ();
    int get_score ();
    // . Reset
    void reset_remaining_movement ();
    void reset_remaining_battery ();
    void reset_remaining_charge_time ();
    void reset_remaining_coverage ();
    void reset_score ();
    // . Update
    void update_location (int index);
    void update_score (int update);

    int remaining_movement;
    int remaining_coverage;
    int remaining_battery;
    int remaining_charge_time;

private:
    std::string id;
    int type;
    bool can_reach_upper;
    int max_turn_movement;
    int max_turn_coverage;
    int max_battery_life;
    int recharge_time;
    int location;
    int score;
};

// . Drone, Quadruped, Gantry classes
/** @class Drone
 * @brief inherits from Robot class and populates with set values for attributes
 */
class Drone : public Robot
{
public:
    Drone(std::string handle);
};

/** @class Quadruped
 * @brief inherits from Robot class and populates with set values for attributes
 */
class Quadruped : public Robot
{
public:
    Quadruped(std::string handle);
};

/** @class Gantry
 * @brief inherits from Robot class and populates with set values for attributes
 */
class Gantry : public Robot
{
public:
    Gantry(std::string handle);
};

// . Instantiation function
/** @struct Party
 * @brief contains a map of Robot players with keys that match the elements of a vector that tracks playing order 
 */
struct Party
{
    std::unordered_map<std::string, Robot> players;
    std::vector<std::string> playing_order;
};

/** Randmly shuffles a party
 * @brief randomly shuffles the playing order of a given party
 * @param party Party type object with a playing order to be shuffled 
 */
void randomShufflePlayingOrder (agents::Party &party);

/** Instantiates the party for the current game
 * @brief creates a party object with players and a playing order for a game
 * @param num_drones the number of Drone type players to instantiate
 * @param num_quadrupeds the number of Drone type players to instantiate
 * @param num_gantries the number of Drone type players to instantiate
 * @param random_shuffle tells whether the playing order should be randomly shuffled after instantiation or not
 * @return Party type, as map with players and an order in which they play
 */
Party instantiatePlayers (const unsigned int num_drones, const unsigned int num_quadrupeds, const unsigned int num_gantries, bool random_shuffle=true);
}

// #endif // GAME_PLAYERS_H
