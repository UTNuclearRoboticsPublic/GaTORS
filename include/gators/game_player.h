#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/trigger.hpp>

#include "gators/common.h"
#include "gators/point.h"
#include "board.hpp"
#include "agents.hpp"
#include "game_manager.hpp"
#include "mcts.hpp"
#include "game_visualizer.h"
// #include <fstream>

class GamePlayer : public rclcpp::Node {
  public:
    GamePlayer();

  private:
    // Callback functions for ROS services
    void playRandomGame(std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void playGame(std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void playMCTSGame(std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void playNGames(std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void resetGame(std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void exhaustiveSearch(std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void customSearch(std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void clearGameVisualizer(std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void testGameVisualizer(std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,  std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // Additional functions
    void loadGame();

    void loadGame(int n_drones, int n_quadrupeds, int n_gantries);

    // Private variables for ROS services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr play_random_game_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr play_game_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_game_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_game_visualizer_service_;

    GameManager *manager_;
    sensor_msgs::msg::PointCloud2 map_;
    sensor_msgs::msg::PointCloud2 marked_;

    void declareParams();
    void instantiateVisualizer();

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_visualizer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr marked_visualizer_;

    std::vector<float_t> getLocationVector (const std::string &id);
    std::vector<float_t> getLocationVector (const int &id);
    std::vector<float_t> pointToVector (const PointT &point);

    void playMCTSGame(bool should_visualize=true);
    void takeTurnMCTS(int search_duration_ms, int num_candidates, int search_depth, float uct_c, bool should_visualize=true);
    void visualizeTurn (TurnSequence sequence);
    void interpolatePath (const int &start_node, const int &end_node);
};
