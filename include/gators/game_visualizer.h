#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "point.h"
#include <random>

struct Visualizer_t {
  std::string id;
  visualization_msgs::msg::Marker marker;
  sensor_msgs::msg::PointCloud2 points;
  std::vector<int16_t> points_rgb{0, 0, 0};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
};

class GameVisualizer {
  public:
    /** Constructor for the game visualzer class
     * @brief TODO
     * @param node node pointer for ROS utilities
     * @return None
     */
    GameVisualizer(rclcpp::Node::SharedPtr node);

    /** Clears the game visualizer markers and points
     * @brief TODO
     * @param None
     * @return None
     */
    void clearVisualizer(void);

    /** Clears the map containers for environments and players
     * @brief TODO
     * @param None
     * @return None
     */
    void clearObjects(void);

    /** Creates and adds an visualizer object to the environments container
     * @brief TODO
     * @param env_id string identifier for the environment
     * @param points_rgb RGB color for visualizer point cloud
     * @return None
     */
    void addEnvironment(std::string env_id, std::vector<int16_t>points_rgb);

    /** Adds environment marker to a visualizer object.
     * @brief TODO
     * @param env_id string identifier for the environment
     * @param mesh filepath to object mesh
     * @param position vector of catesian coordinates <x, y, z>
     * @param alpha alpha color value describing transparency
     * @return None
     */
    void addEnvironmentMarker(std::string env_id, std::string mesh, std::vector<float_t> position, float alpha=1.0);

    /** Publish environment marker message.
     * @brief TODO
     * @param env_id string identifier for the environment
     * @return None
     */
    void publishEnvironmentMarker(std::string env_id);

    /** Append environment points to a visualizer object.
     * @brief TODO
     * @param env_id string identifier for the environment
     * @param pcd string containing point cloud data
     * @return None
     */
    void addEnvironmentPoints(std::string env_id, std::string pcd);

    /** Append environment points to a visualizer object.
     * @brief TODO
     * @param env_id string identifier for the environment
     * @param cloud point cloud message pointer
     * @return None
     */
    void addEnvironmentPoints(std::string env_id, sensor_msgs::msg::PointCloud2 &cloud);

    /** Append environment points to a visualizer object.
     * @brief TODO
     * @param env_id string identifier for the environment
     * @param pcl_points point cloud datatype pointer
     * @return None
     */
    void addEnvironmentPoints(std::string env_id, PointCloud::Ptr &pcl_points);

    /** Publish environment point cloud message
     * @brief TODO
     * @param env_id string identifier for the environment
     * @return None
     */
    void publishEnvironmentPoints(std::string env_id);

    /** Helper function to randomize RGB colors.
     * @brief TODO
     * @param None
     * @return Vector of RGB color
     */
    std::vector<int16_t> inputColors();

    /** Creates and adds a visualizer object to the players container
     * @brief TODO
     * @param player_id string identifier for the player
     * @param mesh filepath to object mesh
     * @param input_color boolean flag to assign or randomize point cloud color
     * @param use_color_for_player boolean flag to assign point cloud color
     * @return None
     */
    void addPlayer(std::string player_id, std::string mesh, bool input_color=false, bool use_color_for_player=true, int type=-1);

    /** Update the marker position of a player
     * @brief TODO
     * @param player_id string identifier for the player
     * @param position vector of cartesian coordinates <x, y, z>
     * @return None
     */
    void movePlayerMarker(std::string player_id, std::vector<float_t> position);

    /** Publish player marker message.
     * @brief TODO
     * @param env_id string identifier for the player
     * @return None
     */
    void publishPlayerMarker(std::string player_id);

    /** Append player points to a visualizer object.
     * @brief TODO
     * @param env_id string identifier for the player
     * @param points point cloud message
     * @return None
     */
    void addPlayerPoints(std::string player_id, sensor_msgs::msg::PointCloud2 points);

    /** Publish player point cloud message
     * @brief TODO
     * @param env_id string identifier for the player
     * @return None
     */
    void publishPlayerPoints(std::string player_id);

  private:
    rclcpp::Node::SharedPtr node_;

    std::string marker_topic_ = "/marker";
    std::string points_topic_ = "/points";
    std::string frame_id_ = "map";

    std::map<std::string, Visualizer_t> envs_;
    std::map<std::string, Visualizer_t> players_;
};
