#pragma once

#include "gators/common.h"
#include "gators/point.h"
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/octree/octree_pointcloud_density.h>
#include <pcl/octree/octree_pointcloud_pointvector.h>
#include <random>
#include <math.h>
#include <unordered_map>
#include "agents.hpp"

// . Data Structures
/** @struct OctreeData
 * @brief used to store extracted octree data for graph construction
 */
struct OctreeData
{
    std::string name;
    float discretization;
    std::vector<sensor_msgs::msg::PointCloud2> clusters;
    std::vector<PointT, Eigen::aligned_allocator<PointT>> centroids;
};

// . Helpful spatial relations
/** @struct SpatialNode
 * @brief contains positive and negative nearest node ids in x, y, and z directions and their Euclidean distance
 */
struct SpatialNode
{
    int x_pos{-1};
    float x_pos_d{0};
    int x_neg{-1};
    float x_neg_d{0};
    int y_pos{-1};
    float y_pos_d{0};
    int y_neg{-1};
    float y_neg_d{0};
    int z_pos{-1};
    float z_pos_d{0};
    int z_neg{-1};
    float z_neg_d{0};
};
using SpatialGraph = std::unordered_map<int, SpatialNode>;

// . Movement game board
/** @struct Space
 * @brief contains relevant info related to a specific movement-discretized region in space
 * @var Space::id
 * 'id' matches the key used in a higher map to access the volume
 * @var Space::centroid
 * 'centroid' contains the geometric center of the repair volume
 * @var Space::cloud
 * 'cloud' contains the associated surface geometry to be repaired
 * @var Space::gantry_edges
 * 'gantry_edges' denotes the id's of edges that can be reached by a gantry-type system
 * @var Space::quadruped_edges
 * 'quadruped_edges' denotes the id's of edges that can be reached by a quadruped-type system
 * @var Space::drone_edges
 * 'drone_edges' denotes the id's of edges that can be reached by a drone-type system
 * @var Space::is_ground_level
 * 'is_ground_level' denotes whether the volume is at ground level
 * @var Space::repair_edges
 * 'repair_edges' contains the id's of repair volumes located within the Space
 */
struct Space
{
    int id;
    SpatialNode neighbors;
    std::vector<int> gantry_edges;
    std::vector<int> quadruped_edges;
    std::vector<int> drone_edges;
    PointT centroid;
    sensor_msgs::msg::PointCloud2 cloud;
    bool is_ground_level{false};
    std::vector<int> repair_edges;
};
using MoveBoard = std::unordered_map<int, Space>;

// . Repair action board
/** @struct RepairVolume
 * @brief contains relevant info related to a specific repair-discretized region in space
 * @var RepairVolume::id
 * 'id' matches the key used in a higher map to access the volume
 * @var RepairVolume::centroid
 * 'centroid' contains the geometric center of the repair volume
 * @var RepairVolume::cloud
 * 'cloud' contains the associated surface geometry to be repaired
 * @var RepairVolume::covered
 * 'covered' denotes wheteher a volume has already been repaired
 */
struct RepairVolume
{
    int id;
    PointT centroid;
    sensor_msgs::msg::PointCloud2 cloud;
    bool covered{false};
};
using RepairBoard = std::unordered_map<int, RepairVolume>;

// . Overall board struct
/** @struct Board
 * @brief contains a map of Robot players with keys that match the elements of a vector that tracks playing order
 * @var Board::movement_spaces
 * 'movement_spaces' contains the static movement graph of the current game
 * @var Board::repair_spaces
 * 'repair_spaces' contains the dynamic repair graph of the current game
 */
struct Board
{
    MoveBoard movement_spaces;
    RepairBoard repair_spaces;
};

// . Possible turns struct
/** @struct Action
 * @brief denotes a movement or repair id associated with an action to be taken
 * @var Action::move_id
 * 'move_id' denotes a node id from the movement graph to move to
 * @var Action::repair_id
 * 'repair_id' denotes a node id from the repair graph to repair
 */
struct Action
{
    int move_id{-1};
    int repair_id{-1};
};
using MoveOptions = std::vector<Action>;
using TurnSequence = std::queue<Action>;
using TurnOptions = std::vector<TurnSequence>;

namespace board_utils
{

    // . Function Prototypes
    /** Extracts octree data from a given point cloud
     * @brief extract octree data from a given point cloud
     * @param cloud point cloud of interest
     * @param octree_resolution octree lowest-level discretization
     * @param output octree data storage type to be filled
     */
    void extractOctreeData(const sensor_msgs::msg::PointCloud2 &cloud, const float &octree_resolution, OctreeData &output);

    /** Assigns spatial edges to a graph node
     * @brief assigns spatial edges to a node
     * @param spatial_graph graph of nodes for which to generate relations
     * @param node space for which to assign edges
     */
    void assignEdges(const SpatialGraph &spatial_graph, Space &node);

    /** Colors a centroid and cloud data the same random color
     * @brief randomly colors a centroid and cloud
     * @param cloud cloud to be colored
     * @param centroid centroid to be colored
     * @return colored centroid point
     */
    PointT colorPairRandomUniform(PointCloud::Ptr &cloud, const PointT &centroid);

    /** Colors a cloud a random color
     * @brief Colors a cloud a random color
     * @param cloud cloud to be colored
     */
    void colorCloudRandomUniform(PointCloud::Ptr &cloud);

    /** Creates randomly colored clouds to show the movment and repair spaces of a board
     * @brief Creates colorful movement and repair clouds for visualization
     * @param board board from which cloud information is extracted
     * @param map_cloud cloud msg for map modified in place
     * @param marked_cloud cloud msg for repair modified in place
     */
    void buildColoredClouds (Board &board, sensor_msgs::msg::PointCloud2 &map_cloud, sensor_msgs::msg::PointCloud2 &marked_cloud);

    /** Saves octree data in easily-visualizable format
     * @brief saves octree data into mulit-colored clouds
     * @param input octree data to be converted
     * @param dir write directory for output clouds
     */
    void saveOctreeDataClouds(const OctreeData &input, const std::string &dir);

    /** Loads PCD file as sensor_msgs::msg::PointCloud2
     * @brief loads pcl file as ros msg
     * @param dir read directory for input cloud
     * @param msg msg to be filled
     */
    void loadCloudasMsg(const std::string &dir, sensor_msgs::msg::PointCloud2 &msg);

    /** Assigns spatial edges to a graph based on distance
     * @brief assigns spatial edges to a graph
     * @param data octree data used to populate graph
     * @param index id of current node
     * @param tolerance azimuth of tolerance cone within which to search for neighbors in a particular direction
     */
    SpatialNode findNeighbors(const OctreeData &data, const int &index, const double &tolerance = 0.174533);

    /** Generates a spatial graph
     * @brief generates a spatial graph given input octree data
     * @param data octree data used to populate graph
     */
    SpatialGraph generateSpatialGraph(const OctreeData &data);

    /** Assigns robot-specific edges to a graph node
     * @brief generates a spatial graph given input octree data
     * @param spatial_graph spatial graph used to populate node
     * @param node movement space to be populated
     */
    void assignEdges(SpatialGraph &spatial_graph, Space &node);
    void assignGantryEdges(SpatialGraph &spatial_graph, Space &node);
    void assignQuadrupedEdges(SpatialGraph &spatial_graph, Space &node);
    void assignDroneEdges(SpatialGraph &spatial_graph, Space &node);

    /** Assigns repair edges to a graph node
     * @brief assigns repair edges to a graph node
     * @param centroid centroid of repair volume, used to find owner with shortest Euclidean distance
     * @param node movement space to be populated
     */
    int assignRepairEdge(const PointT &centroid, const std::vector<PointT, Eigen::aligned_allocator<PointT>> &centroids);

    /** Generates a movement board
     * @brief generates a movement board
     * @param data octree data used to construct movement graph
     * @param spatial_graph spatial data used to construct movement graph
     * @param cloud cloud from which to generate movement graph
     * @param octree_discretization discretization used to generate movement graph
     * @return MoveBoard type
     */
    MoveBoard generateMoveBoard(const OctreeData &data, SpatialGraph &spatial_graph);
    MoveBoard generateMoveBoard(const OctreeData &data);
    MoveBoard generateMoveBoard(const sensor_msgs::msg::PointCloud2 &cloud, const float &octree_resolution);

    /** Generates a game board
     * @brief generates a game board
     * @param move_data octree data used to construct movement graph
     * @param spatial_graph spatial data used to construct movement graph
     * @param repair_data octree data used to construct repair graph
     * @param move_cloud cloud from which to generate movement graph
     * @param move_octree_discretization discretization used to generate movement graph
     * @param repair_cloud cloud from which to generate repair graph
     * @param repair_octree_discretization discretization used to generate repair graph
     * @return MoveBoard type
     */
    Board generateBoard(const OctreeData &move_data, SpatialGraph &spatial_graph, const OctreeData &repair_data);
    Board generateBoard(const OctreeData &move_data, const OctreeData &repair_data);
    Board generateBoard(const sensor_msgs::msg::PointCloud2 &move_cloud, const float &move_octree_resolution, const sensor_msgs::msg::PointCloud2 &repair_cloud, const float &repair_octree_resolution);

}
