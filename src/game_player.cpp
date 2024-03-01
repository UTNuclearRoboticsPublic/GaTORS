#include "gators/game_player.h"

// #define environment_mesh_ "package://gators/models/meshes/facility.stl"
// #define drone_mesh_ "package://gators/models/meshes/drone.dae"
// #define quadruped_mesh_ "package://gators/models/meshes/quadruped.dae"
// #define gantry_mesh_ "package://gators/models/meshes/gantry.dae"

// GameManager *manager_;
GameVisualizer *visualizer_;

GamePlayer::GamePlayer() : Node("play_game") {

  declareParams();

  // create publishers
  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();  
  map_visualizer_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map", qos_profile);
  marked_visualizer_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/marked", qos_profile);

  // Create services to play games
  play_random_game_service_ = this->create_service<std_srvs::srv::Trigger>("~/play_random_game", std::bind(&GamePlayer::playRandomGame, this, std::placeholders::_1, std::placeholders::_2));

  play_game_service_ = this->create_service<std_srvs::srv::Trigger>("~/play_game", std::bind(&GamePlayer::playGame, this, std::placeholders::_1, std::placeholders::_2));

  reset_game_service_ = this->create_service<std_srvs::srv::Trigger>("~/reset_game", std::bind(&GamePlayer::resetGame, this, std::placeholders::_1, std::placeholders::_2));

  // Create services for the game visualizer
  clear_game_visualizer_service_ = this->create_service<std_srvs::srv::Trigger>("~/clear_game_visualizer", std::bind(&GamePlayer::clearGameVisualizer, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "[GamePlayer] Up and ready.");
}

void GamePlayer::playRandomGame(std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/, std::shared_ptr<std_srvs::srv::Trigger::Response> response) 
{
  loadGame(); 
  manager_->playRandomGame();
  response->success = true;
  response->message = "Played a random game to completion!";
}

void GamePlayer::playGame(std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  loadGame();
  playMCTSGame();
  response->success = true;
  response->message = "Played a series of moves using MCTS!";
}

void GamePlayer::resetGame(std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/, std::shared_ptr<std_srvs::srv::Trigger::Response> response) 
{
  delete manager_;
  std::shared_ptr<std_srvs::srv::Trigger::Request> srv_request;
  std::shared_ptr<std_srvs::srv::Trigger::Response> srv_response;
  clearGameVisualizer(srv_request, srv_response);
  response->success = true;
  response->message = "Reset game!";
}

void GamePlayer::clearGameVisualizer(std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  // Clear visualizer
  visualizer_->clearVisualizer();

  rclcpp::sleep_for(std::chrono::milliseconds(200));

  visualizer_->clearObjects();

  response->success = true;
  response->message = "Game Visualizer cleared";
}


void GamePlayer::interpolatePath (const int &start_node, const int &end_node)
{
  // ros::Rate r(20);
  int num_steps {10};
  switch (manager_->party_.players.at(manager_->playingNow()).get_type()) {
    case 0:
    {
      num_steps = 10;
      break;
    }
    case 1:
    {
      num_steps = 15;
      break;
    }
    case 2:
    {
      num_steps = 20;
      break;
    }
  }

  std::vector<float_t> location {getLocationVector(start_node)};
  std::vector<float_t> goal {getLocationVector(end_node)};

  float_t x_step = (goal[0] - location[0]) / num_steps;
  float_t y_step = (goal[1] - location[1]) / num_steps;
  float_t z_step = (goal[2] - location[2]) / num_steps;

  for (int i = 0; i < num_steps; i++) {
    location[0] += x_step;
    location[1] += y_step;
    location[2] += z_step;
    visualizer_->movePlayerMarker(manager_->playingNow(), location);
    // r.sleep();
    rclcpp::sleep_for(std::chrono::milliseconds(50));
  }
}

void GamePlayer::visualizeTurn (TurnSequence sequence)
{
  // ros::Rate r(1);
  int turn_start_location {manager_->party_.players.at(manager_->playingNow()).get_location()};
  while (!sequence.empty()) {
    Action action {sequence.front()};
    if (action.move_id != -1) {
      // - handle movements, move appropriate player marker to target location
      interpolatePath(turn_start_location, action.move_id);
      turn_start_location = action.move_id;
    } else {
      // - handle repairs and add to score
      visualizer_->addPlayerPoints(manager_->playingNow(), manager_->board_.repair_spaces.at(action.repair_id).cloud);
      // r.sleep();
      rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
    sequence.pop();
  }
}

void GamePlayer::takeTurnMCTS(int search_duration_ms, int num_candidates, int search_depth, float uct_c, bool should_visualize)
{
  if (manager_->hasSufficientBattery()) {
    MCTS mcts {MCTS(manager_->board_, manager_->party_, manager_->player_turn_, search_duration_ms, num_candidates, search_depth, uct_c)};

    TurnSequence sequence {mcts.search()};
    // manager_.printSequence(sequence);
    if (should_visualize) {
      visualizeTurn(sequence);
    }
    manager_->playSequence(sequence);
    manager_->printScoreboard();
  }
  manager_->startNext();
}

void GamePlayer::playMCTSGame(bool should_visualize)
{
  int search_duration_ms {(int)(this->get_parameter("gators.mcts.search_duration_ms").as_int())};
  int num_candidates {(int)(this->get_parameter("gators.mcts.num_candidates").as_int())};
  int search_depth {(int)(this->get_parameter("gators.mcts.search_depth").as_int())};
  float uct_c {(float)(this->get_parameter("gators.mcts.uct_c").as_double())};

  while (!manager_->isOver()) {
    takeTurnMCTS(search_duration_ms, num_candidates, search_depth, uct_c, should_visualize);
  }
  std::vector<std::string> winners {manager_->determineWinners()};
  std::cout << "[Manager]\n------------------------------------------------------------\nGame has reached terminal state after " << manager_->total_turns_  << " turns!" << "\n------------------------------------------------------------" << std::endl;
  std::cout << "\tWinner(s):";
  for (std::string winner : winners) {
      std::cout << winner << " (" << manager_->party_.players.at(winner).get_score() << "),";
  }
  std::cout << std::endl;
}

void GamePlayer::loadGame()
{
  int n_drones {(int)(this->get_parameter("gators.party.n_drones").as_int())};
  int n_quadrupeds {(int)(this->get_parameter("gators.party.n_quadrupeds").as_int())};
  int n_gantries {(int)(this->get_parameter("gators.party.n_gantries").as_int())};
  loadGame(n_drones, n_quadrupeds, n_gantries);
}

void GamePlayer::loadGame(int n_drones, int n_quadrupeds, int n_gantries) 
{
  manager_ = new GameManager();

  std::string ws_dir {this->get_parameter("gators.paths.pkg_path").as_string()};
  float movement_discretization {(float)(this->get_parameter("gators.board.movement.discretization").as_double())};
  float repair_discretization {(float)(this->get_parameter("gators.board.repair.discretization").as_double())};
  int starting_location {(int)(this->get_parameter("gators.party.starting_location").as_int())};
  std::string rel_map_dir {this->get_parameter("gators.paths.clouds.raw.map").as_string()};
  std::string map_dir {ws_dir + rel_map_dir};
  std::string rel_marked_dir {this->get_parameter("gators.paths.clouds.raw.marked").as_string()};
  std::string marked_dir {ws_dir + rel_marked_dir};

  manager_->instantiateBoard(map_dir, movement_discretization, map_, marked_dir, repair_discretization, marked_);
  manager_->instantiatePlayers(n_drones, n_quadrupeds, n_gantries, starting_location);

  map_.header.frame_id = "map";
  marked_.header.frame_id = "map";
  instantiateVisualizer ();
}

void GamePlayer::instantiateVisualizer()
{
  bool user_input_color {this->get_parameter("gators.party.user_input_color").as_bool()};
  float environment_alpha {(float)(this->get_parameter("gators.board.mesh_alpha").as_double())};

  map_visualizer_->publish(map_);
  marked_visualizer_->publish(marked_);

  // - map environment
  visualizer_->addEnvironment("environment", std::vector<int16_t>{100, 100, 100});
  visualizer_->addEnvironmentMarker("environment", "package://gators/models/meshes/facility.stl", std::vector<float_t>{40.0, 0.0, -15.0}, environment_alpha);
  visualizer_->publishEnvironmentMarker("environment");
  visualizer_->addEnvironmentPoints("environment", map_);
  visualizer_->publishEnvironmentPoints("environment");

  // - repair environment
  visualizer_->addEnvironment("repair", std::vector<int16_t>{255, 255, 255});
  visualizer_->addEnvironmentPoints("repair", marked_);
  visualizer_->publishEnvironmentPoints("repair");

  // - create player visualizations
  for (std::size_t i = 0; i < manager_->party_.playing_order.size(); i++) {
    switch (manager_->party_.players.at(manager_->party_.playing_order[i]).get_type()) {
      case 0:
      {
        std::cout << "Trying to add marker for drone type" << std::endl;
        visualizer_->addPlayer(manager_->party_.playing_order[i], "package://gators/models/meshes/drone.dae", user_input_color);
        // visualizer_->addPlayer(manager_->party_.playing_order[i], "package://gators/models/meshes/drone.dae", user_input_color, true, 0);
        visualizer_->movePlayerMarker(manager_->party_.playing_order[i], getLocationVector(manager_->party_.playing_order[i]));
        break;
      }
      case 1:
      {
        std::cout << "Trying to add marker for quadruped type" << std::endl;
        visualizer_->addPlayer(manager_->party_.playing_order[i], "package://gators/models/meshes/quadruped.dae", user_input_color);
        // visualizer_->addPlayer(manager_->party_.playing_order[i], "package://gators/models/meshes/quadruped.dae", user_input_color, true, 1);
        visualizer_->movePlayerMarker(manager_->party_.playing_order[i], 
        getLocationVector(manager_->party_.playing_order[i]));
        break;
      }
      case 2:
      {
        std::cout << "Trying to add marker for gantry type" << std::endl;
        visualizer_->addPlayer(manager_->party_.playing_order[i], "package://gators/models/meshes/gantry.dae", user_input_color);
        // visualizer_->addPlayer(manager_->party_.playing_order[i], "package://gators/models/meshes/gantry.dae", user_input_color, true, 2);
        visualizer_->movePlayerMarker(manager_->party_.playing_order[i], getLocationVector(manager_->party_.playing_order[i]));
        break;
      }
    }
  }
}

std::vector<float_t> GamePlayer::getLocationVector (const int &id)
{
  return pointToVector(manager_->board_.movement_spaces.at(id).centroid);
}

std::vector<float_t> GamePlayer::getLocationVector (const std::string &id)
{
  int location {manager_->party_.players.at(id).get_location()};
  return pointToVector(manager_->board_.movement_spaces.at(location).centroid);
}

std::vector<float_t> GamePlayer::pointToVector (const PointT &point)
{
  std::vector<float_t> location {point.x, point.y, point.z + static_cast<float>(0.8)};
  return location;
}

void GamePlayer::declareParams()
{ 
  this->declare_parameter<std::string>("gators.paths.pkg_path");
  this->declare_parameter<float>("gators.board.movement.discretization");
  this->declare_parameter<float>("gators.board.repair.discretization");
  this->declare_parameter<int>("gators.party.starting_location");
  this->declare_parameter<std::string>("gators.paths.clouds.raw.map");
  this->declare_parameter<std::string>("gators.paths.clouds.raw.marked");

  this->declare_parameter<bool>("gators.party.user_input_color");
  this->declare_parameter<bool>("gators.party.use_color_for_player");
  this->declare_parameter<float>("gators.board.mesh_alpha");

  this->declare_parameter<int>("gators.party.n_drones");
  this->declare_parameter<int>("gators.party.n_quadrupeds");
  this->declare_parameter<int>("gators.party.n_gantries");
  
  this->declare_parameter<int>("gators.mcts.search_duration_ms");
  this->declare_parameter<int>("gators.mcts.num_candidates");
  this->declare_parameter<int>("gators.mcts.search_depth");
  this->declare_parameter<float>("gators.mcts.uct_c");
  
  this->declare_parameter<int>("gators.party.num_games");
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GamePlayer>();
  visualizer_ = new GameVisualizer(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
