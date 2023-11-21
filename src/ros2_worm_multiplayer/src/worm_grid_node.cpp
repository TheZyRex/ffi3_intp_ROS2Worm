// ############################################################################
// INCLUDES
// ############################################################################

#include <cstdio>
#include <ctime>
#include <chrono>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#include "worm_constants.hpp"
#include "ros2_worm_multiplayer/msg/board.hpp"
#include "ros2_worm_multiplayer/msg/row.hpp"
#include "ros2_worm_multiplayer/msg/element.hpp"
#include "ros2_worm_multiplayer/msg/player_input.hpp"
#include "ros2_worm_multiplayer/msg/direction.hpp"

#include "ros2_worm_multiplayer/srv/join_server.hpp"

extern "C" {
#include <curses.h>
}

// ############################################################################
// DEFINITIONS IN FILE SCOPE
// ############################################################################

typedef struct {
  int headIndex;
  std::vector<std::pair<int, int>> positions;
  std::pair<int, int> currMove;
} Worm;

static const std::pair<int, int> INVALID_POS = std::make_pair(std::numeric_limits<int>::max(), std::numeric_limits<int>::max());
static const char TEMP_WORM_CHARACTER = '@';

/**
 * @brief Return true with the given probability in percent.
*/
bool randomChance(int percentage) {
  if (rand() % 100 < percentage) {
    return true;
  } else {
    return false;
  }
}

// ############################################################################
// GAME EXIT EXCEPTION CLASS DECLARATION
// ############################################################################

class GameExitException : public std::exception {
  public:
    const char* what() {
      return "Exiting game.";
    }
};


// ############################################################################
// GRID NODE CLASS DECLARATION
// ############################################################################

class WormGridNode : public rclcpp::Node {
  public:
    WormGridNode();  // Constructor

    enum GameState {
      INIT,
      LOBBY,
      GAME,
      ENDED
    };

    GameState currentGameState = GameState::INIT;

    const int32_t gameId = std::rand();

    void handleJoin(
      const std::shared_ptr<ros2_worm_multiplayer::srv::JoinServer::Request> request,
      std::shared_ptr<ros2_worm_multiplayer::srv::JoinServer::Response> response
    );

    std::vector<int32_t> joinedPlayers;
    std::map<int32_t, Worm> worms;  // dictionary of the joined worms and associated data structures

  private:
    // callback groups
    rclcpp::CallbackGroup::SharedPtr main_cbg_;
    rclcpp::CallbackGroup::SharedPtr joinService_cbg_;

    // publishers
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr gameId_publisher_;
    rclcpp::Publisher<ros2_worm_multiplayer::msg::Board>::SharedPtr boardInfo_publisher_;

    // subscribers
    std::thread playerInput_thread;
    rclcpp::Subscription<ros2_worm_multiplayer::msg::PlayerInput>::SharedPtr playerInput_subscription_;

    // services
    
    rclcpp::Service<ros2_worm_multiplayer::srv::JoinServer>::SharedPtr joinService_;

    // timer for generating time ticks
    rclcpp::TimerBase::SharedPtr tick_timer_;

    // logical representation of the board
    ros2_worm_multiplayer::msg::Board Board;

    // methods to implement gameplay
    void runLobby();
    void runGame();
    void endGame();

    // methods to implement specific gameplay functions
    void generateLevel();
    void generateNewWorm(int32_t wormId);
    void generateFood();

    // callback methods for publishing
    void GameIdPublishCallback();
    void BoardInfoPublishCallback();

    // callback methods for subscribing
    void PlayerInputCallback(const ros2_worm_multiplayer::msg::PlayerInput input);

    // method combining all the routines to be run in 1 tick
    void RunTick();
};


// ############################################################################
// GRID NODE METHOD DEFINITIONS
// ############################################################################

/**
 * @brief Construct the Node and initialize instance members.
*/
WormGridNode::WormGridNode() : Node("worm_grid_node") {
  gameId_publisher_ = this->create_publisher<std_msgs::msg::Int32>(WormTopics::GameStart, WormConstants::GRID_MESSAGE_QUEUE_LENGTH);
  boardInfo_publisher_ = this->create_publisher<ros2_worm_multiplayer::msg::Board>(WormTopics::BoardInfo, WormConstants::GRID_MESSAGE_QUEUE_LENGTH);

  // initialize main callback group
  main_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // initialize tick timer
  tick_timer_ = this->create_wall_timer(
    WormConstants::TICK_TIME,
    std::bind(
      &WormGridNode::RunTick, 
      this
    ),
    main_cbg_
  );

  // initialize player input subscription 
  rclcpp::SubscriptionOptions playerInput_options;
  playerInput_options.callback_group = main_cbg_;
  playerInput_subscription_ = this->create_subscription<ros2_worm_multiplayer::msg::PlayerInput>(
    WormTopics::PlayerInput, 
    WormConstants::GRID_MESSAGE_QUEUE_LENGTH, 
    std::bind(
      &WormGridNode::PlayerInputCallback,
      this,
      std::placeholders::_1
    ),
    playerInput_options
  );

  // initialize join service server
  joinService_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  joinService_ = this->create_service<ros2_worm_multiplayer::srv::JoinServer>(
    WormServices::JoinService,
    std::bind(
      &WormGridNode::handleJoin,
      this,
      std::placeholders::_1,
      std::placeholders::_2
    ),
    rmw_qos_profile_services_default,
    joinService_cbg_
  );

  // initialize board
  Board = ros2_worm_multiplayer::msg::Board();
  
  auto boardVector = std::vector<ros2_worm_multiplayer::msg::Row>();
  auto currentRow = std::vector<ros2_worm_multiplayer::msg::Element>();
  auto currentElement = ros2_worm_multiplayer::msg::Element();
  
  for (int y = 0; y < WormConstants::BOARD_HEIGHT; y++) {

    for (int x = 0; x < WormConstants::BOARD_LENGTH; x++) {
      currentElement.set__color(COLOR_BLACK);
      currentElement.set__zeichen(WormConstants::WormCharacters::EMPTY);
      currentElement.set__worm_id(WormConstants::INVALID_WORM_ID);
      currentRow.push_back(currentElement);
    }
    boardVector.push_back(ros2_worm_multiplayer::msg::Row().set__row(currentRow));
    currentRow.clear();
  }
  Board.set__board(boardVector);

  // but gameplay elements onto the board
  generateLevel();

  // initialize player list
  joinedPlayers = std::vector<int32_t>();

  // send message of server starting to console
  RCLCPP_INFO(this->get_logger(), "Worm Grid Node started! GameId: %d", gameId);

  // start the lobby
  currentGameState = GameState::LOBBY;

}

/**
 * @brief Lobby for players to wait in.
*/
void WormGridNode::runLobby() {
  if (joinedPlayers.size() < WormConstants::MAX_PLAYERS) {
    GameIdPublishCallback();
  } else {
    RCLCPP_INFO(this->get_logger(), "Starting game.");
    
    for (auto id: joinedPlayers) {
      generateNewWorm(id);
    }

    currentGameState = GameState::GAME;
  }
}

/**
 * @brief Run the logic of the game.
*/
void WormGridNode::runGame() {


  // remove all worms
  for (int y = 0; y < WormConstants::BOARD_HEIGHT; y++) {
    for (int x = 0; x < WormConstants::BOARD_LENGTH; x++) {
      if (Board.board.at(y).row.at(x).worm_id != WormConstants::INVALID_WORM_ID) {
        Board.board.at(y).row.at(x).zeichen = TEMP_WORM_CHARACTER;
        Board.board.at(y).row.at(x).color = COLOR_BLACK;
        Board.board.at(y).row.at(x).worm_id = WormConstants::INVALID_WORM_ID;
      }
    }
  }

  // process worm movement
  /* fix: we need to have references of id and worm instead of a local copy */
  for (auto it = worms.begin(); it != worms.end();) {
    auto& [id, worm] = *it;
    std::pair<int, int> headPos = worm.positions.at(worm.headIndex);

    headPos.first += worm.currMove.first;
    headPos.second += worm.currMove.second;

    // determine if and what the worm hits
    switch (Board.board.at(headPos.second).row.at(headPos.first).zeichen)
    {
    case WormConstants::WormCharacters::EMPTY:
      worm.headIndex++;
      if (worm.headIndex >= worm.positions.size()) {
        worm.headIndex = 0;
      }
      worm.positions.at(worm.headIndex) = headPos;
      ++it;
      break;
    
    case WormConstants::WormCharacters::FOOD_1:
      worm.positions.resize(worm.positions.size() + 1, INVALID_POS);
      worm.headIndex++;
      if (worm.headIndex >= worm.positions.size()) {
        worm.headIndex = 0;
      }
      worm.positions.at(worm.headIndex) = headPos;
      ++it;
      break;

    case WormConstants::WormCharacters::FOOD_2:
      worm.positions.resize(worm.positions.size() + 2, INVALID_POS);
      worm.headIndex++;
      if (worm.headIndex >= worm.positions.size()) {
        worm.headIndex = 0;
      }
      worm.positions.at(worm.headIndex) = headPos;
      break;

    case WormConstants::WormCharacters::FOOD_3:
      worm.positions.resize(worm.positions.size() + 3, INVALID_POS);
            worm.headIndex++;
      if (worm.headIndex >= worm.positions.size()) {
        worm.headIndex = 0;
      }
      worm.positions.at(worm.headIndex) = headPos;
      ++it;
      break;

    case TEMP_WORM_CHARACTER:
      if (worm.currMove != std::make_pair(0, 0)) {
        joinedPlayers.erase(std::remove(joinedPlayers.begin(), joinedPlayers.end(), id));
        /* set the iterator to the next element after erasing */
        it = worms.erase(it);
      }
      else
      {
        /* Move to next element in map */
        ++it;
      }
      break;

    default:
      joinedPlayers.erase(std::remove(joinedPlayers.begin(), joinedPlayers.end(), id));
      /* set the iterator to the next element after erasing */
      it = worms.erase(it);
      break;
    }    
  }

  // redraw changed worms
  for (auto [id, worm]: worms) {
    for (int posIndex = 0; posIndex < worm.positions.size(); posIndex++) {
      if (worm.positions.at(posIndex) == INVALID_POS) {
        continue;
      }

      if (posIndex == worm.headIndex) {
        Board.board.at(worm.positions.at(posIndex).second).row.at(worm.positions.at(posIndex).first).zeichen = WormConstants::WormCharacters::WORM_HEAD;
        Board.board.at(worm.positions.at(posIndex).second).row.at(worm.positions.at(posIndex).first).color = WormConstants::WormColors::COLOR_HEAD;
      } else {
        Board.board.at(worm.positions.at(posIndex).second).row.at(worm.positions.at(posIndex).first).zeichen = WormConstants::WormCharacters::WORM_BODY;
        Board.board.at(worm.positions.at(posIndex).second).row.at(worm.positions.at(posIndex).first).color = WormConstants::WormColors::COLOR_BODY;
      }
      Board.board.at(worm.positions.at(posIndex).second).row.at(worm.positions.at(posIndex).first).worm_id = id;
    }
  }

  // remove temp elements on the board
  for (int y = 0; y < WormConstants::BOARD_HEIGHT; y++) {
    for (int x = 0; x < WormConstants::BOARD_LENGTH; x++) {
      if (
        Board.board.at(y).row.at(x).zeichen == TEMP_WORM_CHARACTER
      ) {
        Board.board.at(y).row.at(x).zeichen = WormConstants::WormCharacters::EMPTY;
      }
    }
  }

  if (randomChance(WormConstants::FOOD_SPAWN_CHANCE_PERCENTAGE)) {
    generateFood();
  }

  if (joinedPlayers.size() == 1) {
    if (worms[joinedPlayers.at(0)].positions.size() >= WormConstants::MIN_LENGTH_WIN_CONDITION) {
      currentGameState = GameState::ENDED;
    }
  } else if (joinedPlayers.size() == 0) {
    currentGameState = GameState::ENDED;
  }
}

/**
 * @brief End the game and stop the grid node.
*/
void WormGridNode::endGame() {
  // let players exit
  if (joinedPlayers.size() > 0) {
    return;
  }

  throw GameExitException();
}

/**
 * @brief Put barriers and other elements on the board.
*/
void WormGridNode::generateLevel() {
  // put barriers all around the board
  for (int x = 0; x < WormConstants::BOARD_LENGTH; x++) {
    // top of board
    Board.board.at(0).row.at(x).color = WormConstants::WormColors::COLOR_BARRIER;
    Board.board.at(0).row.at(x).zeichen = WormConstants::WormCharacters::BARRIER;
    Board.board.at(0).row.at(x).worm_id = WormConstants::INVALID_WORM_ID;

    // bottom of board
    Board.board.at(WormConstants::BOARD_HEIGHT - 1).row.at(x).color = WormConstants::WormColors::COLOR_BARRIER;
    Board.board.at(WormConstants::BOARD_HEIGHT - 1).row.at(x).zeichen = WormConstants::WormCharacters::BARRIER;
    Board.board.at(WormConstants::BOARD_HEIGHT - 1).row.at(x).worm_id = WormConstants::INVALID_WORM_ID;
  }

  for (int y = 1; y < WormConstants::BOARD_HEIGHT - 1; y++) {
    // left side of board
    Board.board.at(y).row.at(0).color = WormConstants::WormColors::COLOR_BARRIER;
    Board.board.at(y).row.at(0).zeichen = WormConstants::WormCharacters::BARRIER;
    Board.board.at(y).row.at(0).worm_id = WormConstants::INVALID_WORM_ID;
    
    // right side of board
    Board.board.at(y).row.at(WormConstants::BOARD_LENGTH - 1).color = WormConstants::WormColors::COLOR_BARRIER;
    Board.board.at(y).row.at(WormConstants::BOARD_LENGTH - 1).zeichen = WormConstants::WormCharacters::BARRIER;
    Board.board.at(y).row.at(WormConstants::BOARD_LENGTH - 1).worm_id = WormConstants::INVALID_WORM_ID;
  }
}

/**
 * @brief Put a new player on the board.
*/
void WormGridNode::generateNewWorm(int32_t wormId) {
  int currX = rand() % WormConstants::BOARD_LENGTH;
  int currY = rand() % WormConstants::BOARD_HEIGHT;

  while (Board.board.at(currY).row.at(currX).zeichen != WormConstants::EMPTY) {
    currX = rand() % WormConstants::BOARD_LENGTH;
    currY = rand() % WormConstants::BOARD_HEIGHT;
  }
  Worm currWorm;
  currWorm.headIndex = 0;

  std::vector<std::pair<int, int>> positions;
  positions.push_back(std::make_pair(currX, currY));

  currWorm.currMove = std::make_pair(0, 0);

  currWorm.positions = positions;
  worms.insert(std::make_pair(wormId, currWorm));
}

/**
 * @brief Put food at an unpopulated spot on the board.
*/
void WormGridNode::generateFood() {
  int x = rand() % WormConstants::BOARD_LENGTH;
  int y = rand() % WormConstants::BOARD_HEIGHT;

  while (Board.board.at(y).row.at(x).zeichen != WormConstants::EMPTY) {
    x = rand() % WormConstants::BOARD_LENGTH;
    y = rand() % WormConstants::BOARD_HEIGHT;
  }

  int foodType = rand() % 3;
  switch (foodType) {
  case 0:
    Board.board.at(y).row.at(x).zeichen = WormConstants::WormCharacters::FOOD_1;
    Board.board.at(y).row.at(x).color = WormConstants::WormColors::COLOR_FOOD_1;
    break;
  case 1:
    Board.board.at(y).row.at(x).zeichen = WormConstants::WormCharacters::FOOD_2;
    Board.board.at(y).row.at(x).color = WormConstants::WormColors::COLOR_FOOD_2;
    break;
  case 2:
    Board.board.at(y).row.at(x).zeichen = WormConstants::WormCharacters::FOOD_3;
    Board.board.at(y).row.at(x).color = WormConstants::WormColors::COLOR_FOOD_3;
    break;
  }
  Board.board.at(y).row.at(x).worm_id = WormConstants::INVALID_WORM_ID;
}

/**
 * @brief Callback method to send GameId when waiting for players.
*/
void WormGridNode::GameIdPublishCallback() {
  static std_msgs::msg::Int32 message = std_msgs::msg::Int32();
  message.data = gameId;

  gameId_publisher_->publish(message);
  RCLCPP_INFO(this->get_logger(), "Publishing GameId %d!", gameId);
}

/**
 * @brief Callback method to send board info each tick.
*/
void WormGridNode::BoardInfoPublishCallback() {
  boardInfo_publisher_->publish(Board);
}

/**
 * @brief Callback method to process player inputs.
*/
void WormGridNode::PlayerInputCallback(const ros2_worm_multiplayer::msg::PlayerInput input) {
  RCLCPP_INFO(this->get_logger(), "Player %d: Received input (%d/%d).", input.wormid, input.dir.dx, input.dir.dy);
  // check if player is actuall in game
  if (worms.find(input.wormid) == worms.end()) {
    return;
  }

  worms[input.wormid].currMove.first = input.dir.dx;
  worms[input.wormid].currMove.second = input.dir.dy;
}

/**
 * @brief Run all methods that need to be run in a tick.
*/
void WormGridNode::RunTick() {  
  // run the tick according to GameState
  switch (currentGameState) {
  case GameState::LOBBY:
    runLobby();
    break;

  case GameState::GAME:
    runGame();
    break;
  
  case GameState::ENDED:
    endGame();
    break;
  
  default:
    break;
  }

  // publish the game board after all changes have had an effect
  BoardInfoPublishCallback();
}

/**
 * @brief Handle joins and disconnects through the JoinServer service.
*/
void WormGridNode::handleJoin(
  const std::shared_ptr<ros2_worm_multiplayer::srv::JoinServer::Request> request,
  std::shared_ptr<ros2_worm_multiplayer::srv::JoinServer::Response> response
) {
  // Handle joining
  if (request->srv_request == WormConstants::ServiceRequests::SRV_JOIN) {
    // Only allow joining when GameState is LOBBY
    if (currentGameState != GameState::LOBBY) {
      response->set__wormid(WormConstants::INVALID_WORM_ID);
      RCLCPP_INFO(this->get_logger(), "Player was not allowed to join. Lobby full.");
      return;
    }

    int32_t newWormId = std::rand();
    while (std::find(joinedPlayers.begin(), joinedPlayers.end(), newWormId) != joinedPlayers.end()) {
      // Increment newWormId until it is unique
      newWormId++;
    }
    RCLCPP_INFO(this->get_logger(), "Player joined. Given ID: %d", newWormId);
    response->set__wormid(newWormId);
    joinedPlayers.push_back(newWormId);
    return;

  // Handle disconnecting
  } else if (request->srv_request == WormConstants::ServiceRequests::SRV_DISCONNECT) {
    RCLCPP_INFO(this->get_logger(), "Player %d left.", request->wormid);
    
    while (std::find(joinedPlayers.begin(), joinedPlayers.end(), request->wormid) != joinedPlayers.end()) {
      joinedPlayers.erase(
        std::remove(joinedPlayers.begin(), joinedPlayers.end(), request->wormid),
        joinedPlayers.end()
      );
    }

    while (worms.find(request->wormid) != worms.end()) {
      worms.erase(worms.find(request->wormid));
    }
    
    response->set__wormid(WormConstants::INVALID_WORM_ID);
  }
}


// ############################################################################
// MAIN
// ############################################################################

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  
  getchar();
  std::cout << "test" << std::endl;

  rclcpp::init(argc, argv);

  std::srand(std::time(nullptr));

  rclcpp::executors::MultiThreadedExecutor executor;
  auto worm_grid_node = std::make_shared<WormGridNode>();
  executor.add_node(worm_grid_node);
  try {
    executor.spin();
  }
  catch(const GameExitException& e)
  {
    RCLCPP_INFO(rclcpp::Node("EXIT").get_logger(), "Exiting game.");
  }
  
  rclcpp::shutdown();
  
  return 0;
}