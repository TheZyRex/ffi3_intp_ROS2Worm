/**
 * Author: jop6462
 * Last Change: 20.11.2023
*/


#include <cstdio>
#include <curses.h>
#include <cinttypes>
#include <string>
#include <iostream>
#include <vector>
#include <csignal>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "ros2_worm_multiplayer/msg/player_input.hpp"
#include "ros2_worm_multiplayer/srv/join_server.hpp"
#include "worm_constants.hpp"


extern "C" {
#include "prep.h"
}

/**
 * @brief Custom Node Class for the 
*/
class NavigationNode : public rclcpp::Node
{
	public:
		NavigationNode();

	private:

		/* Gamestate */
		enum gamestates 
		{
			PRELOBBY,
			LOBBY,
			INGAME,
			POSTGAME,
			QUIT
		};

		/* */
		void gameserver_selection();

		/* */
		void join_gameserver();

		/* */
		void user_input_loop();

		/* */
		void quit_gameserver();

		/* */
		void shutdown_node();

		/* */
		void gamestart_callback(const std_msgs::msg::Int32& msg);

		/* */
		void timer_callback();
		
		/* helper methods */
		void add_gameid_to_vector(const std_msgs::msg::Int32& msg);

		/* Game Server IDs */
		std::vector<int> game_ids_;
		int cur_game_id;

		/* */
		int cur_wormid;

		/* */
		bool use_python_display;

		/* */
		enum gamestates cur_gamestate;

		/* Msg type that includes direction that a specifc worm goes */
		ros2_worm_multiplayer::msg::PlayerInput pInput;

		/* Timer for keyboard read */
		rclcpp::TimerBase::SharedPtr timer_;

		/* Publisher for PlayerInputs */
		rclcpp::Publisher<ros2_worm_multiplayer::msg::PlayerInput>::SharedPtr pInput_pub_;

		/* Subscriber to GameStart */
		rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gamestart_sub_;

		/* Join Service Client */
		rclcpp::Client<ros2_worm_multiplayer::srv::JoinServer>::SharedPtr client_;  

		/* Callback Groups */
		rclcpp::CallbackGroup::SharedPtr client_cb_group_;
		rclcpp::CallbackGroup::SharedPtr sub_cb_group_;
};


/**
 * @brief Initialize navigation node 
*/
NavigationNode::NavigationNode()
: Node("navigation_node"), cur_game_id{-1}, cur_gamestate{PRELOBBY}
{
	/* NCurses Init */
	initializeCursesApplication();

	/* Set parameters */
	this->declare_parameter("pyDisplay", true);
	this->get_parameter("pyDisplay", this->use_python_display);

	/* Callback Groups init */
	this->client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	this->sub_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

	/* Create Subscriber for GameStart waiting for a gameid to join */
	rclcpp::SubscriptionOptions sub_options;
	sub_options.callback_group = this->sub_cb_group_;
	this->gamestart_sub_ = this->create_subscription<std_msgs::msg::Int32>(
		WormTopics::GameStart, 10, std::bind(&NavigationNode::gamestart_callback, this, std::placeholders::_1), sub_options);

	/* Create Client for join request */
	this->client_ = this->create_client<ros2_worm_multiplayer::srv::JoinServer>(WormServices::JoinService,
		rmw_qos_profile_services_default, this->client_cb_group_);

	/* rclcpP::QoS(10) provides default QoS profile with history depth of 10 */
	this->pInput_pub_ = this->create_publisher<ros2_worm_multiplayer::msg::PlayerInput>(WormTopics::PlayerInput, rclcpp::QoS(10));

	/* Create a timer to check for keyboard input every 100ms */
	this->timer_ = create_wall_timer(WormConstants::TICK_TIME, std::bind(&NavigationNode::timer_callback, this));
}


/**
 * @brief Requests a Server to join their game
*/
void NavigationNode::join_gameserver()
{
	if (this->cur_game_id > -1)
	{
		int retry_count {0};
		bool timedout {false};

		/* Create the actual join request */
		auto join_request = std::make_shared<ros2_worm_multiplayer::srv::JoinServer::Request>();
		join_request->gameid = this->cur_game_id;
		join_request->wormid = WormConstants::INVALID_WORM_ID;
		join_request->srv_request = WormConstants::ServiceRequests::SRV_JOIN;

		/* Waiting for service to wake up */
		while (!this->client_->wait_for_service(std::chrono::seconds(1)))
		{
			clear();
			attron(COLOR_PAIR(1));
			printw("Waiting for server...\n");
			attroff(COLOR_PAIR(1));
			refresh();
		}

		/* sending actual request to the service */
		auto future_response = this->client_->async_send_request(join_request);

		/* Wait for server reply */
		while (future_response.wait_for(WormConstants::RESPONSE_TIMEOUT) != std::future_status::ready)
		{
		  attron(COLOR_PAIR(1));
		  printw("Server not responding... \n");
		  attroff(COLOR_PAIR(1));
		  refresh();

			/* Check for recurring timeouts */
			if ((retry_count += 1) > WormConstants::RETRIES_UNTIL_TIMEOUT)
			{
				clear();
				printw("We were not able to join this server...\n");
				printw("Going back to game selection.");
				refresh();

				this->game_ids_.clear();
				this->cur_gamestate = PRELOBBY;
				timedout = true;

				/* Display Info for 1s */
				rclcpp::sleep_for(std::chrono::milliseconds(2000));

				break;
			}
		}

		/* Only proceed if no timeout occured */
		if (!timedout)
		{
		  auto response = future_response.get();

		  attron(COLOR_PAIR(1));
		  printw("Successfully joined server: %d\n playing as wormid: %d\n",this->cur_game_id, response->wormid);
		  attroff(COLOR_PAIR(1));
		  refresh();

		  /* Store wormid for this node until the end of the game */
		  this->cur_wormid = response->wormid;
		  this->pInput.wormid = this->cur_wormid;

			std::ostringstream start_command;

			/* depending on start parameters, start the python or ncurses display */
			if (this->use_python_display)
			{
			  start_command << "gnome-terminal --geometry=" << std::to_string(WormConstants::BOARD_LENGTH) << "x" << std::to_string(WormConstants::BOARD_HEIGHT) 
			  							<< " -- bash -i -c 'ros2 run ros2_worm_multiplayer worm_display_node.py --ros-args -p wormID:=" << std::to_string(this->cur_wormid) << "'";

			}
			else
			{

			  start_command << "gnome-terminal --geometry=" << std::to_string(WormConstants::BOARD_LENGTH) << "x" << std::to_string(WormConstants::BOARD_HEIGHT) 
			  							<< " -- bash -i -c 'ros2 run ros2_worm_multiplayer worm_display_node --ros-args -p wormID:=" << std::to_string(this->cur_wormid) << "'";

			}

			/* Start display node in separate terminal */
    	system(start_command.str().c_str());

		  /* Change the gamestate to INGAME */
		  this->cur_gamestate = INGAME;
		}
	}
}


/**
 * @brief Requests to quit the server and goes back to gameselection state
*/
void NavigationNode::quit_gameserver()
{
	if (this->cur_wormid != WormConstants::INVALID_WORM_ID)
	{
		int retry_count {0};
		bool timedout {false};

  	auto quit_request = std::make_shared<ros2_worm_multiplayer::srv::JoinServer::Request>();
  	quit_request->gameid = this->cur_game_id;
  	quit_request->wormid = this->pInput.wormid;
  	quit_request->srv_request = WormConstants::SRV_DISCONNECT;

  	auto future_response = this->client_->async_send_request(quit_request);

  	while (future_response.wait_for(WormConstants::RESPONSE_TIMEOUT) != std::future_status::ready)
  	{
			if ((retry_count += 1) > WormConstants::RETRIES_UNTIL_TIMEOUT) { timedout = true; break; }
  	}

		if (timedout)
		{
			this->cur_wormid = WormConstants::INVALID_WORM_ID;
			this->cur_game_id = WormConstants::INVALID_GAME_ID;
			this->game_ids_.clear();
		}
		else
		{
		  auto response = future_response.get();

		  this->cur_wormid = response->wormid;
		  this->cur_game_id = WormConstants::INVALID_GAME_ID;
		  this->game_ids_.clear();
		}

		this->cur_gamestate = PRELOBBY;

		clear();
		refresh();
	}
}


/**
 * @brief Shutdown and cleanup
*/
void NavigationNode::shutdown_node()
{
	cleanupCursesApp();
	rclcpp::shutdown();
}

/**
 * @brief Callback for the subscriber
*/
void NavigationNode::gamestart_callback(const std_msgs::msg::Int32& msg)
{
	this->add_gameid_to_vector(msg);
}


/**
 * @brief Displays a small lobby with available servers in the network
*/
void NavigationNode::gameserver_selection()
{
	char input;

	if (this->game_ids_.size() > 0)
	{
	  clear();
	  attron(COLOR_PAIR(1));

	  printw("Game Lobby: \n");
	  for (int i = 0; i < this->game_ids_.size(); i++)
	  {
	   	printw("%d - Game ID: %d\n", i, this->game_ids_.at(i));
	  }

		/* Move cursor to the bottom left */
	  move(getmaxy(stdscr) - 2, 0);
	  printw("Chose a game to join %d - %ld: ", 0, this->game_ids_.size() - 1);
	  move(getmaxy(stdscr) - 1, 0);
	  printw("Press 'q' to quit...");

		/* Get user inputs in a non-blocking manner */
		input = getch();
	  if (input != ERR && isdigit(input))
	  {
	  	input = std::atoi(&input);
	  	if (input < this->game_ids_.size())
	  	{
	  		this->cur_game_id = this->game_ids_.at(input);

				this->cur_gamestate = LOBBY;
	  	}
	  }
		else if (input == 'q') /* q to quit */
		{
			this->cur_gamestate = QUIT;
		}

		attroff(COLOR_PAIR(1));
		refresh();
	}
	else /* if no gameservers are available, display a waiting message */
	{
		int mid_y = getmaxy(stdscr) / 2;
		int mid_x = getmaxx(stdscr) / 2;
		clear();

	  move(mid_y, mid_x);

		attron(COLOR_PAIR(1));
		printw("Looking for Gameservers...\n");

		move(mid_y + 1, mid_x);
	  printw("Press 'q' to quit...");

		input = getch();
		if (input == 'q')
		{
			this->cur_gamestate = QUIT;
		}

		attroff(COLOR_PAIR(1));
		refresh();
	}

	return;
}


/**
 * @brief
*/
void NavigationNode::timer_callback()
{
	switch (this->cur_gamestate)
	{
		case PRELOBBY:
			this->gameserver_selection();
			break;

		case LOBBY: 
			this->join_gameserver();
			break;

		case INGAME:
			this->user_input_loop();
			break;

		case POSTGAME:
			this->quit_gameserver();
			break;

		case QUIT:
			this->shutdown_node();
			break;
	}
}


/**
 * @brief Adds a game id to the list of available game servers if it doesnt exist already
*/
void NavigationNode::add_gameid_to_vector(const std_msgs::msg::Int32& msg)
{
	if (std::find(this->game_ids_.begin(), this->game_ids_.end(), msg.data) == this->game_ids_.end())
	{
		this->game_ids_.push_back(msg.data);
	}
}


/**
 * @brief
*/
void NavigationNode::user_input_loop()
{
	int key;
	bool dirty = false;
	static ros2_worm_multiplayer::msg::PlayerInput last; 

	/* TODO Game Menu */
	clear();
	mvprintw(1, 1, "Worm Game Instructions");
	mvprintw(3, 1, "Use the Arrow Keys to Navigate:");
	mvprintw(5, 1, "UP    : Move Up");
	mvprintw(6, 1, "DOWN  : Move Down");
	mvprintw(7, 1, "LEFT  : Move Left");
	mvprintw(8, 1, "RIGHT : Move Right");
	mvprintw(10, 1, "Press Q to Quit the Game");
	refresh();

	if ((key = getch()) != ERR)
	{
	  switch (key)
	  {
	  case 'q': /* Quit the Game */
			this->cur_gamestate = POSTGAME;
	  	break;
	
	  case KEY_UP :
			this->pInput.dir.dx = 0;
			this->pInput.dir.dy = -1;
			dirty = true;
	  	break;

	  case KEY_DOWN :
			this->pInput.dir.dx = 0;
			this->pInput.dir.dy = 1;
			dirty = true;
	  	break;

	  case KEY_LEFT :
			this->pInput.dir.dx = -1;
			this->pInput.dir.dy = 0;
			dirty = true;
	  	break;

	  case KEY_RIGHT :
			this->pInput.dir.dx = 1;
			this->pInput.dir.dy = 0;
			dirty = true;
	  	break;
	  }

		/* if there is change in direction, publish user input */
		if (dirty == true && this->pInput != last)
		{
			this->pInput_pub_->publish(this->pInput);
			last = this->pInput;
			dirty = false;
		}
	}
	return;
}


/**
 * @brief Catch SIGINT event generated by CTRL+C to cleanup nCurses and shutdown
 * the node correctly
*/
void signalHandler(int signum)
{
	cleanupCursesApp();
	rclcpp::shutdown();
	exit(signum);
}

/* */
int main(int argc, char** argv)
{
	signal(SIGINT, signalHandler);
	rclcpp::init(argc, argv);

	auto navigation_node = std::make_shared<NavigationNode>();

	/* Use a multithreaded executor to run more threads at once */
	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(navigation_node);

	executor.spin();

	rclcpp::shutdown();

	return 0;
}