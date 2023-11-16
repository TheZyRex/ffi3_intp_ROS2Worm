# ffi3_intp_ROS2Worm

## Navigation Node
publishes to the following topics:

- PlayerInput -- using Message Type: ros2_worm_multiplayer::msg::PlayerInput

subscribes to the following topics:

- GameStart -- using message type: std_msgs::msg:Int32

Will make a request to a server to join the game -- using Interface: ros2_worm_multiplayer::srv::JoinServer

## Display Node(s)

publishes to the following topics:

- None
  
subscribes to the following topics:

- BoardInfo -- using Message Type: ros2_worm_multiplayer::msg::Board

## Grid Node

publishes to the following topics:

- GameStart  -- using Message Type: std_msgs::msg::Int32  
  - used to transmit a gameId to identify a starting game
- BoardInfo  -- using Message Type: ros2_worm_multiplayer::msg::Board  
  - used to transmit all of the contents of the game board in the Board datastructure

subscribes to the following topics:

- PlayerInput  -- using Message Type: ros2_worm_multiplayer::msg::PlayerInput  

Will respond to a Client, giving them their wormid -- using Interface: ros2_worm_multiplayer::srv::JoinServer

# dockerized ROS2 Application
docker image available from: hub.docker.com

image name: thezyrex/ros2_worm_multiplayer:latest

docker compose file is included in this repository and can be found in ./docker-image
