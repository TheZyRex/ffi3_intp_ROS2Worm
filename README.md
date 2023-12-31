# ffi3_intp_ROS2Worm

- [Navigation Node](#navigation-node)
- [Display Node(s)](#display-nodes)
- [Grid Node](#grid-node)
- [ARM64 & AMD64 Dockerimage](#arm64--amd64-dockerimage)

---

## Navigation Node

The Navigation Node handles the direction of the worm and joins a game.

### Publishes

- **Topic:** `PlayerInput`
  - **Message Type:** `ros2_worm_multiplayer::msg::PlayerInput`

### Subscribes

- **Topic:** `GameStart`
  - **Message Type:** `std_msgs::msg::Int32`

### Requests

The Navigation Node interacts with a server by making a request to join the game.

- **Service Interface:** `ros2_worm_multiplayer::srv::JoinServer`

This request enables seamless integration with the gaming environment, enhancing the overall functionality of the system.

### How to use

```bash
# first source the overlay
source install/setup.bash

# start the navigation node with parametersm
# true = use python display -- false = use ncurses display
ros2 run ros2_worm_multiplayer worm_navigation_node --ros-args -p pyDisplay:=true
```

---

## Display Node(s)

The Display Node(s) play a vital role in visualizing the game state.

### Commandline Arguments

- **wormID:**
  - Example: `ros2 run ros2_worm_multiplayer worm_display_node --ros-args -p wormID:=549528476`

### Publishes

The Display Node(s) do not publish to any topics.

### Subscribes

- **Topic:** `BoardInfo`
  - **Message Type:** `ros2_worm_multiplayer::msg::Board`

```bash
# first source the overlay
source install/setup.bash

# If you wish to use the python node, please use worm_display_node.py instead of worm_display_node

# start the display node with parameters
# wormID:=*YOUR_ID* -> dye specified worm with unique color, example:
ros2 run ros2_worm_multiplayer worm_display_node --ros-args -p wormID:=549528476

# or start it without parameters
ros2 run ros2_worm_multiplayer worm_display_node
```

---

## Grid Node

The Grid Node is responsible for managing game grid-related functionality.

### How to use

```bash
source install/setup.bash

# start the grid node with parameters
# numPlayers: number of players, must be greater than or equal to 1
ros2 run ros2_worm_multiplayer worm_grid_node --ros-args -p numPlayers:=2
```

### Command line Arguments
- numPlayers: int
-> e.g. ros2 run ros2_worm_multiplayer worm_grid_node --ros-args -p numPlayers:=1

### Publishes

- **Topic:** `GameStart`
  - **Message Type:** `std_msgs::msg::Int32`
    - Used to transmit a gameId to identify the start of a game.
- **Topic:** `BoardInfo`
  - **Message Type:** `ros2_worm_multiplayer::msg::Board`
    - Used to transmit all contents of the game board in the `Board` data structure.

### Subscribes

- **Topic:** `PlayerInput`
  - **Message Type:** `ros2_worm_multiplayer::msg::PlayerInput`

### Responds To

The Grid Node responds to a client, providing them with their wormID.

- **Service Interface:** `ros2_worm_multiplayer::srv::JoinServer`

---

## ARM64 & AMD64 Dockerimage

A Docker image for hosting the game server is accessible on hub.docker.com.

```bash
# clone this repository
git clone https://github.com/TheZyRex/ffi3_intp_ROS2Worm.git

# change into git repo
cd ffi3_intp_ROS2Worm/docker-images

# start the container using docker compose - depending on which platform you are using amd64/arm64
docker compose -f docker-compose-amd64.yml up
```
