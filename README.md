# ffi3_intp_ROS2Worm

- [Navigation Node](#navigation-node)
- [Display Node(s)](#display-nodes)
- [Grid Node](#grid-node)
- [ARM64 & AMD64 Dockerimage](#arm64--and64-dockerimage)

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

---

## Display Node(s)

The Display Node(s) play a vital role in visualizing the game state.

### Commandline Arguments

- **wormID (cpp version only):**
  - Example: `ros2 run ros2_worm_multiplayer worm_display_node --ros-args -p wormID:=549528476`

### Publishes

The Display Node(s) do not publish to any topics.

### Subscribes

- **Topic:** `BoardInfo`
  - **Message Type:** `ros2_worm_multiplayer::msg::Board`

---

## Grid Node

The Grid Node is responsible for managing game grid-related functionality.

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

# depending on which platform u are using: amd64 or arm64
cd ffi3_intp_ROS2Worm/docker-images/amd64

# start the container using docker compose
docker compose -f docker-compose-amd64.yml up
```