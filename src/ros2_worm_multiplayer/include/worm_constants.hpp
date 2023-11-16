#pragma once

#ifndef _WORM_CONSTANTS_HPP
#define _WORM_CONSTANTS_HPP

#include <chrono>

extern "C" {
#include <curses.h>
}

/* Namespace for constants used in this project */
namespace WormConstants 
{
  constexpr std::chrono::milliseconds TICK_TIME{100};
  constexpr std::chrono::milliseconds RESPONSE_TIMEOUT{1000};
  constexpr const int GRID_MESSAGE_QUEUE_LENGTH = 10;
  constexpr const int FOOD_SPAWN_CHANCE_PERCENTAGE = 2;  // Chance a food item will spawn in a given tick

  /* Size of board */
  constexpr const int BOARD_HEIGHT = 40;
  constexpr const int BOARD_LENGTH = 80;

  /* Constants concerning limits of the game*/
  constexpr const int MAX_PLAYERS = 1;

  typedef enum ServiceRequests {
    SRV_JOIN,
    SRV_DISCONNECT,
  } ServiceRequests; 

  /* Invalid WormId to check for in join requests. */
  constexpr const int INVALID_WORM_ID = -1;
  constexpr const int INVALID_GAME_ID = -1;

  /* Enumeration for characters to be displayed on the board */
  typedef enum WormCharacters {
    WORM_HEAD = '0',
    WORM_BODY = 'o',
    BARRIER = '#',
    FOOD_1 = '1',
    FOOD_2 = '2',
    FOOD_3 = '3',

    EMPTY = ' '
  } WormCharacters;

  /* Color palette for characters to be displayed on the board */
  typedef enum WormColors {
    COLOR_HEAD = COLOR_GREEN,
    COLOR_BODY = COLOR_GREEN,
    COLOR_BARRIER = COLOR_RED,
    COLOR_FOOD_1 = COLOR_YELLOW,
    COLOR_FOOD_2 = COLOR_MAGENTA,
    COLOR_FOOD_3 = COLOR_CYAN,

    COLOR_EMPTY = COLOR_BLACK
  } WormColors;
}

/* Namespace for topics used in this project */
namespace WormTopics
{
  constexpr const char* PlayerInput = "PlayerInput";
  constexpr const char* GameStart = "GameStart";
  constexpr const char* BoardInfo = "BoardInfo";
}

/* Namespace for services used in this porject */
namespace WormServices
{
  constexpr const char* JoinService = "JoinService";
}

#endif