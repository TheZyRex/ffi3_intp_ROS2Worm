#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ros2_worm_multiplayer/msg/board.hpp"
#include "worm_constants.hpp"
#include <curses.h>


#define WORM_ID_PARAMETER wormid;

using std::placeholders::_1;
class WormDisplayNode : public rclcpp::Node
{
  public:

    WormDisplayNode() : Node("display_node")
    {
      declare_parameter("wormID", WormConstants::INVALID_WORM_ID); // defaults to INVALID_WORM_ID
      subscription_ = this->create_subscription<ros2_worm_multiplayer::msg::Board>(WormTopics::BoardInfo, 10, std::bind(&WormDisplayNode::refreshD, this, _1));


      get_parameter("wormID", observedWormID);

      startup();
    }

    ~ WormDisplayNode()
    {
      shutdown();
    }

  private:

    rclcpp::Subscription<ros2_worm_multiplayer::msg::Board>::SharedPtr subscription_;

    int width, height;
    bool cursesInitialized = false;
    int observedWormID = WormConstants::INVALID_WORM_ID;

    void startup();
    void initializeCursesApplication();
    void initializeColors();
    void place(int y, int x, chtype symbol, int color_pair);
    void cleanupCursesApp(void);
    void shutdown();
    void refreshD(const ros2_worm_multiplayer::msg::Board& msg);

};



void WormDisplayNode::startup()
{
  initializeCursesApplication();
  initializeColors();
}

void WormDisplayNode::initializeColors() {
  // Define colors of the game
  start_color();
  //Loop through all 7 Colors and generate a Color Pair by their own ID with background Black
  for(int i = 0; i<=COLOR_WHITE; i++)
  {
    init_pair(i, i, COLOR_BLACK);
  }
}

void WormDisplayNode::initializeCursesApplication() {

  initscr(); // Initialize the curses screen

  // Note:
  // The call to initscr() defines various global variables of the curses framework.
  // stdscr, LINES, COLS, TRUE, FALSE

  noecho();  // Characters typed ar not echoed
  cbreak();  // No buffering of stdin
  nonl();    // Do not translate 'return key' on keyboard to newline character
  keypad(stdscr, FALSE); // Disable the keypad
  curs_set(0);          // Make cursor invisible
  // Begin in non-single-step mode (getch will not block)
  nodelay(stdscr, TRUE);  // make getch to be a non-blocking call
}

void WormDisplayNode::place(int y, int x, chtype symbol, int color_pair) {
  //  Store symbol on the display (symbol code)
  move(y, x);                         // Move cursor to (y,x)
  attron(COLOR_PAIR(color_pair));     // Start writing in selected color
  addch(symbol);                      // Store symbol on the virtual display
  attroff(COLOR_PAIR(color_pair));    // Stop writing in selected color
}

void WormDisplayNode::shutdown()
{
  cleanupCursesApp();
}

void WormDisplayNode::cleanupCursesApp(void)
{
  standend();   // Turn off all attributes
  refresh();    // Write changes to terminal
  curs_set(1);  // Set cursor state to normal visibility
  endwin();     // Terminate curses application
  
}

void WormDisplayNode::refreshD(const ros2_worm_multiplayer::msg::Board& msg)
{
  clear();
  
  if ( LINES < height || COLS < width )
  {
  
    mvprintw(0, 0,"Fenster zu klein. Benötigte Displaygrösse: %d/%d ", width, height);
  
  } 
  else
  {
    int y = 0;
    int yCentralizer = (LINES-height)/2;
  
    for(auto &row : msg.board)
    {
      
      int x = 0;
      int xCentralizer = (COLS-width)/2;
      for(auto &element : row.row)
      {
        int32_t color = element.color;

        if(element.worm_id != WormConstants::INVALID_WORM_ID && element.worm_id == observedWormID)
        {
          color = COLOR_MAGENTA; //drawing selected worm in Magenta
        }

        place(y+yCentralizer, x+xCentralizer, element.zeichen, color);
        x++;
      }
      width = x;
      y++;
    }
    height = y;
  }
  refresh();
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WormDisplayNode>());
  rclcpp::shutdown();
  return 0;
}