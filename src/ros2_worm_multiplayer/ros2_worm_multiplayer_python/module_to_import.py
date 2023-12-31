import rclpy
from rclpy.node import Node
from ros2_worm_multiplayer.msg import Board
import tkinter as tk



# Mapping of curses color codes to Tkinter color names or RGB values
CURSES_TO_TK_COLOR = {
    0: "black",
    1: "red",
    2: "green",
    3: "yellow",
    4: "blue",
    5: "magenta",
    6: "cyan",
    7: "white"
}

class MyNode(Node):

    GUI = None
    displayCache = []
    displayCacheInit = False
    observedWorm = -1
    
    def __init__(self, gui):
        super().__init__("worm_display_node")
        self.GUI = gui
        self.subscription = self.create_subscription(
            Board,
            'BoardInfo',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.declare_parameter('wormID', -1)
        self.observedWorm = self.get_parameter('wormID').get_parameter_value().integer_value
    
    def listener_callback(self, msg):
        canvas = self.GUI.canvas
   
        cell_size = 20  # Adjusts the size of a pixel
        board_height = len(msg.board)
        board_width = len(msg.board[0].row)

        if(not self.displayCacheInit):
            self.displayCacheInit = True
            self.displayCache = [[0 for j in range(board_width)] for i in range(board_height)]
            canvas_width = board_width * cell_size
            canvas_height = board_height * cell_size

            canvas.config(width=canvas_width, height=canvas_height)
       


        for i, row in enumerate(msg.board):
            for j, element in enumerate(row.row):
                curses_color = element.color 
                if(isinstance(self.displayCache[i][j], tuple) and self.displayCache[i][j][1] == curses_color):
                    pass
                else:
                    
                    x1, y1 = j * cell_size, i * cell_size
                    x2, y2 = x1 + cell_size, y1 + cell_size
                    
                        
                    tk_color = CURSES_TO_TK_COLOR.get(curses_color, "black")
                    if(self.observedWorm!=-1 and self.observedWorm == element.worm_id):
                        tk_color = "coral"
                    
                    rect_id = canvas.create_rectangle(x1, y1, x2, y2, fill=tk_color, outline="black")
                    if(isinstance(self.displayCache[i][j], tuple)): canvas.delete(self.displayCache[i][j][0])
                    self.displayCache[i][j] = (rect_id, curses_color)
                
   
class MainFrame(tk.Frame):
    def __init__(self, root=None):
        tk.Frame.__init__(self, master=root)
        self.canvas = tk.Canvas(self)
        self.canvas.pack()



