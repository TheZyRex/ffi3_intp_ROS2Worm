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
    
    def __init__(self, gui):
        super().__init__("worm_display_node")
        self.GUI = gui
        self.subscription = self.create_subscription(
            Board,
            'BoardInfo',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
    
    def listener_callback(self, msg):
        canvas = self.GUI.canvas
        #canvas.delete("all")  # Clear previous drawings
        # ->> very slow
        cell_size = 20  # Adjust the size based on your preference
        board_height = len(msg.board)
        board_width = len(msg.board[0].row)

        canvas_width = board_width * cell_size
        canvas_height = board_height * cell_size

        canvas.config(width=canvas_width, height=canvas_height)

        photo = tk.PhotoImage(width=canvas_width, height=canvas_height)
        canvas.create_image((canvas_width // 2, canvas_height // 2), image=photo, state="normal")

        for i, row in enumerate(msg.board):
            for j, element in enumerate(row.row):
                curses_color = element.color  # Assuming color is an attribute in the message
                x1, y1 = j * cell_size, i * cell_size
                x2, y2 = x1 + cell_size, y1 + cell_size
                tk_color = CURSES_TO_TK_COLOR.get(curses_color, "black")
                canvas.create_rectangle(x1, y1, x2, y2, fill=tk_color, outline="black")
                #self.get_logger().info('I heard: "%s"' % element.zeichen)
   
class MainFrame(tk.Frame):
    def __init__(self, root=None):
        tk.Frame.__init__(self, master=root)
        self.canvas = tk.Canvas(self)
        self.canvas.pack()



