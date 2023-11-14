#!/usr/bin/env python3

import threading
import tkinter
import rclpy
from ros2_worm_multiplayer_python.module_to_import import MyNode, MainFrame

def main(args=None):
    rclpy.init(args=args)
    root = tkinter.Tk()
    root.title("ROS2 Worm Display")
    GUI = MainFrame(root=root)
    node = MyNode(gui = GUI)
    GUI.pack()
    thread_spin = threading.Thread(target=rclpy.spin, args=(node, ))
    thread_spin.start()
    
    
    
   
    root.mainloop()


    #rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    thread_spin.join()

if __name__ == "__main__":
    main()