#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import threading

WAYPOINT_NAMES = ['station_a', 'station_b', 'station_c', 'station_d', 'docking_station', 'home']

class RosGuiNode(Node):
    def __init__(self):
        super().__init__('gui_node')
        self.seq_pub = self.create_publisher(String, 'waypoint_sequence', 10)
        self.cancel_pub = self.create_publisher(Bool, 'cancel_nav', 10)
        self.status_sub = self.create_subscription(String, 'waypoint_status', self.status_cb, 10)
        self.latest_status = ''

    def status_cb(self, msg):
        self.latest_status = msg.data

class WaypointGUI:
    def __init__(self, ros_node):
        self.ros_node = ros_node
        self.root = tk.Tk()
        self.root.title("Waypoint Navigator")
        self.selected = {name: tk.BooleanVar(value=False) for name in WAYPOINT_NAMES}

        frm = ttk.Frame(self.root, padding=12)
        frm.grid()

        # Buttons for waypoints (toggle)
        r = 0
        for name in WAYPOINT_NAMES:
            cb = ttk.Checkbutton(frm, text=name, variable=self.selected[name])
            cb.grid(column=0, row=r, sticky='w', pady=2)
            r += 1

        # Control buttons
        go_btn = ttk.Button(frm, text="Go", command=self.on_go)
        go_btn.grid(column=1, row=0, padx=8)
        stop_btn = ttk.Button(frm, text="Stop / Cancel", command=self.on_stop)
        stop_btn.grid(column=1, row=1, padx=8)

        self.status_label = ttk.Label(frm, text="Status: Idle", wraplength=300)
        self.status_label.grid(column=0, row=r, columnspan=2, pady=(10,0))
        r += 1

        # update status periodically from ROS
        self._update_status_loop()

    def on_go(self):
        chosen = [name for name, var in self.selected.items() if var.get()]
        if not chosen:
            # default: go home if nothing selected
            chosen = ['home']
        msg = String()
        msg.data = ','.join(chosen)
        self.ros_node.seq_pub.publish(msg)
        self.status_label.config(text=f"Sent: {msg.data}")

    def on_stop(self):
        msg = Bool()
        msg.data = True
        self.ros_node.cancel_pub.publish(msg)
        self.status_label.config(text="Cancel sent")

    def _update_status_loop(self):
        # called in Tkinter mainloop to refresh status from ROS node
        try:
            st = self.ros_node.latest_status
            if st:
                self.status_label.config(text=f"Status: {st}")
        except Exception:
            pass
        # call again after 300ms
        self.root.after(300, self._update_status_loop)

    def run(self):
        self.root.mainloop()

def ros_thread(node):
    rclpy.spin(node)

def main():
    rclpy.init()
    ros_node = RosGuiNode()

    # spin ROS in a separate thread so Tkinter mainloop runs on main thread
    t = threading.Thread(target=ros_thread, args=(ros_node,), daemon=True)
    t.start()

    gui = WaypointGUI(ros_node)
    gui.run()

    # on exit
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
