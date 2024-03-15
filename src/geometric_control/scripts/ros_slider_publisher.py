#!/bin/python3

import tkinter as tk
import time
import rospy
from std_msgs.msg import Float32




class ros_slider:
    def __init__(self):
        
        rospy.init_node('slider_pub', anonymous=True)
        self.velocity_pub = rospy.Publisher('velocity',Float32,queue_size=1)
        self.steering_pub = rospy.Publisher('steering_rad',Float32,queue_size=1)
        
        self.drive_speed = Float32()
        self.steering_angle = Float32()
        self.drive_speed.data = self.steering_angle.data = 50



    def update_drive_value(self,value):
        drive_label.config(text=f"Drive: {value}%")

        self.drive_speed.data = float(value)
        data_to_print = "d:%3.d" %   (int(self.drive_speed.data))
        data_to_print += " ,s:%3.d" % (int(self.steering_angle.data))
        data_to_print += "\n"
        print(data_to_print)

        self.velocity_pub.publish(self.drive_speed)
        self.steering_pub.publish(self.steering_angle)

    
    def update_steering_value(self,value):
        
        steering_label.config(text=f"Steering: {value}%")

        self.steering_angle.data = float(value)
        data_to_print = "d:%3.d" %   (int(self.drive_speed.data))
        data_to_print += " ,s:%3.d" % (int(self.steering_angle.data))
        data_to_print += "\n"
        print(data_to_print)

        self.velocity_pub.publish(self.drive_speed)
        self.steering_pub.publish(self.steering_angle)

    def close_window(self,event):
        print("Serial communication stopped.")
        # ser.close()
        if event.keysym in ["Escape", "q", "Q"]:
            window.destroy()

if __name__ == '__main__':
    
    slider = ros_slider()
    # Create a Tkinter window
    window = tk.Tk()
    window.title("Slider Demo")
    window.geometry("800x600")  # Width x Height

    # Create drive slider
    drive_label = tk.Label(window, text="Drive: 0%", font=("Arial", 16))
    drive_label.pack()
    drive_slider = tk.Scale(window, from_=100, to=0, orient="vertical", command=slider.update_drive_value,length=300)
    drive_slider.set(50)  # Set the default value to 50
    drive_slider.pack()

    # Create steering slider
    steering_label = tk.Label(window, text="Steering: 0%", font=("Arial", 16))

    steering_label.pack()
    steering_slider = tk.Scale(window, from_=0, to=100, orient="horizontal", command=slider.update_steering_value,length=300)
    steering_slider.set(50)  # Set the default value to 50

    steering_slider.pack()
    # Bind the keys to close the window
    window.bind("<Escape>", slider.close_window)
    window.bind("q", slider.close_window)
    window.bind("Q", slider.close_window)

    # Start the Tkinter main loop
    window.mainloop()



