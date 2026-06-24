1. [Wheels/Wifi/SSH](https://github.com/lrbailey58-coder/SceneFlowLangFork/blob/main/README.md#wheelswifissh)
2. [Camera/Web UI](https://github.com/lrbailey58-coder/SceneFlowLangFork/blob/main/README.md#cameraweb-ui)
3. [LiDAR]()

This repository was forked off of a repository that was used for a paper at the University of Virginia ("Scene Flow Specifications: Encoding and Monitoring Rich Temporal Safety Properties of Autonomous Systems"). There may be some libraries you need to download that are not listed in these instructions. If that is the case, just use pip install to install those libraries (you might also need to install pip).

# Setup Steps for Husarion ROSbot XL using ROS2 snap/jazzy

These instructions are primarily based on the [ROSbot XL quick start guide](https://husarion.com/tutorials/howtostart/rosbotxl-quick-start/). These instructions are more specific, as they are only for the **Husarion ROSbot XL using ROS2 snap/jazzy**. If you ever prompt AI to ask questions aboutnthis setup, it is highly reccommended that, in every prompt, you remind the AI model that you are using a ROSbot XL running ROS2 snap/jazzy.  

## Wheels/Wifi/SSH

1. Open the case and unwrap the robot

2. Flip the robot over and unscrew the battery cover (you'll need a size T15 screwdriver). Plug the battery into the port in its compartment. Replace the battery cover. Flip back over.

3. Plug in the robot using the included charging cable

4. Attach the two antennae to the ports on the back of the robot

5. Access the robot's terminal. Do one of the following. The password is always `husarion`:
   * Connect a keyboard, mouse, and monitor to the robot. Log into the Husarion account using the default password, `husarion`. Open the terminal using `Ctrl + Alt + T`
   * Connect a laptop (or other computer) directly to the robot using the included ethernet cable. On the laptop, open the terminal and input `ssh husarion@192.168.77.2`. Enter the password (when you type, the letters won't show up, but they are still being tracked).

6. If you are looking directly at the robot's Ubuntu system (not ssh'ed in), go to the settings (top right, gear) and change the date and time to match the current date/time (system --> date & time)

7. Scan for wifi using the following two commands in order:
```
sudo nmcli dev wifi rescan
sudo nmcli dev wifi
```

8. Open the Netplan configuration file:
```
sudo nano /etc/netplan/01-network-manager-all.yaml
```

9. Put the network SSID and password in the corresponding locations near the bottom of the page. You should connect the device to the lab’s local network or WM_Welcome. If Dr. Woodlief has added the robot to WM_Welcome, use that network. If not, use the lab network (SSID and password are on the router). If there is no password delete the auth section and include “ {}” directly after the SSID’s closing quotation. **DO NOT DELETE ANYTHING FROM THE NETPLAN FILE EXCEPT THE PARTS YOU ARE REPLACING. DO NOT USE TABS TO INDENT, USE SPACES.** If you need to reset the netplan file, go to the quick start guide linked at the top of this page.

10. Press `Ctrl + O`, then `Enter`, then `Ctrl + X`. This is how you save and quit when editing with nano.

11. Test the connection. Use:
```
sudo netplan try
```

12. When prompted, press `Enter` in order to make the changes to the network settings. If it throws an error, fix it and try again. If you need to reset the netplan file, the default netplan file is copyable from the quick start guide. If you need to force commit the changes you made, use:
```
sudo netplan apply
```

13. Verify the conncection. Use the first command to check if the robot has recieved an IP address (write it down). Use the second command to ping a public DNS server (not nescessary if using the lab network):
```
ip a show wlan0
ping -c 3 8.8.8.8
```

14. Access the robot remotely. Use the following command on your laptop in command line (replacing `<ROSBOT_IPv4>` with the ROSbot's IP):
```
ssh husarion@<ROSBOT_IPv4>
```

15. Run the following in the ROSbot's terminal in order. The first command will take minute to finish. The final command will launch the teleop, which (if everything has gone well) should allow you to control the robot from your laptop
```
~/flash_firmware.sh
sudo rosbot.start
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

## Camera/Web UI
These instructions are for the Oak D Pro, setup instructions may be different for different cameras

1. Mount camera (use included camera mount) and plug it into one of the internal usb ports

2. View the snap services on the ROSbot:
```
snap services
```

3. Make sure husarion-depthai and husarion-webui are active. If one of them is not acive, run the cooresponding command:
```
sudo husarion-depthai.start
sudo husarion-webui.start
```

4. Look for the topics associated with the camera:
```
ros2 topic list
```

5. Look through the list and find the topics relating to the camera (for Oak D pro, should be `/oak/rgb/image_raw`)

6. Check if the camera is sending data by running the folloing command (replace `/oak/rgb/image_raw` with the topic from step 5)
```
ros2 topic hz /oak/rgb/image_raw
```

7. View the camera in the web interface. On your laptop (which needs to be on the same network as the robot), go to `http://<ROSBOT_IP>:8080/ui`, where `<ROSBOT_IP>` is the IPv4 of the ROSbot

8. If the camera doesn't show up right away, click on the log panel's three dots. Go to change panel and choose 'Image'. You may need to go to the image panel's settings and change the topic to `/oak/rgb/image_raw/compressed`.

## LiDAR
These instructions are for the RPLIDAR S3. If you have a different LiDAR, the instuctions may be different

1. Put the robot in autonomy mode:
```
sudo snap set rosbot driver.configuration=autonomy
```

2. Mirror the ros2 settings over to the LiDAR
```
sudo snap set husarion-rplidar $(xargs -a /var/snap/rosbot/common/ros_snap_args)
```

3. Set the baud rate (specific to RPLIDAR S3, check the baud rate for your specific LiDAR)
```
sudo snap set husarion-rplidar driver.serial-baudrate=1000000
```

4. Turn on the LiDAR
```
sudo snap start husarion-rplidar
```

5. Make sure the LiDAR is enabled (look at the husarion-rplidar row)
```
snap services
```

6. Look through topics and make sure that `/scan` is there
```
ros2 topic list
```

7. Open the Foxglove Webui (see above instructions) and go to the settings of the 3D panel. In the "Panel" section, scroll down to /scan and make it visible). You should now see the LiDAR output in the 3D panel.

8. If the LiDAR is inactive (step 5) or otherwise not working, connect the robot to the internet and update the LiDAR software to the edge (beta) brach of the snap (to undo, replace `edge` with `stable`).
```
sudo snap refresh husarion-rplidar --channel=jazzy/edge
```


