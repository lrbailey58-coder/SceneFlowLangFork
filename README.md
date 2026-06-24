This repository was forked off of a repository that was used for a paper at the University of Virginia ("Scene Flow Specifications: Encoding and Monitoring Rich Temporal Safety Properties of Autonomous Systems"). There may be some libraries you need to download that are not listed in these instructions. If that is the case, just use pip install to install those libraries (you might also need to install pip).

# Setup Steps for Husarion ROSbot XL using ROS2 snap/jazzy

These instructions are primarily based on the [ROSbot XL quick start guide](https://husarion.com/tutorials/howtostart/rosbotxl-quick-start/). These instructions are more specific, as they are only for the **Husarion ROSbot XL using ROS2 snap/jazzy**. If you ever prompt AI to ask questions aboutnthis setup, it is highly reccommended that, in every prompt, you remind the AI model that you are using a ROSbot XL running ROS2 snap/jazzy.  

1. Open the case and unwrap the robot

2. Flip the robot over and unscrew the battery cover (you'll need a size T15 screwdriver). Plug the battery into the port in its compartment. Replace the battery cover. Flip back over.

3. Plug in the robot using the included charging cable

4. Attach the two antennae to the ports on the back of the robot

5. Access the robot's terminal. Do one of the following. The password is always "husarion":
   * Connect a keyboard, mouse, and monitor to the robot. Log into the Husarion account using the default password, "husarion". Open the terminal using Ctrl + Alt + T.
   * Connect a laptop (or other computer) directly to the robot using the included ethernet cable. On the laptop, open the terminal and input "ssh husarion@192.168.77.2". Enter the password (when you type, the letters won't show up, but they are still being tracked).

6. If you are looking directly at the robot's Ubuntu system (not ssh'ed in), go to the settings (top right, gear) and change the date and time to match the current date/time (system --> date & time)

7. Scan for wifi using the following two commands in order:
   * sudo nmcli dev wifi rescan
   * sudo nmcli dev wifi

8. Open the Netplan configuration file:
   * sudo nano /etc/netplan/01-network-manager-all.yaml

9. Put the network SSID and password in the corresponding locations near the bottom of the page. You should connect the device to the lab’s local network or WM_Welcome. If Dr. Woodlief has added the robot to WM_Welcome, use that network. If not, use the lab network (SSID and password are on the router). If there is no password delete the auth section and include “ {}” directly after the SSID’s closing quotation. **DO NOT DELETE ANYTHING FROM THE NETPLAN FILE EXCEPT THE PARTS YOU ARE REPLACING. DO NOT USE TABS TO INDENT, USE SPACES.** If you need to reset the netplan file, go to the quick start guide linked at the top of this page.

10. Press **Ctrl + O**, then **Enter**, then **Ctrl + X**. This is how you save and quit when editing with nano.

11. Test the connection. Use:
    * sudo netplan try
   
12. When prompted, press **Enter** in order to make the changes to the network settings. If it throws an error, fix it and try again. If you need to reset the netplan file, the default netplan file is copyable from the quick start guide. If you need to force commit the changes you made, use:
    * sudo netplan apply

13. Verify the conncection. Use the first command to check if the robot has recieved an IP address (write it down). Use the second command to ping a public DNS server (not nescessary if using the lab network):
    * ip a show wlan0
    * ping -c 3 8.8.8.8

14. Access the robot remotely. Use the following command on your laptop in command line:
    * ssh husarion@<ROSBOT_IPv4>

15. Run the following in the ROSbot's terminal in order. Make sure that each command completely finished before moving on. The final command will launch the teleop, which (if everything has gone well) should allow you to control the robot from your laptop
    * ~/flash_firmware.sh
    * sudo rosbot.start
    * ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true

### Property 5: Passing a bike too closely
While the three-foot distance cited in the driving code is observed here, it is only barely met.
These violations were identified by increasing the required safety buffer.

#### Bike 1:
![Vehicle comes too close to a bike while passing. Bike 1](./videos/435.gif)

#### Bike 2:
![Vehicle comes too close to a bike while passing. Bike 2](./videos/476.gif)

## Installation
This has been tested on a Ubuntu 20.04.

To install everything needed to run the code, execute the following command:
```bash
./unpack_data.sh
```
The installation script will do the following:
1) Unpack the included study data from `study_data.7z` and `study_timing_data.sh`
2) Create the conda environments as needed.
3) Install [mona](https://www.brics.dk/mona/) using the `install_mona.sh` script

## Replication
To reproduce the results of the paper, execute the following command:
```bash
source run.sh
```
This will run for ~6 hours. If you are running on a machine with at least 10 cores, you can substantially reduce this time by using the multithreaded version below.
```bash
source run_threaded.sh
```

This script will do the following:
1) Activate the conda environments as needed.
2) Unpack the scene graphs used in the experiment for RQ2 and RQ3.
3) Check the properties specified in the paper, located in the `symbolic_properties.py` file, using the scene graphs and the monitor instantiation. The violations will appear in `./results/`
4) Generate tables that show the property violations for each RQ.


### Replicating the timing figures (Fig. 7)
The times taken to evaluate each from of the SG as described in RQ4 are stored in `./study_timing_data/`. 
To reproduce Fig. 7, and the equivalent version including monitoring for all vehicles, run:
```bash
conda activate tcp_env
python3 time_parser.py
```

This will create:
* `frame_time_hist_ego_only.pdf` (Fig. 7)
* `frame_time_hist_all.pdf` (Equivalent to Fig. 7, but with comparing properties checking all vehicles and only ego)

Both of these files have been included in the repo.
