# LinkerHand-Python-SDK

## Overview  
LinkerHand Python SDK  

## Caution  
- Ensure that the LinkerHand is not under other controls, such as `linker_hand_sdk_ros`, motion capture glove control, or other topics controlling the LinkerHand, to avoid conflicts.  
- Secure the LinkerHand to prevent it from falling during movement.  
- Ensure the LinkerHand power supply and USB-to-CAN connection are correct.  

## Installation  
You can run the examples after installing the requirements.txt. Only Python3 is supported.  
- Download  

  ```bash  
  git clone https://github.com/linkerbotai/linker_hand_python_sdk.git  
  ```  

- Install  

  ```bash  
  pip3 install -r requirements.txt  
  ```  

## CAN or RML485 Protocol Switching  
Note: Due to a bug in the Python 485 interface of the current RM65 from Reeman, this feature is temporarily unsupported.  
Edit the `config/setting.yaml` configuration file and modify the parameters according to the comments in the file. RML (Reeman API2) controls the LinkerHand via 485 protocol communication through the Reeman robotic arm.  
MODBUS: "None" or "RML"  

## Related Documentation  
[Linker Hand API for Python Document](doc/API-Reference.md)  

## Release Notes  
- > ### 2.1.4  
  - 1. Added support for L21  
  - 2. Added support for matrix pressure sensors  
  - 3. Added support for L10 Mujoco simulation  

- > ### 1.3.6  
  - Added support for LinkerHand L7/L20/L25 versions  

- > ### 1.1.2  
  - Added support for LinkerHand L10 version  
  - Added GUI control for L10 LinkerHand  
  - Added GUI display for L10 LinkerHand pressure sensor graphical data  
  - Added partial example source code  

- Position and Finger Joint Mapping Table  

  L7: ["Thumb flexion", "Thumb abduction", "Index flexion", "Middle flexion", "Ring flexion", "Little flexion", "Thumb rotation"]  

  L10: ["Thumb base", "Thumb abduction", "Index base", "Middle base", "Ring base", "Little base", "Index abduction", "Ring abduction", "Little abduction", "Thumb rotation"]  

  L20: ["Thumb base", "Index base", "Middle base", "Ring base", "Little base", "Thumb abduction", "Index abduction", "Middle abduction", "Ring abduction", "Little abduction", "Thumb roll", "Reserved", "Reserved", "Reserved", "Reserved", "Thumb tip", "Index tip", "Middle tip", "Ring tip", "Little tip"]  

  L21: ["Thumb base", "Index base", "Middle base", "Ring base", "Little base", "Thumb abduction", "Index abduction", "Middle abduction", "Ring abduction", "Little abduction", "Thumb roll", "Reserved", "Reserved", "Reserved", "Reserved", "Thumb middle", "Reserved", "Reserved", "Reserved", "Reserved", "Thumb tip", "Index tip", "Middle tip", "Ring tip", "Little tip"]  

  L25: ["Thumb base", "Index base", "Middle base", "Ring base", "Little base", "Thumb abduction", "Index abduction", "Middle abduction", "Ring abduction", "Little abduction", "Thumb roll", "Reserved", "Reserved", "Reserved", "Reserved", "Thumb middle", "Index middle", "Middle middle", "Ring middle", "Little middle", "Thumb tip", "Index tip", "Middle tip", "Ring tip", "Little tip"]  

## [L10_Example](example/L10)  

Before running, please modify the configuration information in [setting.yaml](LinkerHand/config/setting.yaml) to match the actual LinkerHand you are controlling.  

- #### [0000-gui_control](example/gui_control/gui_control.py) # python3 gui_control.py  
- #### [0001-linker_hand_fast](example/L10/gesture/linker_hand_fast.py)  
- #### [0002-linker_hand_finger_bend](example/L10/gesture/linker_hand_finger_bend.py)  
- #### [0003-linker_hand_fist](example/L10/gesture/linker_hand_fist.py)  
- #### [0004-linker_hand_open_palm](example/L10/gesture/linker_hand_open_palm.py)  
- #### [0005-linker_hand_opposition](example/L10/gesture/linker_hand_opposition.py)  
- #### [0006-linker_hand_sway](example/L10/gesture/linker_hand_sway.py)  

- #### [0007-linker_hand_get_force](example/L10/get_status/get_force.py) # python3 get_force.py --hand_joint L10 --hand_type right  
- #### [0008-linker_hand_get_speed](example/L10/get_status/get_set_speed.py) # python3 get_set_speed.py --hand_joint L10 --hand_type right --speed 100 123 211 121 222 (Note: For L7, the speed parameter requires 7 values; others require 5.)  
- #### [0009-linker_hand_get_state](example/L10/get_status/get_set_state.py) # python3 get_set_state.py --hand_joint L10 --hand_type right --position 100 123 211 121 222 255 255 255 255 255 (The number of position parameters should match the Position and Finger Joint Mapping Table.)  

- #### [0010-linker_hand_dynamic_grasping](example/L10/grab/dynamic_grasping.py)  

## API Documentation  
[Linker Hand API for Python Document](doc/API-Reference.md)