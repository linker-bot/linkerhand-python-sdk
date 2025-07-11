
# LinkerHand-Python-SDK

## Overview
LinkerHand Python SDK

## Caution
- Ensure no other control methods are active for the dexterous hand (e.g., `linker_hand_sdk_ros`, motion capture glove control, or other ROS topics). Conflicts may occur.。
- Securely mount the dexterous hand to prevent falls during operation.
- Verify proper power supply and USB-to-CAN connection for the dexterous hand.

## Installation
&ensp;&ensp;Run the examples after installing dependencies (Python 3 only)
- download

  ```bash
  git clone https://github.com/linkerbotai/linker_hand_python_sdk.git
  ```

- install

  ```bash
  pip3 install -r requirements.txt
  ```

## Documentation
[Linker Hand API for Python Document](doc/API-Reference.md)

## Version Information
- > ### release_2.1.8
 - 1. Fix occasional frame collision issues
 
- > ### 1.3.6

  - Compatible with L7/L20/L25 Dexterous Hand Models 
  
- > ### 1.1.2
  - Compatible with L10 Dexterous Hand Models 
  - Supports GUI-Based Control for L10 Dexterous Hands
  - Added GUI pressure display for L10 hand
  - Included partial example source code
  


## [L10_Example](example/L10)

&ensp;&ensp; __Before running, update the [setting.yaml](LinkerHand/config/setting.yaml) configuration to match your actual LinkerHand hardware setup.__

- #### [0000-gui_control](example/gui_control/gui_control.py)
- #### [0001-linker_hand_fast](example/L10/gesture/linker_hand_fast.py)
- #### [0002-linker_hand_finger_bend](example/L10/gesture/linker_hand_finger_bend.py)
- #### [0003-linker_hand_fist](example/L10/gesture/linker_hand_fist.py)
- #### [0004-linker_hand_open_palm](example/L10/gesture/linker_hand_open_palm.py)
- #### [0005-linker_hand_opposition](example/L10/gesture/linker_hand_opposition.py)
- #### [0006-linker_hand_sway](example/L10/gesture/linker_hand_sway.py)

- #### [0007-linker_hand_get_force](example/L10/get_status/get_force.py)
- #### [0008-linker_hand_get_speed](example/L10/get_status/get_speed.py)
- #### [0009-linker_hand_get_state](example/L10/get_status/get_state.py)

- #### [0010-linker_hand_dynamic_grasping](example/L10/grab/dynamic_grasping.py)




## API Document
[Linker Hand API for Python Document](doc/API-Reference.md)



