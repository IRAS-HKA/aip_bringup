# AIP Bringup Tutorial

## Prequisites

For a detailed information to the Bring Up of the AIP application, please check out the AIP_Wiki, especially "[How to start AIP](https://github.com/IRAS-HKA/aip_wiki/blob/main/docs/how_to_start_aip.md)".

## Quick Start Instruction for AIP KUKA KR10 Robot

1. Establish communication 
   - KUKA HMI (SmartPad) must be set in AUT-Mode for operation 
   - EKI_HW_Interface file must be selected and running as active project 
   - Please pay attention to the specified velocities!
 
2. Start RVIZ
   - Pay attention to the hardware flag
    - **True** = Actions will only be performed in RViz simulation 
    - **False** = Actions will be performed on the physical KR10. 
        ``` bash
        # in aip_bringup docker 
        ros2 launch aip_cell_description aip.launch.py use_fake_hardware:=false robot_ip:=10.166.32.145
        ``` 

3. Start the Bosch Gripper node to enable gripper movements 
    ``` bash
    # in aip_bringup docker 
    ros2 run aip_bosch_gripper aip_bosch_gripper_node 
    ``` 

4. Execute the gripper via service calls from the command line
   ``` bash
   # Service call for ROS node OpenGripper cylinder 1 and 2 
   ros2 service call /open_gripper iras_interfaces/srv/MoveGripper '{cylinder_ids: [1,2]}'
   ```
    ``` bash
    # Service call for ROS node CloseGripper cylinder 1 and 2 
    ros2 service call /close_gripper iras_interfaces/srv/MoveGripper '{cylinder_ids: [1,2]}'
    ```

5. Start Behaviour Tree
    ``` bash
    # in aip_coordinator docker
    ros2 launch aip_coordinator aip.launch.py

    # the to be executed behavior tree can be adjusted in the params.yaml file located in src/aip_coordinator/config/params.yaml
    ```


