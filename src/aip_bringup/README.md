# AIP Bringup

This includes only the launch files and configs necessary to launch the AIP Demo.

## How to start

```bash
# ~/workspace/roboception_driver$ 
./start_docker.sh
```

```bash
# ~/workspace/aip_bringup$
./start_docker.sh
ros2 launch aip_bringup aip.launch.py use_fake_hardware:=false robot_ip:=10.166.32.145

# for simulation
ros2 launch aip_bringup aip.launch.py
```

```bash
# ~/workspace/object_detector_tensorflow$ 
./start_docker.sh
ros2 launch object_detector_tensorflow detect_and_transform.launch.py
```

```bash
# ~/workspace/aip_coordinator$ 
./start_docker.sh
ros2 launch aip_coordinator demo.launch.py

# for simulation
ros2 launch aip_coordinator simulation.launch.py
```