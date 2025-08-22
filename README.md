# LUCAS: Lightweight framework for UAV Control And Supervision

<div>
  <a href="https://github.com/catec/lucas/tree/main"><img src="https://img.shields.io/badge/ROS1-Noetic-blue" /></a>
  <a href="https://github.com/catec/lucas/tree/ros2"><img src="https://img.shields.io/badge/ROS2-Humble-blue" /></a>
</div>

Framework for UAV autonomous flight compatible with ArduPilot. The two modules that composed it are:

- **CATEC Control Manager:** Finite-state-machine for mission and behavior control.
- **Cascade PID Controller:** Module for trajectory control that interfaces with CATEC Control Manager and sent attitude set-points to autopilot via MAVROS.

Please check the README of each package for usage guidelines and more detailed information.

## Citation

If you use this data for any academic work, please cite our original paper:

```bibtex
@INPROCEEDINGS{11007803,
  author={Murillo, J.I. and Montes, M.A. and Zahinos, R. and Trujillo, M.A. and Viguria, A. and Heredia, G.},
  booktitle={2025 International Conference on Unmanned Aircraft Systems (ICUAS)}, 
  title={Simplifying Autonomous Aerial Operations: LUCAS, a Lightweight Framework for UAV Control and Supervision}, 
  year={2025},
  volume={},
  number={},
  pages={854-861},
  keywords={Buildings;Inspection;Autonomous aerial vehicles;Aircraft navigation;Libraries;Autopilot;Trajectory;Middleware;Monitoring;Robots},
  doi={10.1109/ICUAS65942.2025.11007803}
}
```
