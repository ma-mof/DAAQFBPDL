# DAAQFBPDL
Master thesis: Development and analysis of autonomous quadcopter flight based on perception and deep learning

This thesis investigates the development and validation of an autonomous multirotor unmanned system capable of perception-driven motion control using embedded visual processing. The primary goal is to evaluate the feasibility of combining deep learning-based visual perception with a widely used open-source flight control system, while preserving operational safety, modularity, and real-time performance on embedded hardware with limited computational and power resources. The experimental system is built around a quadrotor platform controlled by an ArduPilot autopilot running on a Pixhawk control computer, complemented by an NVIDIA Jetson Nano co-computer responsible for perception and high-level decision-making. The software architecture uses the modular design of ArduPilot, with communication between the flight controller and the co-computer achieved via the MAVLink protocol. DroneKit was used as a high-level interface for issuing flight commands and processing telemetry data. Visual perception was implemented using a convolutional neural network based on the ResNet18-body architecture.

Keywords: Autonomous flight, visual processing, quadcopter, Nvidia Jetson Nano, ArduPilot, ResNet

