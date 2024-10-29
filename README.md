# ROS 2 Iron - Payload Dispenser Package

This package contains various ROS 2 Iron nodes for controlling and visualizing a payload dispenser system. Below are the available nodes and their functions.

## Available Nodes

- **keyboard_publisher**:  
  Node to publish the payload deployment signal.  
  Enter an integer in the terminal and press `ENTER` to send the signal.

- **mesh_visualizer**:  
  Node to visualize both the payload and the payload dispenser in RVIZ2.

- **tf_to_xyz**:  
  Node to publish the payload coordinates in XYZ format in RVIZ2 for position visualization.

- **static_camera_frame**:  
  Node to publish the static frame of the camera in RVIZ2.

## Usage

1. Launch each node as described, ensuring that RVIZ2 is open for visualization.
2. Use the `keyboard_publisher` node to send the deployment signal.

Make sure ROS 2 Iron is installed and properly configured before running the nodes.

---

Happy deploying!
