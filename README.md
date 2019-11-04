# baxter_modified_controller
Modified Python controllers for Rethink Baxter allowing for position or torque-based control

- Controllers modified to accomodate the ARC Lab Baxter's different joint limits.
- Simultaneous limb control was also implemented as the originals could only control one limb at a time.
- Finally, torque-based control was implemented with the Baxter robot
  - This allowed for the sending of specific joint-torques which was crucial in most of the ARC Lab experiments 
  - Torque control allowed for less loss of fidelity compared to position based control
- Velocity based control was implemented but eventually scrapped due to the violent motions which would occur.
