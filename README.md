# mavros_pose_control
### Cascaded PID controller for setpoint position
  1. Install package <code> mavros_pose_control </code> from <code> westpoint robotics </code> on github
  2. Launch mocap_optitrack to receive data from mocap.
  2. Launch the <code> px4.launch </code> from mavros package.
  3. Launch <code> teleop_controller_test.launch </code> from mavros_pose_control package.
  4. publish the setpoint target(pose message) on <code>/mav_pose_control/pose setpoint </code> i.e.<br>
     <code> rostopic pub /mav_pose_control/pose_setpoint geometry_msgs/Pose "position: x:0.1 y:0.2 z:0.9 orientation: x:0.0 y:0.0 z:0.0 w:0.0" </code>


### Steps for demo
  1. Arm the quad
  2. Switch to offboard
  3. Control the quad using joystick 
  4. push and hold button 1 to go to the target setpoint and stay(setpoint position mode).
  5. releasing the button will give control back to the joystick(offboard joystick mode).
  6. Check the button map for several default setpoints.


### Additional commands
  1. See/change the gains for the controller using the following command
    <p><code> rosrun rqt_reconfigure rqt_reconfigure </code></p>

### Controller Logic
  * will be posted soon *


###Important Links 
 1. http://stackoverflow.com/questions/20615962/2d-world-velocity-to-2d-local-velocity

###Button map
  Button map              |       Button Number       |         Description
  ------------------------|:-------------------------:|-----------------------------------------------------------
  'teleop_toggle'         |       0                   |  Switches to position control mode
  'clear_integrals'       |        1                  |Clear the intergrals before switching to position control( Good practice)
  'set_takeoff_setpoint'  |       2                   |Take off to 1 meter from current location
  'set_land_setpoint'     |       3                   |Land setpoint(just above the ground) from current position
  'arm'                   |6                          |arming the quad
  'disarm'                |   7                       | disarming the quad
  's1'                    | 8                         |pre programmed setpoint( one of the corners of 1*1 square)
  's2'                    | 9                         |pre programmed setpoint( one of the corners of 1*1 square)
  's3'                    | 10                        |pre programmed setpoint( one of the corners of 1*1 square)
  's4'                    | 11                        |pre programmed setpoint( one of the corners of 1*1 square)

*Printed button numbers on the joystick starts with 1 and ends with 12 *  


### ( check back for more updates)
