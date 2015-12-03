##** Read this document if you have used this package before 12/01/2015** **Details are changing continuously**
-----------------------------------------------------------------------------------------------------------
# mavros_pose_control
### Cascade PID controller for setpoint position
  1. Install package <code> mavros_pose_control </code> from <code> westpoint robotics </code> on github
  2. Launch mocap_optitrack to receive data from mocap.
  2. Launch the <code> px4.launch </code> from mavros package.
  3. Launch <code> teleop_controller_test.launch </code> from mavros_pose_control package.
  4. publish the setpoint target(pose message) on <code>/mav_pose_control/pose setpoint </code> i.e.<br>
     <code> rostopic pub /mav_pose_control/pose_setpoint geometry_msgs/Pose "position: x:0.1 y:0.2 z:0.9 orientation: x:0.0 y:0.0 z:0.0 w:0.0" </code>
    or
  4. Use one of the setpoints assigned to the joystick keys.(Refer table below)


### Steps for demo( pre flight checks mentioned below are highly recommended) 
  1. Arm the quad
  2. Check PID gain values in rqt perspective.(If it shows the default value then refer to the sample_config_values.cfg in this package to set up the gains manually. Custom values are the 3rd last parameter)
  3. Switch to offboard
  4. Control the quad using joystick 
  5. Either publish a setpoint using command given above or use any of the pre programmed setpoints on the joystick.
  6. Reset the integrals before switching to setpoint position mode
  7. push and hold button 1 to go to the target setpoint and stay(setpoint position mode).
  8. releasing the button will give control back to the joystick(offboard joystick mode).
  9. Check the button map for several default setpoints.
  10. <b>Absolutely </b> make sure that quad is <b> disarmed </b> before you close any of the nodes ( mavros or mavros_pose_control) as closing any of the nodes before you disarmed the vehicle will cause vehicle to take off without any control if quad is offboard mode.

### Pre flight Checks
  1. Check for Mocap data if it is streamed without freezing
   ( if the data freezes frequently then on the windows side change the setting of camera group and change the point cloud engine to version 4.0 and its better to uncheck 'broadcast frames' checkbox and check it again)
  2. Before switching to offboard make sure the joystick throttle is minimum position
  3. Check conditions of propellers on the quad and change if it looks damaged.
  4. <b>Currently when you run the mavros_pose_control for the first time the gains are set to default. Change the gains and You don't have to modify this again until you close the current roscore session.</b>
  5. Once you swtich to offboard joystick mode, slowly increase the throttle to check if quad is trying to point towards east direction. Once it does points towards east,it means quad is ready to take off. if it starts turning in opposite direction then there might be something wrong and do the pre flight checks.

### Additional commands
  1. See/change the gains for the controller using the following command
    <p><code> rosrun rqt_reconfigure rqt_reconfigure </code></p><b>or</b>
  2. Manually change the gains in the rqt perspective.
  

### Controller Logic
  * will be posted soon *
  * for PID tuning techniques follow link 2 and 3 for reference.


###Useful Links 
 1. http://stackoverflow.com/questions/20615962/2d-world-velocity-to-2d-local-velocity
 2. http://flitetest.com/articles/p-i-and-sometimes-d-gains-in-a-nutshell
 3. http://blog.oscarliang.net/quadcopter-pid-explained-tuning/

###Button map
  Button map              |       Button Number       |Actual Joystick Key No   |         Description
  ------------------------|:-------------------------:|:-----------------------:|----------------------------------
  'teleop_toggle'         |       0                   | 1| Switches to position control mode
  'clear_integrals'       |        1                  |2|Clear the intergrals before switching to position control( Good practice)
  'set_takeoff_setpoint'  |       2                   |3|Take off to 1 meter from current location
  'set_land_setpoint'     |       3                   |4|Land setpoint(just above the ground) from current position
  'arm'                   |6                          |7|arming the quad
  'disarm'                |   7                       |8|disarming the quad
  's1'                    | 8                         |9|pre programmed setpoint( one of the corners of 1*1 square)
  's2'                    | 9                         |10|pre programmed setpoint( one of the corners of 1*1 square)
  's3'                    | 10                        |11|pre programmed setpoint( one of the corners of 1*1 square)
  's4'                    | 11                        |12|pre programmed setpoint( one of the corners of 1*1 square)

*Printed button numbers on the joystick starts with 1 and ends with 12 *  


### ( check back for more updates)
