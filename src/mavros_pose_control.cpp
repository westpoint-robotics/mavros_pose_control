#include <mavros_pose_control/mavros_pose_control.h>
#include <signal.h>

mavrosPoseController::mavrosPoseController(){
  z_pos_.reset( new PIDController(1.0, 0.01, 0.0, ros::Time::now().toSec(), 5.0, -5.0 ) );
  z_vel_.reset( new PIDController(0.1, 0.01, 0.0, ros::Time::now().toSec(), 1.0, -1.0 ) );

  y_pos_.reset( new PIDController(1.0, 0.01, 0.0, ros::Time::now().toSec(), 1.0, -1.0 ) );
  y_vel_.reset( new PIDController(0.1, 0.01, 0.0, ros::Time::now().toSec(), 0.1, -0.1 ) );

  x_pos_.reset( new PIDController(1.0, 0.01, 0.0, ros::Time::now().toSec(), 1.0, -1.0 ) );
  x_vel_.reset( new PIDController(0.1, 0.01, 0.0, ros::Time::now().toSec(), 0.1, -0.1 ) );

  cur_pose_.reset( new geometry_msgs::PoseStamped() );
  cur_twist_.reset( new geometry_msgs::TwistStamped() );
  desired_set_point_.reset( new geometry_msgs::Pose() );

  ros::NodeHandle n;

  f = boost::bind( &mavrosPoseController::configCb, this, _1, _2 );
  server.setCallback(f);

  attThrottlePub = n.advertise<std_msgs::Float64>("/mpc/pid/throttle", 1000);
  attRollPub = n.advertise<std_msgs::Float64>("/mpc/pid/roll", 1000);
  attPitchPub = n.advertise<std_msgs::Float64>("/mpc/pid/pitch", 1000);

  ctPub = n.advertise<geometry_msgs::TwistStamped>("/mavros/vision_pose/twist",100);
  attitudePub = n.advertise<geometry_msgs::PoseStamped>("/mpc/pid/attitude",100);

  feedbackPoseSub = n.subscribe("/mavros/vision_pose/pose", 100, &mavrosPoseController::feedbackPoseCb, this);
  setpointPoseSub = n.subscribe("/mpc/pose_setpoint", 100, &mavrosPoseController::setpointPoseCb, this);
  resetSub = n.subscribe("/mpc/reset_integrals", 1, &mavrosPoseController::integralResetCb, this);
  control_thread_ = std::thread( &mavrosPoseController::control_loop, this );
}

mavrosPoseController::~mavrosPoseController(){
  alive_.lock();
}

void mavrosPoseController::integralResetCb( std_msgs::Empty msg ) {
    z_pos_->resetIntegral();
    y_pos_->resetIntegral();
    x_pos_->resetIntegral();
    z_vel_->resetIntegral();
    y_vel_->resetIntegral();
    x_vel_->resetIntegral();
}

void mavrosPoseController::configCb( mavros_pose_control::ControlTunerConfig &config, uint32_t level ){
  y_pos_->setGains( config.yp1, config.yi1, config.yd1 );
  y_vel_->setGains( config.yp2, config.yi2, config.yd2 );
  x_pos_->setGains( config.xp1, config.xi1, config.xd1 );
  x_vel_->setGains( config.xp2, config.xi2, config.xd2 );
  z_pos_->setGains( config.zp1, config.zi1, config.zd1 );
  z_vel_->setGains( config.zp2, config.zi2, config.zd2 );
  z_throttle_zero_ = config.throttle_zero;
  z_throttle_divider_ = config.throttle_divd;
}

void mavrosPoseController::setpointPoseCb( geometry_msgs::Pose msg ){
 setpoint_data_lock_.lock();
 desired_set_point_.reset( new geometry_msgs::Pose( msg ) );
 setpoint_data_lock_.unlock();
}

void mavrosPoseController::feedbackPoseCb( geometry_msgs::PoseStamped msg ){
  feedback_data_lock_.lock();
  // start the velocity calculation
  geometry_msgs::TwistStamped delta;
  ros::Duration ros_dt = (msg.header.stamp - cur_pose_->header.stamp );
  double dt = ros_dt.toSec();
  delta.header = msg.header;
  // msg is most recent  --- cur_pose_ is leftover from the last time this was called
  delta.twist.linear.x = (-cur_pose_->pose.position.x + msg.pose.position.x)/(dt);
  delta.twist.linear.y = (-cur_pose_->pose.position.y + msg.pose.position.y)/(dt);
  delta.twist.linear.z = (-cur_pose_->pose.position.z + msg.pose.position.z)/(dt);

  delta.twist.angular.x = 0.0;
  delta.twist.angular.y = 0.0;
  delta.twist.angular.z = 0.0;

  // get previous roll pitch yaw
  double pr, pp, py;
  tf::Quaternion q( cur_pose_->pose.orientation.x, cur_pose_->pose.orientation.y, cur_pose_->pose.orientation.z, cur_pose_->pose.orientation.w );
  tf::Matrix3x3 m(q);
  m.getRPY(pr, pp, py);

  // get current roll pitch yaw
  double cr, cp, cy;
  q = tf::Quaternion( msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w );
  m = tf::Matrix3x3(q);
  m.getRPY(cr, cp, cy);

  // calculate the angular velocity
  delta.twist.angular.x = (cr - pr)/dt;
  delta.twist.angular.y = (cp - pp)/dt;
  delta.twist.angular.z = (cy - py)/dt;

  ctPub.publish( delta );

  // send the current data to the control loop
  cur_twist_.reset( new geometry_msgs::TwistStamped(delta) );
  cur_pose_.reset( new geometry_msgs::PoseStamped(msg) );
  feedback_data_lock_.unlock();
}

void mavrosPoseController::control_loop(){
  ros::Rate r( rate_ );
  while( alive_.try_lock() && ros::ok() )
  {
    feedback_data_lock_.lock();
    std::shared_ptr<geometry_msgs::PoseStamped> tmp_pose( cur_pose_ );
    std::shared_ptr<geometry_msgs::TwistStamped> tmp_twist( cur_twist_ );
    feedback_data_lock_.unlock();
    setpoint_data_lock_.lock();
    std::shared_ptr<geometry_msgs::Pose> tmp_setpoint( desired_set_point_ );
    setpoint_data_lock_.unlock();

    // current time
    double ct = ros::Time::now().toSec();

    // process the cascaded z pid loops
    ROS_INFO_THROTTLE(1,"::::::::::::::::::::::::::::::::::::::::::::");
    ROS_INFO_THROTTLE(1,"::::::::::::::::::::[pos]:::::::::::::::::::");

    z_pos_->setSetPoint( tmp_setpoint->position.z );
    double desired_z_vel = z_pos_->process( ct, tmp_pose->pose.position.z );
    ROS_INFO_THROTTLE(1, "[CE]::%f  [DE]::%f  [SE]::%f", z_pos_->getCE(), z_pos_->getDE(), z_pos_->getSE() );

    y_pos_->setSetPoint( tmp_setpoint->position.y );
    double desired_y_vel = y_pos_->process( ct, tmp_pose->pose.position.y );

    x_pos_->setSetPoint( tmp_setpoint->position.x );
    double desired_x_vel = x_pos_->process( ct, tmp_pose->pose.position.x );
    ROS_INFO_THROTTLE(1, "[x]::%f  [y]::%f  [z]::%f", desired_x_vel, desired_y_vel, desired_z_vel );
    ROS_INFO_THROTTLE(1, "::::::::::::::::::::[vel]:::::::::::::::::::");
    z_vel_->setSetPoint( desired_z_vel );
    y_vel_->setSetPoint( desired_y_vel );
    x_vel_->setSetPoint( desired_x_vel );
    double z_cmd = z_vel_->process( ct, tmp_twist->twist.linear.z );
    ROS_INFO_THROTTLE(1, "[desired_z_cmd]::%f", (z_cmd)/z_throttle_divider_ + z_throttle_zero_ );
    std_msgs::Float64 throttle;
    throttle.data = ((z_cmd)/z_throttle_divider_) + z_throttle_zero_;
    attThrottlePub.publish(throttle);
    


    double y_world_cmd = y_vel_->process( ct, tmp_twist->twist.linear.y );
    double x_world_cmd = x_vel_->process( ct, tmp_twist->twist.linear.x );

    double mag = sqrt(x_world_cmd*x_world_cmd + y_world_cmd*y_world_cmd);
    double theta = atan2(y_world_cmd, x_world_cmd);
    ROS_INFO_THROTTLE(1, "[x_world_cmd]::%f  [y_world_cmd]::%f  [mag]::%f  [theta]::%f", x_world_cmd, y_world_cmd, mag, theta);

    double cr, cp, yaw;
    tf::Quaternion q = tf::Quaternion( tmp_pose->pose.orientation.x, tmp_pose->pose.orientation.y, tmp_pose->pose.orientation.z, tmp_pose->pose.orientation.w );
    tf::Matrix3x3  m = tf::Matrix3x3(q);
    m.getRPY(cr, cp, yaw);

    ROS_INFO_THROTTLE(1, "[yaw]::%f", yaw);

    double pitch_cmd = (sin(yaw)*x_world_cmd + cos(yaw)*y_world_cmd);
    double roll_cmd  = cos(yaw)*x_world_cmd - sin(yaw)*y_world_cmd;
    ROS_INFO_THROTTLE(1, "[pitch_cmd]::%f  [roll_cmd]::%f", (pitch_cmd), roll_cmd );

    geometry_msgs::PoseStamped att_twist;
    att_twist.header.stamp = ros::Time::now();
    att_twist.pose.orientation.x = roll_cmd;
    att_twist.pose.orientation.y = pitch_cmd;
    att_twist.pose.orientation.w = 1.0;

    attitudePub.publish(att_twist);

    // some debugging publishers
    std_msgs::Float64 roll;
    roll.data = roll_cmd;
    attRollPub.publish(roll);
    std_msgs::Float64 pitch;
    pitch.data = pitch_cmd;
    attPitchPub.publish(pitch);

    alive_.unlock();
    r.sleep();
  }
}

mavrosPoseController* mpc;

void signal_handler( int sig )
{
  delete mpc;
}

int main( int argc, char** argv ){
  ros::init(argc, argv, "mpc");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  mpc = new mavrosPoseController();

  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);
  signal(SIGKILL, signal_handler);

  ros::waitForShutdown();
}
