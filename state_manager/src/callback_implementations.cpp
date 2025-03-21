void StateManager::viconCallback( const TFStamped::ConstPtr &msg )
{
  /* Handle most of the state information directly, except for velocity
    and that needs to be computed after filtering positions */
  
  /* Note that Vicon gives us ENU, while we use NED */
  
  double time_since = (ros::Time::now() - lastUpdateTime_).toSec();
  
  px_ = msg -> transform.translation.y;
  py_ = msg -> transform.translation.x;
  pz_ = -(msg -> transform.translation.z);

  /*
  prev_pn_.erase( prev_pn_.begin() );
  prev_pn_.push_back( x );
  prev_pe_.erase( prev_pe_.begin() );
  prev_pe_.push_back( y );
  prev_pd_.erase( prev_pd_.begin() );
  prev_pd_.push_back( z );

  pose_filter_.filterObservations( prev_pn_, prev_pe_, prev_pd_, x, y, z );
  */

  /* positions
  state_vector_[0] = x;
  state_vector_[1] = y;
  state_vector_[2] = z; */
  
  /* velocities 
  double vx, vy, vz;
  vx = ( x - last_pn_ )/time_since; //[3]
  vy = ( y - last_pe_ )/time_since; //[4]
  vz = ( z - last_pd_ )/time_since;
  prev_vn_.erase( prev_vn_.begin() );
  prev_vn_.push_back( vx );
  prev_ve_.erase( prev_ve_.begin() );
  prev_ve_.push_back( vy );
  prev_vd_.erase( prev_vd_.begin() );
  prev_vd_.push_back( vz );

  rate_filter_.filterObservations( prev_vn_, prev_ve_, prev_vd_, vx, vy, vz );

  state_vector_[3] = vx;
  state_vector_[4] = vy;
  state_vector_[5] = vz;
  */ 
  vx_ = (px_ - px_old_)/time_since;
  vy_ = (py_ - py_old_)/time_since;
  vz_ = (pz_ - pz_old_)/time_since;

  /* fetch components from quaternion, make rotation matrix */
  //NED quaternion- take components of ENU quaternion, swap axes. Quaternion is in form [w x y z] = [qw qx qy qz]
  float qw, qx, qy, qz;
  qw = msg -> transform.rotation.w;
  qx = msg -> transform.rotation.y;
  qy = msg -> transform.rotation.x;
  qz = -1*(msg -> transform.rotation.z);
  //for simplicity, define squares of terms
  float qw2 = std::pow(qw, 2);
  float qx2 = std::pow(qx, 2);
  float qy2 = std::pow(qy, 2);
  float qz2 = std::pow(qz, 2);
  //form rotation matrix
  RAB << (1-2*(qy2 - qz2)), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw),
      2*(qx*qy + qz*qw), (1-2*(qx2 - qz2)), 2*(qy*qz - qx*qw),
      2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), (1-2*(qx2 - qy2));
  /*
  Eigen::MatrixXf Rab_roll;
  Eigen::MatrixXf Rab_pitch;
  Eigen::MatrixXf Rab_yaw;
  Eigen::MatrixXf Rab_roll_old;
  Eigen::MatrixXf Rab_pitch_old;
  Eigen::MatrixXf Rab_yaw_old;
  Rab_roll << 1, 0, 0,
            0, std::cos(drone_roll_), -1*std::sin(drone_roll_),
            0, std::sin(drone_roll_), std::cos(drone_roll_);
  Rab_pitch << std::cos(drone_pitch_), 0, std::sin(drone_pitch_),
          0, 1, 0
          -1*std::sin(drone_pitch_), 0, std::cos(drone_pitch_);
  Rab_yaw << std::cos(drone_yaw_), -1*std::sin(drone_yaw_), 0,
              std::sin(drone_yaw_), std::cos(drone_yaw_), 0,
              0, 0, 1;
  RAB = Rab_roll*Rab_pitch*Rab_yaw;
  //
  Rab_roll_old << 1, 0, 0,
            0, std::cos(drone_roll_old_), -1*std::sin(drone_roll_old_),
            0, std::sin(drone_roll_old_), std::cos(drone_roll_old_);
  Rab_pitch_old << std::cos(drone_pitch_old_), 0, std::sin(drone_pitch_old_),
          0, 1, 0
          -1*std::sin(drone_pitch_old_), 0, std::cos(drone_pitch_old_);
  Rab_yaw_old << std::cos(drone_yaw_old_), -1*std::sin(drone_yaw_old_), 0,
              std::sin(drone_yaw_old_), std::cos(drone_yaw_old_), 0,
              0, 0, 1;
  RAB_old = Rab_roll_old*Rab_pitch_old*Rab_yaw_old; */

  RAB_dot = (RAB - RAB_old)/time_since;
  
  /* rpy rates (no filter yet, use with caution!)
  state_vector_[9] = ( roll - last_roll_ )/time_since;
  state_vector_[10] = ( pitch - last_pitch_ )/time_since;
  state_vector_[11] = ( yaw - last_yaw_ )/time_since;
  */
  /* age of this data
  state_vector_[12] = time_since;
  */
  
  /* Update the current time this happened */
  lastUpdateTime_ = ros::Time::now();
  
  /*update old variables*/
  px_old_ = px_;
  py_old_ = py_;
  pz_old_ = pz_;
  RAB_old = RAB;
  /*
  drone_roll_old_ = drone_roll_;
  drone_pitch_old_ = drone_pitch_;
  drone_yaw_old_ = drone_yaw_;*/
  /* bookkeeping stuff 
  last_pn_ = state_vector_[0];
  last_pe_ = state_vector_[1];
  last_pd_ = state_vector_[2];
  last_roll_ = roll;
  last_pitch_ = pitch;
  last_yaw_ = yaw; */
  
  /* Copy over and publish right away
  static freyja_msgs::CurrentState state_msg;
  state_msg.header.stamp = ros::Time::now();
  for( uint8_t idx = 0; idx < STATE_VECTOR_LEN; idx++ )
    state_msg.state_vector[idx] = state_vector_[idx];
  state_pub_.publish( state_msg );
  */
}

void StateManager::asctecDataCallback( const freyja_msgs::AsctecData::ConstPtr &msg )
{
  double time_since = (ros::Time::now() - lastUpdateTime_).toSec();
  double x, y, z, vx, vy, vz;
  
  if( (msg -> motor1rpm) > 0 && !have_location_fix_ )
  {
    have_location_fix_ = true;
    home_lat_ = (msg -> best_lat)/10000000.0;
    home_lon_ = (msg -> best_lon)/10000000.0;
    ROS_INFO( "[StateManager]: Asctec: Home location set!" );
  }
  
  if( !have_location_fix_ )
    return;
  
  x = ( (msg->best_lat)/10000000.0 - home_lat_ )*111050.51;
  y = ( (msg->best_lon)/10000000.0 - home_lon_ )*84356.28;
  z = -(msg -> hgt)/1000.0;
  
  vy = (msg -> best_sp_x)/1000.0;
  vx = (msg -> best_sp_y)/1000.0;
  vz = -(msg -> best_sp_z)/1000.0;
  prev_vn_.erase( prev_vn_.begin() );
  prev_vn_.push_back( vx );
  prev_ve_.erase( prev_ve_.begin() );
  prev_ve_.push_back( vy );
  prev_vd_.erase( prev_vd_.begin() );
  prev_vd_.push_back( vz );
  rate_filter_.filterObservations( prev_vn_, prev_ve_, prev_vd_, vx, vy, vz );

  /* positions */
  state_vector_[0] = x;
  state_vector_[1] = y;
  state_vector_[2] = z;
  
  /* velocities from vehicle */
  state_vector_[3] = vx; //(msg -> best_sp_x)/1000.0;
  state_vector_[4] = vy; //(msg -> best_sp_y)/1000.0;
  state_vector_[5] = vz; //(msg -> best_sp_z)/1000.0;
  
  /* Attitude */
  state_vector_[6] = (msg -> roll_angle)/1000.0;
  state_vector_[7] = (msg -> pitch_angle)/1000.0;
  double yaw_c = DEG2RAD( (msg -> yaw_angle)/1000.0 );
  state_vector_[8] = ( yaw_c > F_PI )? yaw_c - 2*F_PI : yaw_c;
  
  /* rpy rates */
  state_vector_[9] = 0.0;
  state_vector_[10] = 0.0;
  state_vector_[11] = 0.0;
  
  /* age */
  state_vector_[12] = time_since;
  
  /* Update the current time this happened */
  lastUpdateTime_ = ros::Time::now();
  
  /* bookkeeping stuff */
  last_pn_ = state_vector_[0];
  last_pe_ = state_vector_[1];
  last_pd_ = state_vector_[2];
  
  /* Copy over and publish right away 
  freyja_msgs::CurrentState state_msg;
  state_msg.header.stamp = ros::Time::now();
  for( uint8_t idx = 0; idx < STATE_VECTOR_LEN; idx++ )
    state_msg.state_vector[idx] = state_vector_[idx];
  state_pub_.publish( state_msg );
  */
}

void StateManager::mavrosGpsRawCallback( const sensor_msgs::NavSatFix::ConstPtr& msg )
{
  /*
  if( msg -> status.status >= 0 && !have_location_fix_ )
  {
    // first location fix
    home_lat_ = msg -> latitude;
    home_lon_ = msg -> longitude;
    have_location_fix_ = true;
    ROS_INFO( "[StateManager]: APM: Home location set!" );
  }
  else
    return;

  state_vector_[0] = ( (msg->latitude)/10000000.0 - home_lat_ )*111050.51;
  state_vector_[1] = ( (msg->longitude)/10000000.0 - home_lon_ )*84356.28;
  */
}



void StateManager::mavrosCompassCallback( const std_msgs::Float64::ConstPtr &msg )
{
  compass_yaw_ = msg -> data;
}

void StateManager::mavrosGpsOdomCallback( const nav_msgs::Odometry::ConstPtr &msg )
{
  double time_since = (ros::Time::now() - lastUpdateTime_).toSec();
  px_ = msg -> pose.pose.position.y;
  py_ = msg -> pose.pose.position.x;
  pz_ = -(msg -> pose.pose.position.z);

  vx_ = (px_ - px_old_)/time_since;
  vy_ = (py_ - py_old_)/time_since;
  vz_ = (pz_ - pz_old_)/time_since;
  //my way of getting rotation matrix
  float qw, qx, qy, qz;
  qw = msg -> pose.pose.orientation.w;
  qx = msg -> pose.pose.orientation.y;
  qy = msg -> pose.pose.orientation.x;
  qz = -1*(msg -> pose.pose.orientation.z);
  //for simplicity, define squares of terms
  float qw2 = std::pow(qw, 2);
  float qx2 = std::pow(qx, 2);
  float qy2 = std::pow(qy, 2);
  float qz2 = std::pow(qz, 2);
  //form rotation matrix
  RAB << (1-2*(qy2 + qz2)), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw),
      2*(qx*qy + qz*qw), (1-2*(qx2 + qz2)), 2*(qy*qz - qx*qw),
      2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), (1-2*(qx2 + qy2));
  
  //original freyja way of getting roll and pitch
  tf::Quaternion q;
  tf::quaternionMsgToTF( msg->pose.pose.orientation, q);
  double yaw_placeholder;
  tf::Matrix3x3(q).getRPY(roll_actual, pitch_actual, yaw_placeholder);
  
  RAB_dot = (RAB - RAB_old)/time_since;
  /* Update the current time this happened */
  lastUpdateTime_ = ros::Time::now();

  /*update old variables*/
  px_old_ = px_;
  py_old_ = py_;
  pz_old_ = pz_;
  RAB_old = RAB;
  
  /* Callback for odometry direct from the Pixhawk.
  Note that this is Pixhawk's best estimate of where it is in the world,
  there is no need to filter it, only structure it to our format.
  Likely topics:
        /mavros/global_position/local --> does not zero at arming.
        /mavros/local_position/local  --> zeros at arming, has IMU frame quirk.
  
  
  static double pn, pe, pd, vn, ve, vd;
  
  // update containers anyway (needed for capturing arming location)
  gps_odom_pn_ = msg -> pose.pose.position.y;
  gps_odom_pe_ = msg -> pose.pose.position.x;
  gps_odom_pd_ = -( msg -> pose.pose.position.z );
  
  if( !have_arming_origin_ )
  {
    ROS_INFO_THROTTLE( 5, "StateManager:: Waiting for arming .." );
    return;
  }
  
  // armed at this point
  pn = gps_odom_pn_ + map_rtk_pn_ - arming_gps_pn_;
  pe = gps_odom_pe_ + map_rtk_pe_ - arming_gps_pe_;
  pd = gps_odom_pd_ + map_rtk_pd_ - arming_gps_pd_;
  
  vn = msg -> twist.twist.linear.y;
  ve = msg -> twist.twist.linear.x;
  vd = -( msg -> twist.twist.linear.z );
  
  static freyja_msgs::CurrentState state_msg;
  state_msg.state_vector[0] = pn;
  state_msg.state_vector[1] = pe;
  state_msg.state_vector[2] = pd;
  state_msg.state_vector[3] = vn;
  state_msg.state_vector[4] = ve;
  state_msg.state_vector[5] = vd;
  state_msg.state_vector[8] = DEG2RAD( compass_yaw_ );
  
  state_msg.header.stamp = ros::Time::now();
  state_pub_.publish( state_msg );
  */
}


void StateManager::mavrosRtkBaselineCallback( const geometry_msgs::Vector3::ConstPtr &msg )
{
  rtk_baseoffset_pn_ = msg -> x;
  rtk_baseoffset_pe_ = msg -> y;
  rtk_baseoffset_pd_ = msg -> z;
}

bool StateManager::maplockArmingHandler( BoolServReq& rq, BoolServRsp& rp )
{
  /*  Service handler for when the vehicle is armed or disarmed.
     This must lock in/release all the map offsets needed by manager.
  */
  if( !have_arming_origin_ )
  {
    lockArmingGps();  // set this locatin as origin
    lockMapRTK();     // capture current offset from rtk base (zero if no rtk)
    have_arming_origin_ = true;
    ROS_WARN( "StateManager::Origin set, locking RTK map-frame!" );
  }
  else
  {
    lockArmingGps( false );
    lockMapRTK( false );
    have_arming_origin_ = false;
    ROS_WARN( "StateManager::Origin cleared, unlocking RTK map-frame!" );
  }
  return true;
}
//void StateManager::payloadCallback( const std_msgs::Float32MultiArray::ConstPtr &msg )
void StateManager::payloadCallback( const sensor_msgs::JointState::ConstPtr &msg )
{
  /*float payload_roll_deg = msg -> data[0];
  float payload_pitch_deg = msg -> data[1];
  float payload_roll_dot_deg = msg -> data[2];
  float payload_pitch_dot_deg = msg -> data[3];
  
  //convert to rad
  float payload_roll_rad = payload_roll_deg*3.14159/180;
  float payload_pitch_rad = payload_pitch_deg*3.14159/180;
  float payload_roll_dot_rad = payload_roll_dot_deg*3.14159/180;
  float payload_pitch_dot_rad = payload_pitch_dot_deg*3.14159/180;
  */
  float payload_roll_rad = msg -> position[0];
  float payload_pitch_rad = msg -> position[1];
  float payload_roll_dot_rad = msg -> velocity[0];
  float payload_pitch_dot_rad = msg -> velocity[1];
  Eigen::Matrix3f Rbc_pitch;
  Eigen::Matrix3f Rbc_roll;
  Eigen::Matrix3f Rbc_yaw;
  /*Because of ENU to NED conversion, apply a yaw to the payload. In ENU convention, drone facing north is 90 degrees yaw.
  When we convert to NED, this becomes -90 degrees yaw. However, we want yaw to be 0 degrees at north, not -90 degrees at north. 
  Therefore, we add a 90 degree yaw to the payload to fix this problem.*/
  float yaw_payload = 90*3.14159/180;
  Rbc_pitch << std::cos(payload_pitch_rad), 0, std::sin(payload_pitch_rad),
              0, 1, 0,
              -1*std::sin(payload_pitch_rad), 0, std::cos(payload_pitch_rad);
  Rbc_roll << 1, 0, 0,
              0, std::cos(payload_roll_rad), -1*std::sin(payload_roll_rad),
              0, std::sin(payload_roll_rad), std::cos(payload_roll_rad);
  Rbc_yaw << std::cos(yaw_payload), -1*std::sin(yaw_payload), 0,
              std::sin(yaw_payload), std::cos(yaw_payload), 0,
              0, 0, 1;
  Eigen::Matrix3f RBC = Rbc_yaw*Rbc_pitch*Rbc_roll;//rotation matrix that transforms C frame (payload) to B frame (drone)
  
  float wp_x = payload_roll_dot_rad*std::cos(payload_pitch_rad);
  float wp_y = payload_pitch_dot_rad;
  float wp_z = -1*payload_roll_dot_rad*std::sin(payload_pitch_rad);
  //calculate total angular velocity
  Eigen::Matrix3f wd_skew; //skew symmetric matrix
  wd_skew = RAB_dot*(RAB.transpose());
  float wd_x, wd_y, wd_z;
  wd_x = -1*wd_skew(1,2);
  wd_y = wd_skew(0,2);
  wd_z = -1*wd_skew(0,1);
  Eigen::Vector3f wp_B; //payload angular velocity in B frame
  wp_B(0,0) = wp_x;
  wp_B(1,0) = wp_y;
  wp_B(2,0) = wp_z;
  Eigen::Vector3f wp_A;
  wp_A = RAB*wp_B;
  float w_total_x, w_total_y, w_total_z;
  w_total_x = wd_x + wp_A(0,0);
  w_total_y = wd_y + wp_A(1,0);
  w_total_z = wd_z + wp_A(2,0);
  //calculate q vector
  Eigen::Vector3f q_(3,1);
  Eigen::Vector3f k_(3,1);
  k_ << 0, 0, 1;
  q_ = RAB*RBC*k_;
  float qx_ = q_(0,0);
  float qy_ = q_(1,0);
  float qz_ = q_(2,0);

  static freyja_msgs::CurrentState state_msg;
  state_msg.header.stamp = ros::Time::now();
  //putting stuff in state vector
  state_msg.state_vector[0] = px_; //pn
  state_msg.state_vector[1] = py_; //pe
  state_msg.state_vector[2] = pz_; //pd
  state_msg.state_vector[3] = vx_; //vn
  state_msg.state_vector[4] = vy_; //ve
  state_msg.state_vector[5] = vz_; //vd
  state_msg.state_vector[6] = qx_ ;//qn
  state_msg.state_vector[7] = qy_ ;//qe
  state_msg.state_vector[8] = qz_ ;//qd
  state_msg.state_vector[9] = w_total_x;//wn
  state_msg.state_vector[10] = w_total_y;//we
  state_msg.state_vector[11] = w_total_z;//wd*/
  state_msg.state_vector[12] = roll_actual;
  state_msg.state_vector[13] = pitch_actual;
  state_pub_.publish( state_msg );
}
void StateManager::cameraUpdatesCallback( const CameraOdom::ConstPtr &msg )
{
  /* Assume that camera directly gives us positions AND velocity. This is
    ok to do, so as to allow different modes of computation at the camera node;
    meaning that it should handle its own velocity calculations (particularly
    useful when using image Jacobians and such). This callback then simply
    reformats the acquired data into a state vector of our style.
    Odometry has to be one of the most ridiculously named structures ..
    nav_msgs::Odometry - pose
                          - pose
                            - position {x,y,z}
                            - orientation {Quaternion}
                          - covariance (6x6)
                       - twist
                          - twist
                            - linear {x,y,z}
                            - angular {x,y,z}
                          - covariance (6x6).
     NOTE: axis convention aligns with camera plane axes!
          x   -- lateral/horizontal/left(-) to right(+)
          y   -- longitudinal/vertical/bottom(-) to top(+)
          z   -- away from the camera (always positive)
          yaw -- not handled yet (set to zero)
          r/p -- not handled
    */
    
  double vx, vy, vz;
  
  state_vector_[0] = msg -> pose.pose.position.y;
  state_vector_[1] = msg -> pose.pose.position.x;
  state_vector_[2] = -( msg -> pose.pose.position.z );
  
  // filter velocities maybe? Prefer a small-length filter ..
  vx = msg -> twist.twist.linear.y;
  vy = msg -> twist.twist.linear.x;
  vz = -( msg -> twist.twist.linear.z );
  prev_vn_.erase( prev_vn_.begin() );
  prev_vn_.push_back( vx );
  prev_ve_.erase( prev_ve_.begin() );
  prev_ve_.push_back( vy );
  prev_vd_.erase( prev_vd_.begin() );
  prev_vd_.push_back( vz );
  rate_filter_.filterObservations( prev_vn_, prev_ve_, prev_vd_, vx, vy, vz );
  state_vector_[3] = vx;
  state_vector_[4] = vy;
  state_vector_[5] = vz;
  
  /* unfiltered:
  state_vector_[3] = msg -> twist.twist.linear.y;
  state_vector_[4] = msg -> twist.twist.linear.x;
  state_vector_[5] = -( msg -> twist.twist.linear.z );
  */
  
  tf::Quaternion q;
  tf::quaternionMsgToTF( msg -> pose.pose.orientation, q );
  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY( roll, pitch, yaw );
  state_vector_[8] = yaw;

  // publish away!
  freyja_msgs::CurrentState state_msg;
  state_msg.header.stamp = ros::Time::now();
  for( uint8_t idx = 0; idx < STATE_VECTOR_LEN; idx++ )
    state_msg.state_vector[idx] = state_vector_[idx];
  state_pub_.publish( state_msg );
}