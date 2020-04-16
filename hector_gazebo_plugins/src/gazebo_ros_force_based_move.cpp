/*
 * Copyright 2015 Stefan Kohlbrecher, TU Darmstadt
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/*
 * Desc: Simple model controller that uses a twist message to exert
 *       forces on a robot, resulting in motion. Based on the
 *       planar_move plugin by Piyush Khandelwal.
 * Author: Stefan Kohlbrecher
 * Date: 06 August 2015
 */

#include <hector_gazebo_plugins/gazebo_ros_force_based_move.h>

namespace gazebo 
{

  GazeboRosForceBasedMove::GazeboRosForceBasedMove() {}

  GazeboRosForceBasedMove::~GazeboRosForceBasedMove() {}

  // Load the controller
  void GazeboRosForceBasedMove::Load(physics::ModelPtr parent,
      sdf::ElementPtr sdf) 
  {

    parent_ = parent;

    /* Parse parameters */

    robot_namespace_ = "";
    if (!sdf->HasElement("robotNamespace")) 
    {
      ROS_INFO("PlanarMovePlugin missing <robotNamespace>, "
          "defaults to \"%s\"", robot_namespace_.c_str());
    }
    else 
    {
      robot_namespace_ = 
        sdf->GetElement("robotNamespace")->Get<std::string>();
    }

    command_topic_ = "cmd_vel";
    if (!sdf->HasElement("commandTopic")) 
    {
      ROS_WARN("PlanarMovePlugin (ns = %s) missing <commandTopic>, "
          "defaults to \"%s\"", 
          robot_namespace_.c_str(), command_topic_.c_str());
    } 
    else 
    {
      command_topic_ = sdf->GetElement("commandTopic")->Get<std::string>();
    }

    odometry_topic_ = "odom";
    if (!sdf->HasElement("odometryTopic")) 
    {
      ROS_WARN("PlanarMovePlugin (ns = %s) missing <odometryTopic>, "
          "defaults to \"%s\"", 
          robot_namespace_.c_str(), odometry_topic_.c_str());
    } 
    else 
    {
      odometry_topic_ = sdf->GetElement("odometryTopic")->Get<std::string>();
    }

    odometry_frame_ = "odom";
    if (!sdf->HasElement("odometryFrame")) 
    {
      ROS_WARN("PlanarMovePlugin (ns = %s) missing <odometryFrame>, "
          "defaults to \"%s\"",
          robot_namespace_.c_str(), odometry_frame_.c_str());
    }
    else 
    {
      odometry_frame_ = sdf->GetElement("odometryFrame")->Get<std::string>();
    }

    // setting PID controllers
    
    force_x_velocity_p_gain_ = 10000.0;
    force_x_velocity_i_gain_ = 0.0;
    force_x_velocity_d_gain_ = 0.0;

    force_y_velocity_p_gain_ = 10000.0;
    force_y_velocity_i_gain_ = 0.0;
    force_y_velocity_d_gain_ = 0.0;   

    torque_yaw_velocity_p_gain_ = 100.0;
    torque_yaw_velocity_i_gain_ = 0.0;
    torque_yaw_velocity_d_gain_ = 0.0;

    if (sdf->HasElement("x_velocity_p_gain"))
      (sdf->GetElement("x_velocity_p_gain")->GetValue()->Get(force_x_velocity_p_gain_));

    if (sdf->HasElement("x_velocity_i_gain"))
      (sdf->GetElement("x_velocity_i_gain")->GetValue()->Get(force_x_velocity_i_gain_));

    if (sdf->HasElement("x_velocity_d_gain"))
      (sdf->GetElement("x_velocity_d_gain")->GetValue()->Get(force_x_velocity_d_gain_));

    if (sdf->HasElement("y_velocity_p_gain"))
      (sdf->GetElement("y_velocity_p_gain")->GetValue()->Get(force_y_velocity_p_gain_));

    if (sdf->HasElement("y_velocity_i_gain"))
      (sdf->GetElement("y_velocity_i_gain")->GetValue()->Get(force_y_velocity_i_gain_));

    if (sdf->HasElement("y_velocity_d_gain"))
      (sdf->GetElement("y_velocity_d_gain")->GetValue()->Get(force_y_velocity_d_gain_));

    if (sdf->HasElement("yaw_velocity_p_gain"))
      (sdf->GetElement("yaw_velocity_p_gain")->GetValue()->Get(torque_yaw_velocity_p_gain_));

    if (sdf->HasElement("yaw_velocity_i_gain"))
      (sdf->GetElement("yaw_velocity_i_gain")->GetValue()->Get(torque_yaw_velocity_i_gain_));

    if (sdf->HasElement("yaw_velocity_d_gain"))
      (sdf->GetElement("yaw_velocity_d_gain")->GetValue()->Get(torque_yaw_velocity_d_gain_));

      
    ROS_INFO_STREAM("ForceBasedMove x axis using: P: " << force_x_velocity_p_gain_ <<
                                               ", I: " << force_x_velocity_i_gain_ <<
                                               ", D: " << force_x_velocity_d_gain_);

    ROS_INFO_STREAM("ForceBasedMove y axis using: P: " << force_y_velocity_p_gain_ <<
                                               ", I: " << force_y_velocity_i_gain_ <<
                                               ", D: " << force_y_velocity_d_gain_);

    ROS_INFO_STREAM("ForceBasedMove z axis using: P: " << torque_yaw_velocity_p_gain_ <<
                                               ", I: " << torque_yaw_velocity_i_gain_ <<
                                               ", D: " << torque_yaw_velocity_d_gain_);

    double integralLimit = 1000.0;
    double commandLimit = 1200.0;

    if (sdf->HasElement("integralLimit"))
      (sdf->GetElement("integralLimit")->GetValue()->Get(integralLimit));

    if (sdf->HasElement("commandLimit"))
      (sdf->GetElement("commandLimit")->GetValue()->Get(commandLimit));

    xPID_.Init(force_x_velocity_p_gain_, force_x_velocity_i_gain_, force_x_velocity_d_gain_, 5*integralLimit, -5*integralLimit, 1.5*commandLimit, -1.5*commandLimit);
    yPID_.Init(force_y_velocity_p_gain_, force_y_velocity_i_gain_, force_y_velocity_d_gain_, 5*integralLimit, -5*integralLimit, 1.5*commandLimit, -1.5*commandLimit);
    zPID_.Init(torque_yaw_velocity_p_gain_, torque_yaw_velocity_i_gain_, torque_yaw_velocity_d_gain_, 5*integralLimit, -5*integralLimit, 3*commandLimit, -3*commandLimit);

    robot_base_frame_ = "base_footprint";
    if (!sdf->HasElement("robotBaseFrame")) 
    {
      ROS_WARN("PlanarMovePlugin (ns = %s) missing <robotBaseFrame>, "
          "defaults to \"%s\"",
          robot_namespace_.c_str(), robot_base_frame_.c_str());
    } 
    else 
    {
      robot_base_frame_ = sdf->GetElement("robotBaseFrame")->Get<std::string>();
    }

    ROS_INFO_STREAM("robotBaseFrame for force based move plugin: " << robot_base_frame_  << "\n");


    x_base_frame_ = "omnitorso_x";
    if (!sdf->HasElement("xBaseFrame")) 
    {
      ROS_WARN("PlanarMovePlugin (ns = %s) missing <xBaseFrame>, "
          "defaults to \"%s\"",
          robot_namespace_.c_str(), x_base_frame_.c_str());
    } 
    else 
    {
      x_base_frame_ = sdf->GetElement("xBaseFrame")->Get<std::string>();
    }

    ROS_INFO_STREAM("xBaseFrame for force based move plugin: " << x_base_frame_  << "\n");


    y_base_frame_ = "omnitorso_y";
    if (!sdf->HasElement("yBaseFrame")) 
    {
      ROS_WARN("PlanarMovePlugin (ns = %s) missing <yBaseFrame>, "
          "defaults to \"%s\"",
          robot_namespace_.c_str(), y_base_frame_.c_str());
    } 
    else 
    {
      y_base_frame_ = sdf->GetElement("yBaseFrame")->Get<std::string>();
    }

    ROS_INFO_STREAM("yBaseFrame for force based move plugin: " << y_base_frame_  << "\n");


    this->link_ = parent->GetLink(robot_base_frame_);
    // this->xLink_ = parent->GetLink(x_base_frame_);
    // this->yLink_ = parent->GetLink(y_base_frame_);

    odometry_rate_ = 20.0;
    if (!sdf->HasElement("odometryRate")) 
    {
      ROS_WARN("PlanarMovePlugin (ns = %s) missing <odometryRate>, "
          "defaults to %f",
          robot_namespace_.c_str(), odometry_rate_);
    } 
    else 
    {
      odometry_rate_ = sdf->GetElement("odometryRate")->Get<double>();
    } 

    this->publish_odometry_tf_ = true;
    if (!sdf->HasElement("publishOdometryTf")) {
      ROS_WARN("PlanarMovePlugin Plugin (ns = %s) missing <publishOdometryTf>, defaults to %s",
               this->robot_namespace_.c_str(), this->publish_odometry_tf_ ? "true" : "false");
    } else {
      this->publish_odometry_tf_ = sdf->GetElement("publishOdometryTf")->Get<bool>();
    }

    this->useRealOdom = true;
    if (!sdf->HasElement("useRealOdom")) {
      ROS_WARN("No useRealOdom, setting value to default", this->useRealOdom ? "true" : "false");
    } else {
      this->useRealOdom = sdf->GetElement("useRealOdom")->Get<bool>();
    }

 
#if (GAZEBO_MAJOR_VERSION >= 8)
    last_odom_publish_time_ = parent_->GetWorld()->SimTime();
    last_odom_pose_ = parent_->WorldPose();
#else
    last_odom_publish_time_ = parent_->GetWorld()->GetSimTime();
    last_odom_pose_ = parent_->GetWorldPose();
#endif
    x_ = 0;
    y_ = 0;
    rot_ = 0;
    prevX_ = 0.0;
    prevY_ = 0.0;
    prevRot_ = 0.0;

    alive_ = true;

    odom_transform_.setIdentity();

    // Ensure that ROS has been initialized and subscribe to cmd_vel
    if (!ros::isInitialized()) 
    {
      ROS_FATAL_STREAM("PlanarMovePlugin (ns = " << robot_namespace_
        << "). A ROS node for Gazebo has not been initialized, "
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    rosnode_.reset(new ros::NodeHandle(robot_namespace_));

    ROS_DEBUG("OCPlugin (%s) has started!", 
        robot_namespace_.c_str());

    tf_prefix_ = tf::getPrefixParam(*rosnode_);

    if (publish_odometry_tf_)
      transform_broadcaster_.reset(new tf::TransformBroadcaster());

    // subscribe to the odometry topic
    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
          boost::bind(&GazeboRosForceBasedMove::cmdVelCallback, this, _1),
          ros::VoidPtr(), &queue_);

    vel_sub_ = rosnode_->subscribe(so);
    odometry_pub_ = rosnode_->advertise<nav_msgs::Odometry>(odometry_topic_, 1);

    // start custom queue for diff drive
    callback_queue_thread_ = 
      boost::thread(boost::bind(&GazeboRosForceBasedMove::QueueThread, this));

    // listen to the update event (broadcast every simulation iteration)
    update_connection_ = 
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboRosForceBasedMove::UpdateChild, this));

  }

  // Update the controller
  void GazeboRosForceBasedMove::UpdateChild()
  {       
    boost::mutex::scoped_lock scoped_lock(lock);
  #if (GAZEBO_MAJOR_VERSION >= 8)
    common::Time current_reg_time = parent_->GetWorld()->SimTime();
  #else
    common::Time current_reg_time = parent_->GetWorld()->GetSimTime();
  #endif
    common::Time seconds_since_last_reg_update = current_reg_time - last_reg_publish_time_;

  #if (GAZEBO_MAJOR_VERSION >= 8)
    ignition::math::Pose3d pose = parent_->WorldPose();

    ignition::math::Vector3d angular_vel = parent_->WorldAngularVel();

    
    // zPID_.SetCmd(rot_);
    // ROS_INFO(std::string("Rotation error: ").append(std::to_string(error)).c_str());
    // prevError = error;

    // link_->AddTorque(ignition::math::Vector3d(0.0, 0.0, -error * torque_yaw_velocity_p_gain_));
    double error = angular_vel.Z() - rot_;
    const double rotControl = zPID_.Update(error, seconds_since_last_reg_update);

    link_->AddTorque(ignition::math::Vector3d(0.0, 0.0, std::isnan(rotControl) ? 0.0 : rotControl));
    float yaw = pose.Rot().Yaw();

    ignition::math::Vector3d linear_vel = parent_->RelativeLinearVel();
    const auto velocity = parent_->WorldLinearVel();

    // xPID_.SetCmd(linear_vel.X());
    // yPID_.SetCmd(linear_vel.Y());

    const double xControl = xPID_.Update(linear_vel.X() - x_, seconds_since_last_reg_update);
    const double yControl = yPID_.Update(linear_vel.Y() - y_, seconds_since_last_reg_update);
    link_->AddRelativeForce(ignition::math::Vector3d(std::isnan(xControl) ? 0.0 : xControl, std::isnan(yControl) ? 0.0 : yControl, 0.0));
    // ROS_INFO_STREAM("x: " << xControl << ", y: " << yControl << ", z: " << rotControl);
    // ROS_INFO(std::string("X linear: ").append(std::to_string(velocity.X())).c_str());
    // ROS_INFO(std::string("Y linear: ").append(std::to_string(velocity.Y())).c_str());

    // ROS_INFO(std::string("X error: ").append(std::to_string(x_ - linear_vel.X())).c_str());
    // ROS_INFO(std::string("Y error: ").append(std::to_string(y_ - linear_vel.Y())).c_str());
    // xLink_->AddRelativeForce(ignition::math::Vector3d((x_ - linear_vel.X())* force_x_velocity_p_gain_, 0.0, 0.0));
    // yLink_->AddRelativeForce(ignition::math::Vector3d(0.0, (y_ - linear_vel.Y())* force_y_velocity_p_gain_, 0.0));
    // link_->AddRelativeForce(ignition::math::Vector3d((x_ - linear_vel.X())* force_x_velocity_p_gain_,
    //                                                  (y_ - linear_vel.Y())* force_y_velocity_p_gain_,
    //                                                  0.0));

#else
    math::Pose pose = parent_->GetWorldPose();

    math::Vector3 angular_vel = parent_->GetWorldAngularVel();

    double error = angular_vel.z - rot_;

    link_->AddTorque(math::Vector3(0.0, 0.0, -error * torque_yaw_velocity_p_gain_));

    float yaw = pose.rot.GetYaw();

    math::Vector3 linear_vel = parent_->GetRelativeLinearVel();

    link_->AddRelativeForce(math::Vector3((x_ - linear_vel.x)* force_x_velocity_p_gain_,
                                          (y_ - linear_vel.y)* force_y_velocity_p_gain_,
                                          0.0));
#endif
    //parent_->PlaceOnNearestEntityBelow();
    //parent_->SetLinearVel(math::Vector3(
    //      x_ * cosf(yaw) - y_ * sinf(yaw),
    //      y_ * cosf(yaw) + x_ * sinf(yaw),
    //      0));
    //parent_->SetAngularVel(math::Vector3(0, 0, rot_));

    if (odometry_rate_ > 0.0) {
#if (GAZEBO_MAJOR_VERSION >= 8)
      common::Time current_time = parent_->GetWorld()->SimTime();
#else
      common::Time current_time = parent_->GetWorld()->GetSimTime();
#endif
      double seconds_since_last_update = 
        (current_time - last_odom_publish_time_).Double();
      if (seconds_since_last_update > (1.0 / odometry_rate_)) {
        publishOdometry(seconds_since_last_update);
        last_odom_publish_time_ = current_time;
      }
    }
  }

  // Finalize the controller
  void GazeboRosForceBasedMove::FiniChild() {
    alive_ = false;
    queue_.clear();
    queue_.disable();
    rosnode_->shutdown();
    callback_queue_thread_.join();
  }

  void GazeboRosForceBasedMove::cmdVelCallback(
      const geometry_msgs::Twist::ConstPtr& cmd_msg) 
  {
    boost::mutex::scoped_lock scoped_lock(lock);
    x_ = cmd_msg->linear.x;
    y_ = cmd_msg->linear.y;
    rot_ = cmd_msg->angular.z;
  }

  void GazeboRosForceBasedMove::QueueThread()
  {
    static const double timeout = 0.01;
    while (alive_ && rosnode_->ok()) 
    {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

//     void GazeboRosForceBasedMove::publishOdometry(double step_time)
//   {
//     ros::Time current_time = ros::Time::now();
//     std::string odom_frame = tf::resolve(tf_prefix_, odometry_frame_);
//     std::string base_footprint_frame = tf::resolve(tf_prefix_, robot_base_frame_);
//     odom_.header.stamp = current_time;
//     odom_.header.frame_id = odom_frame;
//     odom_.child_frame_id = base_footprint_frame;
//     if (transform_broadcaster_.get()){
//       transform_broadcaster_->sendTransform(tf::StampedTransform(odom_transform_, current_time, "odom", "world"));
//     }
      
//      odom_.pose.covariance[0] = 0.001;
//      odom_.pose.covariance[7] = 0.001;
//      odom_.pose.covariance[14] = 1000000000000.0;
//      odom_.pose.covariance[21] = 1000000000000.0;
//      odom_.pose.covariance[28] = 1000000000000.0;
      
// #if (GAZEBO_MAJOR_VERSION >= 8)
//     if (std::abs(angular_vel.Z()) < 0.0001) {
// #else
//     if (std::abs(angular_vel.z) < 0.0001) {
// #endif
//       odom_.pose.covariance[35] = 0.01;
//     } else {
//       odom_.pose.covariance[35] = 100.0;
//     }

//     odom_.twist.covariance[0] = 0.001;
//     odom_.twist.covariance[7] = 0.001;
//     odom_.twist.covariance[14] = 0.001;
//     odom_.twist.covariance[21] = 1000000000000.0;
//     odom_.twist.covariance[28] = 1000000000000.0;

// #if (GAZEBO_MAJOR_VERSION >= 8)
//     if (std::abs(angular_vel.Z()) < 0.0001) {
// #else
//     if (std::abs(angular_vel.z) < 0.0001) {
// #endif
//       odom_.twist.covariance[35] = 0.01;
//     } else {
//       odom_.twist.covariance[35] = 100.0;
//     }

//     if(this->useRealOdom){
//   #if (GAZEBO_MAJOR_VERSION >= 8)
//       ignition::math::Vector3d angular_vel = parent_->RelativeAngularVel();
//       ignition::math::Vector3d linear_vel = parent_->RelativeLinearVel();

//       odom_transform_= odom_transform_ * this->getTransformForMotion(linear_vel.X(), linear_vel.Y(), angular_vel.Z(), step_time);

//       tf::poseTFToMsg(odom_transform_, odom_.pose.pose);
//       odom_.twist.twist.angular.z = angular_vel.Z();
//       odom_.twist.twist.linear.x  = linear_vel.X();
//       odom_.twist.twist.linear.y  = linear_vel.Y();
//   #else
//       math::Vector3 angular_vel = parent_->GetRelativeAngularVel();
//       math::Vector3 linear_vel = parent_->GetRelativeLinearVel();

//       odom_transform_= odom_transform_ * this->getTransformForMotion(linear_vel.x, linear_vel.y, angular_vel.z, step_time);

//       tf::poseTFToMsg(odom_transform_, odom_.pose.pose);
//       odom_.twist.twist.angular.z = angular_vel.z;
//       odom_.twist.twist.linear.x  = linear_vel.x;
//   #endif
//     } else {
//       const auto pose = this->parent_->WorldPose();
//       const auto linVel = this->parent_->WorldLinearVel();
//       const auto angVel = this->parent_->WorldAngularVel();
//       odom_.pose.pose. = pose.pose;
//       odom_.twist.twist.linear.x = linVel.X();
//       odom_.twist.twist.linear.y = linVel.Y();
//       odom_.twist.twist.linear.z = linVel.Z();
//       odom_.twist.twist.angular.x = angVel.X();
//       odom_.twist.twist.angular.y = angVel.Y();
//       odom_.twist.twist.angular.z = angVel.Z();
//     }
//     odometry_pub_.publish(odom_);
//   }




  void GazeboRosForceBasedMove::publishOdometry(double step_time)
  {
    ros::Time current_time = ros::Time::now();
    std::string odom_frame = tf::resolve(tf_prefix_, odometry_frame_);
    std::string base_footprint_frame = tf::resolve(tf_prefix_, robot_base_frame_);

    if(this->useRealOdom){

      #if (GAZEBO_MAJOR_VERSION >= 8)
        ignition::math::Vector3d angular_vel = parent_->RelativeAngularVel();
        ignition::math::Vector3d linear_vel = parent_->RelativeLinearVel();

        odom_transform_= odom_transform_ * this->getTransformForMotion(linear_vel.X(), linear_vel.Y(), angular_vel.Z(), step_time);

        tf::poseTFToMsg(odom_transform_, odom_.pose.pose);
        odom_.twist.twist.angular.z = angular_vel.Z();
        odom_.twist.twist.linear.x  = linear_vel.X();
        odom_.twist.twist.linear.y  = linear_vel.Y();
      #else
        math::Vector3 angular_vel = parent_->GetRelativeAngularVel();
        math::Vector3 linear_vel = parent_->GetRelativeLinearVel();

        odom_transform_= odom_transform_ * this->getTransformForMotion(linear_vel.x, linear_vel.y, angular_vel.z, step_time);

        tf::poseTFToMsg(odom_transform_, odom_.pose.pose);
        odom_.twist.twist.angular.z = angular_vel.z;
        odom_.twist.twist.linear.x  = linear_vel.x;
      #endif

        if (transform_broadcaster_.get()){
//          transform_broadcaster_->sendTransform(tf::StampedTransform(odom_transform_, current_time, "odom", "world"));
            transform_broadcaster_->sendTransform(tf::StampedTransform(odom_transform_, current_time, "odom", robot_base_frame_));
        }
        
        odom_.pose.covariance[0] = 0.001;
        odom_.pose.covariance[7] = 0.001;
        odom_.pose.covariance[14] = 1000000000000.0;
        odom_.pose.covariance[21] = 1000000000000.0;
        odom_.pose.covariance[28] = 1000000000000.0;
         
      #if (GAZEBO_MAJOR_VERSION >= 8)
        if (std::abs(angular_vel.Z()) < 0.0001) {
      #else
        if (std::abs(angular_vel.z) < 0.0001) {
      #endif
          odom_.pose.covariance[35] = 0.01;
        }else{
          odom_.pose.covariance[35] = 100.0;
        }

        odom_.twist.covariance[0] = 0.001;
        odom_.twist.covariance[7] = 0.001;
        odom_.twist.covariance[14] = 0.001;
        odom_.twist.covariance[21] = 1000000000000.0;
        odom_.twist.covariance[28] = 1000000000000.0;

      #if (GAZEBO_MAJOR_VERSION >= 8)
        if (std::abs(angular_vel.Z()) < 0.0001) {
      #else
        if (std::abs(angular_vel.z) < 0.0001) {
      #endif
          odom_.twist.covariance[35] = 0.01;
        } else {
          odom_.twist.covariance[35] = 100.0;
        }
    } else {
      const auto pose = this->parent_->WorldPose();
      const auto linVel = this->parent_->WorldLinearVel();
      const auto angVel = this->parent_->WorldAngularVel();
      odom_.pose.pose.position.x = pose.Pos().X();
      odom_.pose.pose.position.y = pose.Pos().Y();
      odom_.pose.pose.position.z = pose.Pos().Z();
      odom_.pose.pose.orientation.x = pose.Rot().X();
      odom_.pose.pose.orientation.y = pose.Rot().Y();
      odom_.pose.pose.orientation.z = pose.Rot().Z();
      odom_.pose.pose.orientation.w = pose.Rot().W();

      odom_.twist.twist.linear.x = linVel.X();
      odom_.twist.twist.linear.y = linVel.Y();
      odom_.twist.twist.linear.z = linVel.Z();
      odom_.twist.twist.angular.x = angVel.X();
      odom_.twist.twist.angular.y = angVel.Y();
      odom_.twist.twist.angular.z = angVel.Z();

      tf::Transform tmp;
      tmp.setIdentity();
      tmp.setOrigin(tf::Vector3(odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z));
      tmp.setRotation(tf::Quaternion(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W()));

      if (transform_broadcaster_.get()){
//          transform_broadcaster_->sendTransform(tf::StampedTransform(tmp, current_time, "odom", "world"));
            transform_broadcaster_->sendTransform(tf::StampedTransform(tmp, current_time, "odom", robot_base_frame_));
      }
    }

    odom_.header.stamp = current_time;
    odom_.header.frame_id = odom_frame;
    odom_.child_frame_id = base_footprint_frame;
    odometry_pub_.publish(odom_);
  }


  tf::Transform GazeboRosForceBasedMove::getTransformForMotion(double linear_vel_x, double linear_vel_y, double angular_vel, double timeSeconds) const {
    // ROS_INFO(std::string("x_vel: ").append(std::to_string(linear_vel_x)).c_str());
    // ROS_INFO(std::string("y_vel: ").append(std::to_string(linear_vel_y)).c_str());
    // ROS_INFO(std::string("angular_vel: ").append(std::to_string(angular_vel)).c_str());

    tf::Transform tmp;
    tmp.setIdentity();

    if (std::abs(angular_vel) < 0.0001) {
     tmp.setOrigin(tf::Vector3(static_cast<double>(linear_vel_x*timeSeconds), static_cast<double>(linear_vel_y*timeSeconds), 0.0));
    } else {
      const double angleChange = angular_vel * timeSeconds;

      const double distChangeX = linear_vel_x * timeSeconds; 
      const double arcRadiusX = distChangeX / angleChange;
      const double distChangeY = linear_vel_y * timeSeconds;
      const double arcRadiusY = distChangeY / angleChange;

      tmp.setOrigin(tf::Vector3(std::sin(angleChange) * arcRadiusX + arcRadiusY - std::cos(angleChange) * arcRadiusY,
                                arcRadiusX - std::cos(angleChange) * arcRadiusX + std::sin(angleChange) * arcRadiusY,
                                0.0));
      tmp.setRotation(tf::createQuaternionFromYaw(angleChange));
    }
    return tmp;
  }

  GZ_REGISTER_MODEL_PLUGIN(GazeboRosForceBasedMove)
}

