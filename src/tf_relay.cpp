// Standaerd C++ Library
#include <iostream>
// ROS
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
// ROS message
#include <geometry_msgs/TransformStamped.h>
// Boost
#include <boost/bind.hpp>
// USER
#include "tf_relay/tf_relay.h"

namespace tf_relay {

    TFRelay::TFRelay( ros::NodeHandle &nh,
                      ros::NodeHandle &nh_private,
                      tf2_ros::Buffer &tf_buffer,
                      tf2_ros::TransformBroadcaster &tf_broadcaster )
        : nh_(nh), nh_private_(nh_private), tf_buffer_(tf_buffer), tf_broadcaster_(tf_broadcaster)
    {
        this->reference_frame_id_ = "reference_frame";
        if ( nh_private.hasParam("reference_frame_id") ) {
            nh_private.getParam("reference_frame_id", this->reference_frame_id_);
        }

        this->output_frame_id_ = "output_frame";
        if ( nh_private.hasParam("output_frame_id") ) {
            nh_private.getParam("output_frame_id", this->output_frame_id_);
        }

        this->fixed_frame_id_ = "fixed_frame";
        if ( nh_private.hasParam("fixed_frame_id") ) {
            nh_private.getParam("fixed_frame_id", this->fixed_frame_id_);
        }

        double timeout = 0.05;
        if ( this->nh_private_.hasParam("timeout_duration") ) {
            this->nh_private_.getParam("timeout_duration", timeout);
        }
        this->duration_timeout_ = ros::Duration(timeout);

        /**
         *
         */
        this->initialized_ = false;

        ROS_INFO("Initialization finished.");
    }

    void TFRelay::spin()
    {
        double timer_duration = 0.1;
        if ( this->nh_private_.hasParam("timer_duration") ) {
            this->nh_private_.getParam("timer_duration", timer_duration);
        }

        // start broadcaster timer
        ros::Timer timer_tf = this->nh_.createTimer<TFRelay>( 
                                ros::Duration(timer_duration),
                                &TFRelay::callbackTimerTF,
                                this
                            );

        //
        ros::spin();
    }

    void TFRelay::callbackTimerTF( const ros::TimerEvent& )
    {
        try {
            geometry_msgs::TransformStamped temp_transform =
                this->tf_buffer_.lookupTransform(
                        this->fixed_frame_id_,
                        this->reference_frame_id_,
                        ros::Time(0),
                        this->duration_timeout_
                        );
            this->msg_transform_ = temp_transform;
            this->msg_transform_.child_frame_id = this->output_frame_id_;
            if ( not this->initialized_ ) {
                this->initialized_ = true;
            }
        } catch (tf2::TransformException &ex) {
            ROS_DEBUG("%s",ex.what());
        }
        if ( this->initialized_ ) {
            this->msg_transform_.header.stamp = ros::Time::now();
            this->tf_broadcaster_.sendTransform(this->msg_transform_);
        }
        ROS_INFO("callback called.");
    }

}
