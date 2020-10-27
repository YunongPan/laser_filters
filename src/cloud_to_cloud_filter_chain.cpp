//
// Created by yunong on 19.10.20.
//

/*
 * Copyright (c) 2008 Radu Bogdan Rusu <rusu@cs.tum.edu>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: scan_shadows_filter.cpp,v 1.0 2008/12/04 12:00:00 rusu Exp $
 *
 */

/*
\author Radu Bogdan Rusu <rusu@cs.tum.edu>


 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <float.h>



// TF
#include <tf/transform_listener.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

//Filters
#include "filters/filter_chain.h"

/** @b ScanShadowsFilter is a simple node that filters shadow points in a laser scan line and publishes the results in a cloud.
 */
class CloudToCloudFilterChain
{
public:
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

    filters::FilterChain<sensor_msgs::PointCloud2> cloud_filter_chain_;
    ros::Publisher cloud_pub_;

    std::string target_frame_;

    tf::TransformListener tf_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_;
    tf::MessageFilter<sensor_msgs::PointCloud2> filter_;



    ////////////////////////////////////////////////////////////////////////////////
    CloudToCloudFilterChain () : private_nh("~"), filter_(tf_, "", 50),
                                cloud_filter_chain_("sensor_msgs::PointCloud2")
    {
        private_nh.param("target_frame", target_frame_, std::string ("base_link"));



        filter_.setTargetFrame(target_frame_);
        filter_.registerCallback(boost::bind(&CloudToCloudFilterChain::scanCallback, this, _1));
        sub_.subscribe(nh, "pointcloud", 50);
        filter_.connectInput(sub_);

        //cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2> ("/cloud_filtered", 10);
        cloud_filter_chain_.configure("cloud_filter_chain", private_nh);
    }

    // We use a deprecation warning on a timer to avoid warnings getting lost in the noise



    ////////////////////////////////////////////////////////////////////////////////
    void
    scanCallback (const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
    {
        sensor_msgs::PointCloud2 filtered_cloud;
        cloud_filter_chain_.update (*cloud_msg, filtered_cloud);
        //cloud_pub_.publish(filtered_cloud);
    }

} ;



int
main (int argc, char** argv)
{
    ros::init (argc, argv, "cloud_to_cloud_filter_chain");
    ros::NodeHandle nh;

    CloudToCloudFilterChain f;

    ros::spin();

    return (0);
}
