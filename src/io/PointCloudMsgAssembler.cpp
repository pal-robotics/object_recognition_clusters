/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, PAL Robotics, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

 /* Author: Bence Magyar
  * Email:  bence.magyar@pal-robotics.com
  */

#include <vector>
#include <boost/foreach.hpp>

#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>

// ROS includes
#include <sensor_msgs/Image.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <pcl_conversions/pcl_conversions.h>

namespace object_recognition_clusters
{
  /** Cell that takes the output of the tabletop pipeline and publishes the clusters nested in a recognizedobjectarray for later fill-up.
  */
  struct PointCloudMsgAssembler {
    static void
    declare_params(ecto::tendrils& params) {}

    static void
    declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs) {
      inputs.declare(&PointCloudMsgAssembler::clusters3d_,
                     "clusters3d", "The 3dpoints as a cv::Mat_<cv::Vec3f>").required(true);
      inputs.declare(&PointCloudMsgAssembler::image_message_,
                     "image_message", "the image message to get the header").required(true);

      outputs.declare(&PointCloudMsgAssembler::output_msg_, "msg", "Pointcloud ROS message");
    }

    void
    configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs) {
    }

    int
    process(const ecto::tendrils& inputs, const ecto::tendrils& outputs) {
      // Publish the info
      ros::Time time = ros::Time::now();
      object_recognition_msgs::RecognizedObjectArrayPtr msg(new object_recognition_msgs::RecognizedObjectArray());

      std::string frame_id;
      if (*image_message_) {
        frame_id = (*image_message_)->header.frame_id;
        time = (*image_message_)->header.stamp;
      }
      //TODO: figure out what to do if image doesn't give enough info
      //      else
      //        if (!frame_id_->empty())
      //          frame_id = *frame_id_;

      msg->header.stamp = time;

      for (size_t i = 0; i < clusters3d_->size(); ++i){
        // for every table
        std::vector<std::vector<cv::Vec3f> > &clusters = (*clusters3d_)[i];
        for (size_t j = 0; j < clusters.size(); j++){
          // for every cluster
          object_recognition_msgs::RecognizedObject object;

          // Deal with the header
          object.header.stamp = time;

          pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
          pcl_cloud.reserve(clusters[j].size());
          BOOST_FOREACH(const cv::Vec3f& point, clusters[j])
          {
            pcl_cloud.push_back(pcl::PointXYZ(point[0], point[1], point[2]));
          }

          sensor_msgs::PointCloud2 cloud;
          pcl::toROSMsg(pcl_cloud, cloud);
          cloud.header.frame_id = frame_id;
          cloud.header.stamp = time;

          object.point_clouds.push_back(cloud);
          msg->objects.push_back(object);
        }
      }

      *output_msg_ = msg;
      return ecto::OK;
    }
  private:
    /** The image message the initial data is from */
    ecto::spore<sensor_msgs::ImageConstPtr> image_message_;
    /** For each table, a vector of clusters */
    ecto::spore<std::vector<std::vector<std::vector<cv::Vec3f> > > > clusters3d_;
    /** Pointcloud with the header of the image message */
    ecto::spore<object_recognition_msgs::RecognizedObjectArrayConstPtr> output_msg_;
  };
}

ECTO_CELL(io_clusters, object_recognition_clusters::PointCloudMsgAssembler, "PointCloudMsgAssembler",
          "Given object ids and poses, fill the object recognition message.");
