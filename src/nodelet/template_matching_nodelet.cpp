// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) Masaya Kataoka
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
 *   * Neither the name of the JSK Lab nor the names of its
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
 *********************************************************************/

// https://github.com/opencv/opencv/tree/2.4/samples/cpp/tutorial_code/ImgProc/AddingImages.cpp
/**
 * This is a demo of adding image (linear blending).
 */

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <dynamic_reconfigure/server.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "opencv_apps/TemplateMatchingConfig.h"
#include "opencv_apps/nodelet.h"

namespace opencv_apps {
  class TemplateMatchingNodelet : public opencv_apps::Nodelet {
    public:
      TemplateMatchingNodelet(){};
      ~TemplateMatchingNodelet(){};
      virtual void onInit() {
        Nodelet::onInit();
        onInitPostProcess();
      };
      void unsubscribe(){};
      void subscribe(){};
  };
}

namespace template_matching{
  class TemplateMatchingNodelet : public opencv_apps::TemplateMatchingNodelet{
    public:
      TemplateMatchingNodelet(){};
      ~TemplateMatchingNodelet(){};
      virtual void onInit() {
        ROS_WARN("DeprecationWarning: Nodelet template_matching/template_matching is deprecated, "
        "and renamed to opencv_apps/template_matching.");
        opencv_apps::TemplateMatchingNodelet::onInit();
      };
  };
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(opencv_apps::TemplateMatchingNodelet, nodelet::Nodelet);
//PLUGINLIB_EXPORT_CLASS(template_matching::TemplateMatchingNodelet, nodelet::Nodelet);