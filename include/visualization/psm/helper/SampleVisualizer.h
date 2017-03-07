/**

Copyright (c) 2016, Gehrung Joachim, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

# pragma once

// Global includes
#include <map>
#include <vector>

// Package includes
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_datatypes.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Pose.h>

// Local includes3
#include "visualization/psm/helper/AbstractExtendedVisualizer.h"

namespace Visualization
{
  /**
   * Defines how many points should be skipped in visualization.
   */
  static const unsigned int VISUALIZATION_OFFSET = 1;
  
  /**
   * Visualizer class for point clounds representing samples.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class SampleVisualizer : public AbstractExtendedVisualizer
  {
  public:
    
    /**
     * Constructor.
     */
    SampleVisualizer();
    
    /**
     * Destructor.
     */
    ~SampleVisualizer();
    
    /**
     * Sets the absolute pose of the parent object.
     * 
     * @param pPose Absolute pose of the primary scene object.
     */
    void setParentPose(boost::shared_ptr<ResourcesForPsm::Pose> pPose);
    
    /**
     * Publishes a trajectory composed of the samples used for learning.
     *
     * @param pPublisher For publishing the visualization message.
     * @param pMarkerId Serves as unique id for the marker. Any marker sent with the same id will overwrite the old one.
     * @param pSamples The list of all samples.
     */
    void publishTrajectory(boost::shared_ptr<ros::Publisher> pPublisher,
			   unsigned int& pMarkerId,
			   const std::vector<Eigen::Vector3d>& pSamples);
    
    /**
     * Publishes a trajectory element along the given positions.
     *
     * @param pPublisher For publishing the visualization message.
     * @param pMarkerId Serves as unique id for the marker. Any marker sent with the same id will overwrite the old one.
     * @param pSamples The list of all samples.
     */
    void publishTrajectory(boost::shared_ptr<ros::Publisher> mPublisher,
			   unsigned int& pMarkerId,
			   const std::vector<Eigen::Vector3d>& pAbsoluteSample,
			   const std::vector<Eigen::Vector3d>& pAbsoluteParentSample);
  private:
    
    /**
     * Create a visualization marker message that contains the trajectory.
     * 
     * @param pMarkerId Serves as unique id for the marker. Any marker sent with the same id will overwrite the old one.
     * @param pSamples The list of all samples.
     * @return A marker array holding the visualization messages.
     */
    visualization_msgs::Marker generateTrajectoryMarker(unsigned int& pMarkerId, const std::vector<Eigen::Vector3d>& pSamples);
    
    /**
     * Generates an arrow between two points.
     * 
     * @param pMarkerId Serves as unique id for the marker. Any marker sent with the same id will overwrite the old one.
     * @param pFrom The position to start the link from.
     * @param pTo The position to end the link.
     */
    visualization_msgs::Marker generateArrowMessage(unsigned int& pMarkerId, Eigen::Vector3d pFrom, Eigen::Vector3d pTo);
    
  private:
    
    /**
     * Absolute pose of the parent object.
     */
    boost::shared_ptr<ResourcesForPsm::Pose> mParentPose;
  };
}
