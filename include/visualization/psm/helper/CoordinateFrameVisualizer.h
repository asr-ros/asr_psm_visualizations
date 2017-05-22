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
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <Pose.h>
#include <ISM/common_type/Pose.hpp>

// Local includes
#include "visualization/psm/helper/AbstractVisualizer.h"

namespace Visualization
{
  /**
   * Visualizer class for primary scenes objects.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class CoordinateFrameVisualizer : public AbstractVisualizer
  {
  public:
    
    /**
     * Constructor.
     */
    CoordinateFrameVisualizer();
    
    /**
     * Destructor.
     */
    ~CoordinateFrameVisualizer();
    
    /**
     * Publishes a coordinate with the given position and orientation.
     *
     * @param pPublisher For publishing the visualization message.
     * @param pMarkerId Serves as unique id for the marker. Any marker sent with the same id will overwrite the old one.
     * @param pPose The pose of the frame.
     * @param pSize A factor describing the size of the frame.
     */
    void publishFrame(boost::shared_ptr<ros::Publisher> pPublisher,
		      unsigned int& pMarkerId,
              const boost::shared_ptr<ISM::Pose> pPose,
		      double pSize);
    
  private:
    
    /**
     * Generates the visualization message that contains the coordinate system base vectors of the covariance ellipse.
     * 
     * @param pMarkerId An unique id for the marker. Any marker sent with the same id will overwrite the old one.
     * @param pAxis The number of the axis that the message should be created for.
     * @param pPose The last known pose of the associated object.
     * @param pSize A factor describing the size of the frame.
     * @return A marker array holding the visualization messages.
     */
    visualization_msgs::Marker generateAxis(const unsigned int pMarkerId,
					     const unsigned int pAxis,
                         const boost::shared_ptr<ISM::Pose> pPose,
					     double pSize);
  };
}
