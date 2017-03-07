/**

Copyright (c) 2016, Gehrung Joachim, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <vector>

#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <tf/transform_listener.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace Visualization {
  static const int ID_GAUSSIAN = 10;

  class BayesVisualization {
  private:
    ros::Publisher gaussianPublisher;
    ros::Publisher samplePublisher;
    ros::Publisher eigenPublisher;
		
  public:
    BayesVisualization();
    ~BayesVisualization();

    // visualizes a point cloud
    void drawSamples(std::vector<Eigen::Vector3d> samples);

    // draws a 3D gaussian distribution with given mean and covariance
    void drawDistribution(Eigen::Vector3d mean, Eigen::Matrix3d covariance);

    // draws a 3D gaussian distribution over the given points
    void drawDistribution(std::vector<Eigen::Vector3d> samples);

    // draws an arrow between two points
    visualization_msgs::Marker createArrowMarker(const Eigen::Vector3d a, const Eigen::Vector3d b);

    // creates a geometry message for the given point
    geometry_msgs::Point createPointMessage(Eigen::Vector3d point);
  };
}
