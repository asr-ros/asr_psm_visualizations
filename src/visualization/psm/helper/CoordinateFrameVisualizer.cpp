/**

Copyright (c) 2016, Gehrung Joachim, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "visualization/psm/helper/CoordinateFrameVisualizer.h"

namespace Visualization {
  
  CoordinateFrameVisualizer::CoordinateFrameVisualizer()
  : AbstractVisualizer()
  {
  }
  
  CoordinateFrameVisualizer::~CoordinateFrameVisualizer()
  {
  }
  
  void CoordinateFrameVisualizer::publishFrame(boost::shared_ptr<ros::Publisher> pPublisher,
						 unsigned int& pMarkerId,
						 const boost::shared_ptr<ResourcesForPsm::Pose> pPose,
					         double pSize)
  {
    // Check, if publisher is available.
    if(!pPublisher)
      throw std::invalid_argument("Empty ros::publisher cannot be used to publish marker messages.");
    
    // If position pointer.
    if(!pPose)
       throw std::invalid_argument("Invalid pointer: pose of coordinate frame.");
    
    // Container for all messages
    visualization_msgs::MarkerArray msg;
    
    // Generate all three axis of the frame.
    for(unsigned int i = 0; i < 3; i++)
      msg.markers.push_back(generateAxis(pMarkerId++, i, pPose, pSize));
    
    // Publish the markers.
    pPublisher->publish(msg);
  }
    
  visualization_msgs::Marker CoordinateFrameVisualizer::generateAxis(const unsigned int pMarkerId,
								      const unsigned int pAxis,
								      const boost::shared_ptr<ResourcesForPsm::Pose> pPose,
								      double pSize)
  {
    visualization_msgs::Marker msg;
    
    // Use the learners frame_id relative to which all locations in this model are given.
    msg.header.frame_id = getFrameId();
    
    // Set timestamp. See the TF tutorials for information on this.
    msg.header.stamp = ros::Time::now();
    
    // Marker namespace are associated to different types of shapes used to represent object location distributions.
    msg.ns = getNamespace();
    
    // Markers with different namespaces but same id represent same object location distribution with different shapes.
    msg.id = pMarkerId;
    
    // The marker type decides on shape of marker and accordingly how marker parameters are interpreted by RViz.
    // Here we choose a sphere.
    msg.type = visualization_msgs::Marker::ARROW;
    
    // We always want to show a marker by publishing ros messages here.
    msg.action = visualization_msgs::Marker::ADD;
    
    // Arrows should be pretty small. Don't set z here, because we are using start and endpoint here.
    msg.scale.x = msg.scale.y = 0.01 * pSize;
    
    // Create the starting point.
    geometry_msgs::Point start;
    start.x = 0;
    start.y = 0;
    start.z = 0;
    msg.points.push_back(start);
    
    // Create the point to point to ;).
    double length = 0.05 * pSize;
    geometry_msgs::Point stop;
    stop.x = ((pAxis     == 0) ? 1.0 : 0.0) * length;
    stop.y = ((pAxis - 1 == 0) ? 1.0 : 0.0) * length;
    stop.z = ((pAxis - 2 == 0) ? 1.0 : 0.0) * length;
    msg.points.push_back(stop);
    
    // Set position and orientation.
    Eigen::Vector3d position = pPose->getPosition();
    msg.pose.position.x = position[0];
    msg.pose.position.y = position[1];
    msg.pose.position.z = position[2];
    
    Eigen::Quaternion<double> orientation = pPose->getOrientation();
    msg.pose.orientation.w = orientation.w();
    msg.pose.orientation.x = orientation.x();
    msg.pose.orientation.y = orientation.y();
    msg.pose.orientation.z = orientation.z();
    
    // The color depends on the axis number. The color codes RGB stand for the axis XYZ.
    msg.color.r = (pAxis     == 0) ? 1.0 : 0.0;
    msg.color.g = (pAxis - 1 == 0) ? 1.0 : 0.0;
    msg.color.b = (pAxis - 2 == 0) ? 1.0 : 0.0;
    msg.color.a = 1.0;

    // Return the message.
    return msg;
  }
  
}
