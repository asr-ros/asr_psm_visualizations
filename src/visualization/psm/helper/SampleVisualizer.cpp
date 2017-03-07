/**

Copyright (c) 2016, Gehrung Joachim, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "visualization/psm/helper/SampleVisualizer.h"

namespace Visualization {
  
  SampleVisualizer::SampleVisualizer()
  : AbstractExtendedVisualizer()
  {
  }
  
  SampleVisualizer::~SampleVisualizer()
  {
  }
  
  void SampleVisualizer::setParentPose(boost::shared_ptr<ResourcesForPsm::Pose> pPose)
  {
    // Check, if pointer is valid.
    if(!pPose)
      throw std::out_of_range("Invalid pointer: absolute pose of primary scene object.");
    
    // Set absolute position of the primary scene object.
    mParentPose = pPose;
  }
  
  void SampleVisualizer::publishTrajectory(boost::shared_ptr<ros::Publisher> pPublisher,
					   unsigned int& pMarkerId,
					   const std::vector<Eigen::Vector3d>& pSamples)
  {
    // Check, if publisher is available.
    if(!pPublisher)
      throw std::invalid_argument("Empty ros::publisher cannot be used to publish marker messages.");
    
    // ONE MESSAGE TO RULE THEM ALL! Well, kind of.
    visualization_msgs::MarkerArray array;
    
    // Generate trajectory marker message.
    array.markers.push_back(generateTrajectoryMarker(pMarkerId, pSamples));
    
    // Add also an arrow pointing from the parent position to every sample.
    // Draw only every n-th arrow.
//     for(unsigned int i = 0; i < pSamples.size(); i += VISUALIZATION_OFFSET)
//       array.markers.push_back(generateArrowMessage(pMarkerId, mParentPose->getPosition(), pSamples[i]));
    
    // Publish the marker array.
    pPublisher->publish(array);
  }
  
  void SampleVisualizer::publishTrajectory(boost::shared_ptr<ros::Publisher> pPublisher,
					   unsigned int& pMarkerId,
					   const std::vector<Eigen::Vector3d>& pAbsoluteSample,
					   const std::vector<Eigen::Vector3d>& pAbsoluteParentSample)
  {
    // Check, if publisher is available.
    if(!pPublisher)
      throw std::invalid_argument("Empty ros::publisher cannot be used to publish marker messages.");
    
    // Check, if both trajectories are equal in length.
    if(pAbsoluteSample.size() != pAbsoluteParentSample.size())
      throw std::invalid_argument("Unequal number of samples and corresponding parent poses.");
    
    // ONE MESSAGE TO RULE THEM ALL! Well, kind of.
    visualization_msgs::MarkerArray array;
    
    // Visualize the trajectory as a line strip.
    array.markers.push_back(generateTrajectoryMarker(pMarkerId, pAbsoluteSample));
    
    // Draw every support point of the trajectory as arrow.
    for(unsigned int i = 0; i < pAbsoluteSample.size(); i += VISUALIZATION_OFFSET)
      array.markers.push_back(generateArrowMessage(pMarkerId, pAbsoluteParentSample[i], pAbsoluteSample[i]));
    
    // Publish the marker array.
    pPublisher->publish(array);
  }
  
  visualization_msgs::Marker SampleVisualizer::generateTrajectoryMarker(unsigned int& pMarkerId, const std::vector<Eigen::Vector3d>& pSamples)
  {
    visualization_msgs::Marker msg;

    // Set time and frame id.
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = getFrameId();

    // Namespace will represent the nature of the message.
    msg.ns = getNamespace();
    
    // Every sample has its own id.
    msg.id = pMarkerId++;

    // Point is represented by a sphere.
    msg.type = visualization_msgs::Marker::SPHERE_LIST;
    
    // We always want to show a marker by publishing ros messages here.
    msg.action = visualization_msgs::Marker::ADD;
    
    // Iterate over all samples and add them to the visualization marker.
    for(unsigned int i = 0; i < pSamples.size(); i += VISUALIZATION_OFFSET)
    {
      Eigen::Vector3d sample = (mParentPose->getPosition() + mParentPose->getOrientation().toRotationMatrix() * pSamples[i]) * getScaleFactor();
      
      geometry_msgs::Point to;
      to.x = sample[0];
      to.y = sample[1];
      to.z = sample[2];
      msg.points.push_back(to);
    }
    
    // Shall be very small, but must be scaled, too.
    msg.scale.x = msg.scale.y = msg.scale.z = 0.005 * getScaleFactor();
    
    // Samples should be pitch black :).
    msg.color.r = msg.color.g = msg.color.b = 0.0;
    msg.color.a = 0.1;
    
    // Return the message containing all samples.
    return msg;
  }
  
  visualization_msgs::Marker SampleVisualizer::generateArrowMessage(unsigned int& pMarkerId, Eigen::Vector3d pFrom, Eigen::Vector3d pTo)
  {
    visualization_msgs::Marker msg;
    
    // Set time and frame id.
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = getFrameId();
    
    // Scale it to be very small and long.
    msg.scale.x = 0.002;
    msg.scale.y = 0.008;
    msg.scale.z = 0.008;
    
    // This should be an arrow.
    msg.type = visualization_msgs::Marker::ARROW;
    
    // Namespace will represent the nature of the message.
    // Yeah, it's really hard to find comments for every line...
    msg.ns = getNamespace();
    
    // Markers with different namespaces but same id represent same object location distribution with different shapes.
    msg.id = pMarkerId++;
    
    // We always want to show a marker by publishing ros messages here.
    msg.action = visualization_msgs::Marker::ADD;
    
    // Create the starting point.
    geometry_msgs::Point start;
    start.x = pFrom[0] * getScaleFactor();
    start.y = pFrom[1] * getScaleFactor();
    start.z = pFrom[2] * getScaleFactor();
    msg.points.push_back(start);
    
    // Create the end point.
    geometry_msgs::Point stop;
    stop.x = pTo[0] * getScaleFactor();
    stop.y = pTo[1] * getScaleFactor();
    stop.z = pTo[2] * getScaleFactor();
    msg.points.push_back(stop);

    // Take the second color, it stands for the secondary scene object.
    msg.color.r = 0.6;
    msg.color.g = 0.6;
    msg.color.b = 0.6;
    msg.color.a = 1.0;
    
    // Return the link message message.
    return msg;
  }
  
}
