/**

Copyright (c) 2016, Gehrung Joachim, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "visualization/psm/helper/KinematicChainVisualizer.h"

namespace Visualization {
  
  KinematicChainVisualizer::KinematicChainVisualizer()
  : AbstractExtendedVisualizer()
  {
  }
  
  KinematicChainVisualizer::~KinematicChainVisualizer()
  {
  }
  
  void KinematicChainVisualizer::publishObjectPositionAsArrow(boost::shared_ptr<ros::Publisher> pPublisher,
							      unsigned int& pMarkerId,
							      const boost::shared_ptr<ResourcesForPsm::Pose> pPose,
							      double pQualityOfMatch)
  {
    // Check, if publisher is available.
    if(!pPublisher)
      throw std::invalid_argument("Empty ros::publisher cannot be used to publish marker messages.");
    
    visualization_msgs::MarkerArray array;

    // Generate the arrow message that symbolizes the link.
    array.markers.push_back(generatePerpendicularArrowMessage(pMarkerId, pPose, pQualityOfMatch, 0.2));
    
    // Publish the markers.
    pPublisher->publish(array);
  }
  
  void KinematicChainVisualizer::publishObjectPositionAsPoint(boost::shared_ptr<ros::Publisher> pPublisher,
							      unsigned int& pMarkerId,
							      const boost::shared_ptr<ResourcesForPsm::Pose> pPose)
  {
    // Check, if publisher is available.
    if(!pPublisher)
      throw std::invalid_argument("Empty ros::publisher cannot be used to publish marker messages.");
    
    visualization_msgs::MarkerArray array;
    
    // Generate the point message that symbolizes the object position.
    array.markers.push_back(generatePointMessage(pMarkerId, pPose->getPosition()));
    
    // Publish the markers.
    pPublisher->publish(array);
  }
  
  void KinematicChainVisualizer::publishObjectPositionAsPointWithScore(boost::shared_ptr<ros::Publisher> pPublisher,
								       unsigned int& pMarkerId,
								       const boost::shared_ptr<ResourcesForPsm::Pose> pPose,
								       std::vector<double> pScores,
								       double pQualityOfMatch)
  {
    // Check, if publisher is available.
    if(!pPublisher)
      throw std::invalid_argument("Empty ros::publisher cannot be used to publish marker messages.");
    
    visualization_msgs::MarkerArray array;
    
    double arrowLength = 0.2;
    
    // Generate the arrow message that points perpendicular to the object position.
    array.markers.push_back(generatePerpendicularArrowMessage(pMarkerId, pPose, pQualityOfMatch, arrowLength));
    
    // Generate the term markers.
    for(unsigned int i = 0; i < pScores.size(); i++)
      array.markers.push_back(generateTermIndicatorMessage(pMarkerId, pPose, i * (arrowLength / (pScores.size() * 2.0)), pScores[i]));
    
    // Generate the point message that symbolizes the object position.
    array.markers.push_back(generatePointMessage(pMarkerId, pPose->getPosition()));
    
    // Publish the markers.
    pPublisher->publish(array);
  }
  
  void KinematicChainVisualizer::publishLink(boost::shared_ptr<ros::Publisher> pPublisher,
					     unsigned int& pMarkerId,
					     const Eigen::Vector3d pFrom,
					     const Eigen::Vector3d pTo,
					     double pQualityOfMatch)
  {
    // Check, if publisher is available.
    if(!pPublisher)
      throw std::invalid_argument("Empty ros::publisher cannot be used to publish marker messages.");
    
    visualization_msgs::MarkerArray array;
    
    // Generate the arrow message that symbolizes the link.
    array.markers.push_back(generateArrowMessage(pMarkerId, pFrom, pTo, pQualityOfMatch, 0.04, 0.1, 0.10));
    
    // Publish the markers.
    pPublisher->publish(array);
  }
  
  void KinematicChainVisualizer::setBestStatus(bool pStatus)
  {
    mBestStatus = pStatus;
  }
  
  visualization_msgs::Marker KinematicChainVisualizer::generatePointMessage(unsigned int& pMarkerId,
									    const Eigen::Vector3d pPosition)
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
    msg.type = visualization_msgs::Marker::SPHERE;
    
    // We always want to show a marker by publishing ros messages here.
    msg.action = visualization_msgs::Marker::ADD;

    // Draw the sphere at the given position.
    msg.pose.position.x = pPosition[0] * getScaleFactor();
    msg.pose.position.y = pPosition[1] * getScaleFactor();
    msg.pose.position.z = pPosition[2] * getScaleFactor();
    
    // orientation is irrelevant.getScaleFactor
    msg.pose.orientation.w = 1;
    msg.pose.orientation.x = 0;
    msg.pose.orientation.y = 0;
    msg.pose.orientation.z = 0;
    
    // Spheres shall be very small, but must be scaled, too.
    msg.scale.x = msg.scale.y = msg.scale.z = 0.05 * getScaleFactor();

    // Make it gray like the ISM visualization.
    msg.color.r = 0.5;
    msg.color.g = 0.5;
    msg.color.b = 0.5;
    msg.color.a = 0.7;
    
    // Return the point message.
    return msg;
  }
  
  visualization_msgs::Marker KinematicChainVisualizer::generateArrowMessage(unsigned int& pMarkerId,
									    const Eigen::Vector3d pFrom,
									    const Eigen::Vector3d pTo,
									    double pQualityOfMatch,
									    double pScalePeak,
									    double pScaleShaft,
									    double pPeakLength)
  {
    visualization_msgs::Marker msg;
    
    // Set time and frame id.
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = getFrameId();
    
    // Scale it to be very small and long.
    msg.scale.x = pScalePeak;
    msg.scale.y = pScaleShaft;
    msg.scale.z = pPeakLength;
    
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

    // The color depends on the certainty with that the object was detected.
    // High certainty is green, high uncertainty is red!
    msg.color.r = 1.0 - pQualityOfMatch;
    msg.color.g = pQualityOfMatch;
    msg.color.b = 0.0;
    msg.color.a = 1.0;
    
    // Return the link message message.
    return msg;
  }
  
  visualization_msgs::Marker KinematicChainVisualizer::generatePerpendicularArrowMessage(unsigned int& pMarkerId,
											 const boost::shared_ptr<ResourcesForPsm::Pose> pPose,
											 double pQualityOfMatch,
											 double pLength)
  {
    // This is the point where the arrow starts.
    Eigen::Vector3d from = pPose->getPosition();
    Eigen::Vector3d to = pPose->getPosition();
    
    // The arrow should point along the z-axis.
    from[2] -= 0.1 + pLength;
    to[2] -= 0.1;
    
    // Generate the arrow message that symbolizes the link.
    visualization_msgs::Marker msg = generateArrowMessage(pMarkerId, from, to, pQualityOfMatch, 0.055, 0.12, 0.05);
    
    // If it's not the the marker for the best scene object, make it gray.
    if(!mBestStatus)
    {
      msg.color.r = msg.color.g = msg.color.b = 0.7;
      msg.color.a = 0.0;
    }
    return msg;
  }
  
  visualization_msgs::Marker KinematicChainVisualizer::generateTermIndicatorMessage(unsigned int& pMarkerId,
										    const boost::shared_ptr<ResourcesForPsm::Pose> pPose,
										    double pOffset,
										    double pScore)
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
    msg.type = visualization_msgs::Marker::SPHERE;
    
    // We always want to show a marker by publishing ros messages here.
    msg.action = visualization_msgs::Marker::ADD;
    
    // Draw the sphere at the given position.
    msg.pose.position.x = pPose->getPosition()[0] * getScaleFactor();
    msg.pose.position.y = pPose->getPosition()[1] * getScaleFactor();
    msg.pose.position.z = pPose->getPosition()[2] * getScaleFactor() - (0.2 + pOffset);
    
    // orientation is irrelevant.getScaleFactor
    msg.pose.orientation.w = 1;
    msg.pose.orientation.x = 0;
    msg.pose.orientation.y = 0;
    msg.pose.orientation.z = 0;
    
    // Spheres shall be very small, but must be scaled, too.
    msg.scale.x = msg.scale.y = 0.05 * getScaleFactor();
    msg.scale.z = 0.001 * getScaleFactor();

    // If it's not the the marker for the best scene object, make it gray.
    if(mBestStatus)
    {
      msg.color.r = 1.0 - pScore;
      msg.color.g = pScore;
      msg.color.b = 0.0;
      msg.color.a = 1.0;
    } else {
      msg.color.r = msg.color.g = msg.color.b = 0.7;
      msg.color.a = 0.0;
    }
    
    // Return the point message.
    return msg;
  }
  
}
