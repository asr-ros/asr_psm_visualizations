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

#include <Pose.h>

// Local includes
#include "visualization/psm/helper/AbstractExtendedVisualizer.h"

namespace Visualization
{
  /**
   * Visualizer class for the best recognition result.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class KinematicChainVisualizer : public AbstractExtendedVisualizer
  {
  public:
    
    /**
     * Constructor.
     */
    KinematicChainVisualizer();
    
    /**
     * Destructor.
     */
    ~KinematicChainVisualizer();
    
    /**
     * Publishes an arrow that points from top to bottom right onto the given position.
     *
     * @param pPublisher For publishing the visualization message.
     * @param pMarkerId Serves as unique id for the marker. Any marker sent with the same id will overwrite the old one.
     * @param pPose The pose of the object.
     * @param pQualityOfMatch Indicator for the quality of match between the objects shall pose and is pose.
     */
    void publishObjectPositionAsArrow(boost::shared_ptr<ros::Publisher> pPublisher,
				      unsigned int& pMarkerId,
				      const boost::shared_ptr<ResourcesForPsm::Pose> pPose,
				      double pQualityOfMatch);
    
    /**
     * Publishes a point right at the given position.
     *
     * @param pPublisher For publishing the visualization message.
     * @param pMarkerId Serves as unique id for the marker. Any marker sent with the same id will overwrite the old one.
     * @param pPose The pose of the object.
     */
    void publishObjectPositionAsPoint(boost::shared_ptr<ros::Publisher> pPublisher,
				      unsigned int& pMarkerId,
				      const boost::shared_ptr<ResourcesForPsm::Pose> pPose);
    
    /**
     * Publishes a point right at the given position and an arrow standing for the score.
     *
     * @param pPublisher For publishing the visualization message.
     * @param pMarkerId Serves as unique id for the marker. Any marker sent with the same id will overwrite the old one.
     * @param pPose The pose of the object.
     * @param pScores A vector holding the scores of the single terms.
     * @param pQualityOfMatch Indicator for the quality of match between the objects shall pose and is pose.
     */
    void publishObjectPositionAsPointWithScore(boost::shared_ptr<ros::Publisher> pPublisher,
					       unsigned int& pMarkerId,
					       const boost::shared_ptr<ResourcesForPsm::Pose> pPose,
					       std::vector<double> pScores,
					       double pQualityOfMatch);
    
    /**
     * Publishes a link between two objects. The color is determined by the pQualityOfMatch parameter where green is a perfect match and red no match at all.
     *
     * @param pPublisher For publishing the visualization message.
     * @param pMarkerId Serves as unique id for the marker. Any marker sent with the same id will overwrite the old one.
     * @param pFrom The position to start the link from.
     * @param pTo The position to end the link.
     * @param pQualityOfMatch Indicator for the quality of match between the objects shall pose and is pose.
     */
    void publishLink(boost::shared_ptr<ros::Publisher> pPublisher,
		     unsigned int& pMarkerId,
		     const Eigen::Vector3d pFrom,
		     const Eigen::Vector3d pTo,
		     double pQualityOfMatch);
    
    /**
     * Marks the scene object with the best score.
     * 
     * @param pStatus True, to select the scene object as the one with the best score.
     */
    void setBestStatus(bool pStatus);
    
  private:
    
    /**
     * Generates the visualization message that contains a point right at the given position.
     * 
     * @param pMarkerId Serves as unique id for the marker. Any marker sent with the same id will overwrite the old one.
     * @param pPosition The position of the object.
     * @return A marker array holding the visualization messages.
     */
    visualization_msgs::Marker generatePointMessage(unsigned int& pMarkerId,
						    const Eigen::Vector3d pPosition);
    
    /**
     * Generates an arrow between two points. The color is determined by the pQualityOfMatch parameter where green is a perfect match and red no match at all.
     * 
     * @param pMarkerId Serves as unique id for the marker. Any marker sent with the same id will overwrite the old one.
     * @param pFrom The position to start the link from.
     * @param pTo The position to end the link.
     * @param pQualityOfMatch Indicator for the quality of match between the objects shall pose and is pose.
     * @param pScalePeak The scale of the peak part of the arrow.
     * @param pScaleShaft The scale of the shaft.
     * @param pPeakLength The length of the peak.
     */
    visualization_msgs::Marker generateArrowMessage(unsigned int& pMarkerId,
						    const Eigen::Vector3d pFrom,
						    const Eigen::Vector3d pTo,
						    double pQualityOfMatch,
						    double pScalePeak,
						    double pScaleShaft,
						    double pPeakLength);
    
    /**
     * Generates an arrow that points from top to bottom right onto the given position. The color is determined by the pQualityOfMatch parameter where green is a perfect match and red no match at all.
     * 
     * @param pMarkerId Serves as unique id for the marker. Any marker sent with the same id will overwrite the old one.
     * @param pPose The point to point to.
     * @param pQualityOfMatch Indicator for the quality of match between the objects shall pose and is pose.
     * @param pLength The length of the arrow.
     */
    visualization_msgs::Marker generatePerpendicularArrowMessage(unsigned int& pMarkerId,
								 const boost::shared_ptr<ResourcesForPsm::Pose> pPose,
								 double pQualityOfMatch, double pLength);
    
    /**
     * Generates a vertical chain of spheres that are colored depending on the given term scores.
     * 
     * @param pMarkerId Serves as unique id for the marker. Any marker sent with the same id will overwrite the old one.
     * @param pPose The point to start the chain from.
     * @param pOffset The vertical offset to the given position.
     * @param pScores A vector holding the scores of the single terms.
     * @param pLength The length of the chain.
     */
    visualization_msgs::Marker generateTermIndicatorMessage(unsigned int& pMarkerId,
							    const boost::shared_ptr<ResourcesForPsm::Pose> pPose,
							    double pOffset,
							    double pScore);
    
  private:
    
    /**
     * True, if the corresponding scene object is the best one.
     */
    bool mBestStatus;
  };
}
