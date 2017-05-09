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

#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <Pose.h>
#include <ISM/common_type/Pose.hpp>

// Local includes
#include "visualization/psm/ProbabilisticSecondarySceneObjectVisualization.h"

#include "visualization/psm/helper/CoordinateFrameVisualizer.h"
#include "visualization/psm/helper/KinematicChainVisualizer.h"
#include "visualization/psm/helper/SampleVisualizer.h"

#include "helper/ColorHelper.h"

namespace Visualization
{
  /**
   * Visualizer class for primary scenes objects.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class ProbabilisticPrimarySceneObjectVisualization
  {
  public:
    
    /**
     * Constructor.
     * 
     * @param pName The name of this primary scene object.
     */
    ProbabilisticPrimarySceneObjectVisualization(std::string pName);
    
    /**
     * Destructor.
     */
    ~ProbabilisticPrimarySceneObjectVisualization();
    
    /**
     * Appends a scene object visualizer.
     * 
     * @param pVisualizer The scene object visualizer to append.
     */
    void appendVisualizer(boost::shared_ptr<ProbabilisticSecondarySceneObjectVisualization> pVisualizer);
    
    /**
     * Sets the parameter requried for visualizing the secondary scene object.
     * 
     * @param pScale Factor to multiply the kernel with.
     * @param pSigmaMultiplicator Scaling factor for the size of the visualized covariance ellipsoid.
     * @param pFrameId The name of the coordinate frame that should be drawn into.
     */
    void setDrawingParameters(const double pScale, const float pSigmaMultiplicator, const std::string pFrameId);
    
    /**
     * Sets the absolute pose of the primary object associated with this visualizer.
     * 
     * @param pPose Absolute pose of the primary scene object.
     */
    void setPose(boost::shared_ptr<ISM::Pose> pPose);
    
    /**
     * The best absolute pose of the primary object. It was determined by the best scoring hypothesis regarding the assignment of evidence to the slots.
     * 
     * @param pPose Best absolute pose of the primary scene object.
     */
    void setBestPoseCandidate(boost::shared_ptr<ISM::Pose> pPose);
    
    /**
     * Sets the list of samples used for learning the gaussian mixture distribution.
     * 
     * @param pSamples The list of samples used for learning the gaussian mixture distribution.
     */
    void setLearningSamples(const std::vector<Eigen::Vector3d>& pSamples);
    
    /**
     * Draws the probability distribuion indicating the position of the other scene objects. One ring color implies the primary scene object, the other the secondary scene object.
     * 
     * @param pScene The name of the scene. Required for creating un unique frame id.
     * @param pColor The color indicating the scene object.
     */
    void drawInTargetingMode(const std::string pScene, const std_msgs::ColorRGBA& pColor);
    
    /**
     * Draws the kinematic chain for the best scoring hypothesis regarding the assignment of evidence to the slots. A chain for every primary scene object for every scene will be visualized in a separate topic. One ring color implies the primary scene object, the other the secondary scene object.
     * 
     * @param pScene The name of the scene. Required for creating un unique frame id.
     * @param pColor The color indicating the scene object.
     * @param pMarkerId Serves as unique id for the marker. Any marker sent with the same id will overwrite the old one.
     */
    void drawInInferenceMode(const std::string pScene, const std_msgs::ColorRGBA& pColor, unsigned int& pMarkerId);
    
    /**
     * Draws every primary scene object in a distinct coordinate frame. One ring color implies the primary scene object, the other the secondary scene object.
     * 
     * @param pScene The name of the scene. Required for creating un unique frame id.
     * @param pColor The color indicating the scene object.
     */
    void drawInLearningMode(const std::string pScene, const std_msgs::ColorRGBA& pColor);
    
    /**
     * Draws the object trajectories used for learning.
     * 
     * @param pScene The name of the scene. Required for creating un unique frame id.
     */
    void drawInLearningTrajectoryMode(const std::string pScene);
    
    /**
     * Returns the name of the primary scene object associated with this visualizer.
     * 
     * @return The name of the primary scene object.
     */
    std::string getSceneObjectName();
    
    /**
     * Adds the score of one of the terms that form the best hypothesis
     * 
     * @param pScore The score of the appearance term.
     * @param pShapeScore The score of the shape term.
     * @param pExistenceScore The score of the existence term.
     */
    void addTermScore(double pScore);
    
    /**
     * Resets all term scores.
     */
    void resetTermScores();
    
    /**
     * If the given score is better than the last best score, the last best pose is overwritten by the last pose.
     * 
     * @param pHypothesisScore The score of the last hypothesis.
     */
    void setHypothesisScore(double pHypothesisScore);
    
    /**
     * Normalizes the score for the best hypothesis.
     * 
     * @param pNumberOfSlots The number of slots used for normalization.
     */
    void normalizeHypothesisScore(unsigned int pNumberOfSlots);
    
    /**
     * Marks the scene object with the best score.
     * 
     * @param pStatus True, to select the scene object as the one with the best score.
     */
    void setBestStatus(bool pStatus);
    
    /**
     * Resets the candidates and positions.
     */
    void resetHypothesis();
    
  private:
    
    /**
     * The name of this primary scene object.
     */
    std::string mSceneObject;
    
    /**
     * Node handle for inintializing the publisher.
     */
    ros::NodeHandle mHandle;
    
    /**
     * Publisher for all visualization messages generated in this class.
     */
    boost::shared_ptr<ros::Publisher> mPublisher;
    
    /**
     * Visualization helper for coordinate frames.
     */
    CoordinateFrameVisualizer visualizerFrame;
    
    /**
     * Visualization helper for the kinematic chain.
     */
    KinematicChainVisualizer visualizerKinematics;
    
    /**
     * Visualization helper for point clound.
     */
    SampleVisualizer visualizerSamples;
    
    /**
     * The absolute pose of the primary object associated with this visualizer.
     */
    boost::shared_ptr<ISM::Pose> mAbsolutePose;
    
    /**
     * The score of the best hypothesis.
     */
    double mBestHypothesisScore;
    
    /**
     * A list of the current term scores and the best term scores.
     */
    std::vector<double> mTermScores, mBestTermScores;
    
    /**
     * The score of the single terms the best hypothesis score is made of.
     */
    double mAppearanceScore, mShapeScore, mExistenceScore;
    
    /**
     * Best absolute pose of the primary object. It was determined by the best scoring hypothesis regarding the assignment of evidence to the slots.
     */
    boost::shared_ptr<ISM::Pose> mBestPose;
    
    /**
     * Candidate for the best absolute position of the primary object. It was determined by the best scoring hypothesis regarding the assignment of evidence to the slots.
     */
    boost::shared_ptr<ISM::Pose> mBestCandidatePose;
    
    /**
     * A list of visualizers for all appended secondary scene objects.
     */
    std::vector<boost::shared_ptr<ProbabilisticSecondarySceneObjectVisualization> > mSceneObjectVisualizers;
    
    /**
     * The list of samples used for learning the gaussian mixture distribution.
     */
    std::vector<Eigen::Vector3d> mSamples;
  };
}
