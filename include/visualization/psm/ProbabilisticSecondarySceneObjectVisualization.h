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
#include <utility>
#include <vector>
#include <math.h>

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
#include "visualization/psm/helper/CoordinateFrameVisualizer.h"
#include "visualization/psm/helper/GaussianKernelVisualizer.h"
#include "visualization/psm/helper/KinematicChainVisualizer.h"
#include "visualization/psm/helper/SampleVisualizer.h"

namespace Visualization
{
  /**
   * Visualizer class for secondary scenes objects.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class ProbabilisticSecondarySceneObjectVisualization
  {
  public:
    
    /**
     * Constructor.
     */
    ProbabilisticSecondarySceneObjectVisualization();
    
    /**
     * Constructor.
     * 
     * @param pName The name of this secondary scene object.
     */
    ProbabilisticSecondarySceneObjectVisualization(std::string pName);
    
    /**
     * Destructor.
     */
    ~ProbabilisticSecondarySceneObjectVisualization();
    
    /**
     * Sets the parameter requried for visualizing the secondary scene object.
     * 
     * @param pScale Factor to multiply the kernel with.
     * @param pSigmaMultiplicator Scaling factor for the size of the visualized covariance ellipsoid.
     * @param pFrameId The name of the coordinate frame that should be drawn into.
     */
    void setDrawingParameters(const double pScale, const float pSigmaMultiplicator, const std::string pFrameId);
    
    /**
     * Sets the name space bases on the names of scene and scene object.
     * 
     * @param pNamespace The namespace to use for visualization.
     */
    void setNamespace(std::string pNamespace);
    
    /**
     * Sets the colors of the two rings around the covariance ellipse.
     * 
     * @param pFirstRingColor The color of the first ring.
     * @param pSecondRingColor The color of the second ring.
     */
    void setColors(const std_msgs::ColorRGBA& pFirstRingColor, const std_msgs::ColorRGBA& pSecondRingColor);
    
    /**
     * Sets the certainty value that describes the fitting of the associated scene object.
     * 
     * @param pCertainty The certainty betwween 0 and 1.
     */
    void setDetectionCertainty(double pCertainty);
    
    /**
     * Resets the certainty value that describes the fitting of the associated scene object.
     */
    void resetCertainty();
    
    /**
     * Sets the last known pose of the associated object.
     * 
     * @param pPose The last known pose.
     */
    void setLastPose(boost::shared_ptr<ISM::Pose> pPose);
    
    /**
     * Sets a candidate for the absolute pose of the associated object. The primary scene object will decide based on the hypothsis score, if it is the best absolute pose.
     * 
     * @param pPose Best absolute pose of the primary scene object.
     */
    void setBestCandidatePose(boost::shared_ptr<ISM::Pose> pPose);
    
    /**
     * Sets a candidate pScore for the link between this object and its parent. The primary scene object will decide based on the hypothsis score, if it is the best score for the link.
     * 
     * @param pScore The score.
     */
    void setBestPoseCandidate(double pScore);
    
    /**
     * Sets the absolute pose of the parent object.
     * 
     * @param pPose Absolute pose of the primary scene object.
     */
    void setParentPose(boost::shared_ptr<ISM::Pose> pPose);
    
    /**
     * Sets the samples used for model learning and the corresponding poses for the parent object.
     * 
     * @param pRelativeSamples The list of relative poses to the parent.
     * @param pAbsoluteSamples The list of absolute samples used for model learning.
     * @param pAbsoluteParentSamples The list of corresponding poses for the parent object.
     */
    void setAbsoluteLearningSamples(const std::vector<Eigen::Vector3d>& pRelativeSamples,
				    const std::vector<Eigen::Vector3d>& pAbsoluteSamples,
				    const std::vector<Eigen::Vector3d>& pAbsoluteParentSamples);
    
    /**
     * Takes the last stored candidate for the best pose as the last best pose.
     * This method is invoked by the primary scene object.
     */
    void overwriteLastBestPose();
    
    /**
     * Appends a kernel that will be visualized in form of a covariance ellipse.
     * 
     * @param pCovariance A 3x3 eigen matrix representing the corvariance matrix of the gaussian kernel.
     * @param pMean A 3d eigen vector representing the mean of the gaussian kernel.
     */
    void appendKernel(const boost::shared_ptr<Eigen::Vector3d>& pMean, const boost::shared_ptr<Eigen::Matrix3d>& pCovariance);
    
    /**
     * Visualizes all gaussian kernels given to this class.
     * 
     * @param pMarkerId An unique id for the marker. Any marker sent with the same id will overwrite the old one.
     */
    void visualizeGaussianKernels(unsigned int& pMarkerId);
    
    /**
     * Visualizes the position of this secondary scene object.
     *
     * @param pMarkerId Serves as unique id for the marker. Any marker sent with the same id will overwrite the old one.
     */
    void visualizeObjectPositionAsArrow(unsigned int& pMarkerId);
    
    /**
     * Visualizes the kinematic chain.
     * 
     * @param pMarkerId An unique id for the marker. Any marker sent with the same id will overwrite the old one.
     */
    void visualizeKinematicChain(unsigned int& pMarkerId);
    
    /**
     * Visualizes the trajectory composed of the samples used for learning in relation to the parent object.
     * 
     * @param pMarkerId An unique id for the marker. Any marker sent with the same id will overwrite the old one.
     */
    void visualizeRelativeTrajectory(unsigned int& pMarkerId);
    
    /**
     * Visualizes the absolute trajectory.
     * 
     * @param pMarkerId An unique id for the marker. Any marker sent with the same id will overwrite the old one.
     */
    void visualizeAbsoluteTrajectory(unsigned int& pMarkerId);
    
  private:
    
    /**
     * The name of this secondary scene object.
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
     * Visualization helper for gaussian kernels.
     */
    GaussianKernelVisualizer visualizerKernel;
    
    /**
     * Visualization helper for the kinematic chain.
     */
    KinematicChainVisualizer visualizerKinematics;
    
    /**
     * Visualization helper for point clound.
     */
    SampleVisualizer visualizerSamples;
    
    /**
     * The uncertainty describes how good the associated scene object fits into the gaussian distribution.
     */
    double mCertainty;
    
    /**
     * The best score for the link to the parent.
     */
    double mBestScore;
    
    /**
     * Candidate for the best score for the link to the parent.
     */
    double mBestScoreCandidate;
    
    /**
     * The last known pose of the object associated with this visualizer.
     */
    boost::shared_ptr<ISM::Pose> mLastObjectPose;
    
    /**
     * Best absolute pose of the associated object. It was determined by the best scoring hypothesis regarding the assignment of evidence to the slots.
     */
    boost::shared_ptr<ISM::Pose> mBestPose;
    
    /**
     * Candidate for the best absolute position of the associated object. It was determined by the best scoring hypothesis regarding the assignment of evidence to the slots.
     */
    boost::shared_ptr<ISM::Pose> mBestCandidatePose;
    
    /**
     * Absolute pose of the parent object.
     */
    boost::shared_ptr<ISM::Pose> mParentPose;
    
    /**
     * The kernels that will be visualized in form of covariance ellipses.
     */
    std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d> > mKernels;
    
    /**
     * The samples used for model learning and the corresponding poses for the parent object.
     */
    std::vector<Eigen::Vector3d> mRelativeSamples, mAbsoluteSamples, mAbsoluteParentSamples;
  };
}
