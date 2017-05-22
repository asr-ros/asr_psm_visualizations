/**

Copyright (c) 2016, Gehrung Joachim, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "visualization/psm/ProbabilisticPrimarySceneObjectVisualization.h"

namespace Visualization {

  ProbabilisticSecondarySceneObjectVisualization::ProbabilisticSecondarySceneObjectVisualization()
  : mHandle()
  , mCertainty(0.0)
  , mBestScore(0.0)
  {
    mPublisher.reset(new ros::Publisher(mHandle.advertise<visualization_msgs::MarkerArray>("psm", 1)));
  }
  
  ProbabilisticSecondarySceneObjectVisualization::ProbabilisticSecondarySceneObjectVisualization(std::string pName)
  : mSceneObject(pName)
  , mHandle()
  , mCertainty(0.0)
  , mBestScore(0.0)
  {
    mPublisher.reset(new ros::Publisher(mHandle.advertise<visualization_msgs::MarkerArray>("psm", 1)));
  }
	
  ProbabilisticSecondarySceneObjectVisualization::~ProbabilisticSecondarySceneObjectVisualization()
  {
  }
  
  void ProbabilisticSecondarySceneObjectVisualization::setDrawingParameters(const double pScale, const float pSigmaMultiplicator, const std::string pFrameId)
  {
    visualizerFrame.setFrameId(pFrameId);
    
    visualizerKernel.setFrameId(pFrameId);
    visualizerKernel.setScaleFactor(pScale);
    visualizerKernel.setSigmaMultiplicator(pSigmaMultiplicator);
    
    visualizerKinematics.setFrameId(pFrameId);
    visualizerKinematics.setScaleFactor(pScale);
    
    visualizerSamples.setFrameId(pFrameId);
    visualizerSamples.setScaleFactor(pScale);
  }
  
  void ProbabilisticSecondarySceneObjectVisualization::setNamespace(std::string pNamespace)
  {
    std::string ns;
    if(mSceneObject.size() > 0)
      ns = pNamespace + "/" + mSceneObject;
    else
      ns = pNamespace;
    
    visualizerFrame.setNamespace(ns);
    visualizerKernel.setNamespace(ns);
    visualizerKinematics.setNamespace(ns);
    visualizerSamples.setNamespace(ns + "/samples");
  }
  
  void ProbabilisticSecondarySceneObjectVisualization::setColors(const std_msgs::ColorRGBA& pFirstColor,
								  const std_msgs::ColorRGBA& pSecondColor)
  {
    // Create a gray color for the unused ring.
    std_msgs::ColorRGBA color;
    color.r = color.g = color.b = 0.3;
    color.a = 1.0;
    
    // Forward the colors to the visualizers.
    visualizerKernel.setColors(pFirstColor, pSecondColor, color);
    visualizerKinematics.setColors(pFirstColor, pSecondColor, color);
    visualizerSamples.setColors(pFirstColor, pSecondColor, color);
  }
  
  void ProbabilisticSecondarySceneObjectVisualization::setDetectionCertainty(double pCertainty)
  {
    // Certainty must be in [0,1].
    if(pCertainty < 0.0 && pCertainty > 1.0)
      throw std::out_of_range("Certainty is out of range ('" + boost::lexical_cast<std::string>(pCertainty) + "').");
    
    // Take the bigger value.
    if(pCertainty > mCertainty)
      mCertainty = pCertainty;
  }
  
  void ProbabilisticSecondarySceneObjectVisualization::resetCertainty()
  {
    mCertainty = 0.0;
  }
  
  void ProbabilisticSecondarySceneObjectVisualization::setLastPose(boost::shared_ptr<ISM::Pose> pPose)
  {
    // Check, if pointer is valid.
    if(!pPose)
      throw std::out_of_range("Invalid pointer: last observation of associated scene object pose.");
    
    mLastObjectPose = pPose;
  }
  
  void ProbabilisticSecondarySceneObjectVisualization::setBestCandidatePose(boost::shared_ptr<ISM::Pose> pPose)
  {
    // Check, if pointer is valid.
    if(!pPose)
      throw std::out_of_range("Invalid pointer: absolute pose of primary scene object.");
    
    // Set position and orientation.
    mBestCandidatePose = pPose;
  }
  
  void ProbabilisticSecondarySceneObjectVisualization::setBestPoseCandidate(double pScore)
  {
    mBestScoreCandidate = std::min(pScore, 1.0);
  }
  
  void ProbabilisticSecondarySceneObjectVisualization::setParentPose(boost::shared_ptr<ISM::Pose> pPose)
  {
    // Check, if pointer is valid.
    if(!pPose)
      throw std::out_of_range("Invalid pointer: absolute pose of primary scene object.");
    
    // Set absolute position of the primary scene object.
    mParentPose = pPose;
  }
  
  void ProbabilisticSecondarySceneObjectVisualization::setAbsoluteLearningSamples(const std::vector<Eigen::Vector3d>& pRelativeSamples,
										  const std::vector<Eigen::Vector3d>& pAbsoluteSamples,
										  const std::vector<Eigen::Vector3d>& pAbsoluteParentSamples)
  {
    mRelativeSamples = pRelativeSamples;
    mAbsoluteSamples = pAbsoluteSamples;
    mAbsoluteParentSamples = pAbsoluteParentSamples;
  }
  
  void ProbabilisticSecondarySceneObjectVisualization::overwriteLastBestPose()
  {
    mBestScore = mBestScoreCandidate;
    mBestPose = mBestCandidatePose;
  }
  
  void ProbabilisticSecondarySceneObjectVisualization::appendKernel(const boost::shared_ptr<Eigen::Vector3d>& pMean,
								     const boost::shared_ptr<Eigen::Matrix3d>& pCovariance)
  {
    // Check, if pointer is valid.
    if(!pMean)
      throw std::out_of_range("Invalid pointer: mean vector.");
    
    // Check, if pointer is valid.
    if(!pCovariance)
      throw std::out_of_range("Invalid pointer: covariance matrix.");
    
    // Store the kernel in an intern data structure.
    mKernels.push_back(std::pair<Eigen::Vector3d, Eigen::Matrix3d>(*pMean, *pCovariance));
  }
  
  void ProbabilisticSecondarySceneObjectVisualization::visualizeGaussianKernels(unsigned int& pMarkerId)
  {
    // Call to advertise() should not have returned an empty publisher.
    if(!mPublisher)
      throw std::invalid_argument("Empty ros::publisher cannot be used to publish marker messages.");

    // Check, it parent pose is already known.
    if(mParentPose)
    {
      // Forward the parent pose to the visualizer.
      visualizerKernel.setParentPose(mParentPose);
      
      // Draw all kernels stored in this class. Also draw an arrow from the parent position to the kernel mean.
      for(std::pair<Eigen::Vector3d, Eigen::Matrix3d> kernel : mKernels)
	visualizerKernel.publishKernel(mPublisher, pMarkerId, kernel.first, kernel.second);
    }
  }
  
  void ProbabilisticSecondarySceneObjectVisualization::visualizeObjectPositionAsArrow(unsigned int& pMarkerId)
  {
    // Call to advertise() should not have returned an empty publisher.
    if(!mPublisher)
      throw std::invalid_argument("Empty ros::publisher cannot be used to publish marker messages.");
    
    // If last position is known, generate marker for it and publish it on given topic.
    if(mLastObjectPose)
    {
      // Draw the object position as an arrow.
      visualizerKinematics.publishObjectPositionAsArrow(mPublisher, pMarkerId, mLastObjectPose, mCertainty);
    }
  }
  
  
  void ProbabilisticSecondarySceneObjectVisualization::visualizeKinematicChain(unsigned int& pMarkerId)
  {
    // Call to advertise() should not have returned an empty publisher.
    if(!mPublisher)
      throw std::invalid_argument("Empty ros::publisher cannot be used to publish marker messages.");
    
    // Check, it parent pose is already known.
    if(mParentPose && mBestPose)
    {
      // Draw the link between parent object and this object.
      // Apply some normalization term for the color of the kinematic chain.
      visualizerKinematics.publishLink(mPublisher, pMarkerId, mParentPose->point->getEigen(), mBestPose->point->getEigen(), std::min(mBestScore / 0.02, 1.0)); // NOTE The normalization constant here is responsible for coloring the arrow between objects.
    }
  }
  
  void ProbabilisticSecondarySceneObjectVisualization::visualizeRelativeTrajectory(unsigned int& pMarkerId)
  {
    // Call to advertise() should not have returned an empty publisher.
    if(!mPublisher)
      throw std::invalid_argument("Empty ros::publisher cannot be used to publish marker messages.");
    
    // Check, it parent pose is already known.
    if(mParentPose)
    {
      // Forward the parent pose to the visualizer.
      visualizerSamples.setParentPose(mParentPose);
      
      // Draws a small sphere in the color of the scene object it belongs to for every learning sample.
      visualizerSamples.publishTrajectory(mPublisher, pMarkerId, mRelativeSamples);
    }
  }
  
  void ProbabilisticSecondarySceneObjectVisualization::visualizeAbsoluteTrajectory(unsigned int& pMarkerId)
  {
    // Call to advertise() should not have returned an empty publisher.
    if(!mPublisher)
      throw std::invalid_argument("Empty ros::publisher cannot be used to publish marker messages.");
    
    // Draw all samples.
    visualizerSamples.publishTrajectory(mPublisher, pMarkerId, mAbsoluteSamples, mAbsoluteParentSamples);
  }
  
}
