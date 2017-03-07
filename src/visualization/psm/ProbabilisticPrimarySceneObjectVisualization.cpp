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
	
  ProbabilisticPrimarySceneObjectVisualization::ProbabilisticPrimarySceneObjectVisualization(std::string pName)
  : mSceneObject(pName)
  , mHandle()
  , mBestHypothesisScore(0.0)
  {
    mPublisher.reset(new ros::Publisher(mHandle.advertise<visualization_msgs::MarkerArray>("psm", 1)));
  }
	
  ProbabilisticPrimarySceneObjectVisualization::~ProbabilisticPrimarySceneObjectVisualization()
  {
  }
  
  void ProbabilisticPrimarySceneObjectVisualization::appendVisualizer(boost::shared_ptr<ProbabilisticSecondarySceneObjectVisualization> pVisualizer)
  {
    mSceneObjectVisualizers.push_back(pVisualizer);
  }
  
  void ProbabilisticPrimarySceneObjectVisualization::setDrawingParameters(const double pScale, const float pSigmaMultiplicator, const std::string pFrameId)
  {
    // Forward parameters to local visualizers.
    visualizerFrame.setFrameId(pFrameId);
    visualizerKinematics.setFrameId(pFrameId);
    visualizerKinematics.setScaleFactor(pScale);
    visualizerSamples.setFrameId(pFrameId);
    visualizerSamples.setScaleFactor(pScale);
    
    // Forward parameters to secondary scene objects.
    for(boost::shared_ptr<ProbabilisticSecondarySceneObjectVisualization> visualizer : mSceneObjectVisualizers)
      visualizer->setDrawingParameters(pScale, pSigmaMultiplicator, pFrameId);
  }
  
  void ProbabilisticPrimarySceneObjectVisualization::setPose(boost::shared_ptr<ResourcesForPsm::Pose> pPose)
  {
    // Check, if pointer is valid.
    if(!pPose)
      throw std::out_of_range("Invalid pointer: absolute pose of primary scene object.");
    
    // Set position and orientation.
    mAbsolutePose = pPose;
  }
  
  void ProbabilisticPrimarySceneObjectVisualization::setBestPoseCandidate(boost::shared_ptr<ResourcesForPsm::Pose> pPose)
  {
    // Check, if pointer is valid.
    if(!pPose)
      throw std::out_of_range("Invalid pointer: absolute pose of primary scene object candidate.");
    
    // Set position and orientation.
    mBestCandidatePose = pPose;
  }
  
  void ProbabilisticPrimarySceneObjectVisualization::setLearningSamples(const std::vector<Eigen::Vector3d>& pSamples)
  {
    mSamples = pSamples;
  }
  
  void ProbabilisticPrimarySceneObjectVisualization::drawInTargetingMode(const std::string pScene, const std_msgs::ColorRGBA& pColor)
  {
    // Stop, if no primary scene object visualizers registered.
    if(mSceneObjectVisualizers.size() == 0)
      return;
    
    // Only execute when the primary scene object has been found.
    if(mAbsolutePose)
    {
      // Build the namespace.
      std::string myNamespace = "/" + pScene + "/" + mSceneObject;
      
      // The size of a step in the hsv color space.
      double hsvColorSteps = 1.0 / mSceneObjectVisualizers.size();
      
      // Serves as unique id for the marker. Any marker sent with the same id will overwrite the old one.
      unsigned int markerId = 0;
      
      // Draw all secondary scene objects appended to this primary scene in an unique color.
      for(unsigned int i = 0; i < mSceneObjectVisualizers.size(); i++)
      {
	// Create an unique color...
	std_msgs::ColorRGBA color;
	color.a = 1.0;
	
	// ... by converting an index based postion on the hue axis into RGB
	ColorHelper::convertHSVToRGB(color, (double) hsvColorSteps * i, 1.0, 1.0);
	
	// Set the color of the two rings.
	mSceneObjectVisualizers[i]->setColors(pColor, color);
	
	// Forward namespace to secondary scene object visualizers.
	mSceneObjectVisualizers[i]->setNamespace(myNamespace);
	
	// Visualize the covariance ellipse and the last known object position.
	mSceneObjectVisualizers[i]->visualizeObjectPositionAsArrow(markerId);
	mSceneObjectVisualizers[i]->visualizeGaussianKernels(markerId);
      }
    }
  }
  
  void ProbabilisticPrimarySceneObjectVisualization::drawInInferenceMode(const std::string pScene, const std_msgs::ColorRGBA& pColor, unsigned int& pMarkerId)
  {
    // Stop, if no primary scene object visualizers registered.
    if(mSceneObjectVisualizers.size() == 0)
      return;
    
    // Only execute when the primary scene object has been found.
    // DO NOT DRAW FOR BAD RESULTS!
    if(mBestPose)
    {
      // Build the namespace.
      std::string myNamespace = "/" + pScene + "/" + mSceneObject;
      
      // The size of a step in the hsv color space.
      double hsvColorSteps = 1.0 / mSceneObjectVisualizers.size();
      
      // Draw the primary scene object.
      // Apply some normalization term for the color of the arrows.
      visualizerFrame.setNamespace(myNamespace);
      visualizerFrame.publishFrame(mPublisher, pMarkerId, mBestPose, 3.0);
      
      visualizerKinematics.setNamespace("/" + pScene);
      visualizerKinematics.publishObjectPositionAsPointWithScore(mPublisher, pMarkerId, mBestPose, mBestTermScores, std::min(mBestHypothesisScore / 0.000001, 1.0)); 	// NOTE The normalization constant here is responsible for coloring the perpendicular object indircator arrow.
      
      // Draw all secondary scene objects appended to this primary scene in an unique color.
      for(unsigned int i = 0; i < mSceneObjectVisualizers.size(); i++)
      {
	// Create an unique color...
	std_msgs::ColorRGBA color;
	color.a = 1.0;
	
	// ... by converting an index based postion on the hue axis into RGB
	ColorHelper::convertHSVToRGB(color, (double) hsvColorSteps * i, 1.0, 1.0);
	
	// Set the color of the two rings.
	mSceneObjectVisualizers[i]->setColors(pColor, color);
	
	// Forward namespace to secondary scene object visualizers.
	mSceneObjectVisualizers[i]->setNamespace(myNamespace);
	
	// Visualize the kinematic chain.
	mSceneObjectVisualizers[i]->visualizeKinematicChain(pMarkerId);
      }
    }
  }
  
  void ProbabilisticPrimarySceneObjectVisualization::drawInLearningMode(const std::string pScene, const std_msgs::ColorRGBA& pColor)
  {
    // Stop, if no secondary scene object visualizers registered.
    if(mSceneObjectVisualizers.size() == 0)
      return;
    
    // Build the namespace.
    std::string myNamespace = "/" + pScene + "/" + mSceneObject;
    
    // The size of a step in the hsv color space.
    double hsvColorSteps = 1.0 / mSceneObjectVisualizers.size();
    
    // Serves as unique id for the marker. Any marker sent with the same id will overwrite the old one.
    unsigned int markerId = 0;
    
    // Draw all secondary scene objects appended to this primary scene in an unique color.
    for(unsigned int i = 0; i < mSceneObjectVisualizers.size(); i++)
    {
      // Create an unique color...
      std_msgs::ColorRGBA color;
      color.a = 1.0;
      
      // ... by converting an index based postion on the hue axis into RGB
      ColorHelper::convertHSVToRGB(color, (double) hsvColorSteps * i, 1.0, 1.0);
      
      // Set the color of the two rings.
      mSceneObjectVisualizers[i]->setColors(pColor, color);
      
      // Forward namespace to secondary scene object visualizers.
      mSceneObjectVisualizers[i]->setNamespace(myNamespace);
      
      // Visualize the covariance ellipses and the learning data.
      mSceneObjectVisualizers[i]->visualizeGaussianKernels(markerId);
      mSceneObjectVisualizers[i]->visualizeRelativeTrajectory(markerId);
    }
  }
  
  void ProbabilisticPrimarySceneObjectVisualization::drawInLearningTrajectoryMode(const std::string pScene)
  {
    // Stop, if no secondary scene object visualizers registered.
    if(mSceneObjectVisualizers.size() == 0)
      return;
    
    // Build the namespace.
    std::string myNamespace = "/" + pScene + "/" + mSceneObject;
    
    // The size of a step in the hsv color space.
    double hsvColorSteps = 1.0 / mSceneObjectVisualizers.size();
    
    // Serves as unique id for the marker. Any marker sent with the same id will overwrite the old one.
    unsigned int markerId = 0;
    
    // Draw all secondary scene objects appended to this primary scene in an unique color.
    for(unsigned int i = 0; i < mSceneObjectVisualizers.size(); i++)
    {
      // Create an unique color...
      std_msgs::ColorRGBA color;
      color.a = 1.0;
      
      // ... by converting an index based postion on the hue axis into RGB
      ColorHelper::convertHSVToRGB(color, (double) hsvColorSteps * i, 1.0, 1.0);
      
      // Set the color of the two rings.
      mSceneObjectVisualizers[i]->setColors(color, color);
      
      // Forward namespace to secondary scene object visualizers.
      mSceneObjectVisualizers[i]->setNamespace(myNamespace);
      
      // Visualize absolute trajectory.
      mSceneObjectVisualizers[i]->visualizeAbsoluteTrajectory(markerId);
    }
  }
  
  std::string ProbabilisticPrimarySceneObjectVisualization::getSceneObjectName()
  {
    return mSceneObject;
  }
  
  void ProbabilisticPrimarySceneObjectVisualization::addTermScore(double pScore)
  {
  mTermScores.push_back(pScore / 0.000001);	// NOTE The normalization constant here is responsible for coloring the rings around the perpendicular object indircator arrow.
  }
  
  void ProbabilisticPrimarySceneObjectVisualization::resetTermScores()
  {
    mTermScores.clear();
  }
  
  void ProbabilisticPrimarySceneObjectVisualization::setHypothesisScore(double pHypothesisScore)
  {
    // Better score? Overwrite best pose and term scores!
    if(pHypothesisScore > mBestHypothesisScore)
    {
      mBestPose = mBestCandidatePose;
      mBestHypothesisScore = pHypothesisScore;
      
      mBestTermScores.clear();
      mBestTermScores.insert(mBestTermScores.end(), mTermScores.begin(), mTermScores.end());
      
      // Command secondary scene objects to do the same.
      for(unsigned int i = 0; i < mSceneObjectVisualizers.size(); i++)
	mSceneObjectVisualizers[i]->overwriteLastBestPose();
    }
  }
  
  void ProbabilisticPrimarySceneObjectVisualization::normalizeHypothesisScore(unsigned int pNumberOfSlots)
  {
    mBestHypothesisScore /= pNumberOfSlots;
  }
  
  void ProbabilisticPrimarySceneObjectVisualization::resetHypothesis()
  {
      mBestPose.reset();
      mBestHypothesisScore = 0.0;
  }
  
  void ProbabilisticPrimarySceneObjectVisualization::setBestStatus(bool pStatus)
  {
    visualizerKinematics.setBestStatus(pStatus);
  }
  
}
