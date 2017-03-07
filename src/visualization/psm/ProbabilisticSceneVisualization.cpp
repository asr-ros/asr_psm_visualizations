/**

Copyright (c) 2016, Gehrung Joachim, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "visualization/psm/ProbabilisticSceneVisualization.h"

namespace Visualization {
	
  ProbabilisticSceneVisualization::ProbabilisticSceneVisualization(std::string pScene)
  : mScene(pScene)
  {
  }
	
  ProbabilisticSceneVisualization::~ProbabilisticSceneVisualization()
  {
  }
  
  void ProbabilisticSceneVisualization::appendVisualizer(boost::shared_ptr<ProbabilisticPrimarySceneObjectVisualization> pVisualizer)
  {
    mSceneObjectVisualizers.push_back(pVisualizer);
  }
  
  void ProbabilisticSceneVisualization::setDrawingParameters(const double pScale, const float pSigmaMultiplicator, const std::string pFrameId)
  {
    for(boost::shared_ptr<ProbabilisticPrimarySceneObjectVisualization> visualizer : mSceneObjectVisualizers)
      visualizer->setDrawingParameters(pScale, pSigmaMultiplicator, pFrameId);
  }
  
  void ProbabilisticSceneVisualization::drawInTargetingMode()
  {
    // Stop, if no primary scene object visualizers registered.
    if(mSceneObjectVisualizers.size() == 0)
      return;
    
    // The size of a step in the hsv color space.
    double hsvColorSteps = 1.0 / mSceneObjectVisualizers.size();
    
    // Iterate over all scene objects and create a color for each one.
    // But draw only the scene object with the given name.
    for(unsigned int i = 0; i < mSceneObjectVisualizers.size(); i++)
    {
      // Create an unique color...
      std_msgs::ColorRGBA color;
      color.a = 1.0;
      
      // ... by converting an index based postion on the hue axis into RGB
      ColorHelper::convertHSVToRGB(color, (double) hsvColorSteps * i, 1.0, 1.0);
      
      // Draw the scene.
      mSceneObjectVisualizers[i]->drawInTargetingMode(mScene, color);
    }
  }
  
  void ProbabilisticSceneVisualization::drawInInferenceMode()
  {
    // Stop, if no primary scene object visualizers registered.
    if(mSceneObjectVisualizers.size() == 0)
      return;
    
    // The size of a step in the hsv color space.
    double hsvColorSteps = 1.0 / mSceneObjectVisualizers.size();
    
    // Serves as unique id for the marker. Any marker sent with the same id will overwrite the old one.
    unsigned int markerId = 0;
    
    // Iterate over all scene objects and create a color for each one.
    // But draw only the scene object with the given name.
    for(unsigned int i = 0; i < mSceneObjectVisualizers.size(); i++)
    {
      // Create an unique color...
      std_msgs::ColorRGBA color;
      color.a = 1.0;
      
      // ... by converting an index based postion on the hue axis into RGB
      ColorHelper::convertHSVToRGB(color, (double) hsvColorSteps * i, 1.0, 1.0);
      
      // Draw the scene.
      mSceneObjectVisualizers[i]->drawInInferenceMode(mScene, color, markerId);
    }
  }
  
  void ProbabilisticSceneVisualization::drawInLearningMode()
  {
    // Stop, if no primary scene object visualizers registered.
    if(mSceneObjectVisualizers.size() == 0)
      return;
    
    // The size of a step in the hsv color space.
    double hsvColorSteps = 1.0 / mSceneObjectVisualizers.size();
    
    // Iterate over all scene objects and create a color for each one.
    // But draw only the scene object with the given name.
    for(unsigned int i = 0; i < mSceneObjectVisualizers.size(); i++)
    {
      // Create an unique color...
      std_msgs::ColorRGBA color;
      color.a = 1.0;
      
      // ... by converting an index based postion on the hue axis into RGB
      ColorHelper::convertHSVToRGB(color, (double) hsvColorSteps * i, 1.0, 1.0);
      
      // Draw the scene.
      mSceneObjectVisualizers[i]->drawInLearningMode(mScene, color);
    }
  }
  
  void ProbabilisticSceneVisualization::drawInLearningTrajectoryMode()
  {
    for(unsigned int i = 0; i < mSceneObjectVisualizers.size(); i++)
      mSceneObjectVisualizers[i]->drawInLearningTrajectoryMode(mScene);
  }
  
}
