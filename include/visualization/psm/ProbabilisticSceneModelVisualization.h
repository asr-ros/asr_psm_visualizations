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

#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Core>

// Local includes
#include "visualization/psm/ProbabilisticSceneVisualization.h"

#include "helper/ColorHelper.h"

namespace Visualization
{
  /**
   * Visualizer class for probabilistic scenes.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class ProbabilisticSceneModelVisualization
  {
  public:
    
    /**
     * Constructor.
     */
    ProbabilisticSceneModelVisualization();
    
    /**
     * Destructor.
     */
    ~ProbabilisticSceneModelVisualization();
    
    /**
     * Appends a scene visualizer.
     * 
     * @param pVisualizer The scene visualizer to append.
     */
    void appendVisualizer(boost::shared_ptr<ProbabilisticSceneVisualization> pVisualizer);
    
    /**
     * Sets the parameter requried for visualizing the secondary scene object.
     * 
     * @param pScale Factor to multiply the kernel with.
     * @param pSigmaMultiplicator Scaling factor for the size of the visualized covariance ellipsoid.
     * @param pFrameId The name of the coordinate frame that should be drawn into.
     */
    void setDrawingParameters(const double pScale, const float pSigmaMultiplicator, const std::string pFrameId);
    
    /**
     * Draws every primary scene object in a distinct coordinate frame. One ring color implies the primary scene object, the other the secondary scene object. Each primary scene object will be visualized in the specified coordinate frame but in an unique topic, formatted like /scene/sceneobject.
     */
    void drawInTargetingMode();
    
    /**
     * Draws the kinematic chain for the best scoring hypothesis regarding the assignment of evidence to the slots. A chain for every primary scene object for every scene will be visualized in a separate topic. One ring color implies the primary scene object, the other the secondary scene object.
     */
    void drawInInferenceMode();
    
    /**
     * Draws all the object trajectories used for learning and the learned distributions.
     */
    void drawInLearningMode();
    
    /**
     * Draws the object trajectories used for learning.
     */
    void drawInLearningTrajectoryMode();
    
  private:
    
    /**
     * A list of visualizers for all scene objects.
     */
    std::vector<boost::shared_ptr<ProbabilisticSceneVisualization> > mSceneVisualizers;
  };
}
