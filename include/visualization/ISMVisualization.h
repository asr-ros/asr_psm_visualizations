/**

Copyright (c) 2016, Gehrung Joachim, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>
#include <set>
#include <utility>

#include <boost/shared_ptr.hpp>

#include <tf/transform_listener.h>

#include <Eigen/Core>

#include <ISM/RecognitionResult.hpp>

#include "QuadGrid.h"

namespace Visualization {
  static const int ID_PTU_FOV = 0;
  static const int ID_TESSELLATED_SPHERE = 1;
  static const int ID_SAMPLES_2D = 2;
  static const int ID_SAMPLES_3D = 3;
  static const int ID_QUADS = 20000;

  static const int GRID_SCALE = 0.005;
  static const int TS_RADIUS = 0.5:

  class ISMVisualization {
	
  private:
    // The grid containing the votes
    QuadGridPtr grid;

    // Resolution for the tesselated sphere
    double quad_pan_deg;
    double quad_tilt_deg;

    // Publishers for vsualization markers
    ros::Publisher ismPublisher;
    ros::Publisher arrowPublisher;
    ros::Publisher tsPublisher;
    ros::Publisher qgPublisher;
    ros::Publisher fovPublisher;
    ros::Publisher samples2dPublisher;
    ros::Publisher samples3dPublisher;
	
  public:
    ISMVisualization();
    ~ISMVisualization();

    // Draw tesselated sphere as grid
    void drawTesselatedSphere(float panMin, float panMax, float tiltMin, float tiltMax);
		
    // Draws extent of camera view with current PTU state
    void drawFoV(float pan, float tilt, float foVX, float foVY);

    // Draws relations and so on in one recognized scene given as parameter
    void drawRecognitionResult(ISM::RecognitionResultPtr result, int level, ISM::PointPtr parentPoint, std::string prefix, unsigned char action);
		
    // Draws color coded quad ratings
    void drawQuads(float panMin, float panMax, float tiltMin, float tiltMax);

    // Draws object position samples in 3D space and their projection on tesselated sphere
    void drawSamples();

    // Draw arrows from scene reference to object position votes for objects belonging to scene and not found until yet
    void drawUnfoundObjectsVotes();

  private:

    // Adds the given distance to the z-Axis of the point
    ISM::PointPtr applyLevelToPoint(ISM::PointPtr p, int level);
    
    // Creates a visualization message that draws a point as sphere
    visualization_msgs::Marker pointToSphere(ISM::PointPtr point, unsigned char action);
		
    // Creates a visualization message that draws a line between points
    visualization_msgs::Marker lineBetweenPoints(ISM::PointPtr point1, ISM::PointPtr point2, unsigned char action);
	
    // Creates a visualization message that draws an arrow between points
    visualization_msgs::Marker arrowBetweenPoints(const ISM::PointPtr& point1, const ISM::PointPtr& point2, unsigned char action);

    // Creates a visualization message that draws an arrow between points
    visualization_msgs::Marker arrowBetweenPoints2(const ISM::PointPtr& point1, const ISM::PointPtr& point2);
		
    // Creates a geometry message containing a point. Helper function for methods above.
    geometry_msgs::Point pointToPointMsg(ISM::PointPtr point);
		
    // Creates the appropriate color for a given confidence level
    std_msgs::ColorRGBA confidenceToColor(double confidence);
		
    // Calculates the angle to the PTU.
    geometry_msgs::Point angleToPTU(float pan, float tilt, float r);

    // Returns the Quaternion that belongs to the given angle
    tf::Quaternion angleToQuaternion(float pan, float tilt);

    // ???
    Eigen::Vector2f getQuadExtent(Eigen::Vector2f pos, float r, float panSize = 5, float tiltSize = 5);
  };

  typedef boost::shared_ptr<ISMVisualization> ISMVisualizationPtr;
}
