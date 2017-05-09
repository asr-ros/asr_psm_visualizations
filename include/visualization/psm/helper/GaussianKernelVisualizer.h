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

#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_datatypes.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Pose.h>
#include <ISM/common_type/Pose.hpp>

// Local includes
#include "visualization/psm/helper/AbstractExtendedVisualizer.h"

namespace Visualization
{
  /**
   * Visualizer class for gaussian kernels.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class GaussianKernelVisualizer : public AbstractExtendedVisualizer
  {
  public:
    
    /**
     * Constructor.
     */
    GaussianKernelVisualizer();
    
    /**
     * Destructor.
     */
    ~GaussianKernelVisualizer();
    
    /**
     * Sets the sigma multiplicator.
     * 
     * @param pSigmaMultiplicator Scaling factor for the size of the visualized covariance ellipsoid.
     */
    void setSigmaMultiplicator(const float pSigmaMultiplicator);
    
    /**
     * Sets the absolute pose of the parent object.
     * 
     * @param pPose Absolute pose of the primary scene object.
     */
    void setParentPose(boost::shared_ptr<ISM::Pose> pPose);
    
    /**
     * Publishes a single gaussian kernel.
     *
     * @param pPublisher For publishing the visualization message.
     * @param pMarkerId Serves as unique id for the marker. Any marker sent with the same id will overwrite the old one.
     * @param pMean The mean vector of the kernel that should be visualized.
     * @param pCovariance The covariance matrix of the kernel that should be visualized.
     */
    void publishKernel(boost::shared_ptr<ros::Publisher> pPublisher,
		       unsigned int& pMarkerId,
		       const Eigen::Vector3d& pMean,
		       const Eigen::Matrix3d& pCovariance);
    
  private:
    
    /**
     * Returns the sigma multiplicator.
     * 
     * @return Scaling factor for the size of the visualized covariance ellipsoid.
     */
    float getSigmaMultiplicator();
    
    /**
     * Generates the visualization message that contains the coordinate system base vectors of the covariance ellipse.
     * 
     * @param pMarkerId An unique id for the marker. Any marker sent with the same id will overwrite the old one.
     * @param pAxis The number of the axis that the message should be created for.
     * @param pMean The mean vector of the kernel that should be visualized.
     * @param pCovariance The covariance matrix of the kernel that should be visualized.
     * @return A marker array holding the visualization messages.
     */
    visualization_msgs::Marker generateAxis(const unsigned int pMarkerId,
					     const unsigned int pAxis,
					     const Eigen::Vector3d& pMean,
					     const Eigen::Matrix3d& pCovariance);
    
    /**
     * Calculate cross sections of coordinate planes passing through ellipsoid center and having one of the axes of the frame of the ellipsoid as normal with a three dimensional ellipsoid itself. Approximate them with polgygons and return one line strips for each of the ellipses.
     * 
     * @param pEllipsoidHalfAxes Discrete lengths of the ellipsoid axes in x/y/z order relative to pose of ellipsoid.
     * @param pDiscretizationResolution Discrete angle between the subsequent polyline vertices in degrees. It defines the approximation accuracy.
     * @param pEllipsePolygons Contour polygons (middle) containing discrete 3D points (inner) approximating ellipses in the three (outer vector) coordinate planes with normals in x/y/z order.
     */
    void calculateCoordinatePlaneEllipses(const std::vector<int>& pEllipsoidHalfAxes, const int pDiscretizationResolution, std::vector<std::vector<std::vector<int> > >& pEllipsePolygons);
    
    /**
     * Generates the visualization message for a covariance ellipse, the rings and the coordinate system.
     * 
     * @param pMarkerId An unique id for the marker. Any marker sent with the same id will overwrite the old one.
     * @param pMarkerShape Should be either visualization_msgs::Marker::SPHERE for the ellipsoid or LINE_STRIP for the rings.
     * @param pWireFramePolygons Contour polygons (outer) containing 3D points (inner) approximating ellipses in one coordinate planewith normal in x/y/z order. Assign emtpy vector if surface model should be generated.
     * @param pColor The r/g/b/a color for the marker.
     * @param pMean The mean vector of the kernel that should be visualized.
     * @param pCovariance The covariance matrix of the kernel that should be visualized.
     * @return A marker array holding the visualization messages.
     */
    visualization_msgs::Marker generateCovarianceEllipsoidMarker(const unsigned int pMarkerId,
								  const int pMarkerShape,
								  const std::vector<std::vector<float> >& pWireFramePolygons,
								  const std_msgs::ColorRGBA& pColor,
								  const Eigen::Vector3d& pMean,
								  const Eigen::Matrix3d& pCovariance);
    
    /**
     * Create a visualization marker message that contains all samples.
     * 
     * @param pMarkerId Serves as unique id for the marker. Any marker sent with the same id will overwrite the old one.
     * @param pSamples The list of all samples.
     * @return A marker array holding the visualization messages.
     */
    visualization_msgs::MarkerArray generateSamplesMarker(const unsigned int pMarkerId, const std::vector<boost::shared_ptr<Eigen::Quaternion<double> > >& pSamples);
    
    /**
     * Generates an arrow between two points.
     * 
     * @param pMarkerId Serves as unique id for the marker. Any marker sent with the same id will overwrite the old one.
     * @param pFrom The position to start the link from.
     * @param pTo The position to end the link.
     */
    visualization_msgs::Marker generateArrowMessage(unsigned int& pMarkerId, Eigen::Vector3d pFrom, Eigen::Vector3d pTo);
    
    /**
     * Generates a small spherical marker at the given position.
     * 
     * @param pMarkerId Serves as unique id for the marker. Any marker sent with the same id will overwrite the old one.
     * @param pPos The position of the marker.
     */
    visualization_msgs::Marker generateSphereMessage(unsigned int& pMarkerId, Eigen::Vector3d pPos);
    
    /**
     * Converts a 3D std::vector<T> of arbitrary compatible type T into a geometry_msgs/Point.msg from ROS.
     *
     * @param pPositionVector STL representation of vector consisting of concatenated cartesian coordinates.
     * @param pROSPositionMsg ROS message containing position information as cartesian point in 3D space.
     * @return A point message holding the content of the vector.
     */
    template <class T> void stlVectorToPointMsg(const std::vector<T> pPositionVector, geometry_msgs::Point& pROSPositionMsg);
    
  private:
    
    /**
     * Lengths of half-axes of the ellipsoid are pSigmaMultiplicator times the standard deviations associated to the coordinate axes of the ellipsoid.
     */
    float mSigmaMultiplicator;
    
    /**
     * Absolute pose of the parent object.
     */
    boost::shared_ptr<ISM::Pose> mParentPose;
  };
}
