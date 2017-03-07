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
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

namespace Visualization
{
  /**
   * Visualizer class for 3D gaussian normal distributions.
   *
   * @author Joachim Gehrung, Pascal Meissner
   * @version See SVN
   */
  class GaussianVisualization
  {
  public:
    
    /**
     * Constructor.
     */
    GaussianVisualization();
    
    /**
     * Destructor.
     */
    ~GaussianVisualization();
    
    /**
     * Setter function for the mean vector and covariance matrix of the kernel.
     * 
     * @param pCovariance A 3x3 eigen matrix representing the corvariance matrix of the gaussian kernel.
     * @param pMean A 3d eigen vector representing the mean of the gaussian kernel.
     */
    void setKernel(const boost::shared_ptr<Eigen::Vector3d>& pMean, const boost::shared_ptr<Eigen::Matrix3d>& pCovariance);
    
    /**
     * The covariance ellipses tend to be very small, therefore a scaling factor shall be applied to it.
     * 
     * @param pScale Factor to multiply the kernel with.
     */
    void setScaleFactor(double pScale);
    
    /**
     * Setter function for mObjectLocationFrameID.
     * 
     * @param pObjectLocationFrameID Identifier for coordinate frame relative to which visualized covariance ellipsoid are defined.
     */
    void setObjectLocationFrameID(const std::string& pObjectLocationFrameID);
    
    /**
     * Setter function for mAlphaChannel.
     * 
     * @param pAlphaChannel Sets transparency of visualized covariance ellipsoid.
     */
    void setAlphaChannel(const float pAlphaChannel);
    
    /**
     * Setter function for mSigmaMultiplicator.
     * 
     * @param pSigmaMultiplicator Scaling factor for the size of the visualized covariance ellipsoid.
     */
    void setSigmaMultiplicator(const float pSigmaMultiplicator);
    
    /**
     * Calculates covariance ellipsoid from a parametric description of an nD normal distribution. Parameterizes a spherical shaped marker message using mean of distribution, calculated ellipsoid and other parameters of this function. Sends marker message under topic set by publisher argument of function to visualize a hull for some distribution samples.
     *
     * @param pMarkerID Serves as unique id for the marker. Any marker sent with the same id will overwrite the old one.
     * @param pEllipsoidColor Color of the ellipsoid. Specified as r/g/b/a vector, with values in the range of [0, 1].
     */
    void publishCovarianceEllipsoidMarker(const int pMarkerID, const std_msgs::ColorRGBA& pEllipsoidColor);
      
    /**
     * Calculates covariance ellipsoid from a parametric description of an nD normal distribution. Parameterizes a spherical shaped marker message using mean of distribution, calculated ellipsoid and other parameters of this function.
     *
     * @param pMarkerID Serves as unique id for the marker. Any marker sent with the same id will overwrite the old one.
     * @param pMarkerShape Should be either visualization_msgs::Marker::SPHERE for surface model or LINE_STRIP for wire-frame model.
     * @param pWireFramePolygons Contour polygons (middle) containing 3D points (inner) approximating ellipses in the three (outer vector) coordinate planes with normals in x/y/z order. Assign emtpy vector if surface model should be generated.
     * @param pWireFrameWidth Sets the width of the lines representing wire-frame. Ignored if surface model is desired.
     * @param pEllipsoidColor Color of the ellipsoid. Specified as r/g/b/a vector, with values in the range of [0, 1].
     * @return RViz marker message ready for being published.
     */
    boost::shared_ptr<visualization_msgs::Marker> generateCovarianceEllipsoidMarker(const int pMarkerID, const int pMarkerShape, const std::vector<std::vector<std::vector<float> > >& pWireFramePolygons, const float& pWireFrameWidth, const std_msgs::ColorRGBA& pEllipsoidColor);
    
    /**
     * Visualizes the eigenvectors of the covariance matrix which are also the coordinate system of the covariance ellipse.
     *
     * @param pMarkerID Serves as unique id for the marker. Any marker sent with the same id will overwrite the old one.
     */
    void publishCovarianceEllipsoidCoordinateSystemMarker(const int pMarkerID);
    
    /**
     *  Create a visualization marker message that contains an arrow pointing perpendicular to the given point and publishes it.
     *
     * @param pMarkerID Serves as unique id for the marker. Any marker sent with the same id will overwrite the old one.
     * @param pColor Color of the arrow. Specified as r/g/b/a vector, with values in the range of [0, 1].
     * @param pPoint The point to point to ;).
     */
    void publishArrowToPoint(const int pMarkerID, const std_msgs::ColorRGBA& pColor, const boost::shared_ptr<Eigen::Quaternion<double> > pPoint);
    
    /**
     * Create a visualization marker message that contains an arrow pointing perpendicular to the given point.
     * 
     * @param pMarkerID Serves as unique id for the marker. Any marker sent with the same id will overwrite the old one.
     * @param pColor Color of the arrow. Specified as r/g/b/a vector, with values in the range of [0, 1].
     * @param pPoint The point to point to ;).
     */
    visualization_msgs::Marker generateArrowToPoint(const int pMarkerID, const std_msgs::ColorRGBA& pColor, const boost::shared_ptr<Eigen::Quaternion<double> > pPoint);
    
    /**
     *  Create a visualization marker message that contains the point cloud.
     *
     * @param pMarkerID Serves as unique id for the marker. Any marker sent with the same id will overwrite the old one.
     * @param pCloud The point cloud to visualize.
     */
    void publishPointCloud(const int pMarkerID, const std::vector<boost::shared_ptr<Eigen::Quaternion<double> > > pCloud);
    
    /**
     * Create a visualization marker message that contains a point of the point cloud.
     * 
     * @param pMarkerID Serves as unique id for the marker. Any marker sent with the same id will overwrite the old one.
     * @param pId The number of the point in the cloud.
     * @param pPoint The point to visualize.
     */
    visualization_msgs::Marker generatePointCloudPoint(const int pMarkerID, const unsigned int pId, const boost::shared_ptr<Eigen::Quaternion<double> >& pPoint);
    
    /**
     * Converts a 3D std::vector<T> of arbitrary compatible type T into a geometry_msgs/Point.msg from ROS.
     *
     * @param pPositionVector STL representation of vector consisting of concatenated cartesian coordinates.
     * @param pROSPositionMsg ROS message containing position information as cartesian point in 3D space.
     */
    template <class T> void stlVectorToPointMsg(const std::vector<T> pPositionVector, geometry_msgs::Point& pROSPositionMsg);
    
  private:
    
    /**
     * Node handle for inintializing the publisher.
     */
    ros::NodeHandle mHandle;
    
    /**
     * Publisher for the visualization messages containing the covariance ellipses and position marker for observed objects.
     */
    boost::shared_ptr<ros::Publisher> mPublisher;
    
    /**
     * The factor to multiply the kernel with to make it look bigger.
     */
    double mScale;
    
    /**
     * The mean vector of the multivariate gaussian.
     */
    boost::shared_ptr<Eigen::Vector3d> mMean;
    
    /**
     * The covariance matrix of the multivariate gaussian.
     */
    boost::shared_ptr<Eigen::Matrix3d> mCovariance;
    
    /**
     * ROS tf coordinate frame relative to which all processed pose information shall be interpreted.
     */
    std::string mObjectLocationFrameID;
    
    /**
     * Degree of transparency of ellipsoid visualized in rviz. Specified with values in the range of [0, 1] standing for increasing opacity.
     */
    float mAlphaChannel;
    
    /**
     * Lengths of half-axes of the ellipsoid are pSigmaMultiplicator times the standard deviations associated to the coordinate axes of the ellipsoid.
     */
    float mSigmaMultiplicator;
  };
}
