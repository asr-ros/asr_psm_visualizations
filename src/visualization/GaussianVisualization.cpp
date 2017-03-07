/**

Copyright (c) 2016, Gehrung Joachim, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "visualization/GaussianVisualization.h"

namespace Visualization {
	
  GaussianVisualization::GaussianVisualization()
  : mHandle()
  , mScale(1.0)
  {
    mPublisher.reset(new ros::Publisher(mHandle.advertise<visualization_msgs::Marker>("psm_scene_objects", 1)));
  }
	
  GaussianVisualization::~GaussianVisualization()
  {
  }
  
  void GaussianVisualization::setKernel(const boost::shared_ptr<Eigen::Vector3d>& pMean, const boost::shared_ptr<Eigen::Matrix3d>& pCovariance)
  {
    mMean = pMean;
    mCovariance = pCovariance;
  }
  
  void GaussianVisualization::setScaleFactor(double pScale)
  {
    mScale = pScale;
  }
  
  void GaussianVisualization::setObjectLocationFrameID(const std::string& pObjectLocationFrameID)
  {
    mObjectLocationFrameID = pObjectLocationFrameID;
  }
  
  void GaussianVisualization::setAlphaChannel(const float pAlphaChannel)
  {
    mAlphaChannel = pAlphaChannel;
  }
  
  void GaussianVisualization::setSigmaMultiplicator(const float pSigmaMultiplicator)
  {
    // Check that we validly scale variances in frame resulting from eigenvalue decomposition.
    if(pSigmaMultiplicator <= 0.0f)
      throw std::out_of_range("Dimensions of a geometric primitive must be non-negative.");
    
    // Init ellisoid scaling with provided data.
    mSigmaMultiplicator = pSigmaMultiplicator;
  }
  
  void GaussianVisualization::publishCovarianceEllipsoidMarker(const int pMarkerID, const std_msgs::ColorRGBA& pEllipsoidColor)
  {
    // Call to advertise() should not have returned an empty publisher.
    if(!mPublisher)
      throw std::invalid_argument("Empty ros::publisher cannot be used to publish marker messages.");

    // Generate marker for given distribution and publish it on given topic.
    mPublisher->publish((*generateCovarianceEllipsoidMarker(pMarkerID, visualization_msgs::Marker::SPHERE, std::vector<std::vector<std::vector<float> > >(), 0, pEllipsoidColor)));
  }
  
  boost::shared_ptr<visualization_msgs::Marker> GaussianVisualization::generateCovarianceEllipsoidMarker(const int pMarkerID, const int pMarkerShape, const std::vector<std::vector<std::vector<float> > >& pWireFramePolygons, const float& pWireFrameWidth, const std_msgs::ColorRGBA& pEllipsoidColor)
  {
    if(pMarkerShape != visualization_msgs::Marker::SPHERE && pMarkerShape != visualization_msgs::Marker::LINE_LIST)
      throw std::invalid_argument("Chosen marker shape is not supported to visualize.");

    // All color parameter values must be in [0,1].
    if((pEllipsoidColor.r < 0.0f || pEllipsoidColor.r > 1.0f) || (pEllipsoidColor.g < 0.0f || pEllipsoidColor.g > 1.0f) || (pEllipsoidColor.b < 0.0f || pEllipsoidColor.b > 1.0f) || (pEllipsoidColor.a < 0.0f || pEllipsoidColor.a > 1.0f))
      throw std::out_of_range("ColorRGBA message is no valid color information.");
    
    // Data structure representing visualization object.
    boost::shared_ptr<visualization_msgs::Marker> ellipsoidMarker(new visualization_msgs::Marker);     
    
    // Use the learners frame_id relative to which all locations in this model are given.
    ellipsoidMarker->header.frame_id = mObjectLocationFrameID;
    
    // Set timestamp. See the TF tutorials for information on this.
    ellipsoidMarker->header.stamp = ros::Time::now();
    
    // Marker namespace are associated to different types of shapes used to represent object location distributions.
    ellipsoidMarker->ns = "distribution";
    
    // Markers with different namespaces but same id represent same object location distribution with different shapes.
    ellipsoidMarker->id = pMarkerID;
    
    // The marker type decides on shape of marker and accordingly how marker parameters are interpreted by RViz.
    ellipsoidMarker->type = pMarkerShape;
    
    // We always want to show a marker by publishing ros messages here.
    ellipsoidMarker->action = visualization_msgs::Marker::ADD;

    // Columns of eigen matrix are eigen vectors representing the axes of the coordinate frame of the ellipsoid. Therefore eigen matrix is equal to an orientation matrix of the ellipsoid. This solver calculates the eigen matrix and vectors.
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(*mCovariance);
    Eigen::Matrix3d rotationMatrixOrientation = es.eigenvectors();
    
    // The position of the ellipsoid is equal to the mean vector of the normal distribution.
    tf::Vector3 tfEllipsoidPosition((*mMean)[0], (*mMean)[1], (*mMean)[2]);
    
    // TODO take care of
    // Transform ellipsoid orientation matrix to ros tf representation to be able to transform it to a quaternion.
    // Invert first and second eigen vectors as they do not correspond to the eigen vector calculated by wolfram alpha.
//     tf::Matrix3x3 tfRotationMatrixOrientation(-1.0 * rotationMatrixOrientation(0,0), -1.0 * rotationMatrixOrientation(0,1), rotationMatrixOrientation(0,2),
// 					       -1.0 * rotationMatrixOrientation(1,0), -1.0 * rotationMatrixOrientation(1,1), rotationMatrixOrientation(1,2),
// 					       -1.0 * rotationMatrixOrientation(2,0), -1.0 * rotationMatrixOrientation(2,1), rotationMatrixOrientation(2,2));
    
    tf::Matrix3x3 tfRotationMatrixOrientation(rotationMatrixOrientation(0,0), rotationMatrixOrientation(0,1), rotationMatrixOrientation(0,2),
					       rotationMatrixOrientation(1,0), rotationMatrixOrientation(1,1), rotationMatrixOrientation(1,2),
					       rotationMatrixOrientation(2,0), rotationMatrixOrientation(2,1), rotationMatrixOrientation(2,2));
    
    // Get transform needed for conversion to ros pose msg representation.
    tf::Transform tfEllipsoidPose(tfRotationMatrixOrientation, tfEllipsoidPosition);
    
    // Fill marker pose with tf transform.
    tf::poseTFToMsg(tfEllipsoidPose, ellipsoidMarker->pose);
    
    // Scale position.
    ellipsoidMarker->pose.position.x *= mScale;
    ellipsoidMarker->pose.position.y *= mScale;
    ellipsoidMarker->pose.position.z *= mScale;
    
    // Extract eigen values of covariance matrix which represent the relative scaling factors for the axes of the coordinate frame of the ellipoid extracted above or the variances in the diagonalized matrix.
    Eigen::Vector3d squaredEllipsoidLengths = es.eigenvalues();

    // If surface model of ellipsoid is desired, set size of ellipsoid directly in marker using scale parameter
    if(pMarkerShape == visualization_msgs::Marker::SPHERE)
    {
      // Multiply the relative lengths (square roots of corresponding eigen values) of the half axes of the ellipse with a multiplicator parameter set for visualization purposes. Then double value as scale does not represent radius, but diameter of the marker.
      ellipsoidMarker->scale.x = 2.0f * std::sqrt(squaredEllipsoidLengths[0]) * mSigmaMultiplicator * mScale;
      ellipsoidMarker->scale.y = 2.0f * std::sqrt(squaredEllipsoidLengths[1]) * mSigmaMultiplicator * mScale;
      ellipsoidMarker->scale.z = 2.0f * std::sqrt(squaredEllipsoidLengths[2]) * mSigmaMultiplicator * mScale;
    }
    // Otherwise a wire-frame model is desired. In this case its size as well as its general appeareance is encoded in the points constituting the wire-frame polygons.
    else {
      
      // Access one of the ellipse-shaped wires making up the wire-frame.
      std::vector<std::vector<std::vector<float> > >::const_iterator ellipseIterator;

      // Go through all ellipses
      for(ellipseIterator = pWireFramePolygons.begin(); ellipseIterator != pWireFramePolygons.end(); ellipseIterator++) {
	
	// Access a point on the ellipse-shaped polygon.
	std::vector<std::vector<float> >::const_iterator polygonPointIterator = ellipseIterator->begin();

	// Geometry_msgs representation of 3D point for polygon point...
	geometry_msgs::Point rosPolygonPoint;
	
	// ...is needed for insertion into marker message.
	stlVectorToPointMsg(*polygonPointIterator, rosPolygonPoint);

	// Add starting of first segment of ellipse to points array containing all segments as pairs of starting- and endpoints.
	ellipsoidMarker->points.push_back(rosPolygonPoint);

	// Go through all points in the polygon ignoring first try.
	for(polygonPointIterator++; polygonPointIterator != ellipseIterator->end(); polygonPointIterator++) {

	  // Add end point of segment already being processed.
	  stlVectorToPointMsg(*polygonPointIterator, rosPolygonPoint);
	  ellipsoidMarker->points.push_back(rosPolygonPoint);

	  // Add starting point of next segment
	  ellipsoidMarker->points.push_back(rosPolygonPoint);

	}
	// Close the loop with the polygon segment [end(),begin()].
	stlVectorToPointMsg(*(ellipseIterator->begin()), rosPolygonPoint);
	ellipsoidMarker->points.push_back(rosPolygonPoint);
	
      }
      // Nevertheless the width of the wires is set solely using x component of scale parameter.
      ellipsoidMarker->scale.x = pWireFrameWidth;

    }

    // Set the color of the ellipsoid for rviz.
    ellipsoidMarker->color = pEllipsoidColor;
        
    // Overwrite alpha channel.
    ellipsoidMarker->color.a = mAlphaChannel;

    // How long the object should last before being automatically deleted. (Here forever.)
    ellipsoidMarker->lifetime = ros::Duration();

    // Give others access to completed covariance ellipsoid.
    return ellipsoidMarker;
  }
  
  void GaussianVisualization::publishCovarianceEllipsoidCoordinateSystemMarker(const int pMarkerID)
  {
    // Columns of eigen matrix are eigen vectors representing the axes of the coordinate frame of the ellipsoid. Therefore eigen matrix is equal to an orientation matrix of the ellipsoid. This solver calculates the eigen matrix and vectors.
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(*mCovariance);
    
    // Visualize the coordinate system as arrows.
    for(int i = 0; i < 3; i++)
    {
      visualization_msgs::Marker msg;

      // Get the eigenvector.
      Eigen::Vector3d eigenvector = es.eigenvectors().col(i) * es.eigenvalues()[i];
      
      // Set time and frame id.
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = mObjectLocationFrameID;

      // Arrows should be small.
      msg.scale.x = 0.005;
      msg.scale.y = 0.005;
      msg.scale.z = 0.005;

      // This should be an arrow.
      msg.type = visualization_msgs::Marker::ARROW;
      
      // We always want to show a marker by publishing ros messages here.
      msg.action = visualization_msgs::Marker::ADD;

      // Create the starting point.
      geometry_msgs::Point start;
      start.x = (*mMean)[0] * mScale;
      start.y = (*mMean)[1] * mScale;
      start.z = (*mMean)[2] * mScale;
      msg.points.push_back(start);
      
      // Create the point to point to ;).
      geometry_msgs::Point stop;
      stop.x = ((*mMean)[0] + eigenvector[0]) * mScale;
      stop.y = ((*mMean)[1] + eigenvector[1]) * mScale;
      stop.z = ((*mMean)[2] + eigenvector[2]) * mScale;
      msg.points.push_back(stop);
      
      // Set an unique id and namespace for every eigenvector.
      msg.id = i;
      msg.ns = "distribution_frame/" + boost::lexical_cast<std::string>(pMarkerID);
      
      // Set color based on the loop index to get a RGB coordinate frame.
      msg.color.r = (i     == 0) ? 1.0 : 0.0;
      msg.color.g = (i - 1 == 0) ? 1.0 : 0.0;
      msg.color.b = (i - 2 == 0) ? 1.0 : 0.0;
      msg.color.a = 1.0;

      // Publish eigenvector message.
      mPublisher->publish(msg);
    }
  }
  
  void GaussianVisualization::publishArrowToPoint(const int pMarkerID, const std_msgs::ColorRGBA& pColor, const boost::shared_ptr<Eigen::Quaternion<double> > pPoint)
  {
    // Call to advertise() should not have returned an empty publisher.
    if(!mPublisher)
      throw std::invalid_argument("Empty ros::publisher cannot be used to publish marker messages.");

    // Generate marker for given point and publish it on given topic.
    mPublisher->publish(generateArrowToPoint(pMarkerID, pColor, pPoint));
  }
  
  visualization_msgs::Marker GaussianVisualization::generateArrowToPoint(const int pMarkerID, const std_msgs::ColorRGBA& pColor, const boost::shared_ptr<Eigen::Quaternion<double> > pPoint)
  {
    // All color parameter values must be in [0,1].
    if((pColor.r < 0.0f || pColor.r > 1.0f) || (pColor.g < 0.0f || pColor.g > 1.0f) || (pColor.b < 0.0f || pColor.b > 1.0f) || (pColor.a < 0.0f || pColor.a > 1.0f))
      throw std::out_of_range("ColorRGBA message is no valid color information.");
    
    visualization_msgs::Marker msg;

    // Set time and frame id.
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = mObjectLocationFrameID;

    // Scale it to be very small and long.
    msg.scale.x = 0.04;
    msg.scale.y = 0.1;
    msg.scale.z = 0.1;

    // This should be an arrow.
    msg.type = visualization_msgs::Marker::ARROW;
    
    // Namespace will represent the nature of the message.
    // Yeah, it's really hard to find comments for every line...
    msg.ns = "evidence";
    
    // Markers with different namespaces but same id represent same object location distribution with different shapes.
    // Haha, I stole the comment from above and nobody realized ;)!
    msg.id = pMarkerID;
    
    // We always want to show a marker by publishing ros messages here.
    msg.action = visualization_msgs::Marker::ADD;
    
    // Create the starting point.
    geometry_msgs::Point start;
    start.x = pPoint->x() * mScale;
    start.y = pPoint->y() * mScale;
    start.z = pPoint->z() * mScale + 0.5;
    msg.points.push_back(start);
    
    // Create the point to point to ;).
    geometry_msgs::Point stop;
    stop.x = pPoint->x() * mScale;
    stop.y = pPoint->y() * mScale;
    stop.z = pPoint->z() * mScale;
    msg.points.push_back(stop);

    // Set the color of the arrow for rviz.
    msg.color = pColor;
    
    return msg;
  }
  
  void GaussianVisualization::publishPointCloud(const int pMarkerID, const std::vector<boost::shared_ptr<Eigen::Quaternion<double> > > pCloud)
  {
    // Call to advertise() should not have returned an empty publisher.
    if(!mPublisher)
      throw std::invalid_argument("Empty ros::publisher cannot be used to publish marker messages.");

    // Generate point cloud and publish it on given topic.
    for(unsigned int i = 0; i < pCloud.size(); i++)
      mPublisher->publish(generatePointCloudPoint(pMarkerID, i, pCloud[i]));
  }
  
  visualization_msgs::Marker GaussianVisualization::generatePointCloudPoint(const int pMarkerID, const unsigned int pId, const boost::shared_ptr<Eigen::Quaternion<double> >& pPoint)
  {
    visualization_msgs::Marker msg;

    // Set time and frame id.
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = mObjectLocationFrameID;

    // Namespace will represent the nature of the message.
    msg.ns = "samples/" + boost::lexical_cast<std::string>(pMarkerID);
    
    // Every sample has its own id.
    msg.id = pId;

    // Point is represented by a sphere.
    msg.type = visualization_msgs::Marker::SPHERE;
    
    // We always want to show a marker by publishing ros messages here.
    msg.action = visualization_msgs::Marker::ADD;

    // Set the position of the sphere based on the current point and apply visualization scale factor.
    msg.pose.position.x = pPoint->x() * mScale;
    msg.pose.position.y = pPoint->y() * mScale;
    msg.pose.position.z = pPoint->z() * mScale;

    // We don't need somespacial orientation.
    msg.pose.orientation.x = 1;
    msg.pose.orientation.y = 1;
    msg.pose.orientation.z = 1;
    msg.pose.orientation.w = 1;

    // Sphere should be very small.
    msg.scale.x = 0.005;
    msg.scale.y = 0.005;
    msg.scale.z = 0.005;

    // And black! We love black!
    msg.color.a = 1.0;
    msg.color.r = 0.0;
    msg.color.g = 0.0;
    msg.color.b = 0.0;

    // Return the point
    return msg;
  }
  
  template <class T> void GaussianVisualization::stlVectorToPointMsg(const std::vector<T> pPositionVector, geometry_msgs::Point& pROSPositionMsg)
  {
    //pPositionVector must represent a position to be converted.
    if(pPositionVector.size() != 3) 
      throw std::invalid_argument("pPositionVector is not a position in terms of dimensionality.");

    //Copy position information per dimension.
    pROSPositionMsg.x = pPositionVector.at(0);
    pROSPositionMsg.y = pPositionVector.at(1);
    pROSPositionMsg.z = pPositionVector.at(2);
  }
  
}
