/**

Copyright (c) 2016, Gehrung Joachim, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "visualization/psm/helper/GaussianKernelVisualizer.h"

namespace Visualization {
  
  GaussianKernelVisualizer::GaussianKernelVisualizer()
  : AbstractExtendedVisualizer()
  {
  }
  
  GaussianKernelVisualizer::~GaussianKernelVisualizer()
  {
  }
  
  void GaussianKernelVisualizer::setSigmaMultiplicator(const float pSigmaMultiplicator)
  {
    mSigmaMultiplicator = pSigmaMultiplicator;
  }
  
  void GaussianKernelVisualizer::setParentPose(boost::shared_ptr<ResourcesForPsm::Pose> pPose)
  {
    // Check, if pointer is valid.
    if(!pPose)
      throw std::out_of_range("Invalid pointer: absolute pose of primary scene object.");
    
    // Set absolute position of the primary scene object.
    mParentPose = pPose;
  }
  
  void GaussianKernelVisualizer::publishKernel(boost::shared_ptr<ros::Publisher> pPublisher,
					       unsigned int& pMarkerId,
					       const Eigen::Vector3d& pMean,
					       const Eigen::Matrix3d& pCovariance)
  {
    // Check, if publisher is available.
    if(!pPublisher)
      throw std::invalid_argument("Empty ros::publisher cannot be used to publish marker messages.");
    
    // Marker array that will contain the visualization messages for the covariance ellipse and the rings.
    visualization_msgs::MarkerArray modelMarkers;
    
    /************************************
     * Create surface model.
     ************************************/

    // Check for errors with non 3D-positions.
    try {
      // Light gray color for ellipse because we already use color codes in the rings.
      std_msgs::ColorRGBA pEllipseColor;
      pEllipseColor.r = pEllipseColor.g = pEllipseColor.b = 0.45;
      pEllipseColor.a = 0.33;
      
      // Generate marker message that differs for other object locations in same scene by color. 
      modelMarkers.markers.push_back(generateCovarianceEllipsoidMarker(pMarkerId++, visualization_msgs::Marker::SPHERE, std::vector<std::vector<float> >(), pEllipseColor, pMean, pCovariance));
    }
    catch(std::exception& exception) {
      std::cerr << exception.what() << std::endl;
    }
    
    /************************************
     * Create wire frame model (rings).
     ************************************/
    
    // Wire-frame model of ellipsoid expressed by multiple closen 3D point sequences as integers in [mm].
    std::vector<std::vector<std::vector<int> > > integerWireFrame;

    // Access one of the ellipse-shaped wires making up the wire-frame.
    std::vector<std::vector<std::vector<int> > >::iterator wireIterator;
    
    // Access one of the points in the polygon representing a wire in the frame.
    std::vector<std::vector<int> >::iterator pointInterator;
    
    // Access one of the coordinates of a point.
    std::vector<int>::iterator coordinatesIterator;

    // Calculation of ellipse polygon is based on integers, so we express size of ellipsoid that way.
    std::vector<int> ellipsoidHalfAxes;

    // Transform dimensions of ellipsoid back to half-axes and scale from unit usually used in ROS [m] to [mm].
    ellipsoidHalfAxes.push_back(static_cast<int>((0.5f * modelMarkers.markers[0].scale.x * 1000.0f) / mSigmaMultiplicator));
    ellipsoidHalfAxes.push_back(static_cast<int>((0.5f * modelMarkers.markers[0].scale.y * 1000.0f) / mSigmaMultiplicator));
    ellipsoidHalfAxes.push_back(static_cast<int>((0.5f * modelMarkers.markers[0].scale.z * 1000.0f) / mSigmaMultiplicator));
    
    // Calculate integer polygon points for the wire-frame to be visualized. Discretization is hard-coded to 1 degree as this is as we don't care.
    calculateCoordinatePlaneEllipses(ellipsoidHalfAxes, 5, integerWireFrame);

    // Wire-frame model of ellipsoid expressed by multiple closen 3D point sequences as floating points in [m].
    std::vector<std::vector<std::vector<float> > > floatWireFrame;

    // Go through all wires.
    for(wireIterator = integerWireFrame.begin(); wireIterator != integerWireFrame.end(); wireIterator++)
    {
      // One wire of the frame scaled down to [m] newly created for conversion from integer to float.
      std::vector<std::vector<float> > pointSequence;

      // Go through all points of a polygon.
      for(pointInterator = wireIterator->begin(); pointInterator != wireIterator->end(); pointInterator++)
      {
	// One point of the wire scaled down to [m] newly created for conversion from integer to float.
	std::vector<float> point;

	// Go through all coordinates of a point.
	for(coordinatesIterator = pointInterator->begin();coordinatesIterator != pointInterator->end(); coordinatesIterator++)
	{
	  // Here real conversion of data types and scaling to [m] takes place.
	  point.push_back(static_cast<float>(*coordinatesIterator) / 1000.0f);
	}
	
	// Add floating point polygon point to already calculated elements of wire.
	pointSequence.push_back(point);
      }
      
      // Add newly calculated wire to floating point frame.
      floatWireFrame.push_back(pointSequence);
    }
    
    // Check for errors with non 3D-positions.
    try {
      double factor = 1.0;
      std_msgs::ColorRGBA pRingColor;
      pRingColor.r = getColor(0).r * factor;
      pRingColor.g = getColor(0).g * factor;
      pRingColor.b = getColor(0).b * factor;
      pRingColor.a = 1.0;
      
      // Generate all marker messages. Width of wires is set to five [mm] as this seems plausible and we don't think people should care about.
      for(unsigned int i = 0; i < floatWireFrame.size(); i++)
      {
	// Create the marker containing the ring.
	visualization_msgs::Marker marker = generateCovarianceEllipsoidMarker(pMarkerId++, visualization_msgs::Marker::LINE_LIST, floatWireFrame[i], pRingColor, pMean, pCovariance);
	
	// Add marker to marker array.
	modelMarkers.markers.push_back(marker);
      }
    } catch(std::exception& exception) {
      std::cerr << exception.what() << std::endl;
    }
    
    /************************************
     * Create coordinate system.
     ************************************/
    
    // Generate all three axis of the covariance ellipse coordinate system.
//     for(unsigned int i = 0; i < 3; i++)
//       modelMarkers.markers.push_back(generateAxis(pMarkerId++, i, pMean, pCovariance));
    
    /************************************
     * Create arrow from the parent position to the kernel.
     ************************************/
    modelMarkers.markers.push_back(generateArrowMessage(pMarkerId, mParentPose->getPosition(), mParentPose->getPosition() + mParentPose->getOrientation().toRotationMatrix() * pMean));
    modelMarkers.markers.push_back(generateSphereMessage(pMarkerId, mParentPose->getPosition()));
    
    /************************************
     * Publish the results.
     ************************************/
    
    // Publish the marker array.
    pPublisher->publish(modelMarkers);
  }
  
  void GaussianKernelVisualizer::calculateCoordinatePlaneEllipses(const std::vector<int>& pEllipsoidHalfAxes,
								  const int pDiscretizationResolution,
								  std::vector<std::vector<std::vector<int> > >& pEllipsePolygons)
  {
    // Check whether we got size values for all three dimensions.
    if(pEllipsoidHalfAxes.size() != 3)
      throw std::length_error("Only supporting three dimensional ellipsoids at the moment, not " + boost::lexical_cast<std::string>(pEllipsoidHalfAxes.size()) + "D.");

    // Make sure no points remain from other calculations.
    pEllipsePolygons.clear();

    // Vector containing half axis indices of ellipses for each possible coordinate plane.
    std::vector<cv::Point> coordinatePlaneHalfAxisIndices;

    // First use x axis as normal of cross secting plane and choose ellipsoid half axes in that plane as ellipse half axes.
    coordinatePlaneHalfAxisIndices.push_back(cv::Point(1, 2));
    
    // Then y axis.
    coordinatePlaneHalfAxisIndices.push_back(cv::Point(0, 2));
    
    // Then z axis.
    coordinatePlaneHalfAxisIndices.push_back(cv::Point(0, 1));

    // Go through all possible coordinate planes.
    for(unsigned int i = 0; i < coordinatePlaneHalfAxisIndices.size(); i++)
    {
      // Polygon representing ellipse (in coordinate plane currently taken into account) in 3D space.
      std::vector<std::vector<int> > threeDEllipsePolygon;

      // Two dimensional representation (in corresponding coordinate plane) of ellipse polygon.
      std::vector<cv::Point> twoDEllipsePolygon;

      // Multiply the half axes of the ellipsoid with a multiplicator parameter set for visualization purposes. Then assign 3D half axes to 2D half axes according to which coordinate plane is taken into account.
      cv::Size ellipseHalfAxes(static_cast<float>(pEllipsoidHalfAxes.at(coordinatePlaneHalfAxisIndices.at(i).x)) * mSigmaMultiplicator, static_cast<float>(pEllipsoidHalfAxes.at(coordinatePlaneHalfAxisIndices.at(i).y)) * mSigmaMultiplicator);

      // Calculate sequence of points representing entire non-rotated ellipse centered in origin lying in coordinate plane currently taken into account. Estimate half axes based on indices.
      cv::ellipse2Poly(cv::Point(0, 0), ellipseHalfAxes, 0, 0, 360, pDiscretizationResolution, twoDEllipsePolygon);     

      // Go through all 2D points constituting polygon.
      for(unsigned int j = 0; j < twoDEllipsePolygon.size(); j++) {

	// Create three dimensional point initialized with zeros as only two coordinates (in the coordinate plane) are going to be non-zero.
	std::vector<int> threeDPolygonPoint(3, 0);

	// X coordinate of polygon point corresponds to half axis coordinate assigned to x coordinate of coordinate plane.
	threeDPolygonPoint.at(coordinatePlaneHalfAxisIndices.at(i).x) = twoDEllipsePolygon.at(j).x;
	
	// Y coordinate of polygon point corresponds to half axis coordinate assigned to y coordinate of coordinate plane.
	threeDPolygonPoint.at(coordinatePlaneHalfAxisIndices.at(i).y) = twoDEllipsePolygon.at(j).y;

	// Complete 3D coordinate plane ellipse with another polygon point.
	threeDEllipsePolygon.push_back(threeDPolygonPoint);
      }
	
      // Add entire ellipse for one of the three coordinate planes to results container.
      pEllipsePolygons.push_back(threeDEllipsePolygon);
    }
  }
  
  visualization_msgs::Marker GaussianKernelVisualizer::generateCovarianceEllipsoidMarker(const unsigned int pMarkerId,
											 const int pMarkerShape, const std::vector<std::vector<float> >& pWireFramePolygons,
											 const std_msgs::ColorRGBA& pColor,
											 const Eigen::Vector3d& pMean,
											 const Eigen::Matrix3d& pCovariance)
  {
    if(pMarkerShape != visualization_msgs::Marker::SPHERE && pMarkerShape != visualization_msgs::Marker::LINE_LIST)
      throw std::invalid_argument("Chosen marker shape is not supported to visualize.");
    
    // The message that will contain the visualization marker.
    visualization_msgs::Marker msg;
    
    // Use the learners frame_id relative to which all locations in this model are given.
    msg.header.frame_id = getFrameId();
    
    // Set timestamp. See the TF tutorials for information on this.
    msg.header.stamp = ros::Time::now();
    
    // Marker namespace are associated to different types of shapes used to represent object location distributions.
    msg.ns = getNamespace();
    
    // Markers with different namespaces but same id represent same object location distribution with different shapes.
    msg.id = pMarkerId;
    
    // The marker type decides on shape of marker and accordingly how marker parameters are interpreted by RViz.
    // Here we choose a sphere.
    msg.type = pMarkerShape;
    
    // We always want to show a marker by publishing ros messages here.
    msg.action = visualization_msgs::Marker::ADD;
    
    // Columns of eigen matrix are eigen vectors representing the axes of the coordinate frame of the ellipsoid. Therefore eigen matrix is equal to an orientation matrix of the ellipsoid. This solver calculates the eigen matrix and vectors.
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(pCovariance);
    
    // Create the rotation matrix that is required for rotating the base vectors of the covariance ellipse into the world coordinate system.
    Eigen::Matrix3d rotationFromParentToWorld = mParentPose->getOrientation().toRotationMatrix();
    
    // Extract the tree base vectors and rotate them according to the relative orientation of this secondary scene object to the primary scene object.
    Eigen::Vector3d baseX = rotationFromParentToWorld * es.eigenvectors().col(0);
    Eigen::Vector3d baseY = rotationFromParentToWorld * es.eigenvectors().col(1);
    Eigen::Vector3d baseZ = rotationFromParentToWorld * es.eigenvectors().col(2);
    
    // We need to rotate the position of the covariance ellipse based on the orientation of the parent object.
    Eigen::Vector3d positionInParentFrame = rotationFromParentToWorld * pMean;
    
    // The position of the ellipsoid is equal to the mean vector of the normal distribution.
    Eigen::Vector3d position = mParentPose->getPosition();
    tf::Vector3 tfEllipsoidPosition(position[0] + positionInParentFrame[0],
				     position[1] + positionInParentFrame[1],
				     position[2] + positionInParentFrame[2]);
    
    // Calculate the cross product of the first and second eigenvector.
    Eigen::Vector3d cross = baseY.cross(baseX);

    // The new base coordinate system for the covariance ellipse.
    tf::Matrix3x3 tfRotationMatrixOrientation;
    
    // Extract eigen values of covariance matrix which represent the relative scaling factors for the axes of the coordinate frame of the ellipoid extracted above or the variances in the diagonalized matrix.
    Eigen::Vector3d squaredEllipsoidLengths;
    
    // Transform ellipsoid orientation matrix to ros tf representation to be able to transform it to a quaternion.
    // Based on wolfram alpha the first and second eigen vectors should be inverted. But the visual inspection (based on the learning samples
    // and the coordinate frame of the ellipse shows, that the right combination is e0, e1, -e2.
    // If it is not right hand, order is e0,e2,e21 Otherwise order is e0,e1,e2.
    if(std::signbit(cross[0]) != std::signbit(baseZ[0]) ||
       std::signbit(cross[1]) != std::signbit(baseZ[1]) ||
       std::signbit(cross[2]) != std::signbit(baseZ[2]))
    {
      tfRotationMatrixOrientation = tf::Matrix3x3(baseX[0], -baseZ[0], baseY[0],
						   baseX[1], -baseZ[1], baseY[1],
						   baseX[2], -baseZ[2], baseY[2]);
      
      squaredEllipsoidLengths = Eigen::Vector3d(es.eigenvalues()[0], es.eigenvalues()[2], es.eigenvalues()[1]);
      
    } else {
      tfRotationMatrixOrientation = tf::Matrix3x3(baseX[0], baseY[0], -baseZ[0],
						   baseX[1], baseY[1], -baseZ[1],
						   baseX[2], baseY[2], -baseZ[2]);
      
      squaredEllipsoidLengths = es.eigenvalues();
    }
      
    // Get transform needed for conversion to ros pose msg representation.
    tf::Transform tfEllipsoidPose(tfRotationMatrixOrientation, tfEllipsoidPosition);
    
    // Fill marker pose with tf transform.
    tf::poseTFToMsg(tfEllipsoidPose, msg.pose);
    
    // Scale position.
    msg.pose.position.x *= getScaleFactor();
    msg.pose.position.y *= getScaleFactor();
    msg.pose.position.z *= getScaleFactor();
    
    // Apply the given color and alpha value.
    msg.color = pColor;
    
    // If surface model of ellipsoid is desired, set size of ellipsoid directly in marker using scale parameter
    if(pMarkerShape == visualization_msgs::Marker::SPHERE)
    {
      // Multiply the relative lengths (square roots of corresponding eigen values) of the half axes of the ellipse with a multiplicator parameter set for visualization purposes. Then double value as scale does not represent radius, but diameter of the marker.
      msg.scale.x = 2.0f * std::sqrt(squaredEllipsoidLengths[0]) * getScaleFactor() * mSigmaMultiplicator;
      msg.scale.y = 2.0f * std::sqrt(squaredEllipsoidLengths[1]) * getScaleFactor() * mSigmaMultiplicator;
      msg.scale.z = 2.0f * std::sqrt(squaredEllipsoidLengths[2]) * getScaleFactor() * mSigmaMultiplicator;
    } else {
      
      // Access a point on the ellipse-shaped polygon.
      std::vector<std::vector<float> >::const_iterator polygonPointIterator = pWireFramePolygons.begin();

      // Geometry_msgs representation of 3D point for polygon point...
      geometry_msgs::Point rosPolygonPoint;
      
      // ...is needed for insertion into marker message.
      stlVectorToPointMsg(*polygonPointIterator, rosPolygonPoint);

      // Add starting of first segment of ellipse to points array containing all segments as pairs of starting- and endpoints.
      msg.points.push_back(rosPolygonPoint);

      // Go through all points in the polygon ignoring first try.
      for(polygonPointIterator++; polygonPointIterator != pWireFramePolygons.end(); polygonPointIterator++)
      {
	// Add end point of segment already being processed.
	stlVectorToPointMsg(*polygonPointIterator, rosPolygonPoint);
	msg.points.push_back(rosPolygonPoint);

	// Add starting point of next segment
	msg.points.push_back(rosPolygonPoint);
      }
      
      // Close the loop with the polygon segment [end(),begin()].
      stlVectorToPointMsg(*(pWireFramePolygons.begin()), rosPolygonPoint);
      msg.points.push_back(rosPolygonPoint);
	
      // Nevertheless the width of the wires is set solely using x component of scale parameter.
      msg.scale.x = 0.005;
    }
    
    // How long the object should last before being automatically deleted. (Here forever.)
    msg.lifetime = ros::Duration();
     
    return msg;
  }
  
  visualization_msgs::Marker GaussianKernelVisualizer::generateAxis(const unsigned int pMarkerId,
								     const unsigned int pAxis,
								     const Eigen::Vector3d& pMean,
								     const Eigen::Matrix3d& pCovariance)
  {
    visualization_msgs::Marker msg;
    
    // Columns of eigen matrix are eigen vectors representing the axes of the coordinate frame of the ellipsoid. Therefore eigen matrix is equal to an orientation matrix of the ellipsoid. This solver calculates the eigen matrix and vectors.
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(pCovariance);
    
    /***************************************************************************************************************************************************
     * Before we start we need to choose the second and third axis (based on the eigenvectors) so that they form a right hand coordinate system.
     * Therefore we calulate the cross product of the first and every other eigenvector.
     * The eigenvector, that creates a right hand coordinate system, is choosen.
     ***************************************************************************************************************************************************/
    
    // This will be the axis number to use.
    unsigned int axisForRightHandFrame = pAxis;
    
    // Extract the axes.
    Eigen::Vector3d firstAxis = es.eigenvectors().col(0) * std::sqrt(es.eigenvalues()[0]);
    Eigen::Vector3d secondAxis = es.eigenvectors().col(1) * std::sqrt(es.eigenvalues()[1]);
    Eigen::Vector3d thirdAxis = es.eigenvectors().col(2) * std::sqrt(es.eigenvalues()[2]);
    
    // Calculate the cross product of the first and second eigenvector.
    Eigen::Vector3d cross = secondAxis.cross(firstAxis);

    // If it is right hand, order is e0,e1,e2. Otherwise (as considered here) order is e0,e2,e1.
    if(std::signbit(cross[0]) != std::signbit(thirdAxis[0]) ||
       std::signbit(cross[1]) != std::signbit(thirdAxis[1]) ||
       std::signbit(cross[2]) != std::signbit(thirdAxis[2]))
    {
      if(pAxis == 1) axisForRightHandFrame = 2;
      if(pAxis == 2) axisForRightHandFrame = 1;
    }
    
    /***************************************************************************************************************************************************
     * Continue with the visualization.
     ***************************************************************************************************************************************************/
    
    // Use the learners frame_id relative to which all locations in this model are given.
    msg.header.frame_id = getFrameId();
    
    // Set timestamp. See the TF tutorials for information on this.
    msg.header.stamp = ros::Time::now();
    
    // Marker namespace are associated to different types of shapes used to represent object location distributions.
    msg.ns = getNamespace();
    
    // Markers with different namespaces but same id represent same object location distribution with different shapes.
    msg.id = pMarkerId;
    
    // The marker type decides on shape of marker and accordingly how marker parameters are interpreted by RViz.
    // Here we choose a sphere.
    msg.type = visualization_msgs::Marker::ARROW;
    
    // We always want to show a marker by publishing ros messages here.
    msg.action = visualization_msgs::Marker::ADD;
    
    // Arrows should be pretty small. Don't set z here, because we are using start and endpoint here.
    msg.scale.x = msg.scale.y = 0.005;
    
    // Create the rotation matrix that is required for rotating the base vectors of the covariance ellipse into the world coordinate system.
    Eigen::Matrix3d rotationFromParentToWorld = mParentPose->getOrientation().toRotationMatrix();
    
    // Extract the axis vector and rotate it from the parent coordinate frame into world coordinates.
    // The axis vector is composed of the eigenvector scaled by the eigen value.    
    Eigen::Vector3d axis = rotationFromParentToWorld * (es.eigenvectors().col(axisForRightHandFrame) * std::sqrt(es.eigenvalues()[axisForRightHandFrame]));
    
    // We need to rotate the position of the covariance ellipse based on the orientation of the parent object.
    Eigen::Vector3d positionInParentFrame = rotationFromParentToWorld * pMean;
    
    // Extract position of parent object.
    Eigen::Vector3d position = mParentPose->getPosition();
    
    // Create the starting point.
    geometry_msgs::Point start;
    start.x = (position[0] + positionInParentFrame[0]) * getScaleFactor();
    start.y = (position[1] + positionInParentFrame[1]) * getScaleFactor();
    start.z = (position[2] + positionInParentFrame[2]) * getScaleFactor();
    msg.points.push_back(start);
    
    // Create the point to point to ;).
    geometry_msgs::Point stop;
    stop.x = ((position[0] + positionInParentFrame[0]) + (axis[0] * mSigmaMultiplicator)) * getScaleFactor();
    stop.y = ((position[1] + positionInParentFrame[1]) + (axis[1] * mSigmaMultiplicator)) * getScaleFactor();
    stop.z = ((position[2] + positionInParentFrame[2]) + (axis[2] * mSigmaMultiplicator)) * getScaleFactor();
    msg.points.push_back(stop);
    
    // Set the colors based on the axis number. The axis with the biggest eigenvalue will be the new X axis.
    // So the colors RGB stand for the axis XYZ.
    msg.color.b = (pAxis     == 0) ? 1.0 : 0.0;
    msg.color.g = (pAxis - 1 == 0) ? 1.0 : 0.0;
    msg.color.r = (pAxis - 2 == 0) ? 1.0 : 0.0;
    msg.color.a = 1.0;

    // Return the message.
    return msg;
  }
  
  visualization_msgs::MarkerArray GaussianKernelVisualizer::generateSamplesMarker(const unsigned int pMarkerId, const std::vector<boost::shared_ptr<Eigen::Quaternion<double> > >& pSamples)
  {
    // ONE MESSAGE TO RULE THEM ALL! Well, kind of. Stop talking, Sauron :=)!
    visualization_msgs::MarkerArray msgArray;
    visualization_msgs::Marker msg;

    // Set time and frame id.
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = getFrameId();

    // Namespace will represent the nature of the message.
    msg.ns = getNamespace();
    
    // Every sample has its own id.
    msg.id = pMarkerId;

    // Point is represented by a sphere.
    msg.type = visualization_msgs::Marker::SPHERE_LIST;
    
    // We always want to show a marker by publishing ros messages here.
    msg.action = visualization_msgs::Marker::ADD;
    
    // Iterate over all samples and add them to the visualization marker.
    for(boost::shared_ptr<Eigen::Quaternion<double> > sample : pSamples)
    {
      geometry_msgs::Point point;
      point.x = sample->x() * getScaleFactor();
      point.y = sample->y() * getScaleFactor();
      point.z = sample->z() * getScaleFactor();
      msg.points.push_back(point);
    }

    // Draw the points relative to the parent scene object.
    Eigen::Vector3d position = mParentPose->getPosition();
    msg.pose.position.x = position[0] * getScaleFactor();
    msg.pose.position.y = position[1] * getScaleFactor();
    msg.pose.position.z = position[2] * getScaleFactor();
    
    Eigen::Quaternion<double> orientation = mParentPose->getOrientation();
    msg.pose.orientation.w = orientation.w();
    msg.pose.orientation.x = orientation.x();
    msg.pose.orientation.y = orientation.y();
    msg.pose.orientation.z = orientation.z();
    
    // Spheres shall be very small, but must be scaled, too.
    msg.scale.x = msg.scale.y = msg.scale.z = 0.0025 * getScaleFactor();

    // And black! We love black!
    msg.color.r = 0.0;
    msg.color.g = 0.0;
    msg.color.b = 0.0;
    msg.color.a = 1.0;

    // Add message to marker array.
    msgArray.markers.push_back(msg);
    
    // Return the message containing all samples.
    return msgArray;
  }
  
  visualization_msgs::Marker GaussianKernelVisualizer::generateSphereMessage(unsigned int& pMarkerId, Eigen::Vector3d pPos)
  {
    visualization_msgs::Marker msg;
    
    // Set time and frame id.
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = getFrameId();
    
    // Scale it to be very small and long.
    msg.scale.x = msg.scale.y = msg.scale.z = 0.01;
    
    // This should be an arrow.
    msg.type = visualization_msgs::Marker::SPHERE;
    
    // Namespace will represent the nature of the message.
    msg.ns = getNamespace();
    
    // Markers with different namespaces but same id represent same object location distribution with different shapes.
    msg.id = pMarkerId++;
    
    // We always want to show a marker by publishing ros messages here.
    msg.action = visualization_msgs::Marker::ADD;
    
    // Set position.
    msg.pose.position.x = pPos[0];
    msg.pose.position.y = pPos[1];
    msg.pose.position.z = pPos[2];
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 1.0;

    // Take the second color, it stands for the secondary scene object.
    double factor = 0.5;
    msg.color.r = getColor(0).r * factor;
    msg.color.g = getColor(0).g * factor;
    msg.color.b = getColor(0).b * factor;
    msg.color.a = 1.0;
    
    // Return the link message message.
    return msg;
  }
  
  visualization_msgs::Marker GaussianKernelVisualizer::generateArrowMessage(unsigned int& pMarkerId, Eigen::Vector3d pFrom, Eigen::Vector3d pTo)
  {
    visualization_msgs::Marker msg;
    
    // Set time and frame id.
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = getFrameId();
    
    // Scale it to be very small and long.
//     msg.scale.x = 0.006;
//     msg.scale.y = 0.013;
//     msg.scale.z = 0.06;
    msg.scale.x = 0.012;
    msg.scale.y = 0.026;
    msg.scale.z = 0.06;
    
    // This should be an arrow.
    msg.type = visualization_msgs::Marker::ARROW;
    
    // Namespace will represent the nature of the message.
    msg.ns = getNamespace();
    
    // Markers with different namespaces but same id represent same object location distribution with different shapes.
    msg.id = pMarkerId++;
    
    // We always want to show a marker by publishing ros messages here.
    msg.action = visualization_msgs::Marker::ADD;
    
    // Create the starting point.
    geometry_msgs::Point start;
    start.x = pFrom[0] * getScaleFactor();
    start.y = pFrom[1] * getScaleFactor();
    start.z = pFrom[2] * getScaleFactor();
    msg.points.push_back(start);
    
    // Create the end point.
    geometry_msgs::Point stop;
    stop.x = pTo[0] * getScaleFactor();
    stop.y = pTo[1] * getScaleFactor();
    stop.z = pTo[2] * getScaleFactor();
    msg.points.push_back(stop);

    // Take the second color, it stands for the secondary scene object.
    double factor = 0.5;
    msg.color.r = getColor(0).r * factor;
    msg.color.g = getColor(0).g * factor;
    msg.color.b = getColor(0).b * factor;
    msg.color.a = 1.0;
    
    // Return the link message message.
    return msg;
  }
  
  template <class T> void GaussianKernelVisualizer::stlVectorToPointMsg(const std::vector<T> pPositionVector, geometry_msgs::Point& pROSPositionMsg)
  {
    // pPositionVector must represent a position to be converted.
    if(pPositionVector.size() != 3) 
      throw std::invalid_argument("pPositionVector is not a position in terms of dimensionality.");

    // Copy position information per dimension.
    pROSPositionMsg.x = pPositionVector.at(0);
    pROSPositionMsg.y = pPositionVector.at(1);
    pROSPositionMsg.z = pPositionVector.at(2);
  }
}
