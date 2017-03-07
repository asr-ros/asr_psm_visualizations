/**

Copyright (c) 2016, Gehrung Joachim, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "visualization/BayesVisualization.h"

namespace Visualization {
	
  BayesVisualization::BayesVisualization() {
		
    ros::NodeHandle n("~");

    // Init publisher for visualization markers
    gaussianPublisher = n.advertise<visualization_msgs::Marker>("gaussian", 0);
    samplePublisher = n.advertise<visualization_msgs::Marker>("samples", 0);
    eigenPublisher = n.advertise<visualization_msgs::Marker>("eigenvectors", 0);
  }
	
  BayesVisualization::~BayesVisualization() {}

  void BayesVisualization::drawSamples(std::vector<Eigen::Vector3d> samples) {
    for(unsigned int i = 0; i < samples.size(); i++) {
      visualization_msgs::Marker m;

      m.header.frame_id = "/map";
      m.header.stamp = ros::Time::now();
      m.ns = "samples";
      m.id = 100 + i;

      m.type = visualization_msgs::Marker::SPHERE;
      m.action = visualization_msgs::Marker::ADD;

      m.pose.position.x = samples[i][0];
      m.pose.position.y = samples[i][1];
      m.pose.position.z = samples[i][2];

      m.pose.orientation.x = 1;
      m.pose.orientation.y = 1;
      m.pose.orientation.z = 1;
      m.pose.orientation.w = 1;

      m.scale.x = 0.05;
      m.scale.y = 0.05;
      m.scale.z = 0.05;

      m.color.a = 1.0;
      m.color.r = 0.0;
      m.color.g = 0.0;
      m.color.b = 0.0;

      samplePublisher.publish(m);
    }
  }
	
  void BayesVisualization::drawDistribution(Eigen::Vector3d mean , Eigen::Matrix3d covariance) {

    // Calculates eigen vectors of covariance matrix.
    // Use eigenvalues are required for the scale of the sphere,
    // the eigenvectors form a rotation matrix, that will be converted into a quaternion;
    // the latter one is required for rotating the sphere.
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(covariance);    
    Eigen::Quaterniond q(es.eigenvectors());

    // visualize eigen vectors
    for(int i = 0; i < 3; i++) {
      visualization_msgs::Marker e = createArrowMarker(mean, mean + es.eigenvectors().col(i) * es.eigenvalues()[i]);

      e.id = i;
      e.ns = "eigenvectors";
      e.color.r = (i     == 0) ? 1.0 : 0.0;
      e.color.g = (i - 1 == 0) ? 1.0 : 0.0;
      e.color.b = (i - 2 == 0) ? 1.0 : 0.0;
      e.color.a = 1.0;

      eigenPublisher.publish(e);
    }
    
   // TODO remove
    /*    std::cout << "--------------------" << std::endl;
    std::cout << "Mean: " << std::endl << mean << std::endl;
    std::cout << "Covariance: " << std::endl << covariance << std::endl;
    std::cout << "Eigen values: " << std::endl << es.eigenvalues() << std::endl;
    std::cout << "Eigen vectors: " << std::endl << es.eigenvectors() << std::endl;
    std::cout << "Quaternion: " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    std::cout << "X: " << es.eigenvectors().col(0) << std::endl;
    std::cout << "Y: " << es.eigenvectors().col(1) << std::endl;
    std::cout << "Z: " << es.eigenvectors().col(2) << std::endl;*/

    // Visualize distribution
    visualization_msgs::Marker m;

    m.header.frame_id ="/map";
    m.header.stamp = ros::Time::now();
    m.ns = "gaussian_distribution";
    m.id = ID_GAUSSIAN;

    m.type = visualization_msgs::Marker::SPHERE;
    m.action = visualization_msgs::Marker::ADD;

    // add a small offset to prevent the disappearing
    // of the sphere when an eigen value is zero
    m.scale.x = (std::abs(es.eigenvalues()[0]) * 2) + 0.1;
    m.scale.y = (std::abs(es.eigenvalues()[1]) * 2) + 0.1;
    m.scale.z = (std::abs(es.eigenvalues()[2]) * 2) + 0.1;

    m.pose.position.x = mean[0];
    m.pose.position.y = mean[1];
    m.pose.position.z = mean[2];

    m.pose.orientation.w = q.w();
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();

    m.color.r = 0.0;
    m.color.g = 1.0;
    m.color.b = 0.0;
    m.color.a = 0.3;
    
    gaussianPublisher.publish(m);
  }

  void BayesVisualization::drawDistribution(std::vector<Eigen::Vector3d> samples) {
    Eigen::Vector3d mean(0, 0, 0);
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();

    // calculate mean
    for(unsigned int i = 0; i < samples.size(); i++) {
            mean += samples[i];
    }
    mean /= samples.size();

    // calculate covariance matrix
    for(unsigned int i = 0; i < samples.size(); i++) {
      covariance += (samples[i] - mean) * (samples[i].adjoint() - mean.adjoint());
    }
    covariance /= (samples.size());

    // draw samples with the related distribution
    drawDistribution(mean, covariance);
    drawSamples(samples);
  }

  visualization_msgs::Marker BayesVisualization::createArrowMarker(const Eigen::Vector3d a, const Eigen::Vector3d b) {
    visualization_msgs::Marker m;

    m.header.stamp = ros::Time::now();
    m.header.frame_id = "/map";

    m.scale.x = 0.05;
    m.scale.y = 0.05;
    m.scale.z = 0.05;

    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::ADD;

    m.points.push_back(createPointMessage(a));
    m.points.push_back(createPointMessage(b));

    return m;
  }

  geometry_msgs::Point BayesVisualization::createPointMessage(Eigen::Vector3d point) {
    geometry_msgs::Point p;

    p.x = point[0];
    p.y = point[1];
    p.z = point[2];

    return p;
  }

}

