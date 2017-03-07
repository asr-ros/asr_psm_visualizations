/**

Copyright (c) 2016, Gehrung Joachim, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "visualization/EnvironmentVisualization.h"

namespace Visualization {
	
  EnvironmentVisualization::EnvironmentVisualization() {
		
    ros::NodeHandle n("~");
		
    // Init publisher for visualization markers
    meshesPublisher = n.advertise<visualization_msgs::Marker>("dome_meshes", 0);		
  }
	
  EnvironmentVisualization::~EnvironmentVisualization() {}
	
  void EnvironmentVisualization::drawDome() {
    visualization_msgs::Marker m;

    // Create and publish a visualization marker for the dome model
    m.header.frame_id = "/PTU";
    m.header.stamp = ros::Time::now();
    m.ns = "dome_meshes";
    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::MESH_RESOURCE;
    m.mesh_resource = "package://asr_visualization_server/res/dome.dae";
    m.id = ID_DOME;

    m.scale.x = m.scale.y = m.scale.z = 0.1;
    m.color.r = m.color.g = m.color.b = 0.7f;
    m.color.a = 1.0f;

    tf::Quaternion q = tf::createQuaternionFromRPY(-1.5708, 0.0, -0.7854); // -90°, 0°, -45°
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();
    m.pose.orientation.w = q.w();

    m.pose.position.x =   0.3;
    m.pose.position.y = -1.15;
    m.pose.position.z =  1.85;

    meshesPublisher.publish(m);
  }

  void EnvironmentVisualization::drawTable() {
    visualization_msgs::Marker m;

    m.header.frame_id = "/PTU";
    m.header.stamp = ros::Time::now();
    m.ns = "dome_meshes";
    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::MESH_RESOURCE;
    m.mesh_resource = "package://asr_visualization_server/res/table.dae";
    m.id = ID_TABLE;

    m.scale.x = m.scale.y = m.scale.z = 0.1;
    m.color.r = m.color.g = m.color.b = 0.7f;
    m.color.a = 1.0f;

    tf::Quaternion qTable = tf::createQuaternionFromRPY(-1.5708, 0.0, 0.7854); // -90°, 0°, 45°
    m.pose.orientation.x = qTable.x();
    m.pose.orientation.y = qTable.y();
    m.pose.orientation.z = qTable.z();
    m.pose.orientation.w = qTable.w();

    m.pose.position.x =  -0.5;
    m.pose.position.y =  -1.4;
    m.pose.position.z = 1.825;

    meshesPublisher.publish(m);
  }
	
}
