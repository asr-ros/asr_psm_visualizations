/**

Copyright (c) 2016, Gehrung Joachim, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <ros/ros.h>

// Global includes
#include <string>
#include <set>
#include <utility>

// Package includes
#include <boost/shared_ptr.hpp>

#include <tf/transform_listener.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Local includes
#include "visualization/gnuplot/gnuplot-iostream.h"

namespace Visualization {
  
  /**
   * Gnuplot visualizer class. Interface to various visualizations realized through gnuplot that can be filled with data from constellation models. Visualization comprises bar charts for probability tables.
   *
   * @author Pascal Meissner
   * @version See SVN
   */
  class GnuplotVisualization
  {
  public:
    
    /**
     * Constructor.
     */
    GnuplotVisualization();
    
    /**
     * Destructor.
     */
    ~GnuplotVisualization();
    
    /**
     * Create a gnuplot visualization of a bar chart that can be altered during its lifetime and set its properties.
     * 
     * @param pBarLabels Identifiers of all bar that are to be visualized
     * @param pBarChartTitle Title of whole visualization.
     * @param pYLabel Label of y-axis of visualization.
     * @param pYRange Interval represented as pair<min, max> in which bar values are allowed to lie.
     */
    void initAnimatedBarChart(const std::vector<std::string>& pBarLabels, const std::string& pBarChartTitle, const std::string& pYLabel, const std::pair<float, float>& pYRange);
    
    /**
     * Alters the values of the bars in the buffer of bar chart visualizer object.
     *
     * @param pCurrentData Labels of bars for which values are to replaced with the data labels are associated with.
     */
    void updateBarChartValues(const std::map<std::string, float>& pCurrentData);

    /**
     * Shows all values in gnuplot as bar chart that are currently in internal buffer of visualizer. Therefore translates content of buffer to gnuplot representation.
     * 
     * @param pHighlightHighestValue Show highest value in bar chart in other color than the rest of the bar.
     */
    void sendBarChartToGnuplot(const bool pHighlightHighestValue);
    
  private:
    
    /**
      * Interface with which configurations or data is sent to gnuplot.
      */
    boost::shared_ptr<Gnuplot> mGnuplotHandler;
    
    /**
     * Buffer keeping bar chart data in representation compatible to gnuplot.
     * Every array element is a bar with (from left to right), its position on the x-axis, the name of the tic at this position and its hight (concerning y-axis). 
     */
    std::vector<std::pair<float, std::pair<std::string, float> > > mBarChartBuffer;

    /**
     * Range (pair<min,max>) in which all bar values to be visualized have to lie.
     */
    std::pair<float, float> mYRange;
  };
}
