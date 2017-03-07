/**

Copyright (c) 2016, Gehrung Joachim, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "visualization/gnuplot/GnuplotVisualization.h"

namespace Visualization {
  
  GnuplotVisualization::GnuplotVisualization()
  {
  }
  
  GnuplotVisualization::~GnuplotVisualization()
  {
  }
  
  void GnuplotVisualization::initAnimatedBarChart(const std::vector<std::string>& pBarLabels, const std::string& pBarChartTitle, const std::string& pYLabel, const std::pair<float, float>& pYRange)
  {
    //Check whether method argument is a non-empty interval.
    if(pYRange.second < pYRange.first)
      throw std::invalid_argument("Bar value range is not allowed to be empty.");

    //Save value range for later checking of bar value updates.
    mYRange = pYRange;

    //Create a clean interface to gnuplot.
    mGnuplotHandler.reset(new Gnuplot);

    //Empty buffer with data for gnuplot.
    mBarChartBuffer.clear();

    //Set bar chart title
    *(mGnuplotHandler) << "set title \"" << pBarChartTitle << "\"\n";

    //Set label for y-axis
    *(mGnuplotHandler) << "set ylabel \"" << pYLabel << "\"\n";
	  
    //Set range in both x and y direction
    *(mGnuplotHandler) << "set auto x\n";
    *(mGnuplotHandler) << "set yrange [" << pYRange.first <<  ":" << pYRange.second << "]\n";

    //Ask for a grid in vertical direction
    *(mGnuplotHandler) << "set grid ytics\n";
    //Set width of bars around their respective x values (left float in outer pair in mBarChartBuffer).
    *(mGnuplotHandler) << "set boxwidth 0.75\n";
    //Ask for slightly transparent bars uniformly colored with a border.
    *(mGnuplotHandler) << "set style fill transparent solid 0.75 border -1\n";

    //Set that there can be both green and red bars in chart.
    *(mGnuplotHandler) << "set style line 1 lc rgb \"green\"\n";
    *(mGnuplotHandler) << "set style line 2 lc rgb \"red\"\n";

    //Access the labels of the different bars
    std::vector<std::string>::const_iterator barLabelIterator;

    //Go through all labels
    for(barLabelIterator = pBarLabels.begin(); barLabelIterator != pBarLabels.end(); barLabelIterator++) {
      //and create data set containing their position, string identifier surrounded by double quotes and a bar value of zero.
      std::pair<float, std::pair<std::string, float> > zeroBar = std::make_pair(static_cast<float>(std::distance(pBarLabels.begin(), barLabelIterator)), std::make_pair("\"" + *barLabelIterator + "\"", 0.0f));

      //Insert initialized data set into visualization buffer.
      mBarChartBuffer.push_back(zeroBar);
    }
  }
  
  void GnuplotVisualization::updateBarChartValues(const std::map<std::string, float>& pCurrentData)
  {
    //Do not accept to alter visualization with non-existing gnuplot handler, even though this would not lead to null pointer exception.
    if(!mGnuplotHandler)
      throw std::runtime_error("Cannot update non-existing gnuplot visualization.");

    //Access data sets predefined for this gnuplot bar chart visualization.
    std::vector<std::pair<float, std::pair<std::string, float> > >::iterator barChartBufferIterator;

    //Access new bar values given to this method.
    std::map<std::string, float>::const_iterator currentDataIterator;

    //Go through buffer as contrary to input map, labels can occur there multiple times.
    for(barChartBufferIterator = mBarChartBuffer.begin(); barChartBufferIterator != mBarChartBuffer.end(); barChartBufferIterator++) {
      //Look for bar label currently taken into account in input map ignoring its surrounding double quotes.
      currentDataIterator = pCurrentData.find(barChartBufferIterator->second.first.substr(1, barChartBufferIterator->second.first.size() - 2));

      //If there is an update for this bar label
      if(currentDataIterator != pCurrentData.end()) {

	//Check whether new bar value is within range currently allowed for this visualization.
	if(currentDataIterator->second < mYRange.first || currentDataIterator->second > mYRange.second)
	  throw std::invalid_argument("Bar chart value " + boost::lexical_cast<std::string>(currentDataIterator->second) + " is outside allowed range.");
	
	//Update value of bar label in internal buffer with value associated to this label in input map.
	barChartBufferIterator->second.second = currentDataIterator->second;
      }
    }
  }

  void GnuplotVisualization::sendBarChartToGnuplot(const bool pHighlightHighestValue)
  {
    //Prevent system from trying to send data to non-initialized gnuplot handler
    if(!mGnuplotHandler)
      throw std::runtime_error("Cannot show non-existing gnuplot visualization.");

    //Check whether there is any data to be shown.
    if(mBarChartBuffer.empty())
      throw std::logic_error("Chart with no bars cannot be shown.");

    //Every data combination has in common it wants to be plotted.
    *(mGnuplotHandler) << "plot '-' every ::" << 0 << "::" << mBarChartBuffer.size() << " using 1:3:xtic(2) notitle with boxes ls 1";

    //End of gnuplot instructions for defining how bars are to be plotted
    *(mGnuplotHandler) << "\n";
    
    //Push bar chart data to gnuplot.
    mGnuplotHandler->send(mBarChartBuffer).send(mBarChartBuffer);
    mGnuplotHandler->flush();
  }
  
}
