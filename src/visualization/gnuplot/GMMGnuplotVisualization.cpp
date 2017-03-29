/**

Copyright (c) 2017, Gaßner Nikolai, Gehrung Joachim, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "visualization/gnuplot/GMMGnuplotVisualization.h"

namespace Visualization {

GMMGnuplotVisualization::GMMGnuplotVisualization(): mPlotFileHandler("") { }
GMMGnuplotVisualization::~GMMGnuplotVisualization() { }

void GMMGnuplotVisualization::plotOrientationHistogram(const std::string& filename, const std::vector<std::pair<double, double>> data, const std::string& rotaxis)
{
    mPlotFileHandler = PlotFileHandler(filename);
    mPointBuffer.clear();

    double min_x, max_x, min_y, max_y;
    min_x = max_x = min_y = max_y = 0;
    for (std::pair<double, double> datum: data)
    {
        if (datum.first < min_x) min_x = datum.first;
        if (datum.first > max_x) max_x = datum.first;
        if (datum.second < min_y) min_y = datum.second;
        if (datum.second > max_y) max_y = datum.second;
    }
    double xdistance = 0;
    if (data.size() > 1) xdistance = std::abs(data.at(1).first - data.at(0).first);   // Assuming all data points have equal distance on the x axis
    std::pair<double, double> xRange(min_x, max_x + xdistance);                       // Add xdistance to be able to stretch the last point out, see below
    std::pair<double, double> yRange(min_y, max_y + ((double) 1 / data.size()));          // y axis goes up to the bucket of the histogram with the highest value and a little above

    initPlot("Tabular(" + rotaxis + ")", rotaxis, "P(" + rotaxis + ")", xRange, yRange, data.size()); // using data.size() as amount of samples
    for (unsigned int i = 0; i < data.size() - 1; i++)    // add each data point twice for histogram-like look
    {
        addPointToBuffer(data.at(i).first, data.at(i).second);
        addPointToBuffer(data.at(i+1).first, data.at(i).second);
    }
    if (!data.empty())    // add last point twice too
    {
        addPointToBuffer(data.at(data.size() - 1).first, data.at(data.size() - 1).second);
        addPointToBuffer(data.at(data.size() - 1).first + xdistance, data.at(data.size() - 1).second);
    }
    sendPlot();
}

void GMMGnuplotVisualization::initPlot(const std::string& pPlotTitle,
                       const std::string& pXLabel,
                       const std::string& pYLabel,
                       const std::pair<double, double>& pXRange,
                       const std::pair<double, double>& pYRange,
                       unsigned int samples)
{
    mPlotFileHandler.reset();   //Create a clean interface to gnuplot.

    mPlotFileHandler.add("set xlabel \"" + pXLabel + "\"\n");     //Set label for x axis
    mPlotFileHandler.add("set ylabel \"" + pYLabel + "\"\n");     //Set label for y axis
    mPlotFileHandler.add("\n");
    mPlotFileHandler.add("set xrange [" + boost::lexical_cast<std::string>(pXRange.first) +  ":" + boost::lexical_cast<std::string>(pXRange.second) + "]\n");   //Set range in x direction
    mPlotFileHandler.add("set yrange [" + boost::lexical_cast<std::string>(pYRange.first) +  ":" + boost::lexical_cast<std::string>(pYRange.second) + "]\n");   //Set range in y direction
    mPlotFileHandler.add("set nokey\n");                // don't show legend
    mPlotFileHandler.add("set samples " + boost::lexical_cast<std::string>(samples) + "\n");         // set number of samples
    mPlotFileHandler.add("set title \"" + pPlotTitle + "\"\n"); //Set title
    mPlotFileHandler.add("\n");

}

void GMMGnuplotVisualization::addPointToBuffer(double x, double y)
{
    mPointBuffer.push_back(std::make_pair(x, y));
}

void GMMGnuplotVisualization::sendPlot()
{
    ROS_ASSERT(mPlotFileHandler.gnuplotExists());
    //Prevent system from trying to send data to non-initialized gnuplot handler
    if(!mPlotFileHandler.gnuplotExists())
      throw std::runtime_error("Cannot show non-existing gnuplot visualization.");

    mPlotFileHandler.add("plot \"-\" with lines\n");    // Prepare for plotting.

    //Push data to gnuplot and file. "end" is added to end of file automatically, see below.
    mPlotFileHandler.send(mPointBuffer);
    mPlotFileHandler.flush();
}

GMMGnuplotVisualization::PlotFileHandler::PlotFileHandler(std::string filename): mFileBuffer(""), mFilename(filename) { }

void GMMGnuplotVisualization::PlotFileHandler::reset() { mGnuplotHandler.reset(new Gnuplot); }

void GMMGnuplotVisualization::PlotFileHandler::add(std::string in)
{
    *(mGnuplotHandler) << in;
    mFileBuffer += in;
}

void GMMGnuplotVisualization::PlotFileHandler::send(std::vector<std::pair<double, double> > pointBuffer)
{
    // Write to file if requested and possible
    if (mFilename != "")
    {
        std::ofstream file;
        file.open(mFilename);
        if (file.is_open())
        {
            file << mFileBuffer;    // Write plot configuration to file
            for (unsigned int i = 0; i < pointBuffer.size(); i++) {     // Write data to file
                file << pointBuffer.at(i).first << " " << pointBuffer.at(i).second << "\n";
            }
            file << "end\n";    // Add "end" so gnuplot later recognizes the end of the data block
            file << "pause -1 \"Hit return to continue\"";  // Add pause and waiting for user input to the end of the file
            file.close();
        }
        else { ROS_INFO_STREAM("Could not open file " + mFilename + ". Proceeding without writing to it."); }
    }
    // send to gnuplot
    mGnuplotHandler->send(pointBuffer);
}

bool GMMGnuplotVisualization::PlotFileHandler::gnuplotExists()
{
    if(!mGnuplotHandler) return false;
    else return true;
}

void GMMGnuplotVisualization::PlotFileHandler::flush() { mGnuplotHandler->flush(); }

}
