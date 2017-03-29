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
   * Visualizes and writes histograms to file.
   */
class GMMGnuplotVisualization
{
public:
    
    /**
     * Constructor.
     */
    GMMGnuplotVisualization();
    
    /**
     * Destructor.
     */
    ~GMMGnuplotVisualization();
    
    /**
     * Writes the data to the gnuplot file with the given name and displays the gnuplot as a histogram
     * @param filename  Name of the gnuplot file to write into.
     * @param data      Data to be written. Interpreted as buckets over [-pi,pi].
     * @param rotaxis   Name of the axis around which the orientation is given.
     */
    void plotOrientationHistogram(const std::string& filename, const std::vector<std::pair<double, double>> data, const std::string& rotaxis);
    
private:
    // similar to active_scene_recognition/recognizer_prediction_psm asrVisualizer
    /**
     * Initializes the Gnuplot
     * @param pPlotTitle - the name of the plot
     * @param pxLabel - label of the x-axis
     * @param pyLabel - label of the y-axis
     * @param pXRange - x-axis range (min, max)
     * @param pYRange - y-axis range (min, max)
     * @param samples - number of samples
     */
    void initPlot(const std::string& pPlotTitle,
                  const std::string& pXLabel,
                  const std::string& pYLabel,
                  const std::pair<double, double>& pXRange,
                  const std::pair<double, double>& pYRange,
                  unsigned int samples);

    /**
     * Adds a new point to the list of points
     */
    void addPointToBuffer(double x, double y);

    /**
     * Sends the data to the gnuplot and draws the window
     * which contains the plot.
     * Also writes the gnuplot data created and used to file.
     */
    void sendPlot();


    /**
     * Private class handling the writing to file and gnuplot output of the data
     */
    class PlotFileHandler
    {
    public:
        /**
         * Constructor
         * @param filename  Name of the file to write the gnuplot script into
         * If "": Do not write to file
         */
        PlotFileHandler(std::string filename);

        /**
         * Create a clean interface to gnuplot.
         */
        void reset();

        /**
         * Add string to gnuplot and file buffer
         */
        void add(std::string in);

        /**
         * Send data to gnuplot and write them to file
         * @param pointBuffer   The buffer containing the point data
         */
        void send(std::vector<std::pair<double, double>> pointBuffer);

        /**
         * Checks whether gnuplot exists or not
         * @return existence of gnuplot
         */
        bool gnuplotExists();

        /**
         * Flushes the gnuplot.
         */
        void flush();

    private:
        //Interface with which configurations or data is sent to gnuplot.
        boost::shared_ptr<Gnuplot> mGnuplotHandler;
        // Buffer for text to be written to file
        std::string mFileBuffer;
        // Name of the file to write the gnuplot script into
        std::string mFilename;
    };

    //Interface through which configurations or data is sent to gnuplot and file
    PlotFileHandler mPlotFileHandler;

    //buffer for visualization points
    std::vector<std::pair<double, double> > mPointBuffer;
};

}
