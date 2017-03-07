/**

Copyright (c) 2016, Gehrung Joachim, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#pragma once

// Global includes
#include <string>

// Package includes
#include <boost/shared_ptr.hpp>

#include <std_msgs/ColorRGBA.h>

// Local includes
#include "visualization/psm/helper/AbstractVisualizer.h"

namespace Visualization {
  
  /**
   * An extended version of the abstract visualization helper class.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class AbstractExtendedVisualizer : public AbstractVisualizer {
  public:
    
    /**
     * Constructor.
     */
    AbstractExtendedVisualizer();
    
    /**
     * Destructor.
     */
    virtual ~AbstractExtendedVisualizer();
    
    /**
     * Sets the three colors required in visualization.
     * 
     * @param pFirstColor The color of the first ring.
     * @param pSecondColor The color of the second ring.
     * @param pThirdColor The color of the second ring.
     */
    void setColors(const std_msgs::ColorRGBA& pFirstColor,
		   const std_msgs::ColorRGBA& pSecondColor,
		   const std_msgs::ColorRGBA& pThirdColor);
    
    /**
     * Sets the visualization parameters.
     * 
     * @param pScaleFactor Factor to scale the visualization with.
     */
    void setScaleFactor(const double pScaleFactor);
    
  protected:
    
    /**
     * Returns the three colors required in visualization.
     * 
     * @param pNumber The number of the color.
     * @return The three colors required in visualization.
     */
    const std_msgs::ColorRGBA getColor(unsigned int pNumber);
    
    /**
     * Returns the scale factor.
     * 
     * @return The factor to scale the visualization with.
     */
    double getScaleFactor();
    
  private:
    
    /**
     * The factor to scale the visualization with.
     */
    double mScaleFactor;
    
    /**
     * The colors of the three rings around the covariance ellipse.
     */
    std::vector<std_msgs::ColorRGBA> mColors;
  };
}
