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
#include <cmath>

// Package includes
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>

namespace Visualization {
  
  /**
   * Helper class for color operations.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class ColorHelper {
  public:
    
    /**
     * Constructor.
     */
    ColorHelper();
    
    /**
     * Destructor.
     */
    ~ColorHelper();
    
    /**
     * Fills the given RGB color object with the color described in the given hsv values.
     * 
     * @param pColor The color in RGB.
     * @param pH The hue value.
     * @param pS The saturation value.
     * @param pV The value value.
     */
    static void convertHSVToRGB(std_msgs::ColorRGBA& pColor, double pH, double pS, double pV);
    
  };
}
