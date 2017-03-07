/**

Copyright (c) 2016, Gehrung Joachim, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "visualization/psm/helper/AbstractExtendedVisualizer.h"

namespace Visualization {
  
  AbstractExtendedVisualizer::AbstractExtendedVisualizer()
  {
  }
  
  AbstractExtendedVisualizer::~AbstractExtendedVisualizer()
  {
  }
  
  void AbstractExtendedVisualizer::setColors(const std_msgs::ColorRGBA& pFirstColor,
					     const std_msgs::ColorRGBA& pSecondColor,
					     const std_msgs::ColorRGBA& pThirdColor)
  {
    // All color parameter values of the first color must be in [0,1].
    if((pFirstColor.r < 0.0f || pFirstColor.r > 1.0f) || (pFirstColor.g < 0.0f || pFirstColor.g > 1.0f) || (pFirstColor.b < 0.0f || pFirstColor.b > 1.0f) || (pFirstColor.a < 0.0f || pFirstColor.a > 1.0f))
      throw std::out_of_range("First ring color contains no valid color information.");
    
    // All color parameter values of the second color must be in [0,1].
    if((pSecondColor.r < 0.0f || pSecondColor.r > 1.0f) || (pSecondColor.g < 0.0f || pSecondColor.g > 1.0f) || (pSecondColor.b < 0.0f || pSecondColor.b > 1.0f) || (pSecondColor.a < 0.0f || pSecondColor.a > 1.0f))
      throw std::out_of_range("Second ring color contains no valid color information.");
    
    // All color parameter values of the second color must be in [0,1].
    if((pThirdColor.r < 0.0f || pThirdColor.r > 1.0f) || (pThirdColor.g < 0.0f || pThirdColor.g > 1.0f) || (pThirdColor.b < 0.0f || pThirdColor.b > 1.0f) || (pThirdColor.a < 0.0f || pThirdColor.a > 1.0f))
      throw std::out_of_range("Third ring color contains no valid color information.");
    
    // Add the colors to the list.
    mColors.push_back(pFirstColor);
    mColors.push_back(pSecondColor);
    mColors.push_back(pThirdColor);
  }
  
  void AbstractExtendedVisualizer::setScaleFactor(const double pScaleFactor)
  {
    mScaleFactor = pScaleFactor;
  }
  
  const std_msgs::ColorRGBA AbstractExtendedVisualizer::getColor(unsigned int pNumber)
  {
    if(pNumber > (mColors.size() - 1))
      throw std::out_of_range("Unable to access a color. Index out of range.");
    
    return mColors[pNumber];
  }
  
  double AbstractExtendedVisualizer::getScaleFactor()
  {
    return mScaleFactor;
  }
  
}
