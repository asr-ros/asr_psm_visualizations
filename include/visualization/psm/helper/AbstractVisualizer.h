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

namespace Visualization {
  
  /**
   * An abstract visualization helper class.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class AbstractVisualizer {
  public:
    
    /**
     * Constructor.
     */
    AbstractVisualizer();
    
    /**
     * Destructor.
     */
    virtual ~AbstractVisualizer();
    
    /**
     * Sets the frame id.
     * 
     * @param pFrameId The id of the frame the messages should be published in.
     */
    void setFrameId(std::string pFrameId);
    
    /**
     * Sets the namespace.
     * 
     * @param pNamespace The namespace the message should be published in.
     */
    void setNamespace(std::string pNamespace);
    
  protected:
    
    /**
     * Returns the frame id.
     * 
     * @return The id of the frame the messages should be published in.
     */
    std::string getFrameId();
    
    
    /**
     * Returns the namespace.
     * 
     * @return The namespace the message should be published in.
     */
    std::string getNamespace();
    
  private:
    
    /**
     * The id of the frame the messages should be published in.
     */
    std::string mFrameId;
    
    /**
     * The namespace the message should be published in.
     */
    std::string mNamespace;
  };
}
