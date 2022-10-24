/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Poznan University of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Poznan University of Technology nor the names
 *       of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Mateusz Przybyla
 */

#include "laser_scan_merger/scans_merger.h"

using namespace laser_scan_merger;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto merger_node = rclcpp::Node::make_shared("laser_scan_merger");
  try {
    RCLCPP_INFO(merger_node->get_logger(), "[Laser Scan Merger]: Initializing node");
    ScansMerger ot(merger_node, merger_node);
    rclcpp::spin(merger_node);
  }
  catch (const char* s) {
    RCLCPP_FATAL_STREAM(merger_node->get_logger(), "[Laser Scan Merger]: "  << s);
  }
  catch (const std::exception &exc) {
    auto eptr = std::current_exception(); // capture
    RCLCPP_FATAL_STREAM(merger_node->get_logger(), "[Laser Scan Merger]: " << exc.what());
  }
  catch (...){
    RCLCPP_FATAL_STREAM(merger_node->get_logger(), "[Laser Scan Merger]: Unknown error");
  }
  rclcpp::shutdown();
  return 0;
}
