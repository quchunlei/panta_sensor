/******************************************************************************
 * Copyright 2017 robosense All rights reserved.
 * Suteng Innovation Technology Co., Ltd. www.robosense.ai

 * This software is provided to you directly by robosense and might
 * only be used to access robosense LiDAR. Any compilation,
 * modification, exploration, reproduction and redistribution are
 * restricted without robosense's prior consent.

 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL robosense BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#pragma once

#include <string>
#include <array>
#include <vector>

namespace robosense {
namespace common {

struct alignas(16) FreeSpace {
    double timestamp = 0.0;
    uint32_t device_code = 0; //source device code where the underlaying data come from.

    float distance = 0.f;  // distance to the nearest obstacle per section
    float yaw_angle = 0.f; // section yaw angle
    float free_prob = 0.f; // probability for the section to be passable, 0~1, the greater, the better.

    typedef std::shared_ptr<FreeSpace> Ptr;
    typedef std::shared_ptr<const FreeSpace> ConstPtr;
};

/**
   * @brief basic output struct for freespace detection for robosense SDK.
   * The FreespaceMsg is used as the output from freespace detection in perception module
   */
struct alignas(16) FreeSpaceMsg {
    double timestamp = 0.0;
    uint32_t seq = 0;
    std::string frame_id = "";
    std::string parent_frame_id = "";

    std::vector<FreeSpace::Ptr> freespaces;

    typedef std::shared_ptr<FreeSpaceMsg> Ptr;
    typedef std::shared_ptr<const FreeSpaceMsg> ConstPtr;
};

} // namespace common
} // namespace robosense
