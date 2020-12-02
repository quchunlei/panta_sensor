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


#ifndef RS_MESSAGE_FILTER_H
#define RS_MESSAGE_FILTER_H

//#include <ros/ros.h>
#include <atomic>
#include <tuple>
#include <deque>
#include <thread>
#include <mutex>
#include <map>
#include <vector>
#include <condition_variable>
#include <chrono>
#include <functional>
#include <iostream>
//#include <sensor_msgs/PointCloud2.h>
#include <panta_common/msg/rs_msg/lidar_points_msg.h>
#include <panta_common/debug/prompt.h>


namespace robosense
{
namespace preprocessing
{
struct VirtualMsg
{
  VirtualMsg() : stamp(0.0), queue_idx(0), idx_queue(0)
  {
  }
  VirtualMsg(double s, std::size_t t, std::size_t i)
    : stamp(s), queue_idx(t), idx_queue(i)
  {
  }

  double stamp;
  std::size_t queue_idx;  // specifies which queue it belongs to
  std::size_t idx_queue;  // specifies the data index in the queue
};

struct MsgSet
{
  MsgSet() : max_stamp(0), min_stamp(-1.0)
  {
  }

  std::vector<VirtualMsg> virtual_data;
  double max_stamp;
  double min_stamp;
};

struct NullP
{
};


template <typename T>
class MessageFilter
{
public:
  using Starters = std::map<double, VirtualMsg>;
  using FuncType6 = std::function<void(T&, T&, T&, T&, T&, T&)>;

  MessageFilter(std::size_t num);
  ~MessageFilter();

  // input
  template <std::size_t index>  // start from 0
  void callback(const T& input_msg);

  void regRecvCallback(const std::function<void(const T&)> callback);

  void regRecvCallback(const std::function<void(const T&, const T&)> callback);

  void regRecvCallback(
      const std::function<void(const T&, const T&, const T&)> callback);

  void regRecvCallback(
      const std::function<void(const T&, const T&, const T&, const T&)>
          callback);

  void regRecvCallback(const std::function<void(const T&, const T&, const T&,
                                                const T&, const T&)>
                           callback);

  void regRecvCallback(const std::function<void(const T&, const T&, const T&,
                                                const T&, const T&, const T&)>
                           callback);

  typedef std::shared_ptr<MessageFilter<T>> Ptr;
  //关闭线程
  void stop();

private:
  // the MsgSet size is the difference between the latest and earliest time
  // stamp in the set
  double getMsgSetSize(const MsgSet& set);
  template <std::size_t N>  // start from 0
  void add(const T& msg);

  std::atomic<bool> start_;
  bool is_registered_;
  FuncType6 callback_;

  std::thread main_thread_;
  std::vector<std::deque<T>> deques_;
  std::vector<std::mutex> vec_mutex_;  // mutex in every queue
  // std::vector<std::condition_variable> cvs_;
  std::condition_variable cv_;

  // parameters
  const std::size_t NUM_QUEUE_;
  const double INTER_MSG_BOUND_;
  const double AGE_PENALTY_;
  const double MAX_INTERVAL_DURATION_;
  const double MAX_QUEUE_DURATION_;

  void run();
  void addToMsgSet(MsgSet& set, const VirtualMsg& data);
  void deleteQueues(const MsgSet&);
  void deleteOneQueue(const double stamp, const int i);
  T pad_msg_;
};

}  // namespace preprocessing
}  // namespace robosense

#endif  // !MESSAGE_FILTER_H
