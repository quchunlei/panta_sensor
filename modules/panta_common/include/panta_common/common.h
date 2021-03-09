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

#include "panta_common/debug/prompt.h"
#include "panta_common/yaml/yaml_parser.h"
#include "panta_common/debug/error_code.h"
#include <chrono>
#include <mutex>
#include <thread>
#include <math.h>
#include <unistd.h>
namespace robosense
{
namespace common
{
/**
 * @brief  basic class which inheritted by all modules of rs_sdk
 * @note   define the basic functions such like yaml read and so on 
 */
class CommonBase
{
public:
  CommonBase() = default;
  virtual ~CommonBase() = default;

  /**
   * @brief  read target yaml node
   * @note   read the yaml node with the specific key value
   * @param  &yaml: the yaml node
   * @param  &key: the key
   * @param  &out_val: output value
   * @retval true: success false: failed
   */
  template <typename T>
  inline bool yamlRead(const YAML::Node &yaml, const std::string &key, T &out_val)
  {
    if (!yaml[key] || yaml[key].Type() == YAML::NodeType::Null)
    {
      WARNING << name() << " : Not set " << RESET << key;
      WARNING << " value !!!" << RESET << REND;
      return false;
    }
    else
    {
      out_val = yaml[key].as<T>();
      return true;
    }
  }

  /**
   * @brief  read target yaml node
   * @note   read the yaml node with the specific key value, if failed, the progress will end
   * @param  &yaml: the yaml node
   * @param  &key: the key
   * @param  &out_val: output value
   */
  template <typename T>
  inline void yamlReadAbort(const YAML::Node &yaml, const std::string &key, T &out_val)
  {

    if (!yaml[key] || yaml[key].Type() == YAML::NodeType::Null)
    {
      ERROR << name() << " : Not set " << RESET << key;
      ERROR << " value, Aborting!!!" << RESET << REND;
      exit(-1);
    }
    else
    {
      out_val = yaml[key].as<T>();
    }
  }

/**
   * @brief  read target yaml node
   * @note   read the yaml node with the specific key value, if failed, will set the default value as the output value
   * @param  &yaml: the yaml node
   * @param  &key: the key
   * @param  &out_val: output value
   * @retval true: success false: failed
   */
  template <typename T>
  inline bool yamlRead(const YAML::Node &yaml, const std::string &key, T &out_val, const T &default_val)
  {
    if (!yaml[key] || yaml[key].Type() == YAML::NodeType::Null)
    {
      // WARNING << name() << " : Not set " << RESET << key;
      // WARNING << " value, Using default" << RESET << REND;
      out_val = default_val;
      return false;
    }
    else
    {
      out_val = yaml[key].as<T>();
      return true;
    }
  }

/**
   * @brief  get the subnode
   * @note   get the subnode of the input yaml node with the subnode name , if failed, the progress will end
   * @param  &yaml: the yaml node 
   * @param  &node: the name of the subnode 
   * @retval the subnode
   */
  inline YAML::Node yamlSubNodeAbort(const YAML::Node &yaml, const std::string &node)
  {
    YAML::Node ret = yaml[node.c_str()];
    if (!ret)
    {
      ERROR << name() << " : Cannot find subnode " << node << ". Aborting!!!" << REND;
      exit(-1);
    }
    return std::move(ret);
  }
  /**
   * @brief  get the subnode
   * @note   get the subnode of the input yaml node with the subnode id (only use in subnode with list structure), if failed, the progress will end
   * @param  &yaml: the yaml node
   * @param  &id: the id of the subnode
   * @retval the subnode
   */
  inline YAML::Node yamlSubNodeAbort(const YAML::Node &yaml, const unsigned int &id)
  {
    if (id >= yaml.size())
    {
      ERROR << name() << " : Input id is overrange! Aborting!!!" << REND;
      exit(-1);
    }
    YAML::Node ret = yaml[id];
    return std::move(ret);
  }
  inline YAML::Node yamlSubNode(const YAML::Node &yaml, const std::string &node)
  {
    YAML::Node ret = yaml[node.c_str()];
    if (!ret)
    {
      WARNING << name() << " : Cannot find subnode " << node << ". Returning Null" << REND;
      return YAML::Node();
    }
    return std::move(ret);
  }
  inline YAML::Node yamlSubNode(const YAML::Node &yaml, const unsigned int &id)
  {
    if (id >= yaml.size())
    {
      WARNING << name() << " : Input id is overrange! Returning Null" << REND;
      return YAML::Node();
    }
    YAML::Node ret = yaml[id];
    return std::move(ret);
  }

public:
  /**
   * @brief  get the current system time
   * @retval time
   */
  inline double getTime(void)
  {
    const auto t = std::chrono::system_clock::now();
    const auto t_sec = std::chrono::duration_cast<std::chrono::duration<double>>(t.time_since_epoch());
    return (double)t_sec.count();
  }
  /**
   * @brief  check a module is initialized or not
   * @retval true: is initialied false: not initialized
   */
  virtual inline bool isInitialized(void) { return is_initialized_; }
  /**
   * @brief  get the name of the module
   * @retval name of the module
   */
  virtual inline std::string name(void) { return name_; }

protected:
  /**
   * @brief  set the name of current module
   * @param  &name: the name to be set
   * @retval None
   */
  inline void setName(const std::string &name) { name_ = name; }
  /**
   * @brief  set the varible is_initialized_
   * @note   call this function in the module's initializing function, and set the flag to be true
   * @param  flag: the flag to check initilization
   * @retval None
   */
  inline void setinitFlag(bool flag) { is_initialized_ = flag; }

private:
  bool is_initialized_ = false;
  std::string name_ = "rs_common_default";
};
} // namespace common
} // namespace robosense
