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
#include "panta_sdk_demo/panta_sdk_demo.h"
#include "version.h"
#include <signal.h>
robosense::SdkDemo demo; ///< instantiate the SdkDemo, demo

/**
 * @brief  signal handler
 * @note   will be called if receive ctrl+c signal from keyboard during the progress
 *         (all the threads in progress will be stopped and the progress end)
 * @param  sig: the input signal
 * @retval None
 */
static void sigHandler(int sig)
{
  demo.stop();
}

/**
 * @brief  the config file loading function
 * @note   load the yaml file and store the absolute path in the varible config_path
 * @param  &config_path: the absolute path of system_config file
 * @param  &sysconfig: the yaml node to store the config parameters
 * @retval None
 */
void loadYaml(std::string &config_path, YAML::Node &sysconfig)
{

  robosense::common::YamlParser yp; ///< instantiate the YamlParser, yp
  std::string usr_config_name;
  YAML::Node config_name_yaml;
  config_name_yaml = yp.loadFile((std::string)PROJECT_PATH + "/config/usr_config_name.yaml"); ///< load the yaml file
  if (config_name_yaml.Type() == YAML::NodeType::Null)                                        ///< if failed, abort!
  { 
    ERROR << "Load usr_config_name yaml file error with path : " << (std::string)PROJECT_PATH + "/config/usr_config_name.yaml" << REND;
    exit(-1);
  }
  demo.yamlReadAbort(config_name_yaml, "usr_config_name", usr_config_name);
  config_path = (std::string)PROJECT_PATH + "/config/system_config";
  sysconfig = yp.loadFile(config_path + "/general_config/config.yaml");                                    ///< load the system config file with the absolute path
  YAML::Node usrconfig = yp.loadFile((std::string)PROJECT_PATH + "/config/usr_config/" + usr_config_name); ///< load the user config file
  sysconfig = yp.loadAndMerge(sysconfig, usrconfig);                                                       ///< merge the system config file and user config file
  if (sysconfig.Type() != YAML::NodeType::Null)
  {
    TITLE << "******************************************************************" << REND;
    TITLE << "       RS_SDK_VERSION : " << PANTA_SDK_VERSION_MAJOR << "."
          << PANTA_SDK_VERSION_MINOR << "."
          << PANTA_SDK_VERSION_PATCH << REND;
    TITLE << "       Config_file : " << usr_config_name << REND;
    TITLE << "******************************************************************" << REND;
  }
  else
  {
    ERROR << "Load config yaml file error with path : " << (std::string)PROJECT_PATH + "/config/usr_config/" + usr_config_name << REND;
    exit(-1);
  }
}

/**
 * @brief  main function
 * @note the only entrance of the program
 */
int main(int argc, char **argv)
{
  bool use_ros;
  bool use_proto;
  YAML::Node config;
  std::string config_path;    ///< the absolute path of config file
  signal(SIGINT, sigHandler); ///< bind the ctrl+c signal with the the handler function
  loadYaml(config_path, config);
#if (DEBUG_LEVEL > 0)
  DEBUG << config << REND;
#endif
  demo.yamlRead<bool>(config["general"], "use_ros", use_ros, false);
  demo.yamlRead<bool>(config["general"], "use_proto", use_proto, false); ///< read the parameter use_ros & use_proto from yaml config file
#ifdef ROS_FOUND
  if (use_ros)
    ros::init(argc, argv, "panta_sdk_demo"); ///< if use_ros is true, ros::init() will be called
#endif
  demo.init(config, use_ros, use_proto, config_path);
  demo.start();
}
