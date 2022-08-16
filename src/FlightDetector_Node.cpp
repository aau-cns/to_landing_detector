/// Copyright (C) 2022 Martin Scheiber, Control of Networked Systems, University of Klagenfurt, Austria.
///
/// All rights reserved.
///
/// This software is licensed under the terms of the BSD-2-Clause-License with
/// no commercial use allowed, the full terms of which are made available
/// in the LICENSE file. No license in patents is granted.
///
/// You can contact the author at <martin.scheiber@ieee.org>

#include <ros/ros.h>

#include "toland_flight/FlightDetector.hpp"

int main(int argc, char* argv[])
{
  // initialize ros
  ros::init(argc, argv, "toland_flight");

  // set logging level
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
    ros::console::notifyLoggerLevelsChanged();

  // initilize FlightDetector
  toland::FlightDetector flight_detector;

  //  print published/subscribed topics
  ros::V_string topics;
  ros::this_node::getSubscribedTopics(topics);
  std::string nodeName = ros::this_node::getName();
  std::string topicsStr = nodeName + ":\n\tsubscribed to topics:\n";
  for (unsigned int i = 0; i < topics.size(); i++)
    topicsStr += ("\t\t" + topics.at(i) + "\n");

  topicsStr += "\tadvertised topics:\n";
  ros::this_node::getAdvertisedTopics(topics);
  for (unsigned int i = 0; i < topics.size(); i++)
    topicsStr += ("\t\t" + topics.at(i) + "\n");

  ROS_INFO_STREAM("" << topicsStr);

  // spin ROS
  ros::spin();

  return EXIT_SUCCESS;
}
