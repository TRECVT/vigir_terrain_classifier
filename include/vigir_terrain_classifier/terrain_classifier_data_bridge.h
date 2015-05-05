/*
 * Terrain Classifier Data Bridge
 *
 * This node just pings vigir_world model main, gets data
 * and ships it to the terrain_classifier_node to try and get things
 * working
 *
 * John Peterson    jrpeter@vt.edu      April 26, 2015
 */

#ifndef FOOTSTEP_CONVERTER_TERRAIN_CLASSIFIER_DATA_BRIDGE_H
#define FOOTSTEP_CONVERTER_TERRAIN_CLASSIFIER_DATA_BRIDGE_H

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <tf/tf.h>
#pragma GCC diagnostic pop

class TerrainClassifierDataBridge
{
public:
  TerrainClassifierDataBridge(ros::NodeHandle& nh);
  ~TerrainClassifierDataBridge();

  void tCallback(const ros::TimerEvent& e);

  double getCBSeconds() const;

  bool getPointCloud();
protected:
  void set_reset(const std_msgs::Empty& msg);
  void set_terrain_hack(const std_msgs::Empty& msg);

private:
  // subscriber to listen if we should reset
  ros::Subscriber _reset_sub;
  ros::Subscriber _activate_hack_sub;

  // service client to get feet pose
  ros::ServiceClient _feet_client;

  // service client to talk to the world model
  ros::ServiceClient _point_cloud_roi_client;

  // and publish that point cloud to the terrain classifier
  ros::Publisher _point_cloud_pub;
  ros::Publisher _update_cloud_pub;

  // how quickly to spin and how many scans to pull each time
  double _cb_seconds;
  size_t _scan_cnt;
  double _request_dimension;

  // transform from Feet to base of feet
  tf::Transform _left_to_base;
  tf::Transform _right_to_base;

  // selecting between using set_point and update
  bool _reset;

  // hack terrain
  // this will for one time step add a bunch of points
  // in a regular grid in the plane of the feet
  bool _hack_local_terrain;

  double _plane_dimension; // meters to side of square centered
  // on the robot
  size_t _plane_samples;
};
#endif