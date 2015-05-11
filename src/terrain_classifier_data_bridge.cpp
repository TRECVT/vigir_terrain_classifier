/*
 * Terrain Classifier Data Bridge
 *
 * This node just pings vigir_world model main, gets data
 * and ships it to the terrain_classifier_node to try and get things
 * working
 *
 * John Peterson    jrpeter@vt.edu      April 26, 2015
 */

#include <vigir_terrain_classifier/terrain_classifier_data_bridge.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#include <vigir_footstep_planning_msgs/GenerateFeetPoseService.h>
#include <sensor_msgs/PointCloud2.h>
#include <flor_perception_msgs/PointCloudRegionRequest.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_ros/point_cloud.h>
#include <Eigen/Core>
#pragma GCC diagnostic pop

#include <cmath>

namespace
{
  const double DEFAULT_SCAN_HZ = 40.0;
  const size_t DEFAULT_PLANE_SAMPLES = 4000;
  const double DEFAULT_PLANE_DIMENSION = 3.0;
  const double DEFAULT_CB_SECONDS = 1.0;
  const double DEFAULT_REQUEST_DIMENSION = 5.0;
}

TerrainClassifierDataBridge::TerrainClassifierDataBridge(ros::NodeHandle& nh)
  : _reset_sub(),
    _feet_client(),
    _point_cloud_roi_client(),
    _point_cloud_pub(),
    _update_cloud_pub(),
    _cb_seconds(DEFAULT_CB_SECONDS),
    _scan_cnt(static_cast<size_t>(floor(DEFAULT_SCAN_HZ * DEFAULT_CB_SECONDS))),
    _request_dimension(DEFAULT_REQUEST_DIMENSION),
    _left_to_base(tf::Transform::getIdentity()),
    _right_to_base(tf::Transform::getIdentity()),
    _reset(true),
    _hack_local_terrain(true),
    _plane_dimension(DEFAULT_PLANE_DIMENSION),
    _plane_samples(DEFAULT_PLANE_SAMPLES)
{
  // load parameters
  double scan_hz = DEFAULT_SCAN_HZ;
  if (nh.getParam("scan_hz", scan_hz) && (scan_hz <= 0.0))
  {
    ROS_ERROR("TerrainDataBridge: scan_hz was expected to be positive, setting default: %f", DEFAULT_SCAN_HZ);
    scan_hz = DEFAULT_SCAN_HZ;
  }

  if (nh.getParam("update_seconds", _cb_seconds) && (_cb_seconds <= 0.0))
  {
    ROS_ERROR("TerrainDataBridge: update_seconds was expected to be positive, setting default: %f", DEFAULT_CB_SECONDS);
    _cb_seconds = DEFAULT_CB_SECONDS;
  }

  // dimensions for the plane hack
  int plane_samples_int = static_cast<int>(DEFAULT_PLANE_SAMPLES);
  if (nh.getParam("hack_plane_samples", plane_samples_int) && (plane_samples_int <= 0))
  {
    ROS_ERROR("TerrainDataBridge: hack_plane_samples was expected to be positive, setting default %lu", DEFAULT_PLANE_SAMPLES);
    plane_samples_int = static_cast<int>(DEFAULT_PLANE_SAMPLES);
  }
  _plane_samples = static_cast<size_t>(plane_samples_int);

  if (nh.getParam("hack_plane_dimension", _plane_dimension) && (_plane_dimension <= 0.0))
  {
    ROS_ERROR("TerrainDataBridge:: hack_plane_dimension was expected to be positive, setting default %f", DEFAULT_PLANE_DIMENSION);
    _plane_dimension = DEFAULT_PLANE_DIMENSION;
  }

  // transform between the feet and the base of the feet for the plane hack
  double t_x = 0.0;
  double t_y = 0.0;
  double t_z = 0.0;
  double t_roll = 0.0;
  double t_pitch = 0.0;
  double t_yaw = 0.0;

  if (nh.getParam("foot/left/foot_frame/x", t_x) &&
      nh.getParam("foot/left/foot_frame/y", t_y) &&
      nh.getParam("foot/left/foot_frame/z", t_z) &&
      nh.getParam("foot/left/foot_frame/roll", t_roll) &&
      nh.getParam("foot/left/foot_frame/pitch", t_pitch) &&
      nh.getParam("foot/left/foot_frame/yaw", t_yaw))
  {
    _left_to_base.setOrigin(tf::Point(t_x, t_y, t_z));

    tf::Quaternion t_q;
    t_q.setRPY(t_roll, t_pitch, t_yaw);

    _left_to_base.setRotation(t_q);
  }
  else
  {
    ROS_WARN("No transform from left foot to foot sole avaialble");
  }

  if (nh.getParam("foot/right/foot_frame/x", t_x) &&
      nh.getParam("foot/right/foot_frame/y", t_y) &&
      nh.getParam("foot/right/foot_frame/z", t_z) &&
      nh.getParam("foot/right/foot_frame/roll", t_roll) &&
      nh.getParam("foot/right/foot_frame/pitch", t_pitch) &&
      nh.getParam("foot/right/foot_frame/yaw", t_yaw))
  {
    _left_to_base.setOrigin(tf::Point(t_x, t_y, t_z));

    tf::Quaternion t_q;
    t_q.setRPY(t_roll, t_pitch, t_yaw);

    _right_to_base.setRotation(t_q);
  }
  else
  {
    ROS_WARN("No transform from right foot to foot sole avaialble");
  }

  // compute number of scans to pull on each update
  _scan_cnt = static_cast<size_t>(floor(scan_hz * _cb_seconds));

  _feet_client = nh.serviceClient<vigir_footstep_planning_msgs::GenerateFeetPoseService>("generate_feet_pose");
  _point_cloud_roi_client = nh.serviceClient<flor_perception_msgs::PointCloudRegionRequest>("/flor/worldmodel/pointcloud_roi");
  _point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("set_point_cloud", 10, true);
  _update_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_update", 10);

  _reset_sub = nh.subscribe("reset_terrain", 1, &TerrainClassifierDataBridge::set_reset, this);
  _activate_hack_sub = nh.subscribe("plane_hack_once", 1, &TerrainClassifierDataBridge::set_terrain_hack, this);
}

TerrainClassifierDataBridge::~TerrainClassifierDataBridge()
{

}

double TerrainClassifierDataBridge::getCBSeconds() const
{
  return _cb_seconds;
}

void TerrainClassifierDataBridge::tCallback(const ros::TimerEvent& e)
{
  getPointCloud();
}

void TerrainClassifierDataBridge::set_reset(const std_msgs::Empty& msg)
{
  _reset = true;
}

void TerrainClassifierDataBridge::set_terrain_hack(const std_msgs::Empty& msg)
{
  _hack_local_terrain = true;
}

bool TerrainClassifierDataBridge::getPointCloud()
{
  // get the foot pose
  vigir_footstep_planning_msgs::GenerateFeetPoseService get_feet_msg;
  get_feet_msg.request.request.header.stamp = ros::Time::now();
  get_feet_msg.request.request.header.frame_id = "map";

  // this flag returns the current feet position
  get_feet_msg.request.request.flags = vigir_footstep_planning_msgs::FeetPoseRequest::FLAG_CURRENT;

  if (!_feet_client.call(get_feet_msg) || (get_feet_msg.response.status.error != vigir_footstep_planning_msgs::ErrorStatus::NO_ERROR))
  {
    ROS_WARN("TerrainDataBridge: Call to get_feet failed error = %u", get_feet_msg.response.status.error);
    return false;
  }

  // convert left foot to a point
  // get_feet_msg.feet.left.pose // this is a geometry_msgs/Pose
  // get a point cloud
  flor_perception_msgs::PointCloudRegionRequest cloud_srv;

  cloud_srv.request.aggregation_size = _scan_cnt; // default aggregation size

  // need to give it a bounding box
  cloud_srv.request.region_req.header.stamp = get_feet_msg.response.feet.left.header.stamp;
  cloud_srv.request.region_req.header.frame_id = "/world"; // because they use silly frame names

  // bounding box minimum
  cloud_srv.request.region_req.bounding_box_min.x = get_feet_msg.response.feet.left.pose.position.x - _request_dimension;
  cloud_srv.request.region_req.bounding_box_min.y = get_feet_msg.response.feet.left.pose.position.y - _request_dimension;
  cloud_srv.request.region_req.bounding_box_min.z = get_feet_msg.response.feet.left.pose.position.z - _request_dimension;

  // bounding box maximum
  cloud_srv.request.region_req.bounding_box_max.x = get_feet_msg.response.feet.left.pose.position.x + _request_dimension;
  cloud_srv.request.region_req.bounding_box_max.y = get_feet_msg.response.feet.left.pose.position.y + _request_dimension;
  cloud_srv.request.region_req.bounding_box_max.z = get_feet_msg.response.feet.left.pose.position.z + _request_dimension;

  cloud_srv.request.region_req.resolution = 0.0; // default request
  cloud_srv.request.region_req.request_augment = static_cast<uint8_t>(0); // no map augments??

  if (!_point_cloud_roi_client.call(cloud_srv))
  {
    ROS_WARN("TerrainDataBridge:: call to get point cloud roi failed");
    return false;
  }

  sensor_msgs::PointCloud2* cloud_to_pub = &cloud_srv.response.cloud;

  // fire off the hacked terrain once at the beginning
  sensor_msgs::PointCloud2 hacked_cloud;
  if (_hack_local_terrain)
  {
    // messy type conversions
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_int(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(cloud_srv.response.cloud, *cloud_int);

    // we have the left foot position and orientation
    // to start just get the position and use it at the center
    // Making flat ground
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    plane_cloud->width = _plane_samples * _plane_samples;
    plane_cloud->height = 1;
    plane_cloud->points.resize(plane_cloud->width * plane_cloud->height);

    double increment = _plane_dimension / static_cast<double>(_plane_samples);
    double offset = -0.5 * _plane_dimension;

    // apply transform
    tf::Pose foot_pose;
    tf::poseMsgToTF(get_feet_msg.response.feet.left.pose, foot_pose);
    tf::Pose foot_sole = _left_to_base * foot_pose;
    //tf::Pose foot_sole = foot_pose;

    ROS_INFO("Foot sole = [%f, %f, %f]", foot_sole.getOrigin().x(),  foot_sole.getOrigin().y(),  foot_sole.getOrigin().z());
    for (size_t jj = 0; jj < _plane_samples; ++jj)
    {
      for (size_t ii = 0; ii < _plane_samples; ++ii)
      {
        // sample points from plane
        plane_cloud->points[jj * _plane_samples + ii].x = foot_sole.getOrigin().x() + jj * increment + offset;
        plane_cloud->points[jj * _plane_samples + ii].y = foot_sole.getOrigin().y() + ii * increment + offset;
        plane_cloud->points[jj * _plane_samples + ii].z = foot_sole.getOrigin().z();
      }
    }

    *cloud_int += *plane_cloud; // append point cloud

    // convert back to a ros message and publish
    pcl::toROSMsg(*cloud_int, hacked_cloud);

    // set header correctly
    hacked_cloud.header = cloud_srv.response.cloud.header;

    cloud_to_pub = &hacked_cloud;

    _hack_local_terrain = false;
  }

  if (_reset)
  {
    _point_cloud_pub.publish(*cloud_to_pub); // publish on this channel to reset the terrain map
    _reset = false; // since the message is latched, it should get the set message
  }
  else
  {
    _update_cloud_pub.publish(*cloud_to_pub); // just provides an update
  }

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "terrain_classifier_data_bridge");
  ros::NodeHandle nh;

  ros::service::waitForService("/flor/worldmodel/pointcloud_roi");
  ros::service::waitForService("generate_feet_pose");

  // note goal manager can only handle a single thread
  TerrainClassifierDataBridge bridge(nh);

  ros::Timer T = nh.createTimer(ros::Duration(1.0), &TerrainClassifierDataBridge::tCallback, &bridge);

  ros::spin();
  return 0;
}