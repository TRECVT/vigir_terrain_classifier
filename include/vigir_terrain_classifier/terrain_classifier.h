//=================================================================================================
// Copyright (c) 2015, Alexander Stumpf, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef VIGIR_TERRAIN_CLASSIFIER_H__
#define VIGIR_TERRAIN_CLASSIFIER_H__

#include <ros/ros.h>

#include <boost/thread/mutex.hpp>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>

//#include <pcl/surface/mls_omp.h>
#include <pcl/surface/mls.h>

#include <pcl/surface/gp3.h>

#include <pcl/surface/vtk_smoothing/vtk.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>

#include <nav_msgs/OccupancyGrid.h>

#include <vigir_footstep_planning_msgs/footstep_planning_msgs.h>

#include <vigir_footstep_planning_lib/helper.h>

#include <vigir_terrain_classifier/pcl/octree_voxel_grid.h>
#include <vigir_terrain_classifier/pcl/point_cloud_filter.h>
#include <vigir_terrain_classifier/TerrainModelMsg.h>
#include <vigir_terrain_classifier/terrain_model.h>

#include <vigir_terrain_classifier/generator/height_grid_map_generator.h>



namespace vigir_terrain_classifier
{
enum filterMask
{
  FILTER_NONE                   = 0,
  FILTER_PASS_THROUGH_BOX       = 1,
  FILTER_PASS_THROUGH_ELLIPSE   = 2,
  FILTER_STATISTICAL_OUTLIER    = 4,
  FILTER_VOXEL_GRID             = 8,
  FILTER_MLS_SMOOTH             = 16,

  FILTER_ALL                    = 255
};

struct TerrainClassifierParams
{
  TerrainClassifierParams()
  {
  }

  TerrainClassifierParams(const ros::NodeHandle& nh)
  {
    readParams(nh);
  }

  void readParams(const ros::NodeHandle& nh)
  {
    nh.param("filter_mask", (int&)filter_mask, (int)FILTER_ALL);
    nh.param("threads", threads, 4);

    nh.param("world_frame_id", world_frame_id, std::string("/world"));
    vigir_footstep_planning::strip(world_frame_id, '/');

    nh.param("aggregation_res", aggregation_res, 0.01);
    nh.param("min_aggregation_size", (int&)min_aggregation_size, 5000);
    nh.param("max_aggregation_size", (int&)max_aggregation_size, 100000);
    nh.param("aggregation_rx", aggregation_rx, 4.0);
    nh.param("aggregation_ry", aggregation_ry, 2.0);
    nh.param("update_weight", update_weight, 1.0);

    nh.param("normals_res", normals_res, 0.05);

    nh.param("pass_through/field_name", pt_field_name, std::string("z"));
    nh.param("pass_through/min", pt_min, -0.2);
    nh.param("pass_through/max", pt_max, 0.5);

    nh.param("voxel_grid/lx", vg_lx, 0.01);
    nh.param("voxel_grid/ly", vg_ly, 0.01);
    nh.param("voxel_grid/lz", vg_lz, 0.01);

    nh.param("mls_smoother/radius", ms_radius, 0.1);

    nh.param("statistical_outlier/radius", so_radius, 0.01);
    nh.param("statistical_outlier/k", so_k, 25);

    nh.param("normal_estimator/radius", ne_radius, 0.05);

    nh.param("gradient_estimator/gradient_obstacle", ge_obstacle, false);
    nh.param("gradient_estimator/thresh", ge_thresh, 0.5);

    nh.param("edge_detector/radius", ed_radius, 0.05);
    nh.param("edge_detector/max_std", ed_max_std, 0.02);
    nh.param("edge_detector/non_max_supp_radius", ed_non_max_supp_radius, 0.05);
    
    int minimum_neighbor_cnt = 0;
    nh.param("edge_detector/minimum_neighbors", minimum_neighbor_cnt, 0);
    if (minimum_neighbor_cnt >= 0)
    {
      ed_min_neighbor_cnt = static_cast<size_t>(minimum_neighbor_cnt);
    }
    else
    {
      ed_min_neighbor_cnt = 0;
    }

    nh.param("grid_map_generator/resolution", gg_res, 0.02);
    nh.param("grid_map_generator/reconstruct", gg_reconstruct, false);

    nh.param("height_grid_map/resolution", hgm_res, 0.01);
    nh.param("height_grid_map/min_expansion_size", hgm_min_expansion_size, 1.0);
    nh.param("height_grid_map/update_weight", hgm_update_weight, 1.0);

    nh.param("misc/low_res", low_res, 0.05);
  }

  unsigned int filter_mask;

  // global frame id
  std::string world_frame_id;

  // number of threads should be used
  int threads;

  // resolution of aggregated data
  double aggregation_res;

  // maximal size of aggregated input point cloud
  unsigned int min_aggregation_size;
  unsigned int max_aggregation_size;

  // describes ellipse around current robot pose to be considerd
  double aggregation_rx;
  double aggregation_ry;

  // weight of each update (lowpass filtering)
  double update_weight;

  // resolution of normal data
  double normals_res;

  // params for pass through filter
  std::string pt_field_name;
  double pt_min, pt_max;

  // params for voxel grid filter
  double vg_lx, vg_ly, vg_lz;

  // params for moving least squares smoother
  double ms_radius;

  // params for statistical outlier filter
  double so_radius;
  int so_k;

  // params for normal estimator
  double ne_radius;

  // params for gradients estimator
  bool ge_obstacle;
  double ge_thresh;

  // params for edge detector
  double ed_radius;
  double ed_max_std;
  double ed_non_max_supp_radius;
  size_t ed_min_neighbor_cnt;

  // params for grid map generator
  double gg_res;
  bool gg_reconstruct;

  // params for height grid map generation
  double hgm_res;
  double hgm_min_expansion_size;
  double hgm_update_weight;

  // misc params
  double low_res;
};

class TerrainClassifier
{
public:
  TerrainClassifier(ros::NodeHandle& nh, const TerrainClassifierParams& params);
  TerrainClassifier(ros::NodeHandle& nh);
  virtual ~TerrainClassifier();

  void reset();

  void setParams(const TerrainClassifierParams& params);
  void getParams(TerrainClassifierParams& params) const { params = this->params; }

  void filterPointCloudData(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

  void setPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);
  void insertPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);

  bool hasTerrainModel() const;

  // some getters
  pcl::PointCloud<pcl::PointXYZ>::Ptr getInputCloud() const;
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& getCloudProcessed() const;
  const pcl::PointCloud<pcl::PointNormal>::ConstPtr getPointsWithsNormals() const;
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& getGradients() const;
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& getEdges() const;

  void getCloudProcessedLowRes(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const;

  void getTerrainModel(TerrainModelMsg& model) const;
  const nav_msgs::OccupancyGrid::Ptr& getGroundLevelGridMap() const;
  nav_msgs::OccupancyGrid::Ptr getHeightGridMap(double& height_grid_map_scale) const;
  nav_msgs::OccupancyGrid::Ptr getHeightGridMapRescaled(int8_t min_val = 0, int8_t max_val = 100) const;

  const pcl::PolygonMesh::Ptr& getMeshSurface() const;

  const std::string& getFrameId() const { return params.world_frame_id; }

  // data generation
  bool computeNormals(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr update = pcl::PointCloud<pcl::PointXYZ>::ConstPtr());
  bool computeGradients(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr update = pcl::PointCloud<pcl::PointXYZ>::ConstPtr());
  bool detectEdges();
  bool generateGroundLevelGridmap();
  bool generateHeightGridmap(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr update = pcl::PointCloud<pcl::PointXYZ>::ConstPtr());

  bool computeSurfaceMesh();

  // typedefs
  typedef boost::shared_ptr<TerrainClassifier> Ptr;
  typedef boost::shared_ptr<const TerrainClassifier> ConstPtr;

protected:
  void setDataOutdated();

  // some helper
  bool determineCurrentPose(geometry_msgs::Pose& pose);
  double determineCurrentGroundHeight();

//  template <typename PointT> void setupGridMap(const pcl::PointCloud<PointT>& cloud, nav_msgs::OccupancyGrid::Ptr& map, float res) const
//  {
//    double min_x, max_x;
//    double min_y, max_y;
//    double min_z, max_z;

//    min_x = min_y = min_z = FLT_MAX;
//    max_x = max_y = max_z = FLT_MIN;
//    for (size_t i = 0; i < cloud->size(); i++)
//    {
//      const pcl::PointNormal& p = cloud->at(i);
//      min_x = min(min_x, (double)p.x);
//      min_y = min(min_y, (double)p.y);
//      min_z = min(min_z, (double)p.z);
//      max_x = max(max_x, (double)p.x);
//      max_y = max(max_y, (double)p.y);
//      max_z = max(max_z, (double)p.z);
//    }

//    // setting up grid map
//    map.reset(new nav_msgs::OccupancyGrid());

//    map->info.resolution = params.gg_res;
//    map->info.width = ceil((max_x-min_x) * 1/res);
//    map->info.height = ceil((max_y-min_y) * 1/res);

//    map->info.origin.position.x = min_x;
//    map->info.origin.position.y = min_y;
//    map->info.origin.position.z = 0.0;

//    map->data.clear();
//    map->data.resize(map->info.width * map->info.height, -1);
//  }

  // service clients
  ros::ServiceClient generate_feet_pose_client;

  TerrainClassifierParams params;

  TerrainModel::Ptr terrain_model;

  HeightGridMapGenerator::Ptr height_grid_map_generator;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_processed;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_gradients;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_edges;

  OctreeVoxelGrid<pcl::PointXYZ>::Ptr input_octree;
  OctreeVoxelGrid<pcl::PointNormal>::Ptr normals_octree;

  nav_msgs::OccupancyGrid::Ptr ground_level_grid_map;

  pcl::PolygonMesh::Ptr mesh_surface;

  bool cloud_normals_outdated;
  bool cloud_gradients_outdated;
  bool cloud_edges_outdated;

  bool ground_level_grid_map_outdated;
  bool height_grid_grid_map_outdated;

  bool mesh_surface_outdated;
};
}

#endif
