/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 30-07-2024.
 */

#include <ltm_pointcloud_filter/robot_cluster_removal.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <pcl/filters/uniform_sampling.h>
#include <urdf_parser/urdf_parser.h>
#include <pcl_ros/transforms.hpp>

#include <iostream>

namespace LTM {

  RobotClusterRemoval::RobotClusterRemoval(rclcpp::Clock::SharedPtr clock) {
    setClock(clock);
    initializeTransformListener(clock);
  }

  bool RobotClusterRemoval::setRobotModel(const std::string &robot_description) {
    // Load the robot model from the URDF if it exists
    if (!m_robot_model.initFile(robot_description)) {
      std::cout << "Failed to load the robot model from URDF file." << std::endl;
      return false;
    }
    generateRobotMeshes();
    return true;
  }

  RobotClusterRemoval::~RobotClusterRemoval() {
    // Reset the robot model
    m_robot_model.clear();
    m_robot_meshes.clear();

    // Reset the transform listener
    m_tf_buffer.reset();
    m_tf_listener.reset();
  }

  void RobotClusterRemoval::setRobotMeshResolution(const double& resolution) {
    m_robot_mesh_resolution = resolution;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr RobotClusterRemoval::getRobotMesh(
    const std::string &link_name) const {
    // Check if the link exists
    try {
      // Get the point cloud of the link mesh in the link frame
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      *cloud = *m_robot_meshes.at(link_name);
      cloud->header.frame_id = link_name;

      // Transform the point cloud to the world frame
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
      transformPointCloud(cloud, cloud_transformed, "world");

      return cloud_transformed;

    } catch (const std::out_of_range &e) {
      std::cout << "Link " << link_name << " does not exist in the robot model." << std::endl;
      return nullptr;
    }
  }

  void RobotClusterRemoval::transformPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output, const std::string &target_frame) const {
    // // Check if the transform is available
    // if (!m_tf_buffer->canTransform(target_frame, source_frame, rclcpp::Time(0))) {
    //   std::cout << "Transform between " << target_frame << " and " << source_frame << " is not available." << std::endl;
    //   return;
    // }

    // Transform the point cloud
    pcl_ros::transformPointCloud(target_frame, *cloud_input, *cloud_output, *m_tf_buffer);

    // Set the frame ID
    cloud_output->header.frame_id = target_frame;
  }

  void RobotClusterRemoval::generateRobotMeshes() {
    // Initialize uniform sampling
    pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
    uniform_sampling.setRadiusSearch(m_robot_mesh_resolution);

    for (const auto &link : m_robot_model.links_) {
      // Skip links without visual geometry
      if (link.second->visual->geometry->type != urdf::Geometry::MESH) {
        continue;
      }

      // Get the mesh file path
      const auto mesh = std::static_pointer_cast<urdf::Mesh>(link.second->visual->geometry);
      const std::string mesh_path = mesh->filename;

      // Load the mesh
      Assimp::Importer importer;
      const aiScene *scene = importer.ReadFile(mesh_path, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices);

      // Check if the mesh was loaded
      if (!scene || scene->mNumMeshes == 0) {
        continue;
      }

      // Create a new point cloud
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_original(new pcl::PointCloud<pcl::PointXYZ>);

      // Iterate over the mesh vertices
      for (unsigned int i = 0; i < scene->mMeshes[0]->mNumVertices; i++) {
        pcl::PointXYZ point;
        point.x = scene->mMeshes[0]->mVertices[i].x;
        point.y = scene->mMeshes[0]->mVertices[i].y;
        point.z = scene->mMeshes[0]->mVertices[i].z;
        cloud_original->push_back(point);
      }

      // Uniformly sample the point cloud
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sampled(new pcl::PointCloud<pcl::PointXYZ>);
      uniform_sampling.setInputCloud(cloud_original);
      uniform_sampling.filter(*cloud_sampled);

      // Add the point cloud to the map
      m_robot_meshes[link.first] = cloud_sampled;
    }
  }

  void RobotClusterRemoval::initializeTransformListener(const rclcpp::Clock::SharedPtr clock) {
    // Initialize the transform listener
    m_tf_buffer = std::make_shared<tf2_ros::Buffer>(clock);
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
  }

  void RobotClusterRemoval::setClock(const rclcpp::Clock::SharedPtr clock) {
    m_clock = clock;
  }

} // namespace LTM