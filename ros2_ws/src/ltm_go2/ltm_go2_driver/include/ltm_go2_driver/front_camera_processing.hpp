/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 04-08-2024.
 */

#ifndef LTM_GO2_DRIVER__FRONT_CAMERA_PROCESSING_HPP_
#define LTM_GO2_DRIVER__FRONT_CAMERA_PROCESSING_HPP_

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <unitree_go/msg/go2_front_video_data.hpp>

#include <string>

#define IMAGE_ENCODING      "rgb8"
#define IMAGE_IS_BIGENDIAN  false
#define IMAGE_STEP_SCALE    3

#define FRONT_CAMERA_FRAME_ID "front_camera"
#define FRONT_CAMERA_180p_WIDTH   320
#define FRONT_CAMERA_180p_HEIGHT  180
#define FRONT_CAMERA_360p_WIDTH   640
#define FRONT_CAMERA_360p_HEIGHT  360
#define FRONT_CAMERA_720p_WIDTH   1280
#define FRONT_CAMERA_720p_HEIGHT  720

namespace LTM
{
  class FrontCameraProcessing
  {
    public:
      FrontCameraProcessing();
      ~FrontCameraProcessing();

      void updateFrontCameraMsgs(const unitree_go::msg::Go2FrontVideoData::SharedPtr msg,
        const rclcpp::Time& timestamp);

      sensor_msgs::msg::Image::SharedPtr getFrontCamera180pMsg() const;
      sensor_msgs::msg::Image::SharedPtr getFrontCamera360pMsg() const;
      sensor_msgs::msg::Image::SharedPtr getFrontCamera720pMsg() const;

    private:
      void initializeFrontCameraMsgs();

      sensor_msgs::msg::Image::SharedPtr m_front_camera_180p_msg;
      sensor_msgs::msg::Image::SharedPtr m_front_camera_360p_msg;
      sensor_msgs::msg::Image::SharedPtr m_front_camera_720p_msg;

  };  // class FrontCamera
} // namespace LTM

#endif  // LTM_GO2_DRIVER__FRONT_CAMERA_PROCESSING_HPP_