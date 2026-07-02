#pragma once

#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai/pipeline/datatype/CameraControl.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rif_msgs/srv/get_int64.hpp"
#include "rif_msgs/srv/set_int64.hpp"
#include "rif_msgs/srv/set_depth_ai_focus_mode.hpp"
#include "rif_msgs/srv/get_depth_ai_focus_mode.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace dai
{
class Pipeline;
class Device;
class DataInputQueue;
enum class CameraBoardSocket;
class ADatatype;
namespace node
{
class ColorCamera;
class XLinkIn;
}  // namespace node
}  // namespace dai

namespace rclcpp
{
class Node;
class Parameter;
}  // namespace rclcpp

namespace depthai_ros_driver
{
namespace param_handlers
{
class SensorParamHandler;
}
namespace dai_nodes
{

namespace sensor_helpers
{
struct ImageSensor;
class ImagePublisher;
}  // namespace sensor_helpers

class RGB : public BaseNode
{
public:
  explicit RGB(const std::string& daiNodeName, std::shared_ptr<rclcpp::Node> node,
               std::shared_ptr<dai::Pipeline> pipeline, dai::CameraBoardSocket socket,
               sensor_helpers::ImageSensor sensor, bool publish);
  ~RGB();
  void updateParams(const std::vector<rclcpp::Parameter>& params) override;
  void trigger() override;
  void setupQueues(std::shared_ptr<dai::Device> device) override;
  void link(dai::Node::Input in, int linkType = 0) override;
  void setNames() override;
  void setXinXout(std::shared_ptr<dai::Pipeline> pipeline) override;
  void closeQueues() override;
  std::vector<std::shared_ptr<sensor_helpers::ImagePublisher>> getPublishers() override;

  rclcpp::CallbackGroup::SharedPtr setManualFocusCBGroup_;
  void setManualFocusCB(const std::shared_ptr<rif_msgs::srv::SetInt64::Request> req,
                        std::shared_ptr<rif_msgs::srv::SetInt64::Response> res);
  void setFocusModeCB(const std::shared_ptr<rif_msgs::srv::SetDepthAIFocusMode::Request> req,
                      std::shared_ptr<rif_msgs::srv::SetDepthAIFocusMode::Response> res);
  void getFocusModeCB(const std::shared_ptr<rif_msgs::srv::GetDepthAIFocusMode::Request> /* req */,
                      std::shared_ptr<rif_msgs::srv::GetDepthAIFocusMode::Response> res);

  rclcpp::CallbackGroup::SharedPtr setManualExposureCBGroup_;
  void setManualExposureModeCB(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                               std::shared_ptr<std_srvs::srv::SetBool::Response> res);
  void setManualExposureCB(const std::shared_ptr<rif_msgs::srv::SetInt64::Request> req,
                           std::shared_ptr<rif_msgs::srv::SetInt64::Response> res);
  void getExposureValueCB(const std::shared_ptr<rif_msgs::srv::GetInt64::Request> /* req */,
                          std::shared_ptr<rif_msgs::srv::GetInt64::Response> res);

private:
  std::shared_ptr<sensor_helpers::ImagePublisher> rgbPub, previewPub;
  std::shared_ptr<dai::node::ColorCamera> colorCamNode;
  std::unique_ptr<param_handlers::SensorParamHandler> ph;
  std::shared_ptr<dai::DataInputQueue> controlQ;
  std::shared_ptr<dai::node::XLinkIn> xinControl;
  std::string ispQName, previewQName, controlQName;

  dai::CameraControl::AutoFocusMode focus_mode;

  rclcpp::Service<rif_msgs::srv::SetInt64>::SharedPtr setManualFocusSrv;
  rclcpp::Service<rif_msgs::srv::SetDepthAIFocusMode>::SharedPtr setFocusModeSrv;
  rclcpp::Service<rif_msgs::srv::GetDepthAIFocusMode>::SharedPtr getFocusModeSrv;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr setManualExposureModeSrv;
  rclcpp::Service<rif_msgs::srv::SetInt64>::SharedPtr setManualExposureSrv;
  rclcpp::Service<rif_msgs::srv::GetInt64>::SharedPtr getExposureValueSrv;
};

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
