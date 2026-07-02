#include "depthai_ros_driver/dai_nodes/sensors/rgb.hpp"

#include "camera_info_manager/camera_info_manager.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/VideoEncoder.hpp"
#include "depthai/pipeline/node/XLinkIn.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/img_pub.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver/param_handlers/sensor_param_handler.hpp"
#include "rclcpp/node.hpp"

#include "rif_msgs/srv/set_int64.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
RGB::RGB(const std::string& daiNodeName,
         std::shared_ptr<rclcpp::Node> node,
         std::shared_ptr<dai::Pipeline> pipeline,
         dai::CameraBoardSocket socket = dai::CameraBoardSocket::CAM_A,
         sensor_helpers::ImageSensor sensor = {"IMX378", "4k", {"12mp", "4k"}, dai::CameraSensorType::COLOR},
         bool publish = true)
    : BaseNode(daiNodeName, node, pipeline) {
    RCLCPP_DEBUG(getLogger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    colorCamNode = pipeline->create<dai::node::ColorCamera>();
    ph = std::make_unique<param_handlers::SensorParamHandler>(node, daiNodeName, socket);
    ph->declareParams(colorCamNode, sensor, publish);
    setXinXout(pipeline);
    RCLCPP_DEBUG(getLogger(), "Node %s created", daiNodeName.c_str());

    // In your Node class constructor or a suitable scope
    setManualFocusCBGroup_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    setManualFocusSrv = node->create_service<rif_msgs::srv::SetInt64>("~/set_manual_focus",
                                                                      std::bind(&RGB::setManualFocusCB, this, std::placeholders::_1, std::placeholders::_2),
                                                                      rmw_qos_profile_services_default,
                                                                      setManualFocusCBGroup_);

    setFocusModeSrv =
        node->create_service<rif_msgs::srv::SetDepthAIFocusMode>("~/set_focus_mode",
                                                                 std::bind(&RGB::setFocusModeCB, this, std::placeholders::_1, std::placeholders::_2),
                                                                 rmw_qos_profile_services_default,
                                                                 setManualFocusCBGroup_);

    getFocusModeSrv =
        node->create_service<rif_msgs::srv::GetDepthAIFocusMode>("~/get_focus_mode",
                                                                 std::bind(&RGB::getFocusModeCB, this, std::placeholders::_1, std::placeholders::_2),
                                                                 rmw_qos_profile_services_default,
                                                                 setManualFocusCBGroup_);

    bool man_focus_mode = ph->getParam<bool>("r_set_man_focus");
    focus_mode = man_focus_mode ? dai::CameraControl::AutoFocusMode::OFF : dai::CameraControl::AutoFocusMode::CONTINUOUS_VIDEO;
}
RGB::~RGB() = default;
void RGB::setNames() {
    ispQName = getName() + "_isp";
    previewQName = getName() + "_preview";
    controlQName = getName() + "_control";
}

void RGB::setXinXout(std::shared_ptr<dai::Pipeline> pipeline) {
    bool outputIsp = ph->getParam<bool>("i_output_isp");
    bool lowBandwidth = ph->getParam<bool>("i_low_bandwidth");
    std::function<void(dai::Node::Input)> rgbLinkChoice;
    if(outputIsp && !lowBandwidth) {
        rgbLinkChoice = [&](auto input) { colorCamNode->isp.link(input); };
    } else {
        rgbLinkChoice = [&](auto input) { colorCamNode->video.link(input); };
    }
    if(ph->getParam<bool>("i_publish_topic")) {
        utils::VideoEncoderConfig encConfig;
        encConfig.profile = static_cast<dai::VideoEncoderProperties::Profile>(ph->getParam<int>("i_low_bandwidth_profile"));
        encConfig.bitrate = ph->getParam<int>("i_low_bandwidth_bitrate");
        encConfig.frameFreq = ph->getParam<int>("i_low_bandwidth_frame_freq");
        encConfig.quality = ph->getParam<int>("i_low_bandwidth_quality");
        encConfig.enabled = lowBandwidth;

        rgbPub = setupOutput(pipeline, ispQName, rgbLinkChoice, ph->getParam<bool>("i_synced"), encConfig);
        rgbPub->enable_trigger_mode(ph->getParam<bool>("i_trigger_mode"));
    }
    if(ph->getParam<bool>("i_enable_preview")) {
        previewPub = setupOutput(pipeline, previewQName, [&](auto input) { colorCamNode->preview.link(input); });
    }
    xinControl = pipeline->create<dai::node::XLinkIn>();
    xinControl->setStreamName(controlQName);
    xinControl->out.link(colorCamNode->inputControl);
}

void RGB::setupQueues(std::shared_ptr<dai::Device> device) {
    if(ph->getParam<bool>("i_publish_topic")) {
        auto tfPrefix = getOpticalTFPrefix(getSocketName(static_cast<dai::CameraBoardSocket>(ph->getParam<int>("i_board_socket_id"))));
        utils::ImgConverterConfig convConfig;
        convConfig.tfPrefix = tfPrefix;
        convConfig.getBaseDeviceTimestamp = ph->getParam<bool>("i_get_base_device_timestamp");
        convConfig.updateROSBaseTimeOnRosMsg = ph->getParam<bool>("i_update_ros_base_time_on_ros_msg");
        convConfig.lowBandwidth = ph->getParam<bool>("i_low_bandwidth");
        if(ph->getParam<std::string>("i_color_order") == "BGR") {
            convConfig.encoding = dai::RawImgFrame::Type::BGR888i;
        } else {
            convConfig.encoding = dai::RawImgFrame::Type::RGB888i;
        }
        convConfig.addExposureOffset = ph->getParam<bool>("i_add_exposure_offset");
        convConfig.expOffset = static_cast<dai::CameraExposureOffset>(ph->getParam<int>("i_exposure_offset"));
        convConfig.reverseSocketOrder = ph->getParam<bool>("i_reverse_stereo_socket_order");

        utils::ImgPublisherConfig pubConfig;
        pubConfig.daiNodeName = getName();
        pubConfig.topicName = "~/" + getName();
        pubConfig.lazyPub = ph->getParam<bool>("i_enable_lazy_publisher");
        pubConfig.socket = static_cast<dai::CameraBoardSocket>(ph->getParam<int>("i_board_socket_id"));
        pubConfig.calibrationFile = ph->getParam<std::string>("i_calibration_file");
        pubConfig.rectified = false;
        pubConfig.width = ph->getParam<int>("i_width");
        pubConfig.height = ph->getParam<int>("i_height");
        pubConfig.maxQSize = ph->getParam<int>("i_max_q_size");
        pubConfig.publishCompressed = ph->getParam<bool>("i_publish_compressed");

        rgbPub->setup(device, convConfig, pubConfig);
    }
    if(ph->getParam<bool>("i_enable_preview")) {
        auto tfPrefix = getOpticalTFPrefix(getSocketName(static_cast<dai::CameraBoardSocket>(ph->getParam<int>("i_board_socket_id"))));
        utils::ImgConverterConfig convConfig;
        convConfig.tfPrefix = tfPrefix;
        convConfig.getBaseDeviceTimestamp = ph->getParam<bool>("i_get_base_device_timestamp");
        convConfig.updateROSBaseTimeOnRosMsg = ph->getParam<bool>("i_update_ros_base_time_on_ros_msg");

        utils::ImgPublisherConfig pubConfig;
        pubConfig.daiNodeName = getName();
        pubConfig.topicName = "~/" + getName();
        pubConfig.lazyPub = ph->getParam<bool>("i_enable_lazy_publisher");
        pubConfig.socket = static_cast<dai::CameraBoardSocket>(ph->getParam<int>("i_board_socket_id"));
        pubConfig.calibrationFile = ph->getParam<std::string>("i_calibration_file");
        pubConfig.rectified = false;
        pubConfig.width = ph->getParam<int>("i_preview_width");
        pubConfig.height = ph->getParam<int>("i_preview_height");
        pubConfig.maxQSize = ph->getParam<int>("i_max_q_size");
        pubConfig.topicSuffix = "/preview/image_raw";

        previewPub->setup(device, convConfig, pubConfig);
    };
    controlQ = device->getInputQueue(controlQName);
}

void RGB::closeQueues() {
    if(ph->getParam<bool>("i_publish_topic")) {
        rgbPub->closeQueue();
        if(ph->getParam<bool>("i_enable_preview")) {
            previewPub->closeQueue();
        }
    }
    controlQ->close();
}

void RGB::link(dai::Node::Input in, int linkType) {
    if(linkType == static_cast<int>(link_types::RGBLinkType::video)) {
        colorCamNode->video.link(in);
    } else if(linkType == static_cast<int>(link_types::RGBLinkType::isp)) {
        colorCamNode->isp.link(in);
    } else if(linkType == static_cast<int>(link_types::RGBLinkType::preview)) {
        colorCamNode->preview.link(in);
    } else {
        throw std::runtime_error("Link type not supported");
    }
}

std::vector<std::shared_ptr<sensor_helpers::ImagePublisher>> RGB::getPublishers() {
    std::vector<std::shared_ptr<sensor_helpers::ImagePublisher>> publishers;
    if(ph->getParam<bool>("i_synced")) {
        publishers.push_back(rgbPub);
    }
    return publishers;
}

void RGB::updateParams(const std::vector<rclcpp::Parameter>& params) {
    auto ctrl = ph->setRuntimeParams(params);
    controlQ->send(ctrl);
}

void RGB::trigger() {
    rgbPub->trigger();
}

void RGB::setManualFocusCB(const std::shared_ptr<rif_msgs::srv::SetInt64::Request> req,
                           std::shared_ptr<rif_msgs::srv::SetInt64::Response> res) {
  dai::CameraControl ctrl;
  ctrl.setManualFocus(req->data);
  //ctrl.setManualFocusRaw(req->data); // 0.0 - 1.0
  controlQ->send(ctrl);
  res->success = true;
}

void RGB::setFocusModeCB(const std::shared_ptr<rif_msgs::srv::SetDepthAIFocusMode::Request> req,
                         std::shared_ptr<rif_msgs::srv::SetDepthAIFocusMode::Response> res)
{
  dai::CameraControl ctrl;
  ctrl.setAutoFocusMode(static_cast<dai::CameraControl::AutoFocusMode>(req->focus_mode));

  if (req->trigger_auto_focus) {
    ctrl.setAutoFocusTrigger();
  }
  controlQ->send(ctrl);
  focus_mode = static_cast<dai::CameraControl::AutoFocusMode>(req->focus_mode);
  res->success = true;
}

void RGB::getFocusModeCB(const std::shared_ptr<rif_msgs::srv::GetDepthAIFocusMode::Request> /* req */,
                         std::shared_ptr<rif_msgs::srv::GetDepthAIFocusMode::Response> res)
{
  res->focus_mode = static_cast<uint>(focus_mode);
  res->success = true;
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
