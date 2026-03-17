#include "zed_smpl_tracking/ClientPublisher.hpp"
#include <iostream>

ClientPublisher::ClientPublisher() {}

ClientPublisher::~ClientPublisher() {
  zed.close();
}

bool ClientPublisher::open(sl::InputType input,
                           sl::COORDINATE_SYSTEM coord_system,
                           sl::RESOLUTION resolution,
                           int sdk_gpu_id) {
  sl::InitParameters init;
  init.depth_mode = sl::DEPTH_MODE::NEURAL_PLUS;
  init.input = input;
  init.coordinate_units = sl::UNIT::METER;
  init.coordinate_system = coord_system;
  init.camera_resolution = resolution;
  init.sdk_gpu_id = sdk_gpu_id;
  init.depth_maximum_distance = 4.0;

  auto state = zed.open(init);
  if (state != sl::ERROR_CODE::SUCCESS) {
    std::cout << "ZED open error: " << state << std::endl;
    return false;
  }

  // Positional tracking
  sl::PositionalTrackingParameters pt;
  pt.set_as_static = true;
  if (zed.enablePositionalTracking(pt) != sl::ERROR_CODE::SUCCESS)
    return false;

  // Body tracking
  sl::BodyTrackingParameters bt;
  bt.detection_model = sl::BODY_TRACKING_MODEL::HUMAN_BODY_ACCURATE;
  bt.body_format = sl::BODY_FORMAT::BODY_38;
  bt.enable_tracking = true;
  bt.enable_body_fitting = true;

  if (zed.enableBodyTracking(bt) != sl::ERROR_CODE::SUCCESS)
    return false;

  std::cout << "ZED initialized ✅" << std::endl;
  return true;
}

bool ClientPublisher::grab() {
  sl::RuntimeParameters rt;
  rt.confidence_threshold = 50;

  return zed.grab(rt) == sl::ERROR_CODE::SUCCESS;
}

void ClientPublisher::close() {
  zed.close();
}