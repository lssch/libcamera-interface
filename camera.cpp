//
// Created by Lars Schwarz on 10.03.2024.
//

#include "camera.hpp"


Camera::Camera() {}

Camera::~Camera() {}

int Camera::init() {
  camera_manager_ = std::make_unique<CameraManager>();
  int state = camera_manager_->start();
  if (state) {
    std::cout << "Failed to start camera manager: " << ret << std::endl;
    return state;
  }

  cameraId = camera_manager_->cameras()[0]->id();
  camera_ = camera_manager_->get(cameraId);
  if (!camera_) {
    std::cerr << "Camera " << cameraId << " not found" << std::endl;
    return 1;
  }

  if (camera_->acquire()) {
    std::cerr << "Failed to acquire camera " << cameraId << std::endl;
    return 1;
  }

  camera_acquired_ = true;
  return 0;
}

void Camera::configure(int width, int height, int format, int buffercount) {
  std::cout << "Configuring still capture..." << std::endl;

  config_ = camera_->generateConfiguration({StreamRole::StillCapture});

  if (width && height) {
    libcamera::Size size(width, height);
    config_->at(0).size = size;
  }

  config_->at(0).pixelFormat = format;

  if (buffercount)
    config_->at(0).bufferCount = buffercount;

  CameraConfiguration::Status validation = config_->validate();
  if (validation == CameraConfiguration::Invalid)
    throw std::runtime_error("Failed to valid stream configurations");
  else if (validation == CameraConfiguration::Adjusted)
    std::cout << "Stream configuration adjusted" << std::endl;

  std::cout << "Still capture setup complete." << std::endl;
}

int Camera::start() {
  int state = camera_->configure(config_.get())
  if (state < 0) {
    std::cout << "Failed to configure camera" << std::endl;
    return state;
  }

  camera_->requestCompleted.connect(this, &LibCamera::requestComplete);

  allocator_ = std::make_unique<FrameBufferAllocator>(camera_);

  return startCapture();
}

int Camera::reset(int width, int height, PixelFormat format, int buffercount) {
  stopCamera();
  configureStill(width, height, format, buffercount, rotation);
  return startCamera();
}

void Camera::stop() {
  if (camera_) {
    std::lock_guard<std::mutex> lock(camera_stop_mutex_);
    if (camera_started_) {
      if (camera_->stop()) {
        throw std::runtime_error("Failed to stop camera!");
      }
      camera_started_ = false;
    }
    camera_->requestCompleted.disconnect(this, &LibCamera::requestComplete);
  }

  while (!requestQueue.empty()) {
    requestQueue.pop();
  }

  for (auto &iter : mappedBuffers_) {
    std::pair<void *, unsigned int> pair_ = iter.second;
    munmap(std::get<0>(pair_), std::get<1>(pair_));
  }

  mappedBuffers_.clear();
  requests_.clear();
  allocator_.reset();
  controls_.clear();
}

void Camera::close() {
  if (camera_acquired_) {
    camera_->release();
  }

  camera_acquired_ = false;
  camera_.reset();
  camera_manager_.reset();
}