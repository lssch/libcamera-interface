//
// Created by Lars Schwarz on 10.03.2024.
//

#include <fstream>
#include "camera.hpp"


Camera::Camera(int id, std::string path_to_config) {
    std::array<int, 2> resolution;
    int framerate, exposure_time, buffercount;
    float brightness, contrast, lens_position;

    try {
        std::ifstream config_file(path_to_config);
        nlohmann::json config_data = nlohmann::json::parse(config_file);

        resolution = config_data.at("resolution");
        buffercount = config_data.at("buffercount");
        framerate = config_data.at("framerate");
        brightness = config_data.at("brightness");
        contrast = config_data.at("contrast");
        exposure_time = config_data.at("exposure_time");
        lens_position = config_data.at("lens_position");
    } catch (std::exception &ex) {
        throw std::runtime_error(std::string("Error while parsing the config data: ") + ex.what());
    }

    try {
        init(id);
    } catch (std::exception &ex) {
        throw std::runtime_error(std::string("Error while initialising camera: ") + ex.what());
    }

    try {
        configure(resolution, libcamera::formats::RGB888, buffercount);
    } catch (std::exception &ex) {
        throw std::runtime_error(std::string("Error while configure camera: ") + ex.what());
    }
}

Camera::Camera(int id, std::array<int, 2> resolution, libcamera::PixelFormat format, int buffercount) {
    try {
        init(id);
    } catch (std::exception &ex) {
        throw std::runtime_error(std::string("Error while initialising camera: ") + ex.what());
    }

    try {
        configure(resolution, format, buffercount);
    } catch (std::exception &ex) {
        throw std::runtime_error(std::string("Error while configure camera: ") + ex.what());
    }
}

Camera::~Camera() {
    close();
}

void Camera::init(int id) {
    camera_manager_ = std::make_unique<libcamera::CameraManager>();
    int state = camera_manager_->start();
    if (state) {
        throw std::runtime_error("Failed to start camera manager: " + std::to_string(state));
    }

    id_ = camera_manager_->cameras()[id]->id();
    camera_ = camera_manager_->get(id_);

    if (not camera_) {
        throw std::runtime_error("Camera id: " + id_ + " not found");
    }

    if (camera_->acquire()) {
        throw std::runtime_error("Failed to acquire camera at id: " + id_);
    }

    acquired_ = true;
}

void Camera::configure(std::array<int, 2> resolution, libcamera::PixelFormat format, int buffercount) {
  std::cout << "Configuring still capture..." << std::endl;

  libcamera::Size size(resolution.at(0), resolution.at(1));

  config_ = camera_->generateConfiguration({libcamera::StreamRole::StillCapture});
  config_->at(0).size = size;
  config_->at(0).pixelFormat = format;
  config_->at(0).bufferCount = buffercount;

  libcamera::CameraConfiguration::Status validation = config_->validate();
  if (validation == libcamera::CameraConfiguration::Invalid) {
      throw std::runtime_error("Failed to valid stream configurations");
  } else if (validation == libcamera::CameraConfiguration::Adjusted) {
      std::cout << "Stream configuration adjusted" << std::endl;
  }

  std::cout << "Still capture setup complete." << std::endl;
}

int Camera::start() {
  int state = camera_->configure(config_.get());
  if (state < 0) {
    std::cout << "Failed to configure camera" << std::endl;
    return state;
  }

  camera_->requestCompleted.connect(this, &Camera::requestComplete);
  allocator_ = std::make_unique<libcamera::FrameBufferAllocator>(camera_);

  state = startCapture();
  if (state < 0) {
      std::cout << "Failed to configure camera" << std::endl;
      return state;
  }

  libcamera::StreamConfiguration const &cfg = viewfinder_stream_->configuration();
  width_ = cfg.size.width;
  height_ = cfg.size.height;
  stride_ = cfg.stride;

  return 0;
}

int Camera::startCapture() {
    int ret;
    unsigned int nbuffers = UINT_MAX;
    for (libcamera::StreamConfiguration &cfg : *config_) {
        ret = allocator_->allocate(cfg.stream());
        if (ret < 0) {
            std::cerr << "Can't allocate buffers" << std::endl;
            return -ENOMEM;
        }

        unsigned int allocated = allocator_->buffers(cfg.stream()).size();
        nbuffers = std::min(nbuffers, allocated);
    }

    for (unsigned int i = 0; i < nbuffers; i++) {
        std::unique_ptr<libcamera::Request> request = camera_->createRequest();
        if (!request) {
            std::cerr << "Can't create request" << std::endl;
            return -ENOMEM;
        }

        for (libcamera::StreamConfiguration &cfg : *config_) {
            libcamera::Stream *stream = cfg.stream();
            const std::vector<std::unique_ptr<libcamera::FrameBuffer>> &buffers =
                    allocator_->buffers(stream);
            const std::unique_ptr<libcamera::FrameBuffer> &buffer = buffers[i];

            ret = request->addBuffer(stream, buffer.get());
            if (ret < 0) {
                std::cerr << "Can't set buffer for request"
                          << std::endl;
                return ret;
            }
            for (const libcamera::FrameBuffer::Plane &plane : buffer->planes()) {
                void *memory = mmap(NULL, plane.length, PROT_READ, MAP_SHARED, plane.fd.get(), 0);
                mappedBuffers_[plane.fd.get()] = std::make_pair(memory, plane.length);
            }
        }

        requests_.push_back(std::move(request));
    }

    ret = camera_->start(&this->controls_);
    if (ret) {
        std::cout << "Failed to start capture" << std::endl;
        return ret;
    }

    controls_.clear();
    started_ = true;

    for (std::unique_ptr<libcamera::Request> &request : requests_) {
        ret = queueRequest(request.get());
        if (ret < 0) {
            std::cerr << "Can't queue request" << std::endl;
            camera_->stop();
            return ret;
        }
    }

    viewfinder_stream_ = config_->at(0).stream();
    return 0;
}

int Camera::queueRequest(libcamera::Request *request) {
    std::lock_guard<std::mutex> stop_lock(camera_stop_mutex_);
    if (!started_) {
        return -1;
    }

    std::lock_guard<std::mutex> lock(control_mutex_);
    request->controls() = std::move(controls_);

    return camera_->queueRequest(request);
}

void Camera::requestComplete(libcamera::Request *request) {
    if (request->status() == libcamera::Request::RequestCancelled)
        return;
    processRequest(request);
}

void Camera::processRequest(libcamera::Request *request) {
    requestQueue_.push(request);
}

void Camera::returnFrameBuffer(OutData frame) {
    uint64_t request = frame.request;
    auto * req = (libcamera::Request *)request;
    req->reuse(libcamera::Request::ReuseBuffers);
    queueRequest(req);
}

bool Camera::readFrame(OutData &frame){
    std::lock_guard<std::mutex> lock(free_requests_mutex_);

    if (!requestQueue_.empty()){
        libcamera::Request *request = this->requestQueue_.front();

        const libcamera::Request::BufferMap &buffers = request->buffers();
        for (auto it : buffers) {
            libcamera::FrameBuffer *buffer = it.second;
            for (unsigned int i = 0; i < buffer->planes().size(); ++i) {
                const libcamera::FrameBuffer::Plane &plane = buffer->planes()[i];
                const libcamera::FrameMetadata::Plane &meta = buffer->metadata().planes()[i];

                void *data = mappedBuffers_[plane.fd.get()].first;
                int length = std::min(meta.bytesused, plane.length);

                frame.size = length;
                frame.imageData = (uint8_t *)data;
            }
        }
        this->requestQueue_.pop();
        frame.request = (uint64_t)request;
        return true;
    } else {
        libcamera::Request *request = nullptr;
        frame.request = (uint64_t)request;
        return false;
    }
}

void Camera::setControl(libcamera::ControlList controls){
    std::lock_guard<std::mutex> lock(control_mutex_);
    this->controls_ = std::move(controls);
}

int Camera::reset(std::array<int, 2> resolution, libcamera::PixelFormat format, int buffercount) {
  stop();
  configure(resolution, format, buffercount);
  return start();
}

void Camera::stop() {
  if (camera_) {
    std::lock_guard<std::mutex> lock(camera_stop_mutex_);
    if (started_) {
      if (camera_->stop()) {
        throw std::runtime_error("Failed to stop camera!");
      }
      started_ = false;
    }
    camera_->requestCompleted.disconnect(this, &Camera::requestComplete);
  }

  while (!requestQueue_.empty()) {
    requestQueue_.pop();
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
  if (acquired_) {
    camera_->release();
  }

  acquired_ = false;
  camera_.reset();
  camera_manager_.reset();
}

std::string Camera::getId(){
    return id_;
}

unsigned int Camera::getWidth() {
    return width_;
}

unsigned int Camera::getHeight() {
    return height_;
}

unsigned int Camera::getStride() {
    return stride_;
}


void Camera::setFocusTrigger() {
    std::lock_guard<std::mutex> lock(control_mutex_);
    controls_.set(libcamera::controls::AfMode, libcamera::controls::AfModeAuto);
    controls_.set(libcamera::controls::AfTrigger, 0);
}

void Camera::setFocusContinuous() {
    std::lock_guard<std::mutex> lock(control_mutex_);
    controls_.set(libcamera::controls::AfMode, libcamera::controls::AfModeContinuous);
}

void Camera::setFocusManual(float lens_position) {
    std::lock_guard<std::mutex> lock(control_mutex_);
    controls_.set(libcamera::controls::AfMode, libcamera::controls::AfModeManual);
    controls_.set(libcamera::controls::LensPosition, lens_position);
}

void Camera::setFPS(unsigned int fps) {
    std::lock_guard<std::mutex> lock(control_mutex_);
    controls_.set(libcamera::controls::FrameDurationLimits, libcamera::Span<const int64_t, 2>({1000000/fps, 1000000/fps}));
}

void Camera::setBrightness(float brightness) {
    if (brightness > 1 or brightness < -1) {
        std::cerr << "Brightness must be in range [-1, 1]." << std::endl;
    }
    std::lock_guard<std::mutex> lock(control_mutex_);
    controls_.set(libcamera::controls::Brightness, brightness);
}

void Camera::setContrast(float contrast) {
    if (contrast < 1) {
        std::cerr << "Contrast must be greather or equal to 1." << std::endl;
    }
    std::lock_guard<std::mutex> lock(control_mutex_);
    controls_.set(libcamera::controls::Contrast, contrast);
}

void Camera::setExposureTime(unsigned int exposure_time) {
    std::lock_guard<std::mutex> lock(control_mutex_);
    controls_.set(libcamera::controls::ExposureTime, exposure_time);
}