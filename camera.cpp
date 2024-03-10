//
// Created by Lars Schwarz on 10.03.2024.
//

#include "camera.hpp"


Camera::Camera() {}

Camera::~Camera() {}

int Camera::init() {
  camera_manager_ = std::make_unique<libcamera::CameraManager>();
  int state = camera_manager_->start();
  if (state) {
    std::cout << "Failed to start camera manager: " << state << std::endl;
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

  acquired_ = true;
  return 0;
}

std::string Camera::getId(){
    return cameraId;
}


void Camera::configure(int width, int height, libcamera::PixelFormat format, int buffercount) {
  std::cout << "Configuring still capture..." << std::endl;

  config_ = camera_->generateConfiguration({libcamera::StreamRole::StillCapture});

  if (width && height) {
    libcamera::Size size(width, height);
    config_->at(0).size = size;
  }

  config_->at(0).pixelFormat = format;

  if (buffercount) {
      config_->at(0).bufferCount = buffercount;
  }

  libcamera::CameraConfiguration::Status validation = config_->validate();
  if (validation == libcamera::CameraConfiguration::Invalid)
    throw std::runtime_error("Failed to valid stream configurations");
  else if (validation == libcamera::CameraConfiguration::Adjusted)
    std::cout << "Stream configuration adjusted" << std::endl;

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

  return startCapture();
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

void Camera::streamDimensions(libcamera::Stream const *stream, uint32_t *width, uint32_t *height, uint32_t *stride) const {
    libcamera::StreamConfiguration const &cfg = stream->configuration();
    if (width)
        *width = cfg.size.width;
    if (height)
        *height = cfg.size.height;
    if (stride)
        *stride = cfg.stride;
}

libcamera::Stream *Camera::VideoStream(uint32_t *width, uint32_t *height, uint32_t *stride) const {
    streamDimensions(viewfinder_stream_, width, height, stride);
    return viewfinder_stream_;
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
    requestQueue.push(request);
}

void Camera::returnFrameBuffer(LibcameraOutData frameData) {
    uint64_t request = frameData.request;
    auto * req = (libcamera::Request *)request;
    req->reuse(libcamera::Request::ReuseBuffers);
    queueRequest(req);
}

bool Camera::readFrame(LibcameraOutData *frameData){
    std::lock_guard<std::mutex> lock(free_requests_mutex_);

    if (!requestQueue.empty()){
        libcamera::Request *request = this->requestQueue.front();

        const libcamera::Request::BufferMap &buffers = request->buffers();
        for (auto it : buffers) {
            libcamera::FrameBuffer *buffer = it.second;
            for (unsigned int i = 0; i < buffer->planes().size(); ++i) {
                const libcamera::FrameBuffer::Plane &plane = buffer->planes()[i];
                const libcamera::FrameMetadata::Plane &meta = buffer->metadata().planes()[i];

                void *data = mappedBuffers_[plane.fd.get()].first;
                int length = std::min(meta.bytesused, plane.length);

                frameData->size = length;
                frameData->imageData = (uint8_t *)data;
            }
        }
        this->requestQueue.pop();
        frameData->request = (uint64_t)request;
        return true;
    } else {
        libcamera::Request *request = nullptr;
        frameData->request = (uint64_t)request;
        return false;
    }
}

void Camera::set(libcamera::ControlList controls){
    std::lock_guard<std::mutex> lock(control_mutex_);
    this->controls_ = std::move(controls);
}

int Camera::reset(int width, int height, libcamera::PixelFormat format, int buffercount) {
  stop();
  configure(width, height, format, buffercount);
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
  if (acquired_) {
    camera_->release();
  }

  acquired_ = false;
  camera_.reset();
  camera_manager_.reset();
}