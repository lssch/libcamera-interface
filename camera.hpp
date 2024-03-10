//
// Created by Lars Schwarz on 10.03.2024.
//

#ifndef LIBCAMERA_INTERFACE_CAMERA_HPP
#define LIBCAMERA_INTERFACE_CAMERA_HPP

#include <atomic>
#include <iomanip>
#include <iostream>
#include <csignal>
#include <climits>
#include <memory>
#include <cstdint>
#include <string>
#include <vector>
#include <unordered_map>
#include <queue>
#include <sstream>
#include <sys/mman.h>
#include <unistd.h>
#include <ctime>
#include <mutex>

#include "json.h"

#include <libcamera/controls.h>
#include <libcamera/control_ids.h>
#include <libcamera/property_ids.h>
#include <libcamera/libcamera.h>
#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>
#include <libcamera/formats.h>
#include <libcamera/transform.h>

typedef struct {
  uint8_t *imageData;
  uint32_t size;
  uint64_t request;
} OutData;

class Camera {
public:
  Camera(int id=0, std::string path_to_config="camera.json");
  Camera(int id=0, std::array<int, 2> resolution={1280, 720}, libcamera::PixelFormat format=libcamera::formats::RGB888, int buffercount=libcamera::formats::RGB888);
  ~Camera();


  int start();
  int reset(std::array<int, 2> resolution, libcamera::PixelFormat format, int buffercount);
  void stop();
  void close();

  bool readFrame(OutData &frame);
  void returnFrameBuffer(OutData frame);



  void setFocusTrigger();
  void setFocusContinuous();
  void setFocusManual(float lens_position);
  void setFPS(unsigned int fps);
  void setBrightness(float brightness);
  void setContrast(float contrast);
  void setExposureTime(unsigned int exposure_time);

  std::string getId();

  unsigned int getWidth();
  unsigned int getHeight();
  unsigned int getStride();

private:    
std::unique_ptr<libcamera::CameraManager> camera_manager_;
    std::shared_ptr<libcamera::Camera> camera_;
    std::string id_;
    bool acquired_{false};
    bool started_{false};

    unsigned int width_;
    unsigned int height_;
    unsigned int stride_;

    void init(int id);
    void configure(std::array<int, 2> resolution, libcamera::PixelFormat format, int buffercount);


  int startCapture();
  int queueRequest(libcamera::Request *request);
  void requestComplete(libcamera::Request *request);
  void processRequest(libcamera::Request *request);

  std::unique_ptr<libcamera::CameraConfiguration> config_;
  std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
  std::vector<std::unique_ptr<libcamera::Request>> requests_;
  std::map<int, std::pair<void *, unsigned int>> mappedBuffers_;

  std::queue<libcamera::Request *> requestQueue_;

  libcamera::ControlList controls_;
  std::mutex control_mutex_;
  std::mutex camera_stop_mutex_;
  std::mutex free_requests_mutex_;

  libcamera::Stream *viewfinder_stream_ = nullptr;
};


#endif //LIBCAMERA_INTERFACE_CAMERA_HPP
