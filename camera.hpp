//
// Created by Lars Schwarz on 10.03.2024.
//

#ifndef LIBCAMERA_INTERFACE_CAMERA_HPP
#define LIBCAMERA_INTERFACE_CAMERA_HPP

#include <atomic>
#include <iomanip>
#include <iostream>
#include <signal.h>
#include <limits.h>
#include <memory>
#include <stdint.h>
#include <string>
#include <vector>
#include <unordered_map>
#include <queue>
#include <sstream>
#include <sys/mman.h>
#include <unistd.h>
#include <time.h>
#include <mutex>

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
} LibcameraOutData;

class Camera {
public:
  explicit Camera();
  ~Camera();

  int init();
  void configure(int width, int height, PixelFormat format, int buffercount);
  void start();
  int reset(int width, int height, PixelFormat format, int buffercount);
  void stop();
  void close();

  Stream *VideoStream(uint32_t *w, uint32_t *h, uint32_t *stride) const;
  std::string getId();

private:
  int startCapture();
  int queueRequest(Request *request);
  void requestComplete(Request *request);
  void processRequest(Request *request);

  void streamDimensions(Stream const *stream, uint32_t *w, uint32_t *h, uint32_t *stride) const;

  unsigned int cameraIndex_;
  uint64_t last_;
  std::unique_ptr<CameraManager> camera_manager_;
  std::shared_ptr<Camera> camera_;
  bool acquired_{false};
  bool started_{false};
  std::unique_ptr<CameraConfiguration> config_;
  std::unique_ptr<FrameBufferAllocator> allocator_;
  std::vector<std::unique_ptr<Request>> requests_;
  std::map<int, std::pair<void *, unsigned int>> mappedBuffers_;

  std::queue<Request *> requestQueue;

  ControlList controls_;
  std::mutex control_mutex_;
  std::mutex camera_stop_mutex_;
  std::mutex free_requests_mutex_;

  Stream *viewfinder_stream_ = nullptr;
  std::string cameraId;
};


#endif //LIBCAMERA_INTERFACE_CAMERA_HPP
