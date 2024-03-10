#include "opencv2/opencv.hpp"

#include "camera.hpp"

int main() {
    float lens_position = 100;
    float focus_step = 50;
    Camera camera;
    uint32_t width = 1920;
    uint32_t height = 1080;
    uint32_t stride;
    char key;

    cv::namedWindow("libcamera-demo", cv::WINDOW_NORMAL);
    cv::resizeWindow("libcamera-demo", width, height);

    int ret = camera.init();
    camera.configure(width, height, libcamera::formats::RGB888, 1);
    libcamera::ControlList controls_;
    int64_t frame_time = 1000000 / 10;
    // Set frame rate
	controls_.set(libcamera::controls::FrameDurationLimits, libcamera::Span<const int64_t, 2>({ frame_time, frame_time }));
    // Adjust the brightness of the output images, in the range -1.0 to 1.0
    controls_.set(libcamera::controls::Brightness, 0.5);
    // Adjust the contrast of the output image, where 1.0 = normal contrast
    controls_.set(libcamera::controls::Contrast, 1.5);
    // Set the exposure time
    controls_.set(libcamera::controls::ExposureTime, 20000);
    camera.set(controls_);
    if (!ret) {
        bool flag;
        LibcameraOutData frameData;
        camera.start();
        camera.VideoStream(&width, &height, &stride);
        while (true) {
            flag = camera.readFrame(&frameData);
            if (!flag)
                continue;
            cv::Mat im(height, width, CV_8UC3, frameData.imageData, stride);

            imshow("libcamera-demo", im);
            key = cv::waitKey(1);
            if (key == 'q') {
                break;
            } else if (key == 'f') {
                libcamera::ControlList controls;
                controls.set(libcamera::controls::AfMode, libcamera::controls::AfModeAuto);
                controls.set(libcamera::controls::AfTrigger, 0);
                camera.set(controls);
            } else if (key == 'a' || key == 'A') {
                lens_position += focus_step;
            } else if (key == 'd' || key == 'D') {
                lens_position -= focus_step;
            }

            if (key == 'a' || key == 'A' || key == 'd' || key == 'D') {
                libcamera::ControlList controls;
                controls.set(libcamera::controls::AfMode, libcamera::controls::AfModeManual);
				controls.set(libcamera::controls::LensPosition, lens_position);
                camera.set(controls);
            }

            camera.returnFrameBuffer(frameData);
        }
        cv::destroyAllWindows();
        camera.stop();
    }
    camera.close();
    return 0;
}
