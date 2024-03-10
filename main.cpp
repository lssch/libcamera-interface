#include "opencv2/opencv.hpp"

#include "camera.hpp"

int main() {
    const std::string winname = "preview";
    float lens_position = 0;
    float focus_step = 0.5;

    Camera camera{0, "../camera.json"};

    cv::namedWindow(winname, cv::WINDOW_NORMAL);

    //camera.setFPS(20);
    //camera.setBrightness(0.0);
    //camera.setContrast(1.0);
    //camera.setExposureTime(1000);

    camera.start();
    while (true) {
        OutData frameData;
        if (not camera.readFrame(frameData)) continue;

        cv::Mat frame(camera.getWidth(), camera.getHeight(), CV_8UC3, frameData.imageData, camera.getStride());

        //std::cout << "Lens position: " << std::to_string(camera.getFocus()) << std::endl;

        imshow(winname, frame);

        char key = cv::waitKey(1);
        if (key == 'q' or key == 27) {
            break;
        } else if (key == 'f') {
            camera.setFocusContinuous();
        } else if (key == 'a' || key == 'A') {
            lens_position += focus_step;
            camera.setFocusManual(lens_position);
        } else if (key == 'd' || key == 'D') {
            lens_position -= focus_step;
            camera.setFocusManual(lens_position);
        }

        camera.returnFrameBuffer(frameData);
    }
    cv::destroyAllWindows();
    camera.stop();
    camera.close();
    return 0;
}
