#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <ros/ros.h>

#include <signal.h>
void signal_handler(sig_atomic_t s) {
  std::cout << "You pressed Ctrl + C, exiting" << std::endl;
  exit(1);
}


#include <iostream>
#include <chrono> 
using namespace std::chrono; 
using namespace std;


// '''using Gstreamer'''
// for NVargus
// camSet = 'nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=3264, height=2464, framerate=21/1, format=NV12' ! nvvidconv flip-method=2 ! video/x-raw, width=800, height=600, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink' #flip-method=0 or 2

// #wbmode : white balance, 0:off, 1:auto(default)
// #tnr-mode : noise reduction mode, 0:off, 1:Fast(default), 2:High Quality
// #ee-mode : edge enhancement mode, 0:off, 1:Fast(def), 2:High Qual
// #contrast 0~2, brightness-1~1, saturation0~2
// #camSet = 'nvarguscamerasrc sensor-id=0 tnr-mode=2 wbmode=3 ! video/x-raw(memory:NVMM), width=3264, height=2464, framerate=21/1, format=NV12' ! nvvidconv flip-method=2 ! video/x-raw, width=800, height=600, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! videobalance contrast=1.5 brightness=-.15 saturation=1.2 ! appsink' #flip-method=0 or 2

// #cam = cv2.VideoCapture(camSet)

// #for Webcam
// #camSet = 'v4l2src device=/dev/video0 ! video/x-raw, width=800, height=600, framerate=24/1 ! videoconvert ! appsink'
// #cam = cv2.VideoCapture(camSet)

// #when using other codec
// ''' test-> v4l2-ctl -d /dev/video1 --list-formats-ext '''
// # cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'H264'))
// # cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
// # cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1920);
// # cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080);
// # cap.set(cv2.CAP_PROP_FPS, 30);

int main(int argc, char **argv){

	cv::VideoCapture cam(4); //, cv::CAP_GSTREAMER); 
	cam.set(CV_CAP_PROP_FOURCC, cv::VideoWriter::fourcc('H','2','6','4'));
	cam.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
	cam.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
	cam.set(CV_CAP_PROP_FPS, 30);

    cv::Mat frame;

	while (1){
        auto start = high_resolution_clock::now(); 
    	cam >> frame;
        auto stop = high_resolution_clock::now(); 
        auto duration = duration_cast<microseconds>(stop - start);
    	std::cout << frame.rows << " " << frame.cols << " " << duration.count()/1000.0 << " ms " << std::endl;
    	cv::imshow("testing", frame);
    	if (cv::waitKey(1) == 27) break; // pressing ESC;
    }

	cam.release();
	cv::destroyAllWindows();

	return 0;
}
