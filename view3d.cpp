// g++ KinectOneStream.cpp -std=c++11 -o main `pkg-config opencv --cflags --libs` `pkg-config freenect2 --cflags --libs`
#include "view3d.h"
#include <iostream>
#include <stdio.h>
#include <iomanip>
#include <time.h>
#include <signal.h>
#include <opencv2/opencv.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <opencv2/viz/vizcore.hpp>
#include "opencv2/viz/types.hpp"

using namespace std;
using namespace cv;

void what(Mat M) ;

View3d g_view3d;

/****************************************************
 * Main()
 *
 ****************************************************/
int main()
{
	g_view3d.show();
}

/****************************************************
 * show()
 *
 ****************************************************/
void View3d::show(void)
{
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;

    if(freenect2.enumerateDevices() == 0)
    {
        cout << "*** Kinect-2 not found." << std::endl;
        return;
    }

    string serial = freenect2.getDefaultDeviceSerialNumber();
    dev = freenect2.openDevice(serial);

    if(dev == 0)
    {
        cout << "failure opening device!" << std::endl;
        return;
    }

    libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Depth);
//    libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Depth | libfreenect2::Frame::Color);
    libfreenect2::FrameMap frames;

    dev->setIrAndDepthFrameListener(&listener);
    dev->setColorFrameListener(&listener);
    dev->start();

    cout << "device serial: " << dev->getSerialNumber() << std::endl;
    cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

    Mat depthmat;
    Mat pCloud;
    viz::Viz3d viz("Viz window");
    pCloud.create(512, 424, CV_32FC3);	// 512 wide
    // loop while getting and displaying depth frames
    while(waitKey(1) < 0)
    {
        listener.waitForNewFrame(frames);
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
    //    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];

        Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);
        viz.showWidget("text2d", viz::WText("Kinect-2", Point(20, 20), 20, viz::Color::yellow()));

        // convert to 3D mat
        for(int i = 0; i < depthmat.rows; i++)
        {
            for(int j = 0; j < depthmat.cols; j++)
            {
            	pCloud.at<Vec3f>(i,j)[0] = j;
            	pCloud.at<Vec3f>(i,j)[1] = depthmat.rows -i;
            	float p = depthmat.at<float>(i,j);
            	if (p < View3dConsts::NearLimit){p=NAN;}				// limit close
            	if (p > View3dConsts::FarLimit){p=NAN;}					// limit far
            	pCloud.at<Vec3f>(i,j)[2] = p;
            }
        }

        viz::WCloud cloudWidget = viz::WCloud(pCloud, viz::Color::white());
        viz.showWidget("kinect-2", cloudWidget);

        listener.release(frames);
        viz.spinOnce(1,true);

    }
    dev->stop();
    dev->close();
    return;
}

/****************************************************
 * what()
 * Shows what Mat type is
 ****************************************************/
void what(Mat M)
{
  int type = M.type();
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  printf("Matrix: %s %dx%d \n", r.c_str(), M.cols, M.rows );


}

