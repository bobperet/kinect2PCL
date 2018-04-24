/*
 * Copyright Bob Peret 2018
 *
 * A dense pointcloud does not contain NaN or INF
 */
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

#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <boost/thread/thread.hpp>

using namespace pcl;
using namespace std;
using namespace cv;

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
    libfreenect2::FrameMap frames;

    dev->setIrAndDepthFrameListener(&listener);
    dev->setColorFrameListener(&listener);
    dev->start();

    cout << "device serial: " << dev->getSerialNumber() << std::endl;
    cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

    Mat depthmat;

    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.setBackgroundColor (0, 0, 0);
    viewer.initCameraParameters ();

    PointCloud<pcl::PointXYZ>::PointType pt;
    PointCloud<pcl::PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);

    // loop while getting and displaying depth frames
    while(waitKey(1) < 0)
    {
        listener.waitForNewFrame(frames);
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

        Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);
//        imshow("x", depthmat/1024);
        listener.release(frames);

        cloud->clear();
        // convert to PCL
        for (int i = 0; i < depthmat.rows; i++)
        {
            for (int j = 0; j < depthmat.cols; j++)
            {
             	float p = depthmat.at<float>(i,j);
            	if ((p > View3dConsts::NearLimit) && (p < View3dConsts::FarLimit))
            	{
                	pt.x = j;
            	    pt.y = depthmat.rows - i;
            	    pt.z = p;
            	    cloud->push_back(pt);
            	}
            }
        }

    	cloud->width  = View3dConsts::KinectCols;
    	cloud->height = View3dConsts::KinectRows;
    	cloud->is_dense = true;

    	viewer.addPointCloud<pcl::PointXYZ> (cloud, "Kinect2 depth");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Kinect2 depth");
        viewer.spinOnce (100);
	    viewer.removePointCloud("Kinect2 depth");

	    if (viewer.wasStopped())
	    {
	    	break;
	    }
    }
    dev->stop();
    dev->close();
    return;
}

/****************************************************
 * what()
 * Shows what Mat type is
 ****************************************************/
void View3d::what(Mat M)
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

void View3d::test(void)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	  // Fill in the cloud data
	  cloud->width  = 15;
	  cloud->height = 1;
	  cloud->points.resize (cloud->width * cloud->height);

	  // Generate the data
	  for (size_t i = 0; i < cloud->points.size (); ++i)
	  {
	    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
	    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
	    cloud->points[i].z = 1.0;
	  }
//	  boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
	    pcl::visualization::PCLVisualizer viewer("3D Viewer");

	    viewer.setBackgroundColor (0, 0, 0);
	    viewer.addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
	    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	    viewer.addCoordinateSystem (1.0);
	    viewer.initCameraParameters ();
//	    return (viewer);
	    while (!viewer.wasStopped())
	    {
	      viewer.spinOnce (100);
	      boost::this_thread::sleep(boost::posix_time::microseconds (100000));
	  }
}
