#ifndef VIEW3D_H_
#define VIEW3D_H_

#include <opencv2/opencv.hpp>

namespace View3dConsts
{
	const float NearLimit = 1000.0;
	const float FarLimit = 2000.0;
	const uint16_t KinectRows = 424;
	const uint16_t KinectCols = 512;
}

class View3d
{
public:
	View3d(void){};
	~View3d(void){};
	void show(void);
private:
	void what(cv::Mat M);
	void test(void);
};
#endif /* VIEW3D_H_ */
