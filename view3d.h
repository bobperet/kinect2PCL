
#ifndef VIEW3D_H_
#define VIEW3D_H_

namespace View3dConsts
{
	const float NearLimit = 5.0;
	const float FarLimit = 2000.0;
}

class View3d
{
public:
	View3d(void){};
	~View3d(void){};
	void show(void);
private:

};
#endif /* VIEW3D_H_ */
