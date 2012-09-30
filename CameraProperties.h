/**
* Class for defining cameras
*
* Libraries: OpenNI, OpenCV.
*
* Author: Emilio J. Almazan <emilio.almazan@kingston.ac.uk>, 2012
*/
#pragma once
#include "XnCppWrapper.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

using namespace xn;

class CameraProperties
{
public:
	CameraProperties(void);
	~CameraProperties(void);
	Context* getContext() const;
	void projectPoint(XnPoint3D* p3D, XnPoint3D* p2D) const;
	void backProjectPoint(XnPoint3D* centroid, XnPoint3D* centroid3D) const; //A read only function
	DepthGenerator* getDepthNode() const;
	ImageGenerator* getImageNode() const;
	Recorder* getRecorderNode();
	CvMat* getRotationMatrix() const;
	CvMat* getTranslationMatrix() const;
	CvMat* getIntrinsicMatrix();
	CvMat* getRGBIntrinsicMatrix();
	CvMat* getDistortionCoeffs();
	float getFocalLenghtX();
	float getFocalLenghtY();
	float getOx();
	float getOy();
	double getPixelSize();
	void setFocalLenghtX(float);
	void setFocalLenghtY(float);
	void setOx(float);
	void setOy(float);
	void setPixelSize(double);
	void setIntrinsicMatrix(CvMat*);
	void setRGBIntrinsicMatrix(CvMat*);
	void setDistortionCoeffs(CvMat*);
	void setRotationMatrix(CvMat*);
	void setTranslationMatrix(CvMat*);
	void setCamId(int);
	int getCamId() const; //A read only function
	

private:
	Context context;
	DepthGenerator depthNode;
	ImageGenerator imageNode;
	Recorder recordNode;
	CvMat* rotation; //= cvCreateMat(3,3,CV_32FC1);
	CvMat* translation; //= cvCreateMat(1,3,CV_32FC1);
	CvMat* intrinsic_matrix;
	CvMat* distortion_coeffs;
	CvMat* rgbIntrinsic_matrix;
	float fx;
	float fy;
	float ox;
	float oy;
	double pSize;
	int camId;
};

