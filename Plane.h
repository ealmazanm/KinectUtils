/**
* Class for defining planes
*
* Libraries: OpenNI, OpenCV.
*
* Author: Emilio J. Almazan <emilio.almazan@kingston.ac.uk>, 2012
*/
#pragma once
#include "XnCppWrapper.h"
#include "CameraProperties.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "stdio.h"
#include "Utils.h"
#include "filePaths.h"
#include <iostream>
#include <fstream>
#include <string>
#include <new>


using namespace std;
using namespace xn;
using std::string;

class Plane
{
public:
	Plane(void);
	~Plane(void);
	XnPoint3D getInitPoint();
	XnPoint3D getEndPoint();
	void setInitPoint(XnPoint3D);
	void setEndPoint(XnPoint3D);
	void initPlane();

	//Getters and Setters
	const CvMat* getNormal();
	const CvMat* getParameters();
	float getDistance();
	void setNormal(CvMat* n);
	void setParameters(CvMat* p);
	void setDistance(float d);
	bool isROISelected();
	void setCentroid3D(XnPoint3D* centroid);
	XnPoint3D* getCentroid3D();

private:
	//Init ROI
	XnPoint3D initPoint;
	XnPoint3D endPoint;
	XnPoint3D centroid3D;

	CvMat* normal; // Unit normal vector of the plane
	CvMat* parameters; //Ax+By+C = Z
	float dist; // Distance from the origin of coordiantes to the plane

};

