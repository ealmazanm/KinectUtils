/**
* Class for detecting planes
*
* Libraries: OpenNI, OpenCV
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
#include "Plane.h"
#include <iostream>
#include <fstream>
#include <string>
#include <new>
#include <list>


using namespace std;
using namespace xn;
using std::string;

const int MAX_DEPTH_DIFF = 55;
const int MAX_POINTS = 20500;

class DynamicPlane
{
public:
	DynamicPlane(XnPoint3D* p, int width, int height, int maskSize, int* rgbPlane, int planeNum, CameraProperties* cam);
	DynamicPlane(list<XnPoint3D>* pList, int width, int height, int maskSize, int* rgbPlane, int planeNum, CameraProperties* cam);
	DynamicPlane(list<XnPoint3D>* pList, int width, int height, int maskSize, int* rgbPlane, int planeNum, CameraProperties* cam, const vector<vector<double>>* hist, IplImage* hsvImg, ofstream* outDebug);
	~DynamicPlane(void);
	void makePlaneGrow(char* winName, IplImage* depthImage, const XnDepthPixel* pDepthMap);
	list<XnPoint3D>* getPlanePoints();
	CvMat* getPlaneParameters();
	XnPoint3D* getCentroid();


private:
	bool DynamicPlane::isHSVColor(XnPoint3D* p);
	void checkNoise();
	void checkNeighbours(const XnPoint3D* p, const XnDepthPixel* pDepthMap, const CvRect* rect);
	bool isInList(XnPoint3D* p, list<XnPoint3D>* lst);
	bool isInPlane2(XnPoint3D* p);
	bool isInPlane(XnPoint3D* p);
	void initMatrix(CvMat* m, int num);
	bool isInACorner(const XnPoint3D* p, int x, int y);
	void calculatePlaneParams();
	void backProjectPoint(XnPoint3D* p, XnPoint3D* p3D);
	IplImage* binaryImg;
	IplImage* mask;
	int maskSize;
	int* colorPlane;
	list<XnPoint3D> planePoints;
	list<XnPoint3D> points2Check;
	list<XnPoint3D> tmp;
	CvMat* planeParameters; //plane parameters: aX + bY + C = Z
	float pointCoors[MAX_POINTS*3];
	float pointDepth[MAX_POINTS];
	XnPoint3D origin;

	ofstream xPlaneCoor;
	ofstream yPlaneCoor;
	ofstream zPlaneCoor;
	ofstream planeParam;

//	char windowName[50];
	CameraProperties* cam;

	XnPoint3D centroid2D;

	const vector<vector<double>>* histogram;
	IplImage* hsvImage;

};

