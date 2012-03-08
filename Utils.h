/**
* Class with static methods for general purpose.
*
* Libraries: OpenNI, OpenCV, Boost.
*
* Author: Emilio J. Almazan <emilio.almazan@kingston.ac.uk>, 2012
*/
#pragma once
#include "XnCppWrapper.h"
#include "CameraProperties.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "filePaths.h"
#include "stdio.h"
#include <iostream>
#include <fstream>
#include <string>
#include <list>
#include <new>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>

using namespace std;
using namespace xn;
using std::string;

class Utils
{
public:
	Utils(void);

	~Utils(void);
	//Create a matrix 3xn with the values of the vector
	static void fillTheMatrix(vector<XnPoint3D>* src, CvMat* mat);

	//Rotate and translate a point using the values from cam.
	static void transformPoint(XnPoint3D* p3D, CameraProperties* cam);

	//Load the intrinsic and extrinsic parameters for cam (check paths)
	static void loadCameraParameters(CameraProperties* cam);

	//Returns the number of files in folderPath
	static int getNumberOfFiles(char* folderPath);

	//Create an image made of the combination of img1 and img2. The width is img1.width+img2.width
	static void combinedImages(IplImage* dst, const IplImage* img1, const IplImage* img2);

	//
	static void color2rgb(const int* Xn_disparity, unsigned short* depth, char *depth_data);

	//Initializes a 1/3 channel image with the value n
	static void initImage3Channel(IplImage* img, int n);
	static void initImage(IplImage* img , int n);

	//Used for converting depth into color
	static void raw2depth(unsigned short* depth, int maxDepth);

	//Used for converting depth into color
	static void depth2rgb(const XnDepthPixel* Xn_disparity, unsigned short* depth, char *depth_data);

	//Load the intrinsice parameters of cam from the intrinsic matrix
	static void initIntrinsicParameters(CameraProperties* cam);

	//Initialize the context of cam1 and cam2. It also loads the depth and the image node to start retrieve data
	static void rgbdInit(CameraProperties* cam1, CameraProperties* cam2);

	//Same as rgbdInit but align depth and rgb cameras for cam1 and cam2
	static void rgbdInitAligned(CameraProperties* cam1, CameraProperties* cam2);

	//Initialize the context of cam1.
	static void rgbdInit(CameraProperties* cam, int idCam);
	static void runCameraContext(CameraProperties* cam, XnDepthPixel  *pDepthMap, XnRGB24Pixel* pImageMap, IplImage *kinectRGBImage);

	//make a copy of the array.
	static void copyDepthMap(const XnDepthPixel* depthMapIn, XnDepthPixel* depthMapOut);
	static void analyzeDepthMap(const XnDepthPixel* depthMap1, const XnDepthPixel* depthMap2);
	static void drawPoints(IplImage* img, CvPoint2D32f* corners, int numCorners);
	static void analyzeData(const CvPoint2D32f* corners, int nWidth, int nHeight, const XnDepthPixel* pDepthMap, const DepthGenerator* depthNode);
	static void analyzeData(XnPoint3D* roi, const XnDepthPixel* pDepthMap, const DepthGenerator* depthNode);
	static void searchPoint(XnPoint3D* point, const XnDepthPixel* pDepthMap);
	static bool searchNeighbour(const XnPoint3D* p1, XnPoint3D* p2, const XnDepthPixel* pDepthMap);

	//Calculate the distance in the space of p1 and p2.
	static float calculateDistance(XnPoint3D p1, XnPoint3D p2);

	//Write the intrinsic/extrinsic matrix in filepath
	static void writeCameraIntrinsics(CameraProperties* cam, const char* filePath);
	static void writeCameraExtrinsics(CameraProperties* cam, const char* filePath);

	//load the intrinsic/extrinsic matrix in cam
	static void loadCameraIntrinsics(CameraProperties* cam);	
	static void loadCameraExtrinsics(CameraProperties* cam);

	//Fucntions for debuggin purposes
	static void writeMatrixValues(const CvMat* mat, ostream*);
	static void writeListXnPointValues(list<XnPoint3D>*, ostream*);
	static void writeImageValues(const IplImage* img, ostream*);
	static void writeROIImageValues(const IplImage* img, CvRect* rect, ostream* out);
	static void writeXmlMatrix(const CvMat* mat, const char* filePath);

	//Create a extrinsic matrix from the intrinsic and R and t.
	static void createExtrinsicMatrix(CvMat* extrinsic_mat, const CvMat* rotation, const CvMat* translation);

	//Convert a point into homogeneus coordinates.
	static void convertToHomogeneus(XnPoint3D, CvMat*);

	//Create a matrix with the values of the point.
	static void fillTheMatrix(CvMat* mat, XnPoint3D*);
	static void turnToMatrix(CvMat*, XnPoint3D*, int);
	static void normalizeMatrixPoints(CvMat*, CvMat*, XnPoint3D*, int, ostream*);
	static void createProjectionMat(CvMat* projMat, CameraProperties* cam);
	static void findMeanPoint(CvMat* points, XnPoint3D* ref, int numPoints);
	//Change the sign of all the matrix
	static void changeSign(const CvMat* src, CvMat* dst);
	//Generate randomly three integers from 0 to 255 (rgb)
	static void generateRandomColor(int* rgbColor);
	//Create an outstream to write the coordinates
	static void createCoordinateOutStream(ofstream* xPlaneCoor, ofstream* yPlaneCoor, ofstream* zPlaneCoor, const int planeNum, const int camId);
	static void initOutStreamPath(char* xPlane, char* yPlane, char* zPlane);
	static void createFullPath(const int pNum, const int cId, char* plane);
	static void createGeneralOutStream(ofstream* planeOutStream, const char* fileName, const int planeNum, const int camId);
	static int sizeArray(float *);
	static void showsArrayElements(float*, ostream*);

	//Create a matrix from p with rows and cols defined.
	static void fillTheMatrix(CvMat* mat, XnPoint3D* p, int rows, int cols);
	//Project/Backproject a point p2D into the space in the cam coordinate system.
	static void backProjectPoint(XnPoint3D* p2D, XnPoint3D* p3D, CameraProperties* cam);
	static void backProjectPointsRGBD(int, CameraProperties* cam, XnPoint3D* proj, XnPoint3D* backProj);
	static void projectPointsRGBD(int, CameraProperties*, XnPoint3D* rworld, XnPoint3D* proj);

	static void createPlanePath(vector<char*>* filePaths, int camId, char* suffix, int nPlanes);
	static void createPlanePath_II(vector<char*>* filePaths, int camId, char* suffix, int nPlanes,vector<int>* planePostions);
	//Calcuate the distance from the origin to the plane
	static float calculatePlaneDistance(const CvMat* param, const CvMat* normal);

	static void fillImageData(IplImage* image, const XnRGB24Pixel* pImageMap, const XnDepthPixel* pDepthMap);
	static void fillImageDataFull(IplImage* image, const XnRGB24Pixel* pImageMap);
	static int getRandomNumber(int max, int min);
	
	
};


