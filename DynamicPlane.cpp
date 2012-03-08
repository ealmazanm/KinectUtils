#include "DynamicPlane.h"

bool firstTime = true;

DynamicPlane::DynamicPlane(list<XnPoint3D>* pList, int width, int height, int maskS, int* rgbPlane, int planeNum, CameraProperties* camera)
{
	//init global variables
	maskSize = maskS;
	colorPlane = rgbPlane;
	cam = camera;
	firstTime = true;

	//intialializes the arrays
	pointCoors[0] = NULL;
	pointDepth[0] = NULL;

	//store the seed
	points2Check.assign(pList->begin(),pList->end());


	//add points2Check to the plane fitting
	planeParameters = cvCreateMat(3,1,CV_32FC1);
	calculatePlaneParams();
//	checkNoise();

	//initializes the mask and binary image
	binaryImg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
	mask = cvCreateImage(cvSize(maskS, maskS), IPL_DEPTH_8U, 1);
	Utils::initImage(mask, 1);
	Utils::initImage(binaryImg , 0);
	//updates the binaryImage with the points in 'points2Check'
	list<XnPoint3D>::iterator it;
	for (it=points2Check.begin(); it!=points2Check.end(); ++it)
	{
		XnPoint3D p = *it;
		((uchar*)(binaryImg->imageData + (int)p.Y*binaryImg->widthStep))[(int)p.X] = 255;
	}

	////Create window name
	//char windowName[80];
	//char idCam[10];
	//itoa(cam->getCamId(), idCam, 10);
	//strcpy(windowName, "Binary ");
	//strcat(windowName, idCam);

	//cvNamedWindow(windowName, 1);
	//cvShowImage(windowName, binaryImg);
	//cvWaitKey(0);


	//CHOOSE ONE POINT AS A ORIGIN OF THE LOCAL COORDINATE SYSTEM.
	origin = *points2Check.begin();


	Utils::createCoordinateOutStream(&xPlaneCoor, &yPlaneCoor, &zPlaneCoor, planeNum, cam->getCamId());
	Utils::createGeneralOutStream(&planeParam, "Parameters_", planeNum, cam->getCamId());
}

DynamicPlane::DynamicPlane(XnPoint3D* p, int width, int height, int maskS, int* rgbPlane, int planeNum, CameraProperties* camera)
{
	firstTime = true;
	//store the seed
	points2Check.push_back(*p);

	//initializes the mask and binary image
	binaryImg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
	mask = cvCreateImage(cvSize(maskS, maskS), IPL_DEPTH_8U, 1);
	Utils::initImage(mask, 1);
	Utils::initImage(binaryImg , 0);
	((uchar*)(binaryImg->imageData + (int)p->Y*binaryImg->widthStep))[(int)p->X] = 1;
	maskSize = maskS;
	colorPlane = rgbPlane;
	cam = camera;

	//intialializes the arrays
	pointCoors[0] = NULL;
	pointDepth[0] = NULL;

	//add points2Check to the plane fitting
	planeParameters = cvCreateMat(3,1,CV_32FC1);
	calculatePlaneParams();

	Utils::createCoordinateOutStream(&xPlaneCoor, &yPlaneCoor, &zPlaneCoor, planeNum, cam->getCamId());
	Utils::createGeneralOutStream(&planeParam, "Parameters_", planeNum, cam->getCamId());
}


DynamicPlane::~DynamicPlane(void)
{
}

//Public members

CvMat* DynamicPlane::getPlaneParameters()
{
	return planeParameters;
}

/*
Make the region grow. From a seed, it checks its neigbours's depth to add them to the plane list of points
in case its depth is less than a certain threshold 
*/
//void DynamicPlane::makePlaneGrow(char* winName, IplImage* depthImage, const XnDepthPixel* pDepthMap)
//{	
//	int maskOffset = (maskSize/2);
//	int minLimit = maskOffset;
//	int xMaxLimit = XN_VGA_X_RES - maskOffset;
//	int yMaxLimit = XN_VGA_Y_RES - maskOffset;
//	XnPoint3D p3D;
//
//	cvShowImage(winName, depthImage);
//	cvWaitKey(1);
//
//	while (points2Check.size() != 0) //if there is no more points to check->finishes
//	{
//		//check neighbours of 'points2Check' list. (all the points that fits in the plane are store in memory)
//		list<XnPoint3D>::iterator it1;
//		for (it1=points2Check.begin(); it1!=points2Check.end(); ++it1)
//		{	
//			XnPoint3D p =  *it1;
//			if (p.X >= minLimit && p.X < xMaxLimit && p.Y >= minLimit && p.Y < yMaxLimit)
//			{
//				//uchar* ptrImgMask = (uchar*)binaryImg_mask ->imageData + ((int)p.Y)*binaryImg_mask->widthStep + (int)p.X;	
//				if ((int)p.X%maskOffset == 0 && (int)p.Y%maskOffset == 0)
//				{
//					CvRect rect = cvRect(p.X-maskOffset, p.Y-maskOffset, maskSize,maskSize);
//					cvSetImageROI(binaryImg, rect);
//					cvMul(binaryImg, mask, binaryImg);
//					checkNeighbours(&p, pDepthMap, &rect);
//					planePoints.push_back(p); //add to the final list
//					//draw the new pixel in the image
////DEBUG BEGIN
//backProjectPoint(&p, &p3D);
//xPlaneCoor << (int)p3D.X << " ";
//yPlaneCoor << (int)p3D.Y << " ";
//zPlaneCoor << (int)p3D.Z << " ";
////DEBUG END
//					cvCircle(depthImage, cvPoint(p.X, p.Y), 1, cvScalar(colorPlane[0],colorPlane[1],colorPlane[2]));
//					cvShowImage(winName, depthImage);
//					cvWaitKey(1);
//				}
//			}
//		}
//
//		firstTime = false;
//		points2Check.clear();
//		//Take all the points store in memory to the list 'points2Check'
//		points2Check.assign(tmp.begin(),tmp.end());
//		//add points2Check to the plane fitting
//		calculatePlaneParams();
//		tmp.clear();
//	}
//	
//
////	float a = *(planeParameters->data.fl);
////	float b = *(planeParameters->data.fl + planeParameters->step/sizeof(float));
////	float c = *(planeParameters->data.fl + 2*planeParameters->step/sizeof(float));
////	*(planeParameters->data.fl + 2*planeParameters->step/sizeof(float)) = origin.Z - (origin.X*a + origin.Y*b);
//
////DEBUG BEGIN
//for (int i = 0; i < 3; i++)
//{
//	planeParam << *(planeParameters->data.fl + i*planeParameters->step/sizeof(float)) << endl;
//}
////DEBUG END
//
//}


void DynamicPlane::makePlaneGrow(char* winName, IplImage* depthImage, const XnDepthPixel* pDepthMap)
{	
	int maskOffset = (maskSize/2);
	int minLimit = maskOffset;
	int xMaxLimit = XN_VGA_X_RES - maskOffset;
	int yMaxLimit = XN_VGA_Y_RES - maskOffset;
	XnPoint3D p3D;
	while (points2Check.size() != 0) //if there is no more points to check->finishes
	{
		//check neighbours of 'points2Check' list. (all the points that fits in the plane are store in memory)
		list<XnPoint3D>::iterator it1;
		for (it1=points2Check.begin(); it1!=points2Check.end(); ++it1)
		{	
			XnPoint3D p =  *it1;
			if (p.X >= minLimit && p.X < xMaxLimit && p.Y >= minLimit && p.Y < yMaxLimit)
			{

				CvRect rect = cvRect(p.X-maskOffset, p.Y-maskOffset, maskSize,maskSize);
				cvSetImageROI(binaryImg, rect);
				cvMul(binaryImg, mask, binaryImg);
				checkNeighbours(&p, pDepthMap, &rect);
				planePoints.push_back(p); //add to the final list
				//draw the new pixel in the image
//DEBUG BEGIN
backProjectPoint(&p, &p3D);
xPlaneCoor << (int)p3D.X << " ";
yPlaneCoor << (int)p3D.Y << " ";
zPlaneCoor << (int)p3D.Z << " ";
//DEBUG END
				cvCircle(depthImage, cvPoint(p.X, p.Y), 1, cvScalar(colorPlane[0],colorPlane[1],colorPlane[2]));
				cvShowImage(winName, depthImage);
				cvWaitKey(1);
			}
		}

		firstTime = false;
		points2Check.clear();
		//Take all the points store in memory to the list 'points2Check'
		points2Check.assign(tmp.begin(),tmp.end());
		//add points2Check to the plane fitting
		calculatePlaneParams();
		tmp.clear();
	}
	
//DEBUG BEGIN
for (int i = 0; i < 3; i++)
{
	planeParam << *(planeParameters->data.fl + i*planeParameters->step/sizeof(float)) << endl;
}
//DEBUG END

//Calculate centroid
list<XnPoint3D>::iterator it1;
XnPoint3D total;
total.X = 0; total.Y = 0; total.Z = 0;
for (it1=planePoints.begin(); it1!=planePoints.end(); ++it1)
{
	XnPoint3D p = *it1;
	total.X += p.X;
	total.Y += p.Y;
	total.Z += p.Z;
}
centroid2D.X = total.X/planePoints.size();
centroid2D.Y = total.Y/planePoints.size();
centroid2D.Z = total.Z/planePoints.size();

}


XnPoint3D* DynamicPlane::getCentroid()
{
	return &centroid2D;
}

list<XnPoint3D>* DynamicPlane::getPlanePoints()
{
	return &planePoints;
}

//Privats members

/*
The n-connect neighbours of 'p' are checked. The neighbours which have a depth difference wit 'p' equal or smaller than 
'MAX_DEPTH' are added to the plane list
*/
void DynamicPlane::checkNeighbours(const XnPoint3D* p, const XnDepthPixel* pDepthMap, const CvRect* rect)
{
	int endY = rect->y + rect->height;
	int endX = rect->x + rect->width;
	int maskOffset = (maskSize/2);
	int contY = rect->y;
	int contX;
	while (contY < endY)
	{
		contX = rect->x;
		while (contX < endX)
		{
			uchar* ptr = (uchar*)binaryImg->imageData + contY*binaryImg->widthStep + contX;
			if (*ptr == 0) 
			{
				float depth = pDepthMap[contY*XN_VGA_X_RES+contX];	
				XnPoint3D pNe, pNe3D;
				pNe.X = contX; pNe.Y = contY; pNe.Z = depth;
				backProjectPoint(&pNe, &pNe3D);
				
				/*XnPoint3D pNe3D_local;
				pNe3D_local.X = pNe3D.X-origin.X;
				pNe3D_local.Y = pNe3D.Y-origin.Y;
				pNe3D_local.Z = pNe3D.Z-origin.Z;*/

				if (depth != 0 && isInPlane(&pNe3D))
				{
					tmp.push_back(pNe);
					*ptr = 255; 
				}
			}
			contX += 2*maskOffset;
		}
		contY += 2*maskOffset;
	}
}

/*
Check if 'p' is in any of the three possible list
*/
bool DynamicPlane::isInPlane(XnPoint3D* p)
{
	float params[3];
	for (int i = 0; i < 3; i++)
	{
		params[i] = *(planeParameters->data.fl + i*planeParameters->step/sizeof(float));
	}

	float depth = params[0]*p->X + params[1]*p->Y + params[2];

	return (abs(depth - p->Z) <= MAX_DEPTH_DIFF);
}


/*
Initializes a matrix with the value 'num'
*/
void DynamicPlane::initMatrix(CvMat* m, int num)
{
	for (int i = 0; i < m->rows; i++)
	{
		unsigned char* ptr = (unsigned char*)(m->data.ptr + i*m->step);
		for (int j = 0; j < m->cols; j++)
		{
			*ptr = num;
			ptr++;
		}
	}
}


/*
Checks that x and y are a corner coordinates of the mask in 'p'
*/
bool DynamicPlane::isInACorner(const XnPoint3D* p, int x, int y)
{
	int maskOffset = (maskSize/2);

	return (((p->X - maskOffset) == x) && ((p->Y - maskOffset) == y) ||
		((p->X - maskOffset) == x) && ((p->Y + maskOffset) == y) ||
		((p->X + maskOffset) == x) && ((p->Y + maskOffset) == y) ||
		((p->X + maskOffset) == x) && ((p->Y - maskOffset) == y));
}


/*
Fit a plane using the points in 'planePoints' and 'points2Check'
*/
void DynamicPlane::calculatePlaneParams()
{
	list<XnPoint3D>::iterator it;
	it =points2Check.begin();
	int pointCount = planePoints.size();
	XnPoint3D p3D;
	while (it!= points2Check.end() && pointCount < MAX_POINTS)
	{
		XnPoint3D p = *it;
		backProjectPoint(&p, &p3D);
		
		//EXPRESS THE POINT IN THE LOCAL COORDINATE SYSTEM.
	/*	XnPoint3D p3D_local;
		p3D_local.X = p3D.X-origin.X;
		p3D_local.Y = p3D.Y-origin.Y;
		p3D_local.Z = p3D.Z-origin.Z;*/

	//I'm storgin the points in the external coordinate systems, but I'm calculating the plane
		// in the local coordinate system., so I should transform the normal again to the local cs. (leverage)

		pointCoors[3*pointCount] = p3D.X;
		pointCoors[3*pointCount + 1] = p3D.Y;
		pointCoors[3*pointCount + 2] = 1;
		pointDepth[pointCount] = p3D.Z;
		pointCount++;
		it++;

	}
	pointCoors[3*pointCount] = NULL;
	pointDepth[pointCount] = NULL;
	
	CvMat coordinatesMat;
	CvMat depthMat;
	CvMat* coordinatesMatPseudo = cvCreateMat(3, pointCount, CV_32FC1);

	cvInitMatHeader(&coordinatesMat, pointCount, 3, CV_32FC1, pointCoors);
	cvInitMatHeader(&depthMat, pointCount, 1, CV_32FC1, pointDepth);

	cvInvert(&coordinatesMat, coordinatesMatPseudo, CV_SVD);
	cvMatMul(coordinatesMatPseudo, &depthMat, planeParameters);

}


/*
Back project 'p' into the space using the intrinsic parameters provided by 'cam'
*/
void DynamicPlane::backProjectPoint(XnPoint3D* p, XnPoint3D* p3D)
{
	p3D->X = (p->X - cam->getOx())*cam->getPixelSize()*2*p->Z/cam->getFocalLenghtX();
	p3D->Y = (p->Y - cam->getOy())*cam->getPixelSize()*2*p->Z/cam->getFocalLenghtY();
	p3D->Z = p->Z;
}

/*
Check if 'p' is in any of the three possible list
*/
bool DynamicPlane::isInPlane2(XnPoint3D* p)
{
	float params[3];
	for (int i = 0; i < 3; i++)
	{
		params[i] = *(planeParameters->data.fl + i*planeParameters->step/sizeof(float));
	}

	float depth = params[0]*p->X + params[1]*p->Y + params[2];

	return (abs(depth - p->Z) <= 45);
}

/*
Checks all the points again with the parameters and remove all of the points that are not in the plane
*/
void DynamicPlane::checkNoise()
{
	list<XnPoint3D>::iterator it;
	it =points2Check.begin();
	XnPoint3D p3D;
	while (it!= points2Check.end())
	{
		XnPoint3D p = *it;
		backProjectPoint(&p, &p3D);
		if (!isInPlane(&p3D))
			it = points2Check.erase(it);
		else
			it++;
	}
}