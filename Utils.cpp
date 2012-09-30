#include "Utils.h"

Utils::Utils(void)
{
}


Utils::~Utils(void)
{
}

const double Utils::COLORFILTER_THRESHOLD = 0.01;

void Utils::loadCameraParameters(CameraProperties* cam)
{
	loadCameraIntrinsics(cam);
	loadCameraExtrinsics(cam);
}

void Utils::loadCameraExtrinsics(CameraProperties* cam)
{
//	CvMat* transT = cvCreateMat(3,1, CV_32FC1);
//	CvMat* trans = cvCreateMat(1,3, CV_32FC1);
	if (cam->getCamId() == 1)
	{
		cam->setRotationMatrix((CvMat*)cvLoad("D:\\CameraCalibrations\\extrinsics\\rotation_12.yml"));
		cam->setTranslationMatrix((CvMat*)cvLoad("D:\\CameraCalibrations\\extrinsics\\translation_12.yml"));
//		cam->setRotationMatrix((CvMat*)cvLoad(filePaths::CAM1_2_ROTATION_FILEPATH));
//		trans = (CvMat*)cvLoad(filePaths::CAM1_2_TRANSLATION_FILEPATH);
//		cvTranspose(trans, transT);
//		cam->setTranslationMatrix(transT);
	}
	else
	{
		cam->setRotationMatrix((CvMat*)cvLoad("D:\\CameraCalibrations\\extrinsics\\rotation_21.yml"));
		cam->setTranslationMatrix((CvMat*)cvLoad("D:\\CameraCalibrations\\extrinsics\\translation_21.yml"));
//		cam->setRotationMatrix((CvMat*)cvLoad(filePaths::CAM2_1_ROTATION_FILEPATH));
//		trans = (CvMat*)cvLoad(filePaths::CAM2_1_TRANSLATION_FILEPATH);
//		cvTranspose(trans, transT);
//		cam->setTranslationMatrix(transT);
	}
//	cvReleaseMat(&trans);
}
void Utils::loadCameraIntrinsics(CameraProperties* cam)
{
	if (cam->getCamId() == 1)
	{
		cam->setIntrinsicMatrix((CvMat*)cvLoad(filePaths::CAM1_INTRINSIC_FILEPATH));
		cam->setDistortionCoeffs((CvMat*)cvLoad(filePaths::CAM1_DISTORTION_FILEPATH));
	}
	else
	{
		cam->setIntrinsicMatrix((CvMat*)cvLoad(filePaths::CAM2_INTRINSIC_FILEPATH));
		cam->setDistortionCoeffs((CvMat*)cvLoad(filePaths::CAM2_DISTORTION_FILEPATH));
	}

//	cam->getContext()->Init();
	cam->getDepthNode()->Create(*cam->getContext());
	double pSize;
	cam->getDepthNode()->GetRealProperty ("ZPPS", pSize);
	cam->setPixelSize(pSize);
			
	float FxPix = CV_MAT_ELEM( *cam->getIntrinsicMatrix(), float, 0, 0);
	float FyPix = CV_MAT_ELEM( *cam->getIntrinsicMatrix(), float, 1, 1);	
	float FxMM = FxPix*(cam->getPixelSize()*2);
	float FyMM = FyPix*(cam->getPixelSize()*2);
	cam->setFocalLenghtX(FxMM);
	cam->setFocalLenghtY(FyMM);
	cam->setOx(CV_MAT_ELEM( *cam->getIntrinsicMatrix(), float, 0, 2));
	cam->setOy(CV_MAT_ELEM( *cam->getIntrinsicMatrix(), float, 1, 2));
}

void Utils::initIntrinsicParameters(CameraProperties* cam)
{
	if (cam->getCamId() == 1)
	{
		cam->setIntrinsicMatrix((CvMat*)cvLoad(filePaths::CAM1_INTRINSIC_FILEPATH));
		cam->setDistortionCoeffs((CvMat*)cvLoad(filePaths::CAM1_DISTORTION_FILEPATH));
	}
	else
	{
		cam->setIntrinsicMatrix((CvMat*)cvLoad(filePaths::CAM2_INTRINSIC_FILEPATH));
		cam->setDistortionCoeffs((CvMat*)cvLoad(filePaths::CAM2_DISTORTION_FILEPATH));
	}		
	float FxPix = CV_MAT_ELEM( *cam->getIntrinsicMatrix(), float, 0, 0);
	float FyPix = CV_MAT_ELEM( *cam->getIntrinsicMatrix(), float, 1, 1);	
	float FxMM = FxPix*(cam->getPixelSize()*2);
	float FyMM = FyPix*(cam->getPixelSize()*2);
	cam->setFocalLenghtX(FxMM);
	cam->setFocalLenghtY(FyMM);
	cam->setOx(CV_MAT_ELEM( *cam->getIntrinsicMatrix(), float, 0, 2));
	cam->setOy(CV_MAT_ELEM( *cam->getIntrinsicMatrix(), float, 1, 2));
}

void Utils::rgbdInit(CameraProperties* cam, int idCam)
{
	cam->getContext()->Init();
	cam->setCamId(idCam);

	cam->getDepthNode()->Create(*cam->getContext());
	cam->getImageNode()->Create(*cam->getContext());

	XnMapOutputMode mapModeVGA;
	mapModeVGA.nXRes = 640;
	mapModeVGA.nYRes = 480;
	mapModeVGA.nFPS = 30;

	cam->getImageNode()->SetMapOutputMode(mapModeVGA);
	cam->getDepthNode()->SetMapOutputMode(mapModeVGA);

		
	//XnBool isSupported = cam->getDepthNode()->IsCapabilitySupported("AlternativeViewPoint");
	//if (isSupported)
	//	cam->getDepthNode()->GetAlternativeViewPointCap().SetViewPoint(*cam->getImageNode());
	
	//extra parameters
	double pSize;
	cam->getDepthNode()->GetRealProperty ("ZPPS", pSize);
	cam->setPixelSize(pSize);

	if (idCam == 1)
	{
		cam->setIntrinsicMatrix((CvMat*)cvLoad(filePaths::CAM1_INTRINSIC_FILEPATH));
		cam->setDistortionCoeffs((CvMat*)cvLoad(filePaths::CAM1_DISTORTION_FILEPATH));
	}
	else
	{
		cam->setIntrinsicMatrix((CvMat*)cvLoad(filePaths::CAM2_INTRINSIC_FILEPATH));
		cam->setDistortionCoeffs((CvMat*)cvLoad(filePaths::CAM2_DISTORTION_FILEPATH));
	}
			
	float FxPix = CV_MAT_ELEM( *cam->getIntrinsicMatrix(), float, 0, 0);
	float FyPix = CV_MAT_ELEM( *cam->getIntrinsicMatrix(), float, 1, 1);	
	float FxMM = FxPix*(cam->getPixelSize()*2);
	float FyMM = FyPix*(cam->getPixelSize()*2);
	cam->setFocalLenghtX(FxMM);
	cam->setFocalLenghtY(FyMM);
	cam->setOx(CV_MAT_ELEM( *cam->getIntrinsicMatrix(), float, 0, 2));
	cam->setOy(CV_MAT_ELEM( *cam->getIntrinsicMatrix(), float, 1, 2));

//	cam->setRGBIntrinsicMatrix((CvMat*)cvLoad(filePaths::CAM1_RGBINTRINSIC_FILEPATH));
//	cam->setRotationMatrix((CvMat*)cvLoad(filePaths::CAM1_RGBD_ROTATION_FILEPATH));
//	cam->setTranslationMatrix((CvMat*)cvLoad(filePaths::CAM1_RGBD_TRANSLATION_FILEPATH));

}

void Utils::rgbdInitAligned(CameraProperties* cam1, CameraProperties* cam2)
{
	cam1->setCamId(1);
	cam2->setCamId(2);
	cam1->getContext()->Init();
	cam2->getContext()->Init();

	NodeInfoList depth_node_info_list, image_node_info_list;
	Query query;
	query.SetVendor("PrimeSense");
	cam1->getContext()->EnumerateProductionTrees(XN_NODE_TYPE_DEPTH, &query, depth_node_info_list, NULL);
	cam2->getContext()->EnumerateProductionTrees(XN_NODE_TYPE_IMAGE, &query, image_node_info_list, NULL);

	//depth node instantiation
	NodeInfoList::Iterator nodeIt = depth_node_info_list.Begin();
	NodeInfo& firstNode = *nodeIt; // first kinect connected
	nodeIt++;
	NodeInfo& secondNode = *nodeIt; // second kinect connected
	firstNode.GetInstance(*cam1->getDepthNode());
	secondNode.GetInstance(*cam2->getDepthNode());
	cam1->getContext()->CreateProductionTree(firstNode);
	cam2->getContext()->CreateProductionTree(secondNode);

	cam1->getDepthNode()->Create(*cam1->getContext());
	cam2->getDepthNode()->Create(*cam2->getContext());

	//RGB node instantiation
	nodeIt = image_node_info_list.Begin();
	firstNode = *nodeIt; // first kinect connected
	nodeIt++;
	secondNode = *nodeIt; // second kinect connected
	firstNode.GetInstance(*cam1->getImageNode());
	secondNode.GetInstance(*cam2->getImageNode());
	cam1->getContext()->CreateProductionTree(firstNode);
	cam2->getContext()->CreateProductionTree(secondNode);
	
	cam1->getImageNode()->Create(*cam1->getContext());
	cam2->getImageNode()->Create(*cam2->getContext());

	//set parameters of both cameras
	XnMapOutputMode mapModeVGA;
	mapModeVGA.nXRes = 640;
	mapModeVGA.nYRes = 480;
	mapModeVGA.nFPS = 30;

	cam1->getImageNode()->SetMapOutputMode(mapModeVGA);
	cam1->getDepthNode()->SetMapOutputMode(mapModeVGA);

	cam2->getImageNode()->SetMapOutputMode(mapModeVGA);
	cam2->getDepthNode()->SetMapOutputMode(mapModeVGA);

	//Align both sensors (rgb and depth) in all cameras
	XnBool isSupported = cam1->getDepthNode()->IsCapabilitySupported("AlternativeViewPoint");
	if (isSupported)
		cam1->getDepthNode()->GetAlternativeViewPointCap().SetViewPoint(*cam1->getImageNode());

	isSupported = cam2->getDepthNode()->IsCapabilitySupported("AlternativeViewPoint");
	if (isSupported)
		cam2->getDepthNode()->GetAlternativeViewPointCap().SetViewPoint(*cam2->getImageNode());

	//extra parameters
	double pSize1, pSize2;
	cam1->getDepthNode()->GetRealProperty ("ZPPS", pSize1);
	cam2->getDepthNode()->GetRealProperty ("ZPPS", pSize2);
	cam1->setPixelSize(pSize1);
	cam2->setPixelSize(pSize2);
}

void Utils::rgbdInit(CameraProperties* cam1, CameraProperties* cam2)
{
	cam1->setCamId(1);
	cam2->setCamId(2);
	cam1->getContext()->Init();
	cam2->getContext()->Init();

	NodeInfoList depth_node_info_list, image_node_info_list;
	Query query;
	query.SetVendor("PrimeSense");
	cam1->getContext()->EnumerateProductionTrees(XN_NODE_TYPE_DEPTH, &query, depth_node_info_list, NULL);
	cam2->getContext()->EnumerateProductionTrees(XN_NODE_TYPE_IMAGE, &query, image_node_info_list, NULL);

	NodeInfoList::Iterator nodeIt = depth_node_info_list.Begin();
	NodeInfo& firstNode = *nodeIt; // first kinect connected
	nodeIt++;
	NodeInfo& secondNode = *nodeIt; // second kinect connected
	firstNode.GetInstance(*cam1->getDepthNode());
	secondNode.GetInstance(*cam2->getDepthNode());
	cam1->getContext()->CreateProductionTree(firstNode);
	cam2->getContext()->CreateProductionTree(secondNode);

	cam1->getDepthNode()->Create(*cam1->getContext());
	cam2->getDepthNode()->Create(*cam2->getContext());

	nodeIt = image_node_info_list.Begin();
	firstNode = *nodeIt; // first kinect connected
	nodeIt++;
	secondNode = *nodeIt; // second kinect connected
	firstNode.GetInstance(*cam1->getImageNode());
	secondNode.GetInstance(*cam2->getImageNode());
	cam1->getContext()->CreateProductionTree(firstNode);
	cam2->getContext()->CreateProductionTree(secondNode);
	
	cam1->getImageNode()->Create(*cam1->getContext());
	cam2->getImageNode()->Create(*cam2->getContext());

	XnMapOutputMode mapModeVGA;
	mapModeVGA.nXRes = 640;
	mapModeVGA.nYRes = 480;
	mapModeVGA.nFPS = 30;

	cam1->getImageNode()->SetMapOutputMode(mapModeVGA);
	cam1->getDepthNode()->SetMapOutputMode(mapModeVGA);

	cam2->getImageNode()->SetMapOutputMode(mapModeVGA);
	cam2->getDepthNode()->SetMapOutputMode(mapModeVGA);

	//extra parameters
	double pSize1, pSize2;
	cam1->getDepthNode()->GetRealProperty ("ZPPS", pSize1);
	cam2->getDepthNode()->GetRealProperty ("ZPPS", pSize2);
	cam1->setPixelSize(pSize1);
	cam2->setPixelSize(pSize2);
}


/*
Generates an image with the rgb and depth information. If there is noise in the depth
data it sets a 255 value.
*/
void Utils::fillImageData(IplImage* image, const XnRGB24Pixel* pImageMap, const XnDepthPixel* pDepthMap)
{
	for (int y=0; y<XN_VGA_Y_RES; y++)
	{
		uchar *ptr = (uchar*)image->imageData + y*image->widthStep;
		for (int x=0; x<XN_VGA_X_RES; x++)
		{
			if (pDepthMap[y * XN_VGA_X_RES + x] != 0)
			{
				ptr[3*x] = pImageMap->nBlue;
				ptr[3*x + 1] = pImageMap->nGreen;
				ptr[3*x + 2] = pImageMap->nRed;
			}
			else
			{
				ptr[3*x] = 255;
				ptr[3*x + 1] = 255;
				ptr[3*x + 2] = 255;
			}
			pImageMap++;
		}
	}
}


/*
Generates an image with the rgb
*/
void Utils::fillImageDataFull(IplImage* image, const XnRGB24Pixel* pImageMap)
{
		
	for (int y=0; y<XN_VGA_Y_RES; y++)
	{
		uchar *ptr = (uchar*)image->imageData + y*image->widthStep;
		//for (int x=(XN_VGA_X_RES-1); x>= 0; x--)
		for (int x=0; x<XN_VGA_X_RES; x++)
		{
			ptr[3*x] = pImageMap->nBlue;
			ptr[3*x + 1] = pImageMap->nGreen;
			ptr[3*x + 2] = pImageMap->nRed;
			pImageMap++;
		}
	}

}

void Utils::runCameraContext(CameraProperties* cam, XnDepthPixel* pDepthMap, XnRGB24Pixel* pImageMap, IplImage *kinectRGBImage)
{
	cam->getContext()->WaitAndUpdateAll();
	pImageMap = (XnRGB24Pixel*)cam->getImageNode()->GetRGB24ImageMap();
	fillImageData(kinectRGBImage, pImageMap, pDepthMap);
}


void Utils::copyDepthMap(const XnDepthPixel* depthMapIn, XnDepthPixel* depthMapOut)
{
	for (int y=0; y<XN_VGA_Y_RES; y++)
	{
		for (int x=0; x<XN_VGA_X_RES; x++)
		{
			depthMapOut[y * XN_VGA_X_RES + x] = depthMapIn[y * XN_VGA_X_RES + x];	
		}
	}

}

void Utils::analyzeDepthMap(const XnDepthPixel* depthMap1, const XnDepthPixel* depthMap2)
{
	float noiseError = 0.0;
	int noiseCount = 0;
	int pixelCount = 0;
	int totalPixels = 0;
	for (int y=0; y<XN_VGA_Y_RES; y++)
	{
		for (int x=0; x<XN_VGA_X_RES; x++)
		{
			float depth1 = depthMap1[y * XN_VGA_X_RES + x];	
			float depth2 = depthMap2[y * XN_VGA_X_RES + x];
			if (depth1 != 0.0)
			{
				totalPixels++;
				if (depth2 == 0.0)
					noiseCount++;
				else
				{
					noiseError += abs(depth1 - depth2);
					pixelCount++;
				}
			}
		}
	}

	
	float percentage = ((float)(noiseCount*100))/totalPixels;
	float noiseAvg = noiseError/pixelCount;

	cout << "The percentage of pixels corrupted is: " << percentage << "%" << endl;
	cout << "The average of error in the measurement is: " << noiseAvg << "mm"<< endl;

}


void Utils::drawPoints(IplImage* img, CvPoint2D32f* corners, int numCorners)
{
	for (int i = 0; i < numCorners; i++)
		cvCircle(img, cvPoint(corners[i].x, corners[i].y), 2, cvScalar(0,0,255));
}

float Utils::calculateDistance(XnPoint3D p1, XnPoint3D p2)
{
	return sqrt(pow(p2.X - p1.X, 2) + pow(p2.Y - p1.Y, 2) + pow(p2.Z - p1.Z, 2));
}

void Utils::analyzeData(const CvPoint2D32f* corners, int nWidth, int nHeight,  const XnDepthPixel* pDepthMap, const DepthGenerator* depthNode)
{
	//Analize depth.
	int totalDepth = 0;
	int nCorners = nWidth*nHeight;
	int totalP = 0;
	for (int i = 0; i < nCorners; i++)
	{
		int depth = pDepthMap[(int)corners[i].y * XN_VGA_X_RES + (int)corners[i].x];
		if (depth != 0)
		{
			totalDepth += depth;
			totalP++;
		}
	}
	cout << "Average Distance: " << totalDepth/totalP << " mm." << endl;

	//Analize distances

	//Obtain 3d coordinates of all the points;
	XnPoint3D* point2d = new XnPoint3D[nCorners];
	XnPoint3D* point3d = new XnPoint3D[nCorners];
	for (int i = 0; i < nCorners; i++)
	{
		point2d[i].X = corners[i].x;
		point2d[i].Y = corners[i].y;
		point2d[i].Z = pDepthMap[(int)point2d[i].Y*XN_VGA_X_RES+(int)point2d[i].X];
	}
	depthNode->ConvertProjectiveToRealWorld(nCorners, point2d, point3d);



	float sumSize = 0.0;
	int cont = 0;
	//measure distances
	for (int y = 0; y < nHeight; y++)
	{
		for (int x = 0; x < nWidth-1; x++)
		{
			sumSize += calculateDistance(point3d[y*nWidth+x], point3d[y*nWidth+x+1]);
			cont++;
		}
	}
	cout << "Average size of the squares: " << sumSize/(cont-1) << " mm." << endl;

}

void Utils::analyzeData(XnPoint3D* roi, const XnDepthPixel* pDepthMap, const DepthGenerator* depthNode)
{
	//analize depth
	int initX = roi[0].X;
	int endX =  roi[1].X;
	int initY = roi[0].Y;
	int endY =  roi[1].Y;
	int width = endX - initX;
	int height = endY - initY;
	int totalPix = 0;

	int sumDepth = 0;
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			int depth = pDepthMap[(initY+y)*XN_VGA_X_RES+(initX+x)];
			if (depth != 0)
			{
				sumDepth += depth;
				totalPix++;
			}
		}
	}
	cout << "Average Distance: " << sumDepth/totalPix << " mm." << endl;

	//analize measurment
	roi[0].Z = pDepthMap[(int)roi[0].Y*XN_VGA_X_RES+(int)roi[0].X];
	roi[1].Z = pDepthMap[(int)roi[1].Y*XN_VGA_X_RES+(int)roi[1].X];
	XnPoint3D point3d[2];
	depthNode->ConvertProjectiveToRealWorld(2, roi, point3d);
	float dist = calculateDistance(point3d[0], point3d[1]);

	cout << "Distance between corners: " << dist << " mm." << endl;
}


void Utils::searchPoint(XnPoint3D* point, const XnDepthPixel* pDepthMap)
{
	int depth = pDepthMap[(int)point->Y*XN_VGA_X_RES+(int)point->X];

	int cont = 1;
	while (depth == 0.0)
	{
		//Right and Left
		depth = pDepthMap[(int)point->Y*XN_VGA_X_RES+(int)(point->X+cont)];
		if (depth != 0.0) 
		{
			point->X += cont;
			continue;
		}
		depth = pDepthMap[(int)point->Y*XN_VGA_X_RES+(int)(point->X-cont)];
		if (depth != 0.0) 
		{
			point->X -= cont;
			continue;
		}
		// Diagonals
		depth = pDepthMap[(int)(point->Y+cont)*XN_VGA_X_RES+(int)(point->X+cont)];
		if (depth != 0.0) 
		{
			point->X += cont;
			point->Y += cont;
			continue;
		}
		depth = pDepthMap[(int)(point->Y+cont)*XN_VGA_X_RES+(int)(point->X-cont)];
		if (depth != 0.0) 
		{
			point->X -= cont;
			point->Y += cont;
			continue;
		}
		depth = pDepthMap[(int)(point->Y-cont)*XN_VGA_X_RES+(int)(point->X+cont)];
		if (depth != 0.0) 
		{
			point->X += cont;
			point->Y -= cont;
			continue;
		}
		depth = pDepthMap[(int)(point->Y-cont)*XN_VGA_X_RES+(int)(point->X-cont)];
		if (depth != 0.0) 
		{
			point->X -= cont;
			point->Y -= cont;
			continue;
		}
		// Up and down
		depth = pDepthMap[(int)point->Y*XN_VGA_X_RES+(int)(point->X+cont)];
		if (depth != 0.0) 
		{
			point->X += cont;
			continue;
		}
		depth = pDepthMap[(int)(point->Y+cont)*XN_VGA_X_RES+(int)point->X];
		if (depth != 0.0) 
		{
			point->Y += cont;
			continue;
		}
		depth = pDepthMap[(int)(point->Y-cont)*XN_VGA_X_RES+(int)point->X];
		if (depth != 0.0) 
		{
			point->Y -= cont;
			continue;
		}
		cont++;
	}

}


bool Utils::searchNeighbour(const XnPoint3D* p1, XnPoint3D* p2, const XnDepthPixel* pDepthMap)
{
	p2->X = p1->X;
	p2->Y = p1->Y;
	int depth = 0;
	int cont = 1;

	//Right and Left
	depth = pDepthMap[(int)p2->Y*XN_VGA_X_RES+(int)(p2->X + cont)];
	if (depth != 0)
	{
		p2->X += cont; 
		return true;
	}
	depth = pDepthMap[(int)p2->Y*XN_VGA_X_RES+(int)(p2->X - cont)];
	if (depth != 0)
	{
		p2->X -= cont; 
		return true;
	}
	// Up and down
	depth = pDepthMap[(int)(p2->Y+cont)*XN_VGA_X_RES+(int)p2->X];
	if (depth != 0)
	{
		p2->Y += cont; 
		return true;
	}
	depth = pDepthMap[(int)(p2->Y-cont)*XN_VGA_X_RES+(int)p2->X];
	if (depth != 0)
	{
		p2->Y -= cont; 
		return true;
	}
	//Diagonals
	depth = pDepthMap[(int)(p2->Y+cont)*XN_VGA_X_RES+(int)(p2->X+cont)];
	if (depth != 0)
	{
		p2->X += cont;
		p2->Y += cont; 
		return true;
	}
	depth = pDepthMap[(int)(p2->Y-cont)*XN_VGA_X_RES+(int)(p2->X-cont)];
	if (depth != 0)
	{
		p2->X -= cont;
		p2->Y -= cont; 
		return true;
	}
	depth = pDepthMap[(int)(p2->Y+cont)*XN_VGA_X_RES+(int)(p2->X-cont)];
	if (depth != 0)
	{
		p2->X -= cont;
		p2->Y += cont; 
		return true;
	}
	depth = pDepthMap[(int)(p2->Y-cont)*XN_VGA_X_RES+(int)(p2->X+cont)];
	if (depth != 0)
	{
		p2->X += cont;
		p2->Y -= cont; 
		return true;
	}
	return false;
	
}



/*
Auxiliar function for debugging. Shows the values of mat.
*/
void Utils::writeMatrixValues(const CvMat* mat, ostream* out)
{
	for (int i = 0; i < mat->rows; i++)
	{
		float* ptr = (float*)(mat->data.fl + i*mat->step/sizeof(float));
		for (int j = 0; j < mat->cols; j++)
		{
			*out << "(" << i << ", " << j << ") =" << *ptr << "\n";
			ptr++;
		}
	}
}

void Utils::writeImageValues(const IplImage* img, ostream* out)
{

	for (int i = 0; i < img->height; i++)
	{
		uchar* ptr = (uchar*)(img->imageData + i*img->widthStep);
		for (int j = 0; j < img->width; j++)
		{
			*out << "(" << i << ", " << j << ") =" << (int)*ptr << "\n";			
			ptr++;
		}
	}
}

void Utils::writeROIImageValues(const IplImage* img, CvRect* rect, ostream* out)
{
	for (int i = rect->y; i < (rect->y + rect->height); i++) 
	{
		
		for (int j = rect->x; j < (rect->x + rect->width); j++) 
		{
			uchar* ptr = (uchar*)img->imageData + i*img->widthStep + j;
			*out << "(" << j << ", " << i << ") =" << (int)*ptr << "\n";			
		}
    }
}

static void writeXmlMatrix(const CvMat* mat, const char* filePath, const char* matrixType)
{
	char elements[400];
	char buff[30];
	for (int r = 0; r < mat->rows; r++)
		for (int c = 0; c < mat->cols; c++)
		{
			sprintf(buff,"%f",CV_MAT_ELEM( *mat, float, r, c)); 
			if (r == 0 && c == 0)
				strcpy (elements,buff);
			else
			{
				strcat(elements, " ");
				strcat(elements, buff);
			}

		}


	cout << elements << endl;

	ofstream outXmlFile(filePath, ios::out);
	outXmlFile << "<?xml version=\"1.0\"?>" << endl;
	outXmlFile << "<opencv_storage>" << endl;
	outXmlFile << "<" << matrixType <<" type_id=\"opencv-matrix\">" << endl;
	outXmlFile << "<?xml version=\"1.0\"?>" << endl;
	outXmlFile << "<rows>" << mat->rows << "</rows>" << endl;
	outXmlFile << "<cols>" << mat->cols << "</cols>" << endl;
	outXmlFile << "<dt>" << "f" << "</dt>" << endl;
	outXmlFile << "<data>" << endl;
	outXmlFile << elements << "</data></" << matrixType << ">" << endl;
	outXmlFile << "</opencv_storage>" << endl;
}

/*
Creates the extrinsic matrix from the rotation and tranlation matrices
*/
void Utils::createExtrinsicMatrix(CvMat* extrinsic_mat, const CvMat* rotation, const CvMat* translation)
{
		//Fill extrinsic matrix
	CV_MAT_ELEM( *extrinsic_mat, float, 0, 0 ) = CV_MAT_ELEM(*rotation, float, 0, 0);
	CV_MAT_ELEM( *extrinsic_mat, float, 0, 1 ) = CV_MAT_ELEM(*rotation, float, 0, 1);
	CV_MAT_ELEM( *extrinsic_mat, float, 0, 2 ) = CV_MAT_ELEM(*rotation, float, 0, 2);
	CV_MAT_ELEM( *extrinsic_mat, float, 0, 3 ) = CV_MAT_ELEM(*translation, float, 0, 0);
	CV_MAT_ELEM( *extrinsic_mat, float, 1, 0 ) = CV_MAT_ELEM(*rotation, float, 1, 0);
	CV_MAT_ELEM( *extrinsic_mat, float, 1, 1 ) = CV_MAT_ELEM(*rotation, float, 1, 1);
	CV_MAT_ELEM( *extrinsic_mat, float, 1, 2 ) = CV_MAT_ELEM(*rotation, float, 1, 2);
	CV_MAT_ELEM( *extrinsic_mat, float, 1, 3 ) = CV_MAT_ELEM(*translation, float, 0, 1);
	CV_MAT_ELEM( *extrinsic_mat, float, 2, 0 ) = CV_MAT_ELEM(*rotation, float, 2, 0);
	CV_MAT_ELEM( *extrinsic_mat, float, 2, 1 ) = CV_MAT_ELEM(*rotation, float, 2, 1);
	CV_MAT_ELEM( *extrinsic_mat, float, 2, 2 ) = CV_MAT_ELEM(*rotation, float, 2, 2);
	CV_MAT_ELEM( *extrinsic_mat, float, 2, 3 ) = CV_MAT_ELEM(*translation, float, 0, 2);
	CV_MAT_ELEM( *extrinsic_mat, float, 3, 0 ) = 0;
	CV_MAT_ELEM( *extrinsic_mat, float, 3, 1 ) = 0;
	CV_MAT_ELEM( *extrinsic_mat, float, 3, 2 ) = 0;
	CV_MAT_ELEM( *extrinsic_mat, float, 3, 3 ) = 1;
}

void Utils::convertToHomogeneus(XnPoint3D p, CvMat* h)
{
	CV_MAT_ELEM( *h, float, 0, 0 ) = (float) p.X;
	CV_MAT_ELEM( *h, float, 1, 0 ) = (float) p.Y;
	CV_MAT_ELEM( *h, float, 2, 0 ) = (float) p.Z;
	CV_MAT_ELEM( *h, float, 3, 0 ) = (float) 1.0;
}

void Utils::fillTheMatrix(CvMat* mat, const XnPoint3D* p, int rows, int cols)
{
	vector<float> point(3);
	point[0] = p->X;
	point[1] = p->Y;
	point[2] = p->Z;

	for (int r = 0; r < rows; r++)
	{
		float* ptr = (float*)(mat->data.fl + (r*mat->step/sizeof(float)));
		for (int c = 0; c < cols; c++)
		{
			*ptr++ = point[r+c];
		}
	}
}


void Utils::fillTheMatrix(CvMat* mat, XnPoint3D* p)
{
	CV_MAT_ELEM( *mat, float, 0, 0 ) = (float) p->X;
	CV_MAT_ELEM( *mat, float, 1, 0 ) = (float) p->Y;
	CV_MAT_ELEM( *mat, float, 2, 0 ) = (float) p->Z;
}

void Utils::turnToMatrix(CvMat* distortedP, XnPoint3D* proj, int total)
{
	for (int i = 0; i < total; i++)
	{
		distortedP->data.fl[i*2] = proj[i].X;
		distortedP->data.fl[i*2+1] = proj[i].Y;
	}
}

void Utils::normalizeMatrixPoints(CvMat* points, CvMat* pointsNorm, XnPoint3D* ref, int numPoints, ostream* out)
{
	for (int i = 0; i < numPoints; i++)
	{
		CV_MAT_ELEM( *pointsNorm, float, 0, i) = CV_MAT_ELEM( *points, float, 0, i) - ref->X;
		CV_MAT_ELEM( *pointsNorm, float, 1, i) = CV_MAT_ELEM( *points, float, 1, i) - ref->Y;
		CV_MAT_ELEM( *pointsNorm, float, 2, i) = CV_MAT_ELEM( *points, float, 2, i) - ref->Z;
	}
}

void Utils::findMeanPoint(CvMat* points, XnPoint3D* ref, int numPoints)
{
	int x = 0;
	int y = 0;
	int z = 0;
	for (int i = 0; i < numPoints; i++)
	{
		x += CV_MAT_ELEM( *points, float, 0, i);
		y += CV_MAT_ELEM( *points, float, 1, i);
		z += CV_MAT_ELEM( *points, float, 2, i);		
	}
	ref->X = (float)x/numPoints;
	ref->Y =(float) y/numPoints;
	ref->Z = (float)z/numPoints;
}

void Utils::createProjectionMat(CvMat* projMat, CameraProperties* cam)
{
	CvMat* extrinsic = cvCreateMat(3,4, CV_32FC1);
	
	CV_MAT_ELEM( *extrinsic, float, 0, 0) = CV_MAT_ELEM( *cam->getRotationMatrix(), float, 0, 0);
	CV_MAT_ELEM( *extrinsic, float, 0, 1) = CV_MAT_ELEM( *cam->getRotationMatrix(), float, 0, 1);
	CV_MAT_ELEM( *extrinsic, float, 0, 2) = CV_MAT_ELEM( *cam->getRotationMatrix(), float, 0, 2);
	CV_MAT_ELEM( *extrinsic, float, 0, 3) = CV_MAT_ELEM( *cam->getTranslationMatrix(), float, 0, 0);
	CV_MAT_ELEM( *extrinsic, float, 1, 0) = CV_MAT_ELEM( *cam->getRotationMatrix(), float, 1, 0);
	CV_MAT_ELEM( *extrinsic, float, 1, 1) = CV_MAT_ELEM( *cam->getRotationMatrix(), float, 1, 1);
	CV_MAT_ELEM( *extrinsic, float, 1, 2) = CV_MAT_ELEM( *cam->getRotationMatrix(), float, 1, 2);
	CV_MAT_ELEM( *extrinsic, float, 1, 3) = CV_MAT_ELEM( *cam->getTranslationMatrix(), float, 0, 1);
	CV_MAT_ELEM( *extrinsic, float, 2, 0) = CV_MAT_ELEM( *cam->getRotationMatrix(), float, 2, 0);
	CV_MAT_ELEM( *extrinsic, float, 2, 1) = CV_MAT_ELEM( *cam->getRotationMatrix(), float, 2, 1);
	CV_MAT_ELEM( *extrinsic, float, 2, 2) = CV_MAT_ELEM( *cam->getRotationMatrix(), float, 2, 2);
	CV_MAT_ELEM( *extrinsic, float, 2, 3) = CV_MAT_ELEM( *cam->getTranslationMatrix(), float, 0, 2);

	cvMatMul(cam->getIntrinsicMatrix(), extrinsic, projMat);
}


void Utils::changeSign(const CvMat* src, CvMat* dst)
{
	for (int i = 0; i < src->rows; i++)
	{
		float* ptrSrc = (float*)(src->data.fl + i*src->step/sizeof(float));
		float* ptrDst = (float*)(dst->data.fl + i*dst->step/sizeof(float));
		for (int j = 0; j < src->cols; j++)
		{
			*ptrDst = -*ptrSrc;
			ptrSrc++;
			ptrDst++;
		}
	}
}

int Utils::getRandomNumber(int max, int min)
{
	int random_integer; 
//	srand((unsigned)time(0)); 
	random_integer = (rand()%max)+min;

	return random_integer;
}


void Utils::generateRandomColor(int* rgbColor)
{
	rgbColor[0] = (int)rand() % 255 + 1;
	rgbColor[1] = (int)rand() % 255 + 1;
	rgbColor[2] = (int)rand() % 255 + 1;
}

/*
Creates the outStreams for writing in a file the coordinates of the rois in 'filePaths::ROI_PIXELS_FILEPATH'
*/
void Utils::createCoordinateOutStream(ofstream* xPlaneCoor, ofstream* yPlaneCoor, ofstream* zPlaneCoor, const int planeNum, const int camId)
{
	//coordinate path names
	char xplaneCoordiantesStr[100]; 
	char yplaneCoordiantesStr[100];
	char zplaneCoordiantesStr[100];

	initOutStreamPath(xplaneCoordiantesStr, yplaneCoordiantesStr, zplaneCoordiantesStr);
	//create the full path
	createFullPath(planeNum, camId, xplaneCoordiantesStr);
	createFullPath(planeNum, camId, yplaneCoordiantesStr);
	createFullPath(planeNum, camId, zplaneCoordiantesStr);

	//initialize variables with the path
	//Coordinates
	xPlaneCoor->open(xplaneCoordiantesStr);
	yPlaneCoor->open(yplaneCoordiantesStr);
	zPlaneCoor->open(zplaneCoordiantesStr);
}

/*
Initialize the values of the out streams
Type can be: 0 (coordinats of the points of the ROI), 1(parameters of the plane) or 2 (normal of the plane)
This will be used in Matlab for representing the results
*/
void Utils::initOutStreamPath(char* xPlane, char* yPlane, char* zPlane)
{
	char* planeCoor = "Coordinates_";

	strcpy (xPlane, filePaths::ROI_PIXELS_FILEPATH);
	strcpy (yPlane, filePaths::ROI_PIXELS_FILEPATH);
	strcpy (zPlane, filePaths::ROI_PIXELS_FILEPATH);

	strcat(xPlane, "xPlane");
	strcat(yPlane, "yPlane");
	strcat(zPlane, "zPlane");
	
	strcat(xPlane, planeCoor);
	strcat(yPlane, planeCoor);
	strcat(zPlane, planeCoor);
	

}

//Create the full path of the outStream for X Y and Z coordinates
void Utils::createFullPath(const int pNum, const int cId, char* plane)
{
	char strNumPlane[4];
	itoa(pNum, strNumPlane, 10);
	char strCamId[4];
	itoa(cId, strCamId, 10);
	strcat (plane,strCamId);
	strcat (plane,strNumPlane);
	strcat (plane,".txt");
}


/*
Creates an OutStream for the fileName provided in 'filePaths::ROI_PIXELS_FILEPATH'
*/
void Utils::createGeneralOutStream(ofstream* planeOutStream, const char* fileName, const int planeNum, const int camId)
{
	//path names
	char planeNameStr[100]; 
	strcpy (planeNameStr, filePaths::ROI_PIXELS_FILEPATH);
	strcat(planeNameStr, fileName);

	createFullPath(planeNum, camId, planeNameStr);

	planeOutStream->open(planeNameStr);

}

void Utils::writeListXnPointValues(list<XnPoint3D>* lst, ostream* out)
{
	if (lst->size() == 0)
		*out << "The list is empty" << endl;

	list<XnPoint3D>::iterator it;

	for(it = lst->begin(); it != lst->end(); it++)
	{
		*out << "(" << (*it).X << ", " << (*it).Y << ", " << (*it).Z << ")" << endl;
	}
}

int Utils::sizeArray(float *lst)
{
	int cont = 0;
	while (lst[cont] != NULL)
	{
		cont++;
	}

	return cont;
}

void Utils::showsArrayElements(float* lst, ostream* out)
{
	int cont = 0;
	while (lst[cont] != NULL)
	{
		*out << lst[cont++] << ", ";
	}
	*out << endl;
}

/*
Back project 'p' into the space using the intrinsic parameters provided by 'cam'
*/
void Utils::backProjectPoint(XnPoint3D* p2D, XnPoint3D* p3D, CameraProperties* cam)
{
	p3D->X = (p2D->X - cam->getOx())*cam->getPixelSize()*2*p2D->Z/cam->getFocalLenghtX();
	p3D->Y = (p2D->Y - cam->getOy())*cam->getPixelSize()*2*p2D->Z/cam->getFocalLenghtY();
	p3D->Z = p2D->Z;
}

void Utils::createPlanePath(vector<char*>* filePaths, int camId, char* suffix, int nPlanes)
{
	char* fileName = new char[100];
	char* strNplane = new char[50];
	for (int i = 0; i < nPlanes; i++)
	{
		itoa(i, strNplane, 10);
		if (camId == 1)
			strcpy(fileName, filePaths::CAM1_PLANES);
		else
			strcpy(fileName, filePaths::CAM2_PLANES);

		strcat(fileName, strNplane);
		strcat(fileName, suffix);
		strcpy((*filePaths)[i],fileName);
	}
	delete(fileName);
	delete(strNplane);
}

void Utils::createPlanePath_II(vector<char*>* filePaths, int camId, char* suffix, int nPlanes, vector<int>* planePositions)
{
	char strNplane[10];
	char strCamId[10];
	itoa(camId, strCamId, 10);
	for (int i = 0; i < nPlanes; i++)
	{
		int pos = (*planePositions)[i];
		itoa(pos, strNplane, 10);
		strcat((*filePaths)[i], strCamId);
		strcat((*filePaths)[i], strNplane);
		strcat((*filePaths)[i], suffix);;
	}
}

float Utils::calculatePlaneDistance(const CvMat* param, const CvMat* normal)
{

	vector<float> p(3);
	vector<float> n(3);
	for (int i = 0; i < 3; i++)
	{
		p[i] = *(float*)(param->data.fl + (i*param->step/sizeof(float)));
		n[i] = *(float*)(normal->data.fl + (i*normal->step/sizeof(float)));
	}
	
	return -p[2]/(p[0]*n[0] + p[1]*n[1] - n[2]);
}

void Utils::backProjectArrayOfPoints(XnPoint3D* out, XnPoint3D* in, const CameraProperties& cam, int numPoints)
{
	for (int i = 0; i < numPoints; i++)
	{
		cam.backProjectPoint(&in[i], &out[i]);
	}
}

void Utils::backProjectPointsRGBD(int nPoints, CameraProperties* cam, XnPoint3D* proj, XnPoint3D* backProj)
{		
	CvMat* rotation = cvCreateMat(3,3,CV_32FC1);
	cvTranspose(cam->getRotationMatrix(), rotation);
	
	CvMat* translationPos = cvCreateMat(3,1, CV_32FC1);
	cvMatMul(rotation, cam->getTranslationMatrix(), translationPos);
	CvMat* translation = cvCreateMat(3,1,CV_32FC1);
	Utils::changeSign(translationPos, translation);

	for (int i = 0; i < nPoints; i++)
	{	
		XnPoint3D cameraPoint;
		cameraPoint.X = (proj[i].X - cam->getOx())* cam->getPixelSize() *2*proj[i].Z/cam->getFocalLenghtX();
		cameraPoint.Y = (proj[i].Y - cam->getOy())* cam->getPixelSize() *2*proj[i].Z/cam->getFocalLenghtY();
		cameraPoint.Z = proj[i].Z;

		cam->getDepthNode()->ConvertProjectiveToRealWorld(1, &proj[i], &cameraPoint);
		cameraPoint.Y = -cameraPoint.Y;
		CvMat* camMat = cvCreateMat(3,1, CV_32FC1);
		Utils::fillTheMatrix(camMat,&cameraPoint);
		CvMat* tmp = cvCreateMat(3, 1, CV_32FC1);
		cvMatMul(rotation, camMat, tmp);
		CvMat* out = cvCreateMat(3,1, CV_32FC1);
		cvAdd(tmp, translation, out);

		backProj[i].X = CV_MAT_ELEM(*out, float, 0, 0);
		backProj[i].Y = CV_MAT_ELEM(*out, float, 1, 0);
		backProj[i].Z = CV_MAT_ELEM(*out, float, 2, 0);
	}
}

void Utils::projectPointsRGBD(int nPoints, CameraProperties* cam, XnPoint3D* rworld, XnPoint3D* proj)
{
	//extra parameters
	double pSize_rgb, pSize_depth;
	//cam->getImageNode()->GetRealProperty ("ZPPS", pSize_rgb);
	cam->getDepthNode()->GetRealProperty("ZPPS", pSize_rgb);
				
	float FxPix = CV_MAT_ELEM( *cam->getRGBIntrinsicMatrix(), float, 0, 0);
	float FyPix = CV_MAT_ELEM( *cam->getRGBIntrinsicMatrix(), float, 1, 1);	
	float FxMM = FxPix*pSize_rgb*2;
	float FyMM = FyPix*pSize_rgb*2;
	float Ox = CV_MAT_ELEM( *cam->getIntrinsicMatrix(), float, 0, 2);
	float Oy = CV_MAT_ELEM( *cam->getIntrinsicMatrix(), float, 1, 2);

	for (int i = 0; i < nPoints; i++)
	{
		proj[i].X = (int)((FxMM *rworld[i].X)/(pSize_rgb*2*rworld[i].Z) + Ox);
		proj[i].Y = (int)((FyMM*rworld[i].Y)/(pSize_rgb*2*rworld[i].Z) + Oy);	
	}

}

void Utils::initImage3Channel(IplImage* img, int n)
{
	for (int i = 0; i < img->height; i++)
	{
		unsigned char* ptr = (unsigned char*)(img->imageData + i*img->widthStep);
		for (int j = 0; j < img->width; j++)
		{
			*ptr++ = n;
			*ptr++ = n;
			*ptr++ = n;
		}
	}

}

void Utils::initImage(IplImage* img , int n)
{
	for (int i = 0; i < img->height; i++)
	{
		unsigned char* ptr = (unsigned char*)(img->imageData + i*img->widthStep);
		for (int j = 0; j < img->width; j++)
		{
			*ptr = n;
			ptr++;
		}
	}
}






void Utils::raw2depth(unsigned short* depth, int maxDepth)
{
	int i;
	for ( i=0; i<maxDepth; i++) {
		float v = (float)i/maxDepth;//for visualization purposes only
		v = powf(v, 2);
		v = v*36*256;
		depth[i] = v;
	}
}

void Utils::depth2rgb(const XnDepthPixel* Xn_disparity, unsigned short* depth, char *depth_data){
	int i;

	for (i=0; i<307200; i++) {
		int pval = depth[Xn_disparity[i]];
		int lb = pval & 0xff;
		switch (pval>>8) {
		case 0:
			depth_data[3*i+0] = 255;
			depth_data[3*i+1] = 255-lb;
			depth_data[3*i+2] = 255-lb;
			break;
		case 1:
			depth_data[3*i+0] = 255;
			depth_data[3*i+1] = lb;
			depth_data[3*i+2] = 0;
			break;
		case 2:
			depth_data[3*i+0] = 255-lb;
			depth_data[3*i+1] = 255;
			depth_data[3*i+2] = 0;
			break;
		case 3:
			depth_data[3*i+0] = 0;
			depth_data[3*i+1] = 255;
			depth_data[3*i+2] = lb;
			break;
		case 4:
			depth_data[3*i+0] = 0;
			depth_data[3*i+1] = 255-lb;
			depth_data[3*i+2] = 255;
			break;
		case 5:
			depth_data[3*i+0] = 0;
			depth_data[3*i+1] = 0;
			depth_data[3*i+2] = 255-lb;
			break;
		default:
			depth_data[3*i+0] = 0;
			depth_data[3*i+1] = 0;
			depth_data[3*i+2] = 0;
			break;
		}
	}
}


void Utils::combinedImages(IplImage* dst, const IplImage* img1, const IplImage* img2)
{
	//fill the img1
	for (int r = 0; r < img1->height; r++)
	{
		const uchar* img1Ptr = (uchar*)img1->imageData + (r*img1->widthStep);
		uchar* dstPtr = (uchar*)dst->imageData + (r*dst->widthStep);
		for (int c = 0; c < img1->width; c++)
		{
			dstPtr[c*3] = img1Ptr[c*3];
			dstPtr[c*3+1] = img1Ptr[c*3+1];
			dstPtr[c*3+2] = img1Ptr[c*3+2];
		}
	}

	//fill the img2
	for (int r = 0; r < img2->height; r++)
	{
		const uchar* img2Ptr = (uchar*)img2->imageData + (r*img2->widthStep);
		uchar* dstPtr = (uchar*)dst->imageData + (r*dst->widthStep);
		for (int c = 0; c < img2->width; c++)
		{
			dstPtr[XN_VGA_X_RES*3+c*3] = img2Ptr[c*3];
			dstPtr[XN_VGA_X_RES*3+c*3+1] = img2Ptr[c*3+1];
			dstPtr[XN_VGA_X_RES*3+c*3+2] = img2Ptr[c*3+2];
		}
	}
}

void Utils::color2rgb(const int* Xn_disparity, unsigned short* depth, char *depth_data){
	int i;

	for (i=0; i<307200; i++) {
		int pval = depth[Xn_disparity[i]];
		int lb = pval & 0xff;
		switch (pval>>8) {
		case 0:
			depth_data[3*i+0] = 255;
			depth_data[3*i+1] = 255-lb;
			depth_data[3*i+2] = 255-lb;
			break;
		case 1:
			depth_data[3*i+0] = 255;
			depth_data[3*i+1] = lb;
			depth_data[3*i+2] = 0;
			break;
		case 2:
			depth_data[3*i+0] = 255-lb;
			depth_data[3*i+1] = 255;
			depth_data[3*i+2] = 0;
			break;
		case 3:
			depth_data[3*i+0] = 0;
			depth_data[3*i+1] = 255;
			depth_data[3*i+2] = lb;
			break;
		case 4:
			depth_data[3*i+0] = 0;
			depth_data[3*i+1] = 255-lb;
			depth_data[3*i+2] = 255;
			break;
		case 5:
			depth_data[3*i+0] = 0;
			depth_data[3*i+1] = 0;
			depth_data[3*i+2] = 255-lb;
			break;
		default:
			depth_data[3*i+0] = 0;
			depth_data[3*i+1] = 0;
			depth_data[3*i+2] = 0;
			break;
		}
	}
}

int Utils::getNumberOfFiles(char* folderPath)
{
	int out = 0;
	boost::filesystem::directory_iterator end ;
	//depthMap cam1
	for( boost::filesystem::directory_iterator iter(folderPath) ; iter != end ; ++iter )
      if ( !is_directory( *iter ) )
		  out++;

	return out;
}

void Utils::fillTheMatrix(vector<XnPoint3D*>* src, CvMat* mat)
{
	int total = src->size();
	vector<XnPoint3D*>::iterator iter;
	int i = 0;

	//pointer to the begining of each row
	float* ptr_Mat_R1 = (float*) mat->data.fl;
	float* ptr_Mat_R2 = (float*) mat->data.fl + (mat->step/sizeof(float));
	float* ptr_Mat_R3 = (float*) mat->data.fl + (2*mat->step/sizeof(float));

	for (iter = src->begin(); iter != src->end(); iter++)
	{
		XnPoint3D* p = *iter;
		ptr_Mat_R1[i] = p->X;
		ptr_Mat_R2[i] = p->Y;
		ptr_Mat_R3[i] = p->Z;
		i++;
	}
}

void Utils::transformPoint(XnPoint3D* p3D, const CameraProperties& cam)
{
	CvMat* pointMat = cvCreateMat(3,1,CV_32FC1);
	fillTheMatrix(pointMat, p3D, 3,1);

	CvMat* tmp = cvCreateMat(3,1, CV_32FC1);
	cvMatMul(cam.getRotationMatrix(), pointMat, tmp);


//	CvMat* transT = cvCreateMat(3,1, CV_32FC1);
//	cvTranspose(cam.getTranslationMatrix(), transT);
	CvMat* outMat = cvCreateMat(3,1,CV_32FC1);
	cvAdd(tmp, cam.getTranslationMatrix(), outMat);

	p3D->X = *(outMat->data.fl);
	p3D->Y = *(outMat->data.fl + outMat->step/(sizeof(float)));
	p3D->Z = *(outMat->data.fl + 2*outMat->step/(sizeof(float)));

	cvReleaseMat(&pointMat);
	cvReleaseMat(&tmp);
//	cvReleaseMat(&transT);
	cvReleaseMat(&outMat);
}