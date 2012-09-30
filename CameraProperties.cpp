#include "CameraProperties.h"

CameraProperties::CameraProperties(void)
{
	context = NULL;
	depthNode = NULL;
	imageNode = NULL;
	rotation = NULL;
	translation = NULL;
	intrinsic_matrix = NULL;
	rgbIntrinsic_matrix = NULL;
	distortion_coeffs = NULL;
}


CameraProperties::~CameraProperties(void)
{
}

Context* CameraProperties::getContext() const
{
	return (Context*)&context;
}

Recorder* CameraProperties::getRecorderNode()
{
	return &recordNode;
}

DepthGenerator* CameraProperties::getDepthNode() const
{
	return (DepthGenerator*)&depthNode;
}
	
ImageGenerator* CameraProperties::getImageNode() const
{
	return (ImageGenerator*)&imageNode;
}

CvMat* CameraProperties::getRotationMatrix() const
{
	return rotation;
}

CvMat* CameraProperties::getTranslationMatrix() const
{
	return translation;
}

CvMat* CameraProperties::getIntrinsicMatrix()
{
	return intrinsic_matrix;
}

CvMat* CameraProperties::getRGBIntrinsicMatrix()
{
	return rgbIntrinsic_matrix;
}

CvMat* CameraProperties::getDistortionCoeffs()
{
	return distortion_coeffs;
}


void CameraProperties::setIntrinsicMatrix(CvMat* mat)
{
	intrinsic_matrix = mat;
}

void CameraProperties::setRGBIntrinsicMatrix(CvMat* mat)
{
	rgbIntrinsic_matrix = mat;
}

void CameraProperties::setDistortionCoeffs(CvMat* mat)
{
	distortion_coeffs = mat;
}

void CameraProperties::setCamId(int id)
{
	camId = id;
}

int CameraProperties::getCamId() const
{
	return camId;
}

void CameraProperties::setRotationMatrix(CvMat* mat)
{
	rotation = mat;
}
	
void CameraProperties::setTranslationMatrix(CvMat* mat)
{
	translation = mat;
}

float CameraProperties::getFocalLenghtX()
{
	return fx;
}
float CameraProperties::getFocalLenghtY()
{
	return fy;
}
float CameraProperties::getOx()
{
	return ox;
}
	
float CameraProperties::getOy()
{
	return oy;
}
	
double CameraProperties::getPixelSize()
{
	return pSize;
}
	
void CameraProperties::setFocalLenghtX(float fx_t)
{
	fx = fx_t;
}
	
void CameraProperties::setFocalLenghtY(float fy_t)
{
	fy = fy_t;
}
	
void CameraProperties::setOx(float ox_t)
{
	ox = ox_t;
}
	
void CameraProperties::setOy(float oy_t)
{
	oy = oy_t;
}
	
void CameraProperties::setPixelSize(double ps)
{
	pSize = ps;
}

void CameraProperties::backProjectPoint(XnPoint3D* p, XnPoint3D* p3D) const
{
	p3D->X = (p->X - ox)*pSize*2*p->Z/fx;
	p3D->Y = (p->Y - oy)*pSize*2*p->Z/fy;
	p3D->Z = p->Z;
}

void CameraProperties::projectPoint(XnPoint3D* p3D, XnPoint3D* p2D) const
{
	p2D->X = (fx *p3D->X)/(pSize*2*p3D->Z) + ox;
	p2D->Y = (fy*p3D->Y)/(pSize*2*p3D->Z) + oy;	
}