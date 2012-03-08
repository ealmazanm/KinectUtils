#include "Plane.h"


Plane::Plane(void)
{
	initPlane();
	normal = cvCreateMat(3,1, CV_32FC1);
	parameters = cvCreateMat(3,1, CV_32FC1);
}


Plane::~Plane(void)
{
}

void Plane::initPlane()
{
	initPoint.X = -1; initPoint.Y = -1;	initPoint.Z = -1;
	endPoint.X = -1; endPoint.Y = -1; endPoint.Z = -1;
}

XnPoint3D Plane::getInitPoint()
{
	return initPoint;
}

XnPoint3D Plane::getEndPoint()
{
	return endPoint;
}	

void Plane::setInitPoint(XnPoint3D p)
{
	initPoint = p;
}
	
void Plane::setEndPoint(XnPoint3D p)
{
	endPoint = p;
}

const CvMat* Plane::getNormal()
{
	return normal;
}
	
const CvMat* Plane::getParameters()
{
	return parameters;
}

float Plane::getDistance()
{
	return dist;
}
	
void Plane::setNormal(CvMat* n)
{
	normal = n;
}
	
void Plane::setParameters(CvMat* p)
{
	parameters = p;
}
	
void Plane::setDistance(float d)
{
	dist = d;
}

bool Plane::isROISelected()
{
	return (endPoint.X != -1);
}

void Plane::setCentroid3D(XnPoint3D* centroid)
{
	centroid3D.X = centroid->X;
	centroid3D.Y = centroid->Y;
	centroid3D.Z = centroid->Z;
}
	
XnPoint3D* Plane::getCentroid3D()
{
	return &centroid3D;
}