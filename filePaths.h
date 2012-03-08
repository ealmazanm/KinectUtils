/**
* Class to define local paths
*
*
* Author: Emilio J. Almazan <emilio.almazan@kingston.ac.uk>, 2012
*/
#pragma once

class filePaths 
{
public:
	static char* POINT3D_FILEPATH;
	static char* POINT3D_CAM_FILEPATH;
	static char* POINT3D_CAM1_FILEPATH;
	static char* POINT3D_CAM2_FILEPATH;
	static char* CAM1_INTRINSIC_FILEPATH;
	static char* CAM2_INTRINSIC_FILEPATH;
	static char* CAM1_DISTORTION_FILEPATH;
	static char* CAM2_DISTORTION_FILEPATH;
	static char* CAM1_ROTATION_FILEPATH;
	static char* CAM2_ROTATION_FILEPATH;
	static char* CAM1_TRANSLATION_FILEPATH;
	static char* CAM2_TRANSLATION_FILEPATH;
	static char* POINTCLOUD_FILEPATH;
	static char* DEBUG_FILEPATH;
	static char* CAM1_2_ROTATION_FILEPATH;
	static char* CAM1_2_TRANSLATION_FILEPATH;
	static char* CAM2_1_ROTATION_FILEPATH;
	static char* CAM2_1_TRANSLATION_FILEPATH;
	static char* ROI_PIXELS_FILEPATH;
	static char* CAM1_PLANES;
	static char* CAM2_PLANES;
	static char* CAM1_RGBINTRINSIC_FILEPATH;
	static char* CAM1_RGBD_ROTATION_FILEPATH;
	static char* CAM1_RGBD_TRANSLATION_FILEPATH;
	static char* TRAINIGN_IMAGES_FILEPATH;
	static char* PLANE_CALIBRATION_ERROR;
	//CalibrationData
	static char* CAM1_CALIB_DEPTHMAP;
	static char* CAM2_CALIB_DEPTHMAP;

	static char* CAM1_CALIB_RGBMAP;
	static char* CAM2_CALIB_RGBMAP;

	static char* CAM1_CALIB_NORMALS;
	static char* CAM2_CALIB_NORMALS;

	static char* CAM1_CALIB_PARAMETERS;
	static char* CAM2_CALIB_PARAMETERS;

	static char* CAM1_CALIB_SEED;
	static char* CAM2_CALIB_SEED;

	static char* CAM1_CALIB_CENTROIDS;
	static char* CAM2_CALIB_CENTROIDS;

	static char* CAM1_CALIBRATION_DATA;
	static char* CAM2_CALIBRATION_DATA;
};