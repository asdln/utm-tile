#pragma once

#include "utility.h"

class OGRSpatialReference;
class OGRCoordinateTransformation;

class CoordinateTransformation
{
public:

	CoordinateTransformation(const OGRSpatialReference* pSrc, const OGRSpatialReference* pDst);

	bool Transform(int count, double* pX, double* pY, double* pZ = nullptr);

	bool Transform(const Envelop& srcEnv, Envelop& dstEnv);

	~CoordinateTransformation();

private:

	OGRCoordinateTransformation* pOGRCoordinateTransformation_ = nullptr;

	bool need_tranverse_before_ = false;
	bool need_tranverse_after_ = false;
};