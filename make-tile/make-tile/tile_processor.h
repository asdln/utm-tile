#pragma once

#include <string>
#include <list>
#include "utility.h"
#include "dataset.h"

class OGRSpatialReference;

class TileProcessor
{
public:
	
	TileProcessor();
	~TileProcessor();

public:

	static bool DynamicProject(const OGRSpatialReference* pDstSpatialReference, Dataset* pDataset
		, int nBandCount, int bandMap[], const Envelop& env, unsigned char* pData
		, unsigned char* pMaskBuffer, int width, int height);

	static bool ProcessPerPixel(Dataset* ptrDataset
		, const Envelop& ptrEnvelope
		, OGRSpatialReference* ptrVisSRef
		, int nWidth, int nHeight
		, int nBandCount, int bandMap[]
		, unsigned char* memDataOut
		, unsigned char* dataMask
		, RasterResamplingType m_resampType
		, bool bDynProjectToGeoCoord);

};
