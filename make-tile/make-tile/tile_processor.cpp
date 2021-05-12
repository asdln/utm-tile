#include "tile_processor.h"
#include "utility.h"
#include "tiff_dataset.h"
#include "gdal_priv.h"
#include "coordinate_transformation.h"
#include "math.h"
#include <algorithm>

bool IsIntersect(const Envelop& ptrMapEnv, const Envelop& ptrLayerEnv, bool bTag = 1)
{
	return false;
}

TileProcessor::TileProcessor()
{
}

bool TileProcessor::ProcessPerPixel(Dataset* ptrDataset
	, const Envelop& ptrEnvelope
	, OGRSpatialReference* ptrVisSRef
	, int nWidth, int nHeight
	, int nBandCount, int bandMap[]
	, unsigned char* pOutData
	, unsigned char* pMaskData
	, RasterResamplingType m_resampType
	, bool bDynProjectToGeoCoord)
{
	Envelop ptrEnvDataset = ptrDataset->GetExtent();
	OGRSpatialReference* ptrDSSRef = ptrDataset->GetSpatialReference();

	double dScreenResolutionX = ptrEnvelope.GetWidth() / (double)nWidth;
	double dScreenResolutionY = ptrEnvelope.GetHeight() / (double)nHeight;

	int nRasterWid = ptrDataset->GetRasterXSize();
	int nRasterHei = ptrDataset->GetRasterYSize();

	DataType ePixelType = ptrDataset->GetDataType();
	CoordinateTransformation coordTrans(ptrVisSRef, ptrDSSRef);

	DataType eTpye = ptrDataset->GetDataType();
	int nTypeSize = GetDataTypeBytes(eTpye);
	int nDataPixelBytes = nBandCount * nTypeSize;

	bool bValid = false;

	double dRow = ptrEnvelope.GetYMax();// - dbHalfCellY;
	for (int nRow = 0; nRow < nHeight; nRow++)
	{
		double dCol = ptrEnvelope.GetXMin();
		for (int nCol = 0; nCol < nWidth; nCol++)
		{
			double dx = dCol;
			double dy = dRow;

			//�����������귶Χcontinue
			if (bDynProjectToGeoCoord)
			{
				if (dx < -180 || dx>180 || dy < -90 || dy>90)
				{
					pMaskData[nWidth * nRow + nCol] = 0;
					dCol += dScreenResolutionX;
					continue;
				}
			}

			unsigned char* buffer = pOutData + (nWidth * nRow + nCol) * nDataPixelBytes;
			bool bRes = coordTrans.Transform(1, &dx, &dy);
			if (!bRes || std::isnan(dx) || std::isnan(dy))
			{
				pMaskData[nWidth * nRow + nCol] = 0;
				//memset(buffer, 128, nDataPixelBytes);
			}
			else
			{
				double dx1, dy1;
				ptrDataset->World2Pixel(dx, dy, dx1, dy1);
				int nx1 = dx1;
				int ny1 = dy1;

				if (nx1 < 0 || nx1 >= nRasterWid || ny1 < 0 || ny1 >= nRasterHei)
				{
					pMaskData[nWidth * nRow + nCol] = 0;
				}
				else
				{
					bValid = true;

					if (ePixelType < DataType::DT_CInt16 && m_resampType == RasterResamplingType::BILINEAR_INTERPOLATION && nx1 + 1 < nRasterWid && ny1 + 1 < nRasterHei)
					{
						void* pSrc = nullptr;
						double u = dx1 - nx1;
						double v = dy1 - ny1;

						unsigned char* pLinearBuffer = new unsigned char[4 * nDataPixelBytes];
						ptrDataset->Read(nx1, ny1, 2, 2, pLinearBuffer, 2, 2, eTpye, nBandCount, bandMap);

						unsigned char* value1 = pLinearBuffer;
						unsigned char* value2 = pLinearBuffer + 2 * nDataPixelBytes;
						unsigned char* value3 = pLinearBuffer + nDataPixelBytes;
						unsigned char* value4 = pLinearBuffer + 3 * nDataPixelBytes;

						double p1 = (1.0 - u) * (1.0 - v);
						double p2 = (1.0 - u) * v;
						double p3 = u * (1.0 - v);
						double p4 = u * v;

						for (int bandIndex = 0; bandIndex < nBandCount; bandIndex++)
						{
							int offset = bandIndex * nTypeSize;

							switch (eTpye)
							{
							case DT_Byte:
							{
								unsigned char value = p1 * *(value1 + offset) + p2 * *(value2 + offset)
									+ p3 * *(value3 + offset) + p4 * *(value4 + offset) + 0.5;
								memcpy(buffer + offset, &value, nTypeSize);
							}
							break;

							case DT_UInt16:
							{
								unsigned short value = p1 * *((unsigned short*)(value1 + offset)) + p2 * *((unsigned short*)(value2 + offset))
									+ p3 * *((unsigned short*)(value3 + offset)) + p4 * *((unsigned short*)(value4 + offset)) + 0.5;
								memcpy(buffer + offset, &value, nTypeSize);
							}
							break;

							case DT_Int16:
							{
								short value = p1 * *((short*)(value1 + offset)) + p2 * *((short*)(value2 + offset))
									+ p3 * *((short*)(value3 + offset)) + p4 * *((short*)(value4 + offset)) + 0.5;
								memcpy(buffer + offset, &value, nTypeSize);
							}
							break;

							case DT_UInt32:
							{
								unsigned int value = p1 * *((unsigned int*)(value1 + offset)) + p2 * *((unsigned int*)(value2 + offset))
									+ p3 * *((unsigned int*)(value3 + offset)) + p4 * *((unsigned int*)(value4 + offset)) + 0.5;
								memcpy(buffer + offset, &value, nTypeSize);
							}
							break;

							case DT_Int32:
							{
								int value = p1 * *((int*)(value1 + offset)) + p2 * *((int*)(value2 + offset))
									+ p3 * *((int*)(value3 + offset)) + p4 * *((int*)(value4 + offset)) + 0.5;
								memcpy(buffer + offset, &value, nTypeSize);
							}
							break;

							case DT_Float32:
							{
								float value = p1 * *((float*)(value1 + offset)) + p2 * *((float*)(value2 + offset))
									+ p3 * *((float*)(value3 + offset)) + p4 * *((float*)(value4 + offset)) + 0.5;
								memcpy(buffer + offset, &value, nTypeSize);
							}
							break;

							case DT_Float64:
							{
								double value = p1 * *((double*)(value1 + offset)) + p2 * *((double*)(value2 + offset))
									+ p3 * *((double*)(value3 + offset)) + p4 * *((double*)(value4 + offset)) + 0.5;
								memcpy(buffer + offset, &value, nTypeSize);
							}
							break;

							default:
								break;
							}
						}

						delete[] pLinearBuffer;
						pLinearBuffer = nullptr;
					}
					else
					{
						ptrDataset->Read(nx1, ny1, 1, 1, buffer, 1, 1, eTpye, nBandCount, bandMap);
					}
				}
			}

			dCol += dScreenResolutionX;
		}

		dRow -= dScreenResolutionY;
	}

	return bValid;
}

bool TileProcessor::DynamicProject(const OGRSpatialReference* ptrVisSRef, Dataset* pDataset, int nBandCount, int bandMap[], const Envelop& envelope, unsigned char* pData, unsigned char* pMaskData, int nWidth, int nHeight)
{
	const Envelop& ptrEnvDataset = pDataset->GetExtent();
	OGRSpatialReference* ptrDSSRef = pDataset->GetSpatialReference();

	RasterResamplingType resampType = NEAREST_NEIGHBOR;

	bool bDynPrjTrans = false;

	// ����ת��
	if (ptrVisSRef && ptrDSSRef && ptrDSSRef->GetRoot() != nullptr)
	{
// 		if ((ptrDSSRef->GetAttrValue("DATUM") != nullptr && ptrVisSRef->GetAttrValue("DATUM")!= nullptr 
// 			&&  std::string(ptrVisSRef->GetAttrValue("DATUM")).compare(ptrDSSRef->GetAttrValue("DATUM")) == 0) 
// 			|| !ptrVisSRef->IsSame(ptrDSSRef))
// 		{
// 			bDynPrjTrans = true;
// 		}

		if(!ptrVisSRef->IsSame(ptrDSSRef))
			bDynPrjTrans = true;
	}

	bool bDynProjectToGeoCoord = false;
	if (bDynPrjTrans
		&& ptrVisSRef->IsGeographic()
		&& ptrDSSRef->IsProjected())
	{
		bDynProjectToGeoCoord = true;
	}

	Envelop envelopeClone = envelope;
	DataType ePixelType = pDataset->GetDataType();

	bool bTransRes = false;
	if (bDynPrjTrans)
	{
		CoordinateTransformation transformation2(ptrVisSRef, ptrDSSRef);
		bTransRes = transformation2.Transform(envelope, envelopeClone);

		if (bTransRes == false)
		{
			Envelop envelopeClone3 = ptrEnvDataset;
			CoordinateTransformation transformation3(ptrDSSRef, ptrVisSRef);
			bTransRes = transformation3.Transform(ptrEnvDataset, envelopeClone3);

			if (bTransRes && !envelopeClone3.Intersects(envelope))
			{
				return false;
			}
		}

 		double dx1, dy1, dx2, dy2;
 		envelopeClone.QueryCoords(dx1, dy1, dx2, dy2);
 
 		//��Լ������������⴦��
 		if (!bTransRes || std::isnan(dx1) || std::isnan(dy1) || std::isnan(dx2) || std::isnan(dy2))
 		{
 			return ProcessPerPixel(pDataset, envelope, ptrDSSRef, nWidth, nHeight, nBandCount, bandMap, pData, pMaskData, resampType, bDynProjectToGeoCoord);
 		}
	}

	double dScreenResolutionX = envelope.GetWidth() / (double)nWidth;
	double dScreenResolutionY = envelope.GetHeight() / (double)nHeight;

	int nRasterWid = pDataset->GetRasterXSize();
	int nRasterHei = pDataset->GetRasterYSize();

	envelopeClone.Normalize();
	Envelop ptrEnvInterSect;
	bool bInsecResult = ptrEnvDataset.Intersection(envelopeClone, ptrEnvInterSect);
	if (!bInsecResult)
	{
		//ȷ��һ���������ת���Ƿ�ɹ����������������ת��
		Envelop ptrEvnTemp = envelope;
		if (IsIntersect(ptrEnvDataset, ptrEvnTemp))
		{
			return ProcessPerPixel(pDataset, envelope, ptrDSSRef, nWidth, nHeight, nBandCount, bandMap, pData, pMaskData, resampType, bDynProjectToGeoCoord);
		}

		return false;
	}

	double dx1, dy1, dx2, dy2, dx3, dy3, dx4, dy4;
	pDataset->World2Pixel(ptrEnvInterSect.GetXMin(), ptrEnvInterSect.GetYMin(), dx1, dy1);
	pDataset->World2Pixel(ptrEnvInterSect.GetXMin(), ptrEnvInterSect.GetYMax(), dx2, dy2);
	pDataset->World2Pixel(ptrEnvInterSect.GetXMax(), ptrEnvInterSect.GetYMax(), dx3, dy3);
	pDataset->World2Pixel(ptrEnvInterSect.GetXMax(), ptrEnvInterSect.GetYMin(), dx4, dy4);

	double dLeft = std::min(std::min(dx1, dx2), std::min(dx3, dx4));
	double dRight = std::max(std::max(dx1, dx2), std::max(dx3, dx4));

	double dTop = std::min(std::min(dy1, dy2), std::min(dy3, dy4));
	double dBottom = std::max(std::max(dy1, dy2), std::max(dy3, dy4));

	if (dLeft < 0.0)
		dLeft = 0.0;

	if (dLeft > (double)nRasterWid)
		dLeft = (double)nRasterWid;

	if (dRight < 0.0)
		dRight = 0.0;

	if (dRight > (double)nRasterWid)
		dRight = (double)nRasterWid;

	if (dTop < 0.0)
		dTop = 0.0;

	if (dTop > (double)nRasterHei)
		dTop = (double)nRasterHei;

	if (dBottom < 0.0)
		dBottom = 0.0;

	if (dBottom > (double)nRasterHei)
		dBottom = (double)nRasterHei;

	//����դ��ֱ���
	//double dx11, dy11, dx21, dy21;
	double dxTmp[2];
	double dyTmp[2];

	pDataset->Pixel2World(nRasterWid * 0.3, nRasterHei * 0.4, dxTmp[0], dyTmp[0]);
	pDataset->Pixel2World(nRasterWid * 0.7, nRasterHei * 0.4, dxTmp[1], dyTmp[1]);

	if (bDynPrjTrans)
	{
		CoordinateTransformation coordTrans(ptrDSSRef, ptrVisSRef);
		bool bRes = coordTrans.Transform(2, dxTmp, dyTmp, nullptr);
		if (bRes == false)
		{
			return false;
		}
	}

	double dImgResX = sqrt((dxTmp[0] - dxTmp[1]) * (dxTmp[0] - dxTmp[1]) + (dyTmp[0] - dyTmp[1]) * (dyTmp[0] - dyTmp[1])) / (nRasterWid * 0.4);

	//���Һ��ʵĽ���������
	int nPyramidLevel = 0;
	double dScale = 1.0;
	double dResTemp = dImgResX;
	int nMaxLevel = 512;

	if (dImgResX < dScreenResolutionX * 0.5)
	{
		do
		{
			/*if (dResTemp > dScreenResolutionX && dResTemp < dScreenResolutionX * 2.0)*/
			if (dResTemp >= dScreenResolutionX * 0.5 && dResTemp <= dScreenResolutionX)
			{
				break;
			}

			if (nMaxLevel <= 0)
			{
				break;
			}

			dResTemp = dResTemp * 2.0;
			nPyramidLevel++;
			dScale *= 0.5;
			nMaxLevel--;

		} while (1);
	}

	int nSrcWid = std::ceil(dRight) - std::floor(dLeft)/* + 1*/;
	int nSrcHei = std::ceil(dBottom) - std::floor(dTop)/* + 1*/;

	double dDesWid = nSrcWid * dScale/* + 0.5*/;
	double dDesHei = nSrcHei * dScale/* + 0.5*/;

	int nDesWid = std::ceil(dDesWid);
	int nDesHei = std::ceil(dDesHei);

	if (ePixelType < DT_CInt16 && resampType == BILINEAR_INTERPOLATION)
	{
		//˫�����ز��������һ������
		nDesWid++;
		nDesHei++;
	}

	nSrcWid = nDesWid / dScale;
	nSrcHei = nDesHei / dScale;

	int nLeft = dLeft;
	int nTop = dTop;

	//�Ŵ󵽸�����ʱ�����о��Ȳ���
// 		double dOffset = 1.0 / dScale;
// 		int nOff = dOffset;
// 		int nTemp = nLeft / nOff;
// 		nLeft = nTemp * nOff;
// 		nTemp = nTop / nOff;
// 		nTop = nTemp * nOff;

		//������Ȳ������λ�ã����������ȡ�ķ�Χ����1�����ؽ��е�����
		//�п��ܳ��ֲ�һ�����ز������������ʱ��1
	while (nTop + nSrcHei < std::ceil(dBottom))
	{
		nDesHei += 1;
		nSrcHei = nDesHei / dScale;
	}

	while (nLeft + nSrcWid < std::ceil(dRight))
	{
		nDesWid += 1;
		nSrcWid = nDesWid / dScale;
	}

	if (nSrcWid + nLeft > nRasterWid)
	{
		nSrcWid = nRasterWid - nLeft;
		nDesWid = nSrcWid * dScale + 0.5;
	}

	if (nSrcHei + nTop > nRasterHei)
	{
		nSrcHei = nRasterHei - nTop;
		nDesHei = nSrcHei * dScale + 0.5;
	}

	if (nSrcWid <= 0 || nSrcHei <= 0)
	{
		return false;
	}

	if (nDesWid <= 0)
	{
		nDesWid = 1;
	}

	if (nDesHei <= 0)
	{
		nDesHei = 1;
	}

	//SysDataSource::PixelBufferPtr ptrPixelBuffer;

	void* pSrcData = new char[GetDataTypeBytes(ePixelType) * nDesWid * nDesHei * nBandCount];

	//QString pszItem = pDataset->GetMetadataItem("INTERLEAVE", "IMAGE_STRUCTURE");
	//if (pszItem == "BAND" || pszItem == "LINE") // �����bsq����bil��ʽ���򰴲��ηֱ��ȡ
	//{
	//	SysDataSource::PixelDataType eType = pDataset->GetRasterBand(0)->GetRasterDataType();
	//	ptrPixelBuffer.Attach(new SysDataSource::PixelBuffer(nDesWid, nDesHei, vecBands, eType));
	//	char* pData = (char*)(ptrPixelBuffer->GetData());
	//	int nBandCount = vecBands.size();

	//	int nTypeSize = SysDataSource::DataSourceUtil::GetTypeSize(eType);
	//	int nPixelSpace = nTypeSize * nBandCount;
	//	int nLineSpace = nPixelSpace * nDesWid;

	//	for (int i = 0; i < nBandCount; i++)
	//	{
	//		SysDataSource::RasterBandPtr ptrRasterBand = pDataset->GetRasterBand(vecBands[i] - 1);
	//		ptrRasterBand->Read(nLeft, nTop, nSrcWid, nSrcHei, pData + i * nTypeSize, nDesWid, nDesHei, eType, nPixelSpace, nLineSpace);

	//		if (ptrTrack && ptrTrack->IsCanced())
	//		{
	//			return false;
	//		}
	//	}
	//}
	//else
	{
		pDataset->Read(nLeft, nTop, nSrcWid, nSrcHei, pSrcData, nDesWid, nDesHei, ePixelType, nBandCount, bandMap);
	}


	double dx, dy, dbSrcX, dbSrcY;

	double dImagePosX = 0.0;
	double dImagePosY = 0.0;

	int nTypeSize = GetDataTypeBytes(ePixelType);
	int nDataPixelBytes = nBandCount * nTypeSize;

	//�Ż���ʾ�ٶȡ�����Ƕ�̬ͶӰ����ͳһTransform��Ȼ���ٽ�����������
	double* pdx = nullptr;
	double* pdy = nullptr;

	double dbHalfCellX = dScreenResolutionX * 0.5;
	double dbHalfCellY = dScreenResolutionY * 0.5;

	if (bDynPrjTrans)
	{
		CoordinateTransformation coordTrans(ptrVisSRef, ptrDSSRef);

		pdx = new double[nWidth * nHeight];
		pdy = new double[nWidth * nHeight];

		dy = envelope.GetYMax() - dbHalfCellY;
		double dXminTemp = envelope.GetXMin();

		for (int RowIndex = 0; RowIndex < nHeight; ++RowIndex)
		{
			dx = dXminTemp + dbHalfCellX;
			for (int ColIndex = 0; ColIndex < nWidth; ++ColIndex)
			{
				pdx[RowIndex * nWidth + ColIndex] = dx;
				pdy[RowIndex * nWidth + ColIndex] = dy;

				dx += dScreenResolutionX;
			}
			dy -= dScreenResolutionY;
		}

		if (!coordTrans.Transform(nWidth * nHeight, pdx, pdy, 0))
		{
			delete[] pdx;
			delete[] pdy;

			delete[] (char*)pSrcData;
			return false;
		}
	}

	double ddx1, ddy1, ddx2, ddy2;
	ptrEnvDataset.QueryCoords(ddx1, ddy1, ddx2, ddy2);

	double dScaleX = (nDesWid / (double)nSrcWid);
	double dScaleY = (nDesHei / (double)nSrcHei);

	dy = envelope.GetYMax() - dbHalfCellY;
	for (int RowIndex = 0; RowIndex < nHeight; ++RowIndex)
	{
		dx = envelope.GetXMin() + dbHalfCellX;
		for (int ColIndex = 0; ColIndex < nWidth; ++ColIndex)
		{
			if (bDynPrjTrans)
			{
				dbSrcX = pdx[RowIndex * nWidth + ColIndex];
				dbSrcY = pdy[RowIndex * nWidth + ColIndex];

				if (std::isnan(dbSrcX) || std::isnan(dbSrcY))
				{
					dx += dScreenResolutionX;
					pMaskData[nWidth * RowIndex + ColIndex] = 0;
					continue;
				}
			}
			else
			{
				dbSrcX = dx;
				dbSrcY = dy;
			}
			//�����������귶Χcontinue
			if (bDynProjectToGeoCoord)
			{
				if (dx < -180 || dx>180 || dy < -90 || dy>90)
				{
					dx += dScreenResolutionX;
					pMaskData[nWidth * RowIndex + ColIndex] = 0;
					continue;
				}
			}

			//��Ӱ�����صĲ�������Ϊȫ���ڷ�Χ֮���������
			//if (dbSrcX < ddx1 || dbSrcX > ddx2 || dbSrcY < ddy1 || dbSrcY > ddy2)
			if (dbSrcX + dbHalfCellX < ddx1 || dbSrcX - dbHalfCellX > ddx2 || dbSrcY + dbHalfCellY < ddy1 || dbSrcY - dbHalfCellY > ddy2)
			{
				pMaskData[nWidth * RowIndex + ColIndex] = 0;
			}
			else
			{
				pDataset->World2Pixel(dbSrcX, dbSrcY, dImagePosX, dImagePosY);
				//����
				double dColPos = (dImagePosX - nLeft) * dScaleX;
				double dRowPos = (dImagePosY - nTop) * dScaleY;

				long nColPos = dColPos;
				long nRowPos = dRowPos;

				if (dImagePosX < 0.0 || dImagePosX > nRasterWid || dImagePosY < 0.0 || dImagePosY > nRasterHei)
				{
					pMaskData[nWidth * RowIndex + ColIndex] = 0;
				}
				//���ڱ��αȽϴ��ͶӰ��������ͶӰ����ĳЩgf4���ݣ��������ӷ�Χ���ܸ����������ݵ�����������⴦��
				else if (nColPos < 0 || nColPos >= nDesWid || nRowPos < 0 || nRowPos >= nDesHei)
				{
					int nImagePosX = dImagePosX;
					int nImagePosY = dImagePosY;

					unsigned char* buffer = new unsigned char[nDataPixelBytes];
					pDataset->Read(nImagePosX, nImagePosY, 1, 1, buffer, 1, 1, ePixelType, nBandCount, bandMap);

					unsigned char* pDes = pData + (nWidth * RowIndex + ColIndex) * nDataPixelBytes;
					memcpy(pDes, buffer, nDataPixelBytes);
					delete[] buffer;
				}
				else
				{
					void* pSrc = nullptr;
					unsigned char* pDes = pData + (nWidth * RowIndex + ColIndex) * nDataPixelBytes;

					if (ePixelType < DT_CInt16 && resampType == BILINEAR_INTERPOLATION && nColPos + 1 < nDesWid && nRowPos + 1 < nDesHei)
					{
						double u = dColPos - nColPos;
						double v = dRowPos - nRowPos;

						unsigned char* value1 = (unsigned char*)pSrcData + (nDesWid * nRowPos + nColPos) * nDataPixelBytes;
						unsigned char* value2 = (unsigned char*)pSrcData + (nDesWid * (nRowPos + 1) + nColPos) * nDataPixelBytes;
						unsigned char* value3 = (unsigned char*)pSrcData + (nDesWid * nRowPos + nColPos + 1) * nDataPixelBytes;
						unsigned char* value4 = (unsigned char*)pSrcData + (nDesWid * (nRowPos + 1) + nColPos + 1) * nDataPixelBytes;

						double p1 = (1.0 - u) * (1.0 - v);
						double p2 = (1.0 - u) * v;
						double p3 = u * (1.0 - v);
						double p4 = u * v;

						for (int bandIndex = 0; bandIndex < nBandCount; bandIndex++)
						{
							int offset = bandIndex * nTypeSize;
							switch (ePixelType)
							{
							case DT_Byte:
							{
								unsigned char value = p1 * *(value1 + offset) + p2 * v * *(value2 + offset)
									+ p3 * *(value3 + offset) + p4 * *(value4 + offset) + 0.5;
								memcpy(pDes + offset, &value, nTypeSize);
							}
							break;

							case DT_UInt16:
							{
								unsigned short value = p1 * *((unsigned short*)(value1 + offset)) + p2 * *((unsigned short*)(value2 + offset))
									+ p3 * *((unsigned short*)(value3 + offset)) + p4 * *((unsigned short*)(value4 + offset)) + 0.5;
								memcpy(pDes + offset, &value, nTypeSize);
							}
							break;

							case DT_Int16:
							{
								short value = p1 * *((short*)(value1 + offset)) + p2 * *((short*)(value2 + offset))
									+ p3 * *((short*)(value3 + offset)) + p4 * *((short*)(value4 + offset)) + 0.5;
								memcpy(pDes + offset, &value, nTypeSize);
							}
							break;

							case DT_UInt32:
							{
								unsigned int value = p1 * *((unsigned int*)(value1 + offset)) + p2 * *((unsigned int*)(value2 + offset))
									+ p3 * *((unsigned int*)(value3 + offset)) + p4 * *((unsigned int*)(value4 + offset)) + 0.5;
								memcpy(pDes + offset, &value, nTypeSize);
							}
							break;

							case DT_Int32:
							{
								int value = p1 * *((int*)(value1 + offset)) + p2 * *((int*)(value2 + offset))
									+ p3 * *((int*)(value3 + offset)) + p4 * *((int*)(value4 + offset)) + 0.5;
								memcpy(pDes + offset, &value, nTypeSize);
							}
							break;

							case DT_Float32:
							{
								float value = p1 * *((float*)(value1 + offset)) + p2 * *((float*)(value2 + offset))
									+ p3 * *((float*)(value3 + offset)) + p4 * *((float*)(value4 + offset)) + 0.5;
								memcpy(pDes + offset, &value, nTypeSize);
							}
							break;

							case DT_Float64:
							{
								double value = p1 * *((double*)(value1 + offset)) + p2 * *((double*)(value2 + offset))
									+ p3 * *((double*)(value3 + offset)) + p4 * *((double*)(value4 + offset)) + 0.5;
								memcpy(pDes + offset, &value, nTypeSize);
							}
							break;

							default:
								break;
							}
						}
					}
					else
					{
						int nIndex = nDesWid * nRowPos + nColPos;
						int nMaxIndex = nDesWid * nDesHei;
						if (nIndex >= nMaxIndex)
						{
							nIndex = nMaxIndex - 1;
						}

						//���ڶ�ȡ��ԭʼ���ݽ�������������ݣ��˴��������ز��������Ż��ռ䣿
						pSrc = (unsigned char*)pSrcData + nIndex * nDataPixelBytes;
						memcpy(pDes, pSrc, nDataPixelBytes);
					}
				}
			}
			dx += dScreenResolutionX;
		}
		dy -= dScreenResolutionY;
	}

	if (bDynPrjTrans)
	{
		delete[] pdx;
		delete[] pdy;
	}

	delete[] (char*)pSrcData;
	return true;
}

TileProcessor::~TileProcessor()
{
}
