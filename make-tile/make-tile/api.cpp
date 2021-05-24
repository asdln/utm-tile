#include "api.h"
#include "dataset_factory.h"
#include "gdal_priv.h"
#include "ogrsf_frmts.h"
#include "coordinate_transformation.h"
#include "tile_processor.h"

#include<filesystem>

void save_file(const std::string& file_name, double* adfGeoTransform, const OGRSpatialReference* oSRS, int band_count, int* band_map, int size_x, int size_y, DataType type, unsigned char* buffer, double nodata_value)
{
	const char* pszFormat = "GTiff";
	GDALDriver* poDriver;
	char** papszMetadata;
	poDriver = GetGDALDriverManager()->GetDriverByName(pszFormat);
	if (poDriver == NULL)
		exit(1);
	papszMetadata = poDriver->GetMetadata();
	if (CSLFetchBoolean(papszMetadata, GDAL_DCAP_CREATE, FALSE))
		printf("Driver %s supports Create() method.\n", pszFormat);
	if (CSLFetchBoolean(papszMetadata, GDAL_DCAP_CREATECOPY, FALSE))
		printf("Driver %s supports CreateCopy() method.\n", pszFormat);

	GDALDataset* poDstDS;
	char** papszOptions = NULL;
	poDstDS = poDriver->Create(file_name.c_str(), size_x, size_y, band_count, (GDALDataType)type, papszOptions);

	if (poDstDS == nullptr)
		return;

	for (int i = 0; i < band_count; i++)
	{
		poDstDS->GetRasterBand(i + 1)->SetNoDataValue(nodata_value);
	}

	//double adfGeoTransform[6] = { 444720, 30, 0, 3751320, 0, -30 };
	char* pszSRS_WKT = NULL;
	GDALRasterBand* poBand;

	poDstDS->SetGeoTransform(adfGeoTransform);
	oSRS->exportToWkt(&pszSRS_WKT);
	poDstDS->SetProjection(pszSRS_WKT);
	CPLFree(pszSRS_WKT);

	int pixel_space = GetDataTypeBytes(type) * band_count;
	int line_space = pixel_space * size_x;
	int band_space = GetDataTypeBytes(type);

	poDstDS->RasterIO(GF_Write, 0, 0, size_x, size_y, buffer, size_x, size_y
		, (GDALDataType)type, band_count, band_map, pixel_space, line_space, band_space);

	GDALClose(poDstDS);
}

void calc_lat_strides(double lat_min, double lat_max, std::vector<std::string>& strides)
{
	std::vector<std::string> strides_all = {"C", "D", "E", "F", "G", "H", "J", "K", "L", "M", "N", "P", "Q", "R", "S", "T", "U", "V", "W", "X"};

	int lat_min_n = int(std::floor(lat_min / 8.0) + 0.5) + 10;
	int lat_max_n = int(std::floor(lat_max / 8.0) + 0.5) + 10;

	strides.reserve(lat_max_n - lat_min_n + 1);
	for (int i = lat_min_n; i <= lat_max_n; i++)
	{
		strides.push_back(strides_all[i]);
	}
}

void process_block(const OGRSpatialReference* srs_dst, Dataset* dataset, double resolution, int tile_width
	, int tile_height, int band_count, int* band_map, DataType type, const Envelop& env, const std::string& file_path_name)
{
	int pixel_bytes = GetDataTypeBytes(type);
	const size_t nSize = (size_t)tile_width * tile_height * band_count * pixel_bytes;

	unsigned char* buff = new unsigned char[nSize];
	memset(buff, 0, nSize);

	unsigned char* mask_buffer = new unsigned char[(size_t)tile_width * tile_height];
	memset(mask_buffer, 255, (size_t)tile_width * tile_height);

	if (TileProcessor::DynamicProject(srs_dst, dataset, band_count, band_map, env, buff, mask_buffer, tile_width, tile_height))
	{
		//设0为无效值
		// 	for (int i = 0; i < (size_t)tile_width * tile_height; i++)
		// 	{
		// 		if (mask_buffer[i] == 0)
		// 		{
		// 			unsigned char* temp = buff + i * band_count * pixel_bytes;
		// 			memset(temp, 0, band_count * pixel_bytes);
		// 		}
		// 	}

		double adfGeoTransform[6]/* = { 444720, 30, 0, 3751320, 0, -30 }*/;
		adfGeoTransform[0] = env.GetXMin();
		adfGeoTransform[1] = resolution;
		adfGeoTransform[2] = 0;
		adfGeoTransform[3] = env.GetYMax();
		adfGeoTransform[4] = 0;
		adfGeoTransform[5] = -resolution;

		save_file(file_path_name, adfGeoTransform, srs_dst, band_count, band_map, tile_width, tile_height, type, buff, 0.0);
	}

	delete[] buff;
	delete[] mask_buffer;
}

void test_code()
{
	GDALAllRegister();
	auto p = DatasetFactory::OpenDataset("D:/test/GF1_WFV2_E111.1_N34.3_20200503_L1A0004778986/GF1_WFV2_E111.1_N34.3_20200503_L1A0004778986.tiff");

	double dx, dy;
	p->Pixel2World(0, 0, dx, dy);

	Envelop dataset_env_geographic = p->GetExtent();
	int band_count_all = p->GetBandCount();

	auto srs_dst0 = std::make_shared<OGRSpatialReference>();
	srs_dst0->importFromEPSG(32649);
	
	OGRSpatialReference* srs_src = p->GetSpatialReference();
	CoordinateTransformation transformation0(srs_src, srs_dst0.get());

	transformation0.Transform(1, &dx, &dy);

	int x = 0;
}

bool make_tile(const std::string& shp_dir, const std::string& path, const std::string& dst_dir)
{
	//test_code();

	if (!std::filesystem::exists(dst_dir))
	{
		std::filesystem::create_directories(dst_dir);
	}

	std::string file_name = std::filesystem::path(path).filename().u8string();

	//去掉后缀名
	int pos = file_name.rfind(".");
	if (pos != std::string::npos)
	{
		file_name = file_name.substr(0, pos);
	}
	else
	{
		pos = file_name.rfind(".");
		if (pos != std::string::npos)
		{
			file_name = file_name.substr(0, pos);
		}
	}

	GDALAllRegister();
	auto p = DatasetFactory::OpenDataset(path);
	Envelop dataset_env_geographic = p->GetExtent();
	int band_count_all = p->GetBandCount();

	OGRSpatialReference* srs_src = p->GetSpatialReference();
	//转换为地理坐标，计算utm相应分带的epsg
	if (!srs_src->IsGeographic())
	{
		//wgs84
		auto srs_wgs84 = std::make_shared<OGRSpatialReference>();
		srs_wgs84->importFromEPSG(4326);

		Envelop env_src = p->GetExtent();
		Envelop env_dst;

		CoordinateTransformation transformation0(srs_src, srs_wgs84.get());
		transformation0.Transform(env_src, env_dst);

		dataset_env_geographic = env_dst;
	}

	double minx, miny, maxx, maxy;
	dataset_env_geographic.QueryCoords(minx, miny, maxx, maxy);

	double Latitude = (miny + maxy) * 0.5;
	double Longitude = (minx + maxx) * 0.5;

	//根据中心的经纬度计算对应的utm投影的epsg
	int EPSG0 = 32700 - (int)((45.0 + Latitude) / 90.0 + 0.5) * 100 + (int)((183.0 + Longitude) / 6.0 + 0.5);
	//int EPSG0 = 32600 + (int)((183.0 + Longitude) / 6.0 + 0.5);

	auto srs_dst0 = std::make_shared<OGRSpatialReference>();
	srs_dst0->importFromEPSG(EPSG0);

	//计算原始影像的分辨率
	// 1) 选取采样点
	int sample_y = p->GetRasterYSize() * 0.5;
	int sample_x = p->GetRasterXSize() * 0.5;

	// x 方向加1 采样
	int sample2_y = sample_y;
	int sample2_x = sample_x + 1;

	//像素坐标转换到原始影像空间参考坐标
	double dx1, dy1, dx2, dy2;
	p->Pixel2World(sample_x, sample_y, dx1, dy1);
	p->Pixel2World(sample2_x, sample2_y, dx2, dy2);

	//原始影像空间参考坐标转换到目标空间参考坐标
	CoordinateTransformation transformation0(srs_src, srs_dst0.get());
	transformation0.Transform(1, &dx1, &dy1);
	transformation0.Transform(1, &dx2, &dy2);
	
	// 直接算出距离，就是分辨率了
	double resolution = std::sqrt((dy2 - dy1) * (dy2 - dy1) + (dx2 - dx1) * (dx2 - dx1));

	bool use_100km = false;
	//计算幅宽，根据幅宽来计算分幅大小
	double prj_width = p->GetRasterXSize() * resolution;
	//如果幅宽大于300km，则按照100公里分幅。否则按照20公里分幅
	if (prj_width > 300000.0)
	{
		use_100km = true;
	}
	else if (resolution > 12.0)
	{
		use_100km = true;
	}

	//计算跨了几个utm分带
	int min_zone = ((183.0 + minx) / 6.0 + 0.5);
	int max_zone = ((183.0 + maxx) / 6.0 + 0.5);

	std::vector<std::string> strides;
	calc_lat_strides(miny, maxy, strides);

	//生成shp路径
	std::vector<std::string> shp_paths;
	for (int i = min_zone; i <= max_zone; i++)
	{
		for (auto stride : strides)
		{
			std::string zone_string = std::to_string(i);
			if (zone_string.size() < 2) zone_string = "0" + zone_string;

			shp_paths.emplace_back("MGRS_100kmSQ_ID_" + zone_string + stride + ".shp");
		}
	}

// 	//暂时只按照北半球计算投影。相应的偏移值
// 	double False_Easting = 500000.0;
// 	double False_Northing = 0.0;

	double km100 = 100000.0;
	double km20 = 20000.0;

	std::vector<int> band_map;
	for (int i = 0; i < band_count_all; i++)
	{
		band_map.push_back(i + 1);
	}

	int pixel_bytes = GetDataTypeBytes(p->GetDataType());

	for (auto& shp_path : shp_paths)
	{
		std::string shp_path_full = shp_dir + "/" + shp_path;
		GDALDataset* poDS;
		poDS = (GDALDataset*)GDALOpenEx(shp_path_full.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL);

		OGRLayer* poLayer;
		poLayer = poDS->GetLayer(0)/*poDS->GetLayerByName("point")*/;
		auto srs_dst = poLayer->GetSpatialRef();

		CoordinateTransformation transformation(srs_src, srs_dst);

		Envelop env_dst;
		const Envelop& dataset_env = p->GetExtent();
		transformation.Transform(dataset_env, env_dst);

		OGRFeatureDefn* poFDefn = poLayer->GetLayerDefn();

		poLayer->ResetReading();
		OGRFeature* poFeature;
		while ((poFeature = poLayer->GetNextFeature()) != NULL)
		{
			const char* Easting =  poFeature->GetFieldAsString("EASTING");
			const char* Northing = poFeature->GetFieldAsString("NORTHING");

			double False_Easting = std::atof(Easting);
			double False_Northing = std::atof(Northing);

			Envelop env_mgrs;
			env_mgrs.PutCoords(False_Easting, False_Northing, False_Easting + km100, False_Northing + km100);

			if (env_dst.Intersects(env_mgrs))
			{
				std::string mgrs = poFeature->GetFieldAsString("MGRS");

				if (!use_100km)
				{
					int tile_width_20km = km20 / resolution;
					int tile_height_20km = km20 / resolution;

					double real_resolution = km20 / tile_width_20km;

					for (int j = 0; j < 5; j ++)
					{
						for (int i = 0; i < 5; i++)
						{
							Envelop env_mgrs_20km;
							env_mgrs_20km.PutCoords(False_Easting + i * km20, False_Northing + j * km20, False_Easting + (i + 1) * km20, False_Northing + (j + 1) * km20);
							std::string file_path_name = dst_dir + "/" + file_name + "_" + mgrs + std::to_string(i) + std::to_string(j) + ".tif";
							process_block(srs_dst, p.get(), real_resolution, tile_width_20km, tile_height_20km
								, band_count_all, band_map.data(), p->GetDataType(), env_mgrs_20km, file_path_name);
						}
					}
				}
				else
				{
					int tile_width_100km = km100 / resolution;
					int tile_height_100km = km100 / resolution;

					double real_resolution = km100 / tile_width_100km;

					std::string file_path_name = dst_dir + "/" + file_name + "_" + mgrs + ".tif";
					process_block(srs_dst, p.get(), real_resolution, tile_width_100km, tile_height_100km
						, band_count_all, band_map.data(), p->GetDataType(), env_mgrs, file_path_name);
				}
			}

			OGRFeature::DestroyFeature(poFeature);
		}

		GDALClose(poDS);

	}

	return true;
}