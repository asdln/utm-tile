#include <iostream>
#include "dataset_factory.h"
#include "tiff_dataset.h"

std::shared_ptr<Dataset> DatasetFactory::OpenDataset(const std::string& path)
{
	auto p = std::make_shared<TiffDataset>();
	//auto p = std::make_shared<S3Dataset>();
	if (p->Open(path))
		return p;
	else
	{
		return nullptr;
	}
}