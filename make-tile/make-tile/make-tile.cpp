// make-tile.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include "api.h"
#include "CJsonObject.hpp"

int main(int args, char** argv)
{
    std::string exe_path = argv[0];

    int pos = exe_path.rfind("/");
    if (pos != std::string::npos)
    {
        exe_path = exe_path.substr(0, pos);
    }
    else
    {
        pos = exe_path.rfind("\\");
		if (pos != std::string::npos)
		{
			exe_path = exe_path.substr(0, pos);
		}
    }

    std::string argparam;
    std::getline(std::cin, argparam);

    neb::CJsonObject oJson(argparam);

    std::string input_img_file;
    oJson.Get("input_img_file", input_img_file);

    std::string output_path;
	oJson.Get("output_path", output_path);
    
    //make_tile(exe_path + "/shp", "D:/test/GF1C_PMS_E112.2_N35.2_20190407_L1A1021393821/GF1C_PMS_E112.2_N35.2_20190407_L1A1021393821-MUX_approo.tif", "d:/test/make_tile_dir");
    //make_tile(exe_path + "/shp", "D:/test/GF1_WFV2_E111.1_N34.3_20200503_L1A0004778986/GF1_WFV2_E111.1_N34.3_20200503_L1A0004778986.tiff", "d:/test/make_tile_dir");
    make_tile(exe_path + "/shp", input_img_file, output_path);
}

// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

// 入门使用技巧: 
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件
