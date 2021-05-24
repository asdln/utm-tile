cmake_minimum_required(VERSION 2.8)

project(make-tile)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

INCLUDE_DIRECTORIES(../install/include)
LINK_DIRECTORIES(../install/lib)

SET(SRC_LIST api.cpp
    cJSON.c
    CJsonObject.cpp
    coordinate_transformation.cpp
    dataset.cpp
    dataset_factory.cpp
    tiff_dataset.cpp
    tile_processor.cpp
    make-tile.cpp
    utility.cpp)

ADD_EXECUTABLE(${PROJECT_NAME} ${SRC_LIST})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} -lgdal)
