cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(remedi)

find_package(PCL 1.6 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)

add_executable (remedi MACOSX_BUNDLE BackgroundSubtractor.h BackgroundSubtractor.cpp Cloudject.hpp CloudjectDetector.h CloudjectDetector.cpp CloudjectFPFHRecognizer.h CloudjectFPFHRecognizer.cpp CloudjectModel.hpp ColorFrame.h ColorFrame.cpp conversion.h conversion.cpp DepthFrame.h DepthFrame.cpp DetectionOutput.h DetectionOutput.cpp Frame.h Frame.cpp InteractiveRegisterer.h InteractiveRegisterer.cpp Monitorizer.h Monitorizer.cpp MonitorizerParams.hpp Reader.h Reader.cpp Remedi.h Remedi.cpp Sequence.h Sequence.cpp Table.hpp TableModeler.h TableModeler.cpp main.cpp)
target_link_libraries (remedi ${OpenCV_LIBS} ${PCL_LIBRARIES})
