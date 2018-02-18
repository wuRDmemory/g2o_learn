//
// Created by ubuntu on 18-2-14.
//

#ifndef DEMO3_G2O_TUTORIAL_SLAM2D_API_H
#define DEMO3_G2O_TUTORIAL_SLAM2D_API_H

#include "g2o/config.h"

#ifdef _MSC_VER
// We are using a Microsoft compiler:
#ifdef G2O_SHARED_LIBS
#ifdef tutorial_slam2d_library_EXPORTS
#define G2O_TUTORIAL_SLAM2D_API __declspec(dllexport)
#else
#define G2O_TUTORIAL_SLAM2D_API __declspec(dllimport)
#endif
#else
#define G2O_TUTORIAL_SLAM2D_API
#endif

#else
// Not Microsoft compiler so set empty definition:
#define G2O_TUTORIAL_SLAM2D_API
#endif

#endif //DEMO3_G2O_TUTORIAL_SLAM2D_API_H
