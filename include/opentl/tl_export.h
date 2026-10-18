#pragma once

#include "opennurbs.h"

#if defined(_WIN32)
  #if defined(OPENTL_EXPORTS)
    #define TL_CLASS __declspec(dllexport)
    #define TL_FUNCTION __declspec(dllexport)
  #else
    #define TL_CLASS
    #define TL_FUNCTION
  #endif
  #define RH_C_FUNCTION extern "C" __declspec(dllexport)
#else
  #define TL_CLASS __attribute__((visibility("default")))
  #define TL_FUNCTION __attribute__((visibility("default")))
  #define RH_C_FUNCTION extern "C" __attribute__((visibility("default")))
#endif
