#pragma once
//这个文件把很多需要的头文件集中import到这里
#include "taichi/taichi_platform.h"     //区分平台,有Linux,windows,和OSX平台

#include "taichi/taichi_core.h"   //还有太极的内核

#ifdef TI_WITH_VULKAN                       //如果使用了vulkan的backend,那么引用vulkan的头文件
#ifndef TI_NO_VULKAN_INCLUDES           
#include <vulkan/vulkan.h>                  
#endif  // TI_NO_VULKAN_INCLUDES

#include "taichi/taichi_vulkan.h"
#endif  // TI_WITH_VULKAN

#ifdef TI_WITH_OPENGL                         //同样的道理,引用OpenGL的头文件
#ifndef TI_NO_OPENGL_INCLUDES
#include <GL/gl.h>
#endif  // TI_NO_OPENGL_INCLUDES

#include "taichi/taichi_opengl.h"
#endif  // TI_WITH_OPENGL

#ifdef TI_WITH_CUDA                             //cuda的backend的一些东西
#ifndef TI_NO_CUDA_INCLUDES
// Only a few CUDA types is needed, including the entire <cuda.h> is overkill
// for this
typedef void *CUdeviceptr;
#endif  // TI_NO_CUDA_INCLUDES

#include "taichi/taichi_cuda.h"
#endif  // TI_WITH_CUDA

#ifdef TI_WITH_CPU                                //CPU的backend的一些东西
#include "taichi/taichi_cpu.h"
#endif  // TI_WITH_CPU
