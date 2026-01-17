/// @file macro.hpp
/// @brief InfIO 模块内部宏定义

#pragma once

#ifdef INFMVS_USE_OPENMP
  #include <omp.h>
  #define INFMVS_OMP_PARALLEL_FOR _Pragma("omp parallel for schedule(static)")
  #define INFMVS_OMP_PARALLEL _Pragma("omp parallel")
  #define INFMVS_OMP_MAX_THREADS() omp_get_max_threads()
  #define INFMVS_OMP_THREAD_NUM() omp_get_thread_num()
  #define INFMVS_OMP_SET_NUM_THREADS(n) omp_set_num_threads(n)
#else
  #define INFMVS_OMP_PARALLEL_FOR
  #define INFMVS_OMP_PARALLEL
  #define INFMVS_OMP_MAX_THREADS() 1
  #define INFMVS_OMP_THREAD_NUM() 0
  #define INFMVS_OMP_SET_NUM_THREADS(n) ((void)0)
#endif
