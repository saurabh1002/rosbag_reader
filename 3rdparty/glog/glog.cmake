set(BUILD_TESTING OFF)
set(WITH_CUSTOM_PREFIX OFF)
set(WITH_GFLAGS OFF)
set(WITH_GMOCK OFF)
set(WITH_GTEST OFF)
set(WITH_PKGCONFIG OFF)
set(WITH_SYMBOLIZE OFF)
set(WITH_TLS OFF)
set(WITH_UNWIND OFF)

include(FetchContent)
FetchContent_Declare(
  glog_ext
  GIT_REPOSITORY https://github.com/google/glog.git
  GIT_SHALLOW ON
  GIT_TAG v0.6.0)

FetchContent_MakeAvailable(glog_ext)
