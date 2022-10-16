include(FetchContent)
FetchContent_Declare(
  indicators
  PREFIX indicators
  GIT_REPOSITORY https://github.com/p-ranav/indicators.git
  GIT_SHALLOW ON
)
FetchContent_MakeAvailable(indicators)