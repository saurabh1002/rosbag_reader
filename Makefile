.PHONY: cpp

install:
	@pip install --verbose ./python/

uninstall:
	@pip -v uninstall rosbag_reader

editable:
	@pip install scikit-build-core pyproject_metadata pathspec pybind11 ninja cmake
	@pip install --no-build-isolation -ve ./python/

cpp:
	@cmake -Bbuild cpp/rosbag_reader/
	@cmake --build build -j$(nproc --all)
