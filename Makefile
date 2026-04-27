SHELL := /bin/bash
.PHONY: all build  build-pkg clean

PACKAGE ?= demo_nodes_cpp
CORES := $(shell nproc)

all: build

build:
	colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_POSITION_INDEPENDENT_CODE=ON
	@echo "Syncing filesystems and dropping caches..."
	@sync
	@echo "Done"
build-pkg:
	colcon build --symlink-install --packages-select $(PACKAGE) --cmake-args "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" "-DCMAKE_POSITION_INDEPENDENT_CODE=ON"

clean:
	rm -r build/ install/ log/