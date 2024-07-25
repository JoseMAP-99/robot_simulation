SHELL := /bin/bash
MAKEFLAGS += --no-print-directory
.PHONY: help create-package clean build run

# OS
OS := $(shell uname)
ifeq ($(OS),Darwin)
	UID = $(shell id -u)
else ifeq ($(OS),Linux)
	UID = $(shell id -u)
else
	UID = 1000
endif


help: ## Show this help message
	@echo 'usage: make [target]'
	@echo
	@echo 'targets:'
	@egrep '^(.+)\:\ ##\ (.+)' ${MAKEFILE_LIST} | column -t -c 2 -s ':#'

create-package: ## Create a colcon package
ifeq ($(strip $(name)),)
	@echo "A package name is required. Use name=<package_name>" && exit 1
endif
	ros2 pkg create $(name) \
		--build-type ament_python \
		--license "Apache-2.0" \
		--description "Package $(name)" \
		--destination-directory "src/" \
		--maintainer-email "ja@xrf.ai" \
		--maintainer-name "xrf" 

clean: ## Clean ignored files
	rm -rf build install log

build: ## Build sources
	colcon build

update: ## Update sources
	source install/setup.bash

run: ## Execute simulation
ifeq ($(strip $(name)),)
	@echo "A package name is required. Use name=<package_name>" && exit 1
endif
	$(MAKE) build && \
	$(MAKE) update && \
	ros2 launch $(name) display.launch.py
