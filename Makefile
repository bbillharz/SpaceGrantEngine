.PHONY: all check clean test fmt help

all:
	bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

clean:
	rm -rf ./install
	rm -rf ./build
	rm -rf ./log

check: fmt test

test:
	bash -c "source /opt/ros/humble/setup.bash && source ./install/setup.bash && colcon test --event-handlers console_direct+"

fmt:
	bash -c "source /opt/ros/humble/setup.bash && source ./install/setup.bash && python3 -m black ./sgengine/sgengine && python3 -m isort ./sgengine/sgengine && python3 -m ruff --fix ./sgengine/sgengine"
	@echo "DONE - CI PASSED"

help:
	@echo "all - Builds all ros packages (Default)"
	@echo "clean - Clears the workspace (build, install, and log dirs)"
	@echo "check - Runs test & fmt"
	@echo "test - Runs the ROS2 test suite on packages"
	@echo "fmt - Runs formatting corrections & tests, outputs problems to fix"
