.PHONY: engine all check clean test ci help

all:
	bash -c "source ./venv/bin/activate && colcon build --symlink-install --packages-skip python-steamcontroller"

clean:
	sudo rm -rf ./install
	sudo rm -rf ./build
	sudo rm -rf ./log

check: ci test

test:
	bash -c "source ./venv/bin/activate && colcon test --event-handlers console_direct+ --packages-skip python-steamcontroller"

ci:
	bash -c "source ./venv/bin/activate && source ./install/setup.bash && python3 -m black ./sgengine/sgengine && python3 -m isort ./sgengine/sgengine && python3 -m ruff --fix ./sgengine/sgengine"
	@echo "DONE - CI PASSED"

help:
	@echo "all - Builds all ros packages (Default)"
	@echo "clean - Clears the workspace (build, install, and log dirs)"
	@echo "check - Runs test & ci"
	@echo "test_engine - Runs the ROS2 test suite on sgengine packages"
	@echo "ci - Runs the continous integration locally, outputs errors"
