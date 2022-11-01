.PHONY: all clean install

all: clean install

clean:
	rm -rf build
	rm -rf install
	rm -rf log

install: 
	colcon build
