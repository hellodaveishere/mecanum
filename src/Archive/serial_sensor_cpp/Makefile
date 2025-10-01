.PHONY: init build clean

init:
    mkdir -p ../../src
    cd ../../src && ln -s $(PWD) serial_sensor_cpp

build:
    cd ../.. && colcon build --packages-select serial_sensor_cpp

clean:
    cd ../.. && rm -rf build install log
