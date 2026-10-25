# Host-side firmware tests: `make test`. AVR build check: `make avr` (needs avr-gcc + Arduino core).
FW := ../j2_cannon
CXXFLAGS := -std=c++17 -O1 -g -Wall -Wextra -Wno-unused-parameter -Istubs -I$(FW)
SRCS := $(FW)/settings.cpp $(FW)/stepper.cpp $(FW)/pressure.cpp $(FW)/joystick.cpp $(FW)/lcd.cpp $(FW)/serial_io.cpp sim.cpp test_firmware.cpp

test_firmware: $(SRCS) $(FW)/j2_cannon.ino $(wildcard $(FW)/*.h) sim.h $(wildcard stubs/*.h stubs/*/*.h)
	$(CXX) $(CXXFLAGS) -o $@ $(SRCS) -x c++ $(FW)/j2_cannon.ino

fw_host: $(filter-out test_firmware.cpp,$(SRCS)) host_main.cpp $(FW)/j2_cannon.ino $(wildcard $(FW)/*.h) sim.h
	$(CXX) $(CXXFLAGS) -o $@ $(filter-out test_firmware.cpp,$(SRCS)) host_main.cpp -x c++ $(FW)/j2_cannon.ino

test: test_firmware
	./test_firmware

avr:
	./build_avr.sh

clean:
	rm -f test_firmware fw_host
	rm -rf build-avr

.PHONY: test avr clean
