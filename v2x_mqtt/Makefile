CC      = gcc
CFLAGS  = -Wall -Wextra -O2 -std=gnu11 \
          -Icommon -Icore -Ihandler -Imanager -Iservice -Imap -Ican \
          $(shell pkg-config --cflags libcjson libmosquitto 2>/dev/null)
LDLIBS  = $(shell pkg-config --libs libmosquitto 2>/dev/null || echo -lmosquitto) \
          -lpthread -lm \
          $(shell pkg-config --libs libcjson 2>/dev/null || echo -lcjson)

BIN_DIR = bin
TARGET  = $(BIN_DIR)/v2x_mqtt_temp
FAKE_VEHICLE_TARGET = $(BIN_DIR)/fake_vehicle_l15_to_l6

SRCS = \
    core/main.c \
    core/can_rx_thread.c \
    core/mqtt_thread.c \
    core/can_tx_thread.c \
    handler/can_handler.c \
    handler/mqtt_handler.c \
    service/map_service.c \
    manager/self_vehicle_manager.c \
    manager/other_vehicle_manager.c \
    manager/traffic_light_manager.c \
    common/vehicle_codec.c \
    map/IntersectionMap.c

.PHONY: all clean fake_vehicle_l15_to_l6

all: $(TARGET)

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

$(TARGET): $(SRCS) | $(BIN_DIR)
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

fake_vehicle_l15_to_l6: $(FAKE_VEHICLE_TARGET)

$(FAKE_VEHICLE_TARGET): tools/fake_vehicle_l15_to_l6.c common/vehicle_codec.c service/map_service.c manager/self_vehicle_manager.c map/IntersectionMap.c | $(BIN_DIR)
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

clean:
	rm -f $(TARGET) $(FAKE_VEHICLE_TARGET)
