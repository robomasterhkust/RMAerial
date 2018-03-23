# List of all the dji_sdk related files.
DJI_SRC = $(CHIBIOS)/dji_sdk/api/src/dji_ack.cpp \
					$(CHIBIOS)/dji_sdk/api/src/dji_broadcast.cpp \
					$(CHIBIOS)/dji_sdk/api/src/dji_command.cpp \
					$(CHIBIOS)/dji_sdk/api/src/dji_error.cpp \
					$(CHIBIOS)/dji_sdk/api/src/dji_hardware_sync.cpp \
					$(CHIBIOS)/dji_sdk/api/src/dji_mfio.cpp \
					$(CHIBIOS)/dji_sdk/api/src/dji_subscription.cpp \
					$(CHIBIOS)/dji_sdk/api/src/dji_vehicle.cpp \
					$(CHIBIOS)/dji_sdk/api/src/dji_version.cpp \
					$(CHIBIOS)/dji_sdk/api/src/dji_virtual_rc.cpp \
					$(CHIBIOS)/dji_sdk/hal/src/dji_hard_driver.cpp \
					$(CHIBIOS)/dji_sdk/hal/src/dji_memory.cpp \
					$(CHIBIOS)/dji_sdk/hal/src/dji_platform_manager.cpp \
					$(CHIBIOS)/dji_sdk/hal/src/dji_thread_manager.cpp \
					$(CHIBIOS)/dji_sdk/protocol/src/dji_aes.cpp \
					$(CHIBIOS)/dji_sdk/protocol/src/dji_open_protocol.cpp \
					$(CHIBIOS)/dji_sdk/protocol/src/dji_protocol_base.cpp \
					$(CHIBIOS)/dji_sdk/utility/src/dji_circular_buffer.cpp \
					$(CHIBIOS)/dji_sdk/utility/src/dji_singleton.cpp

# Required include directories
DJI_INC = $(CHIBIOS)/dji_sdk/api/inc \
					$(CHIBIOS)/dji_sdk/hal/inc \
					$(CHIBIOS)/dji_sdk/protocol/inc \
					$(CHIBIOS)/dji_sdk/utility/inc
