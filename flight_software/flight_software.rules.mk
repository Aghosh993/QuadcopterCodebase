C_SOURCE_FILES=$(realpath $(wildcard ../flight_software/*.c) $(wildcard ../flight_software/source/*/*.c))

FLIGHT_SW_OBJ_FILES=$(realpath $(wildcard ../flight_software/*.o) $(wildcard ../flight_software/source/*/*.o))
FLIGHT_SW_OTHER_FILES=$(realpath $(wildcard ../flight_software/*.d) $(wildcard ../flight_software/source/*/*.d))

C_INCLUDE_DIRS=$(sort $(dir $(realpath $(wildcard ../flight_software/*.h) $(wildcard ../flight_software/include/*.h) $(wildcard ../flight_software/include/*/*.h) $(wildcard ../flight_software/include/mavlink_headers/common/*.h))))

CFLAGS=$(foreach dir, $(C_INCLUDE_DIRS), -I$(dir))

FLIGHT_SW_OBJS=$(C_SOURCE_FILES:.c=.o)

OBJS=$(FLIGHT_SW_OBJS)
