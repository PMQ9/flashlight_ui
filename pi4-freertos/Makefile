CC = gcc
CFLAGS = -pthread -Wall -O2
LDFLAGS = -pthread

EXECUTABLE = flashlight_relay
SOURCE = flashlight_relay.c

all: $(EXECUTABLE)

$(EXECUTABLE): $(SOURCE)
	$(CC) $(CFLAGS) -o $(EXECUTABLE) $(SOURCE) $(LDFLAGS)

clean:
	rm -f $(EXECUTABLE) *.o

run: $(EXECUTABLE)
	./$(EXECUTABLE)

.PHONY: all clean run
