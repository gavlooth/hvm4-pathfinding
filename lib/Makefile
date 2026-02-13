# Makefile for libhvm4_graph

CC = gcc
CFLAGS = -O3 -Wall -Wextra -std=c11 -march=native -pthread -D_GNU_SOURCE
LDFLAGS = -lm -pthread

# Source files
LIB_SRC = libhvm4_graph.c
LIB_HEADER = libhvm4_graph.h
LIB_OBJ = libhvm4_graph.o
LIB_NAME = libhvm4_graph.a
LIB_SO = libhvm4_graph.so

EXAMPLE_SRC = example.c
EXAMPLE_BIN = example

BENCH_SRC = benchmark.c
BENCH_BIN = benchmark

# Targets
all: $(LIB_NAME) $(LIB_SO) $(EXAMPLE_BIN) $(BENCH_BIN)

# Static library
$(LIB_NAME): $(LIB_OBJ)
	ar rcs $@ $^

# Shared library
$(LIB_SO): $(LIB_SRC) $(LIB_HEADER)
	$(CC) $(CFLAGS) -fPIC -shared -o $@ $(LIB_SRC) $(LDFLAGS)

# Object file
$(LIB_OBJ): $(LIB_SRC) $(LIB_HEADER)
	$(CC) $(CFLAGS) -c -o $@ $(LIB_SRC)

# Example program
$(EXAMPLE_BIN): $(EXAMPLE_SRC) $(LIB_NAME)
	$(CC) $(CFLAGS) -o $@ $(EXAMPLE_SRC) $(LIB_NAME) $(LDFLAGS)

# Benchmark program
$(BENCH_BIN): $(BENCH_SRC) $(LIB_NAME)
	$(CC) $(CFLAGS) -o $@ $(BENCH_SRC) $(LIB_NAME) $(LDFLAGS)

# Clean
clean:
	rm -f $(LIB_OBJ) $(LIB_NAME) $(LIB_SO) $(EXAMPLE_BIN) $(BENCH_BIN)

# Run example
test: $(EXAMPLE_BIN)
	./$(EXAMPLE_BIN)

# Run benchmark
bench: $(BENCH_BIN)
	./$(BENCH_BIN)

.PHONY: all clean test bench
