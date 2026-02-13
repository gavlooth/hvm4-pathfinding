# Makefile for libhvm4_graph
# Builds x86_64 static (.a) and shared (.so) libraries

CC = gcc
CFLAGS = -O3 -Wall -Wextra -std=c11 -march=x86-64 -mtune=generic -pthread -D_GNU_SOURCE -fPIC
LDFLAGS = -lm -pthread

# Source files
LIB_SRC = libhvm4_graph.c
LIB_HEADER = libhvm4_graph.h
LIB_OBJ = libhvm4_graph.o
LIB_STATIC = libhvm4_graph.a
LIB_SHARED = libhvm4_graph.so

EXAMPLE_SRC = example.c
EXAMPLE_BIN = example

BENCH_SRC = benchmark.c
BENCH_BIN = benchmark

# Distribution
DIST_DIR = dist

# Targets
all: $(LIB_STATIC) $(LIB_SHARED) $(EXAMPLE_BIN) $(BENCH_BIN)

# Static library
$(LIB_STATIC): $(LIB_OBJ)
	ar rcs $@ $^
	@echo "Built static library: $@ ($(shell stat -c%s $@ 2>/dev/null || echo '?') bytes)"

# Shared library
$(LIB_SHARED): $(LIB_SRC) $(LIB_HEADER)
	$(CC) $(CFLAGS) -shared -Wl,-soname,$@.1 -o $@ $(LIB_SRC) $(LDFLAGS)
	@echo "Built shared library: $@ ($(shell stat -c%s $@ 2>/dev/null || echo '?') bytes)"

# Object file
$(LIB_OBJ): $(LIB_SRC) $(LIB_HEADER)
	$(CC) $(CFLAGS) -c -o $@ $(LIB_SRC)

# Example program (static linked)
$(EXAMPLE_BIN): $(EXAMPLE_SRC) $(LIB_STATIC)
	$(CC) $(CFLAGS) -o $@ $(EXAMPLE_SRC) -L. -l:$(LIB_STATIC) $(LDFLAGS)

# Benchmark program (static linked)
$(BENCH_BIN): $(BENCH_SRC) $(LIB_STATIC)
	$(CC) $(CFLAGS) -o $@ $(BENCH_SRC) -L. -l:$(LIB_STATIC) $(LDFLAGS)

# Create distribution package
dist: $(LIB_STATIC) $(LIB_SHARED)
	@mkdir -p $(DIST_DIR)/include $(DIST_DIR)/lib $(DIST_DIR)/examples
	cp $(LIB_HEADER) $(DIST_DIR)/include/
	cp $(LIB_STATIC) $(LIB_SHARED) $(DIST_DIR)/lib/
	cp example.c benchmark.c $(DIST_DIR)/examples/
	cp README.md USAGE.md $(DIST_DIR)/
	@echo "Distribution created in $(DIST_DIR)/"
	@ls -lah $(DIST_DIR)/lib/

# Clean
clean:
	rm -f $(LIB_OBJ) $(LIB_STATIC) $(LIB_SHARED) $(EXAMPLE_BIN) $(BENCH_BIN)
	rm -rf $(DIST_DIR)

# Run example
test: $(EXAMPLE_BIN)
	./$(EXAMPLE_BIN)

# Run benchmark
bench: $(BENCH_BIN)
	./$(BENCH_BIN)

# Install to system (requires root)
install: $(LIB_STATIC) $(LIB_SHARED)
	install -d /usr/local/include /usr/local/lib
	install -m 644 $(LIB_HEADER) /usr/local/include/
	install -m 644 $(LIB_STATIC) /usr/local/lib/
	install -m 755 $(LIB_SHARED) /usr/local/lib/
	ldconfig

.PHONY: all clean test bench dist install
