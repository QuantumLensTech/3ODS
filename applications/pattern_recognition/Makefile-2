# Makefile - Pattern Recognition Application
# Author: Jean-Christophe Ané
# Date: December 2025

# Compiler settings
CXX := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -O2 -I.
LDFLAGS := -lm

# Google Test settings
GTEST_FLAGS := -lgtest -lgtest_main -pthread

# Targets
.PHONY: all clean test benchmark help

all: test_pattern_recognition benchmark_pattern_recognition

# Test executable
test_pattern_recognition: test_pattern_recognition.cpp pattern_recognition_octovalent.hpp
	@echo "🔨 Compiling tests..."
	$(CXX) $(CXXFLAGS) test_pattern_recognition.cpp -o test_pattern_recognition $(GTEST_FLAGS) $(LDFLAGS)
	@echo "✅ Tests compiled successfully"

# Benchmark executable
benchmark_pattern_recognition: benchmark_pattern_recognition.cpp pattern_recognition_octovalent.hpp
	@echo "🔨 Compiling benchmarks..."
	$(CXX) $(CXXFLAGS) benchmark_pattern_recognition.cpp -o benchmark_pattern_recognition $(LDFLAGS)
	@echo "✅ Benchmarks compiled successfully"

# Run tests
test: test_pattern_recognition
	@echo ""
	@echo "🧪 Running tests..."
	@echo ""
	./test_pattern_recognition

# Run benchmarks
benchmark: benchmark_pattern_recognition
	@echo ""
	@echo "📊 Running benchmarks..."
	@echo ""
	./benchmark_pattern_recognition

# Clean build artifacts
clean:
	@echo "🧹 Cleaning..."
	rm -f test_pattern_recognition benchmark_pattern_recognition
	rm -f *.o
	@echo "✅ Clean complete"

# Help
help:
	@echo ""
	@echo "Pattern Recognition - Makefile"
	@echo "=============================="
	@echo ""
	@echo "Available targets:"
	@echo "  make              - Build all (tests + benchmarks)"
	@echo "  make test         - Build and run tests"
	@echo "  make benchmark    - Build and run benchmarks"
	@echo "  make clean        - Remove build artifacts"
	@echo "  make help         - Show this help"
	@echo ""
	@echo "Examples:"
	@echo "  make && make test && make benchmark"
	@echo ""
