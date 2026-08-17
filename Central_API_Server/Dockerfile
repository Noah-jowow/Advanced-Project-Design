# Use Python 3.10 slim as base
FROM python:3.10-slim

# Install system dependencies for building C++ projects
RUN apt-get update && apt-get install -y \
    cmake \
    g++ \
    ninja-build \
    libopenblas-dev \
    liblapack-dev \
    sed \
    && rm -rf /var/lib/apt/lists/*

# Set workspace directory
WORKDIR /app

# Copy the entire workspace into the container (including System_Shared_Libraries)
COPY . .

# Install Python dependencies including build tools
RUN pip install --no-cache-dir \
    pybind11 \
    pytest \
    bandit \
    fastapi \
    uvicorn \
    pydantic \
    numpy \
    scipy

# Remove Windows-specific .pyd suffix forcing in CMakeLists.txt to allow Linux .so generation
RUN sed -i '/SUFFIX ".pyd"/d' Aerodynamics_CFD/CMakeLists.txt && \
    sed -i '/SUFFIX ".pyd"/d' Propulsion/CMakeLists.txt

# Compile Radar C++ Core
RUN cd Radar_Systems && \
    mkdir -p build && \
    cd build && \
    cmake .. -GNinja -DCMAKE_BUILD_TYPE=Release && \
    ninja

# Compile Aerodynamics C++ Core
RUN cd Aerodynamics_CFD && \
    mkdir -p build && \
    cd build && \
    cmake .. -GNinja -DCMAKE_BUILD_TYPE=Release && \
    ninja

# Compile Propulsion C++ Core
RUN cd Propulsion && \
    mkdir -p build && \
    cd build && \
    cmake .. -GNinja -DCMAKE_BUILD_TYPE=Release && \
    ninja

# Set environment variables
ENV PYTHONPATH=/app/Central_API_Server

# Expose the API port
EXPOSE 8000

# Run the FastAPI server
WORKDIR /app/Central_API_Server
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
