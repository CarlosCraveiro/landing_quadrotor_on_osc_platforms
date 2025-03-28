# Use Ubuntu 24.04 LTS as the base image
FROM ubuntu:24.04

# Install system dependencies
RUN apt-get update && apt-get install -y \
    wget \
    tar \
    unzip \
    python3 \
    python3-venv \
    python3-pip \
    openmpi-bin \
    libopenmpi-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Julia LTS (update the version as needed; currently using 1.8.5 as an example)
RUN wget https://julialang-s3.julialang.org/bin/linux/x64/1.10/julia-1.10.9-linux-x86_64.tar.gz && \
    tar zxvf julia-1.10.9-linux-x86_64.tar.gz -C opt/ && \
    rm julia-1.10.9-linux-x86_64.tar.gz && \
    ln -s /opt/julia-1.10.9/bin/julia /usr/local/bin/julia

# Precompile IJulia to set up Jupyter integration (this will install Julia’s IJulia package)
RUN julia -e 'using Pkg; Pkg.add("IJulia"); Pkg.precompile()'

# Create a Python virtual environment and install Jupyter Notebook there
RUN python3 -m venv /venv && \
    /venv/bin/pip install --upgrade pip && \
    /venv/bin/pip install notebook

# Set 
ENV PATH="/root/.julia/bin:$PATH"

# Create a working directory for your notebooks
WORKDIR /notebooks

# Expose Jupyter Notebook port
EXPOSE 8888

CMD ["/venv/bin/python", "-m", "notebook", "--ip=0.0.0.0", "--no-browser", "--allow-root", "--notebook-dir=/notebooks"]
