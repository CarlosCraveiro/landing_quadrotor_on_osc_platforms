# Use Ubuntu 24.04 LTS as the base image
FROM ubuntu:24.04

# Install system dependencies
RUN apt-get update && apt-get install -y \
    wget \
    tar \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install Julia LTS (update the version as needed; currently using 1.8.5 as an example)
RUN wget https://julialang-s3.julialang.org/bin/linux/x64/1.11/julia-1.11.4-linux-x86_64.tar.gz && \
    tar zxvf julia-1.11.4-linux-x86_64.tar.gz -C opt/ && \
    rm julia-1.11.4-linux-x86_64.tar.gz && \
    ln -s /opt/julia-1.8.5/bin/julia /usr/local/bin/julia

# Precompile IJulia to set up Jupyter integration (this will install Julia’s IJulia package)
RUN julia -e 'using Pkg; Pkg.add("IJulia"); Pkg.precompile()'

# Alternatively, install Jupyter Notebook via pip (in case you prefer the system installation)
RUN pip3 install notebook

# Create a working directory for your notebooks
WORKDIR /notebooks

# Expose Jupyter Notebook port
EXPOSE 8888

# Start Jupyter Notebook (allow access from any IP and disable launching a browser inside the container)
CMD ["jupyter", "notebook", "--ip=0.0.0.0", "--no-browser", "--allow-root", "--notebook-dir=/notebooks"]

