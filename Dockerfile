# Base: Ubuntu 24.04 LTS
FROM ubuntu:24.04

LABEL org.opencontainers.image.title="quadrotor-landing-mpc"
LABEL org.opencontainers.image.description="Research environment for my undergraduate scientific initiation project: simulation and control of quadrotor landing on oscillating platforms using Model Predictive Control (MPC)."
LABEL org.opencontainers.image.authors="Carlos Craveiro <carlos.craveiro@usp.br>"
LABEL org.opencontainers.image.licenses="GPL-3.0-or-later"

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    JULIA_NUM_THREADS=auto \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    wget tar unzip curl ca-certificates git \
    python3 python3-venv python3-pip \
    openmpi-bin libopenmpi-dev \
    tini \
 && rm -rf /var/lib/apt/lists/*

# Install Julia 1.10.9 manually
RUN wget https://julialang-s3.julialang.org/bin/linux/x64/1.10/julia-1.10.9-linux-x86_64.tar.gz \
 && tar zxvf julia-1.10.9-linux-x86_64.tar.gz -C /opt/ \
 && rm julia-1.10.9-linux-x86_64.tar.gz \
 && ln -s /opt/julia-1.10.9/bin/julia /usr/local/bin/julia

# Pre-compile IJulia
RUN julia -e 'using Pkg; Pkg.add("IJulia"); Pkg.precompile()'

# Python virtual environment isolated with Jupyter Notebook
RUN python3 -m venv /venv \
 && /venv/bin/pip install --upgrade pip \
 && /venv/bin/pip install notebook==7.2.2 ipywidgets==8.1.5


# Default work directory
WORKDIR /root/work

# Expose ports (Jupyter + 3D Visualizer inside project)
EXPOSE 8888 8700-8799

# Healthcheck (opcional)
#HEALTHCHECK --interval=30s --timeout=5s --start-period=20s --retries=10 \
#  CMD curl -fsS "http://localhost:8888/api/status" -H "Authorization: token ${JUPYTER_TOKEN}" || exit 1

# tini como init
ENTRYPOINT ["/usr/bin/tini", "--"]

# Jupyter Notebook como root (note o --allow-root)
CMD ["/venv/bin/python", "-m", "notebook", "--ip=0.0.0.0", "--no-browser", \
     "--allow-root", \
     "--NotebookApp.notebook_dir=/root/work"]
