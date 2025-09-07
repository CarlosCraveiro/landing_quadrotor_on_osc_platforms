# Quadrotor Landing on Oscillating Platforms with MPC

This repository contains the code and simulations of the undergraduate scientific initiation project funded by [FAPESP](https://bv.fapesp.br/en/bolsas/222615/landing-of-a-quadrotor-on-an-oscillating-platform-using-predictive-control-strategy/), focused on the development and evaluation of **Model Predictive Control (MPC)** strategies for quadrotor landing on oscillating platforms, typical of maritime scenarios.

---

## Project Overview

Unmanned Aerial Vehicles (UAVs) with vertical take-off and landing capabilities (VTOL) are increasingly used in the automation of tasks in sectors such as port operations and the oil industry.
In these scenarios, UAVs must operate under adverse conditions, including **gusty winds** and **oscillating landing platforms**, requiring new control strategies to prevent catastrophic failures.

This project proposes the use of **MPC** to handle such conditions, with a specific focus on ensuring safe landing on mobile platforms. Validation is carried out through **simulations in Julia**, comparing the performance of the proposed controller with existing solutions in the literature, such as the LQR controller.

## Repository Structure

* `src/` – simulation and controller code
* `notebooks/` – interactive notebooks for results analysis
* `Dockerfile` / `docker-compose.yml` – reproducible environment via Docker

---

## Dependencies

The entire environment is containerized in **Docker**, but you need to have Docker Engine and Docker Compose installed.

### Ubuntu / Debian

```bash
sudo apt update
sudo apt install -y docker.io docker-compose
sudo systemctl enable --now docker
```

### Fedora

```bash
sudo dnf install -y docker docker-compose
sudo systemctl enable --now docker
```

---

## How to run

Clone the repository and execute:

```bash
docker compose build
docker compose up
```

After initialization, copy and paste into your browser the link displayed in the terminal, for example:

```
http://127.0.0.1:8888/tree?token=...
```

This will open the Jupyter environment with the project notebooks.

---

## Credits

Parts of this project were inspired by or adapted from the lecture notebooks of the course
[Optimal-Control-16-745/lecture-notebooks-2023](https://github.com/Optimal-Control-16-745/lecture-notebooks-2023), licensed under [CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/). Changes and extensions have been made to adapt the code to the scope of this work.

## Acknowledgements

This project was supported by the São Paulo Research Foundation (FAPESP), through the funding of an undergraduate research scholarship.

## Authors
- Student Researcher:
    [ *Carlos Henrique Craveiro Aquino Veras* ](https://bv.fapesp.br/pt/pesquisador/736088/carlos-henrique-craveiro-aquino-veras/)
- Advisor:
    [ *Glauco Augusto de Paula Caurin* ](https://orcid.org/0000-0003-0898-1379)
