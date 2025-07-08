
# Stein Variational Guided Model Predictive Path Integral Control (SVG-MPPI)

### [**Paper**](https://arxiv.org/abs/2309.11040) | [**Video**](https://www.youtube.com/watch?v=ML_aOYQIDL0) 

This package includes ROS implementation of [Stein Variational Guided Model Predictive Path Integral Control: Proposal and Experiments with Fast Maneuvering Vehicles](https://arxiv.org/abs/2309.11040) presented in ICRA 2024.


https://github.com/user-attachments/assets/25bd67c5-3e9c-4c4e-9a79-cacf066f4af5


![Overview](docs/assets/overview_svg_mppi.png)

## Tested Environment
- Ubuntu 20.04, 22.04, and 24.04
- Docker

<details>
<summary>Docker Installation</summary>

[Installation guide](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)

```bash
# Install from get.docker.com
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo groupadd docker
sudo usermod -aG docker $USER
```

</details>

## Quick Start with Docker

### 1. Build Docker Image
```bash
make docker_build
```

### 2. Launch Simulator and Controllers

Launch simulator in a new terminal
```bash
cd proj-svg_mppi
./script/launch_simulator.sh
```

Launch controllers in another terminal
```bash
cd proj-svg_mppi
# Enter Docker container with GUI support
make bash
# Inside container, launch controllers
./script/launch_controllers.sh
```
You can change the all MPPI parameters and settings in [the yaml file](./src/mppi_controller/config/mppi_controller.yaml)

## Run on Native System

### Requirements

- Ubuntu 20.04
- ROS Noetic

### 1. Install Dependencies

```bash
cd proj-svg_mppi
make setup
```

### 2. Build the Project

```bash
cd proj-svg_mppi
make build
```

### 3. Launch Simulator and Controllers

Launch simulator in the Docker container
```bash
cd proj-svg_mppi
./script/launch_simulator.sh
```

Launch controllers in another terminal
```bash
cd proj-svg_mppi
./script/launch_controllers.sh 
```

## Evaluation

You can reproduce all simulation results in the paper by one command: 
```bash
cd proj-svg_mppi/script
./eval_all.sh
```

Or, You can evaluate a fixed parameters by this command:
```bash
cd proj-svg_mppi/script
./eval.sh
```

**Note**: The evaluation is used asynchronous simulation using ROS. So, the results can be slightly changed even if all seeds are fixed.


## Citation

```bibtex
@inproceedings{honda2024stein,
  title={Stein Variational Guided Model Predictive Path Integral Control: Proposal and Experiments with Fast Maneuvering Vehicles},
  author={Honda, Kohei and Akai, Naoki and Suzuki, Kosuke and Aoki, Mizuho and Hosogaya, Hirotaka and Okuda, Hiroyuki and Suzuki, Tatsuya},
  booktitle={2024 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={7020--7026},
  year={2024},
  organization={IEEE}
}
```
