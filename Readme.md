Cave Simulation and Reconstruction
This repository contains Python scripts for simulating and reconstructing 3D cave structures using Open3D and Perlin noise. The main functionalities include creating and deforming a cylinder mesh to represent a cave tunnel, sampling points from the mesh, and visualizing the results.

## Table of Contents
- [Installation](#installation)
- [Usage](#usage)
    - [Cave Simulation](#cave-simulation)
    - [3D Reconstruction](#3d-reconstruction)

## Installation
Clone the repository:
```bash
git clone https://github.com/souvik0306/LiDAR-Projects.git
```

Install the required libraries:
```bash
pip install -r requirements.txt
```

## Usage

### Cave Simulation
The `cave_simulation.py` script creates a 3D cave tunnel using a cylinder mesh, deforms the walls using Perlin noise, and visualizes the result.

To run the simulation:
```bash
python cave_simulation.py
```

### 3D Reconstruction
The `cave_3d_reconstruction.py` script performs a similar task but includes additional steps such as recomputing normals, inverting normals, reversing triangle winding, assigning colors to vertices, and downsampling the point cloud.

To run the 3D reconstruction:
```bash
python cave_3d_reconstruction.py
```

## Project Structure
- `cave_3d_reconstruction.py`: Script for 3D reconstruction of the cave.
- `cave_internal_structure.py`: Placeholder for internal structure simulation script.
- `cave_simulation.py`: Script for simulating the cave tunnel.
- `requirements.txt`: List of required libraries and their versions.

