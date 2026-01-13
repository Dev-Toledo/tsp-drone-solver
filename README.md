# Hybrid Truck & Drone Delivery Optimization 🚚🚁

![Python](https://img.shields.io/badge/Python-3.8%2B-blue)
![Solver](https://img.shields.io/badge/Solver-Gurobi-red)
![License](https://img.shields.io/badge/license-MIT-green)

A Mixed-Integer Linear Programming (MILP) solution for the **Flying Sidekick Traveling Salesman Problem (FSTSP)**. This project optimizes last-mile delivery routes by coordinating a truck and a drone to minimize total operation time.

## 📌 Problem Overview
In modern logistics, combining traditional trucks with drones can significantly reduce delivery times. The truck acts as a moving launchpad, performing deliveries while launching a drone to serve customers within a radius $R$. The drone travels $K$ times faster than the truck and must return to the vehicle at a rendezvous node.

**Key Constraints:**
- Each customer is visited exactly once (by truck OR drone).
- The drone has a limited flight range ($R$) and capacity (1 package).
- Subtour elimination constraints ensure valid routing.
- The objective is to minimize the total time ($Z$) required to serve all $N$ customers and return to the depot.

## 🧮 Mathematical Model
The problem is modeled using **Gurobi Optimizer**.

**Objective Function:**
$$\min \sum_{i,j} (K \cdot d_{ij} \cdot x_{ij}) + \sum_{i,j} (2 \cdot d_{ij} \cdot y_{ij})$$

Where:
- $x_{ij} \in \{0,1\}$: Truck travels from $i$ to $j$.
- $y_{ij} \in \{0,1\}$: Drone delivers to $j$ launching from $i$.
- $d_{ij}$: Euclidean distance.
- $K$: Speed factor (Truck/Drone ratio).

## 🚀 Results & Visualization
We tested the model on random instances (15 to 35 nodes). The hybrid approach demonstrated significant efficiency gains compared to the traditional Truck-Only TSP.

| Instance | Nodes | Truck Only Cost | Hybrid Cost | Improvement |
|:---:|:---:|:---:|:---:|:---:|
| `inst_35_0` | 35 | 3736.83 | 3324.46 | **~11%** |
| `inst_20_1` | 20 | 2870.62 | 2677.91 | **~6.7%** |

### Optimized Route Visualization
*Blue lines: Truck Path | Red dashed lines: Drone Flight*

![Route Visualization](images/Figura1_inst_35_0.png)

## 🛠️ Installation & Usage

1. **Clone the repository:**
   ```bash
   git clone [https://github.com/Dev-Toledo/tsp-drone-solver.git](https://github.com/Dev-Toledo/tsp-drone-solver.git)

2. **Install dependencies**
   ```bash
   pip install -r requirements.txt

3. **Run the solver:** (Note: A valid Gurobi License is required. Academic licenses are free.)
   ```bash
   python src/solver.py

## 👨‍💻 Tech Stack
Language: Python

Solver: Gurobi Optimizer (gurobipy)

Visualization: Matplotlib & Folium

## 📄 License
This project is licensed under the MIT License.
