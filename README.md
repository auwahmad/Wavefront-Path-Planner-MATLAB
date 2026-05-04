# Wavefront Path Planning Toolkit

A comprehensive MATLAB-based suite for exploring and implementing the Wavefront (Grassfire) path planning algorithm. This repository includes a modular function, an interactive GUI application, and a Live Script for educational purposes.

## 📖 What is the Wavefront Algorithm?
The **Wavefront Path Planner** is a grid-based navigation technique used in robotics to find the shortest path from a starting point to a goal. 

It operates in two distinct phases:
1.  **Wave Propagation:** Starting at the goal, the algorithm assigns an increasing "cost" value to adjacent free cells. This "wave" spreads across the grid until it reaches the start point.
2.  **Backtracking:** From the start point, the agent follows the steepest descent (moving to the neighbor with the lowest cost) until it reaches the goal.

## 🛠️ Repository Overview
This project provides three distinct ways to utilize the algorithm:
*   **`wavefront_planner.m`**: A modular MATLAB function for integration into external scripts.
*   **`waveFront_Algorithm_App.mlapp`**: An interactive GUI for building maps and visualizing the wave in real-time.
*   **`Wavefront_Tutorial.mlx`**: A Live Script that breaks down the math and logic step-by-step.

---

## ⚡ Algorithm Visualization

### **4-Connectedness vs. 8-Connectedness**
The algorithm supports different neighbor connectivity, which dictates how the "wave" spreads.

*   **4-Connected:** Moves only Up, Down, Left, and Right (Manhattan distance).
*   **8-Connected:** Includes diagonal movements (Chebyshev distance), resulting in shorter, more direct paths.

| 4-Connectedness | 8-Connectedness |
| :--- | :--- |
| ![4-Way Path](images/4_connected_result.png) | ![8-Way Path](images/8_connected_result.png) |

### **Obstacle Handling & Blocked Paths**
The planner intelligently navigates around objects (Value = 1). If the goal is completely enclosed by obstacles, the algorithm will terminate and notify the user that no path is possible.

![Blocked Path Example](images/blocked_path.png)

---

## 💻 Function Usage
The core logic is contained in `wavefront_planner.m`. This is ideal for research or larger simulation projects where a GUI is not required.

```matlab
% Example: Planning a path on a 10x10 grid
map = zeros(10, 10);     
map(3:7, 5) = 1;         % Create a vertical wall obstacle
start = [1, 1];
goal = [10, 10];
connectivity = 8;        % Choose 4 or 8

% Run the planner
[path, costMap, pathLen] = wavefront_planner(map, start, goal, connectivity);

% Display Result
disp('Path Indexes:');
disp(path);
fprintf('Path calculated with %d steps.\n', pathLen);
