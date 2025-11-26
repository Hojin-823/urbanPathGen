# 🚁 urbanPathGen

[![Python](https://img.shields.io/badge/Python-3.8%2B-blue)](https://www.python.org/)

This repository contains the official Python implementation for the paper:

> **Drag-Aware Route Planning for Unmanned Aerial Vehicles in Dynamic Urban Environments** 
---

## 📖 Abstract
Unmanned Aerial Vehicles (UAVs) play a pivotal role in modern society, yet their flight time is limited by battery constraints. This study proposes a **drag-based path prediction method** using a **Graph Convolutional Neural Network (GCNN)**. 

We aim to enhance realism by:
1. Accurately predicting wind flow around complex urban buildings.
2. Utilizing the resulting **aerodynamic drag** as a key cost function for path planning.
3. Demonstrating significant energy savings compared to conventional distance-based algorithms.

---

## 🚀 Key Features

- **🌪️ GCNN-based Wind Prediction**: Utilizes Graph Neural Networks to predict complex wind fields around urban geometries (trained with LB simulation data).
- **🔋 Energy-Efficient Pathfinding**: Implements a modified **Dijkstra algorithm** that considers aerodynamic drag ($F_d$) instead of just Euclidean distance.
- **📊 Quantitative Analysis**: Calculates **Normalized Energy Consumption** and **Work Savings (%)** relative to a no-wind baseline.

---

## 📂 Directory Structure

```bash
urban/
├── data/               # Place NetCDF files here
├── src/                # Source codes
|   ├── __init__.py
│   ├── physics.py      # Physical constants & Drag coefficient ($C_d$) interpolation
│   ├── environment.py  # Data loading & Building detection
│   ├── pathfinding.py  # Modified Dijkstra algorithm (Opt 1 & Opt 2)
│   └── visualization.py # Plotting tools
├── main.py             # Main execution script
└── requirements.txt    # Python dependencies
