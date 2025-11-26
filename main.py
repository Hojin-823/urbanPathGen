import argparse
import numpy as np
from src.environment import UrbanFlowEnv
from src.pathfinding import dijkstra_pathfinding, calculate_total_energy
from src.visualization import setup_plot_style, plot_results

def main():
    parser = argparse.ArgumentParser(description="Drag-Aware UAV Path Planning")
    parser.add_argument('--info', type=str, required=True, help='Path to Info.Netcdf file')
    parser.add_argument('--pv', type=str, required=True, help='Path to PV.Netcdf file')
    args = parser.parse_args()

    # 1. Initialize Environment
    env = UrbanFlowEnv(args.info, args.pv)

    # 2. Define Scenarios
    start_pos = np.array([0, -2.5])
    goal_pos = np.array([-2.5, -2.5])
    stopovers = [np.array([0, 2.5]), np.array([0.5, 0.5]), np.array([2.5, 0.5])]

    scenarios = {
        'Scenario_1': [start_pos, stopovers[0], goal_pos],
        # Add more scenarios...
    }

    setup_plot_style()

    # 3. Run Pathfinding
    for name, waypoints in scenarios.items():
        print(f"\nProcessing {name}...")

        # Opt 1: Distance
        full_path1 = []
        for i in range(len(waypoints)-1):
            seg = dijkstra_pathfinding(waypoints[i], waypoints[i+1], env, 'distance')
            if seg: full_path1.extend(seg)

        # Opt 2: Energy
        full_path2 = []
        for i in range(len(waypoints)-1):
            seg = dijkstra_pathfinding(waypoints[i], waypoints[i+1], env, 'energy')
            if seg: full_path2.extend(seg)

        # Calculate Energy & Print
        e1 = calculate_total_energy(full_path1, env)
        e2 = calculate_total_energy(full_path2, env)

        print(f"Opt 1 Energy: {e1:.2f}")
        print(f"Opt 2 Energy: {e2:.2f}")
        if e1 > 0:
            print(f"Savings: {((e1-e2)/e1)*100:.2f}%")

        # Visualization
        plot_results(env, full_path1, full_path2, waypoints, name)

if __name__ == "__main__":
    main()
