import pandas as pd
import matplotlib
matplotlib.use('Agg')  # Prevents Linux/ROS GUI deadlocks
import matplotlib.pyplot as plt
import pandas as pd
import math
import os
import numpy as np

# --- TITLE MAPPING based on PolicyCfg ---
COLUMN_TITLES = {
    # Proprioception
    "obs_0": "Joint 1 Pos", "obs_1": "Joint 2 Pos", "obs_2": "Joint 3 Pos",
    "obs_3": "Joint 4 Pos", "obs_4": "Joint 5 Pos", "obs_5": "Joint 6 Pos", "obs_6": "Joint 7 Pos",
    "obs_7": "Joint 1 Vel", "obs_8": "Joint 2 Vel", "obs_9": "Joint 3 Vel",
    "obs_10": "Joint 4 Vel", "obs_11": "Joint 5 Vel", "obs_12": "Joint 6 Vel", "obs_13": "Joint 7 Vel",
    "obs_14": "Gripper Open Amount (0=Open, 1=Closed)",
    "obs_15": "Gripper Cooldown",
    
    # Task State: EE to Handle (9D)
    "obs_16": "EE-to-Handle Pos X (Local)", "obs_17": "EE-to-Handle Pos Y (Local)", "obs_18": "EE-to-Handle Pos Z (Local)",
    "obs_19": "Handle X-axis in EE (X)", "obs_20": "Handle X-axis in EE (Y)", "obs_21": "Handle X-axis in EE (Z)",
    "obs_22": "Handle Z-axis in EE (X)", "obs_23": "Handle Z-axis in EE (Y)", "obs_24": "Handle Z-axis in EE (Z)",
    
    # Task State: EE to Hinge (3D) & Hinge Axis (3D)
    "obs_25": "EE-to-Hinge Pos X (Local)", "obs_26": "EE-to-Hinge Pos Y (Local)", "obs_27": "EE-to-Hinge Pos Z (Local)",
    "obs_28": "Hinge Z-axis in EE (X)", "obs_29": "Hinge Z-axis in EE (Y)", "obs_30": "Hinge Z-axis in EE (Z)",
    
    # Task State: Door Pos
    "obs_31": "Door Hinge Position (Rad)",
    
    # Task State: Last Actions
    "obs_32": "Last Action 1", "obs_33": "Last Action 2", "obs_34": "Last Action 3", "obs_35": "Last Action 4",
    "obs_36": "Last Action 5", "obs_37": "Last Action 6", "obs_38": "Last Action 7", "obs_39": "Last Action 8 (Gripper)",
    
    # Mystery Extra Observation
    "obs_40": "Unknown Obs (Index 40)",
    
    # Current Actions
    "action_0": "Cmd Action 1", "action_1": "Cmd Action 2", "action_2": "Cmd Action 3",
    "action_3": "Cmd Action 4", "action_4": "Cmd Action 5", "action_5": "Cmd Action 6",
    "action_6": "Cmd Action 7", "action_7": "Cmd Action 8 (Gripper)"
}

def plot_comparisons(sim_csv="rollout_full_DR_on_procedural.csv", real_csv="rollout_full_DR_on_reconstructed.csv"):
    if not os.path.exists(sim_csv) or not os.path.exists(real_csv):
        print("Error: Missing one of the CSV files!")
        return

    sim_df = pd.read_csv(sim_csv)
    real_df = pd.read_csv(real_csv)

    min_len = min(len(sim_df), len(real_df))
    print(f"Truncating plots to {min_len} steps (Sim had {len(sim_df)}, Real had {len(real_df)})")

    # --- THE FOOLPROOF FIX ---
    # Force extract ONLY the last 48 columns of data (40 obs + 8 actions).
    # This ignores any shifted headers or hidden row indices at the front.
    # --- REPLACEMENT FOR THE iloc LOGIC ---
    # Get the actual column names from the CSVs
    sim_obs_cols = [f"obs_{i}" for i in range(40)]
    real_obs_cols = [f"obs_{i}" for i in range(40)]
    # --- THE ALIGNMENT FIX ---
    # Check if hardware has a "Ghost Column" at the start (common with 'index=True' in pandas)
    # Or if it simply has more columns than headers.
    
    # Extract data for Sim (Standard)
    sim_data = sim_df[[f"obs_{i}" for i in range(40)]].iloc[:min_len].values
    
    # Extract data for Hardware (Offset check)
    # If obs_16 is the square wave that should be obs_15, we need to skip the first data column
    real_data_raw = real_df[[f"obs_{i}" for i in range(40)]].iloc[:min_len].values
    
    # Try shifting the hardware data back by 1 if you see a mismatch
    # Change 'offset = 1' if the hardware has a leading timestamp/index column
    offset = 1 
    
    # We grab columns 1 through 40 instead of 0 through 39
    # This effectively makes 'obs_1' the new 'obs_0'
    real_data_shifted = real_df.iloc[:min_len, offset : offset+40].values
    
    # Use the shifted data for the matrix
    sim_matrix = sim_data
    real_matrix = real_data_shifted
    
    # Do the same for actions if you are plotting them
    sim_act = sim_df[[f"action_{i}" for i in range(8)]].iloc[:min_len].values
    real_act = real_df[[f"action_{i}" for i in range(8)]].iloc[:min_len].values

    # Combine for the plotting loop
    sim_matrix = np.hstack([sim_matrix, sim_act])
    real_matrix = np.hstack([real_matrix, real_act])

    min_len = min(len(sim_df), len(real_df))
    print(f"Truncating plots to {min_len} steps (Sim had {len(sim_df)}, Real had {len(real_df)})")

    # --- THE TRULY FOOLPROOF FIX ---
    # Define the exact column names we want
    expected_cols = [f"obs_{i}" for i in range(40)] + [f"action_{i}" for i in range(8)]
    
    # Extract data by strictly requesting the named columns.
    # This ignores any 'Unnamed: 0' index columns completely.
    sim_matrix = sim_df[expected_cols].iloc[:min_len].values
    real_matrix = real_df[expected_cols].iloc[:min_len].values

    plots_per_fig = 16
    num_figs = math.ceil(len(expected_cols) / plots_per_fig)
    
    for f in range(num_figs):
        fig, axes = plt.subplots(4, 4, figsize=(18, 12))
        fig.tight_layout(pad=5.0)
        axes = axes.flatten()
        
        for i in range(plots_per_fig):
            col_idx = f * plots_per_fig + i
            ax = axes[i]
            
            if col_idx < len(expected_cols):
                col_name = expected_cols[col_idx]
                
                # Plot directly from our aligned numpy matrices
                ax.plot(sim_matrix[:, col_idx], label='Sim', color='blue', alpha=0.7, linewidth=2)
                ax.plot(real_matrix[:, col_idx], label='Hardware', color='red', alpha=0.8, linewidth=2, linestyle='--')
                
                pretty_title = COLUMN_TITLES.get(col_name, col_name)
                ax.set_title(pretty_title, fontweight='bold')
                ax.grid(True, linestyle=':', alpha=0.6)
                
                if i == 0:
                    ax.legend()
            else:
                ax.axis('off')
        
        save_name = f"comparison_part_{f+1}.png"
        plt.savefig(save_name, dpi=150)
        print(f"Saved {save_name}")
        plt.close()

if __name__ == "__main__":
    plot_comparisons()