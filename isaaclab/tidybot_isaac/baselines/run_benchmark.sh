#!/bin/bash

# --- Configuration ---
# Point this to the isaaclab.sh script in your Isaac Lab root
ISAAC_PYTHON="../../isaaclab/isaaclab.sh -p" 
BASELINES_DIR="."
EVAL_SCRIPT="./eval_policy.py"
OUTPUT_CSV="final_evaluation_results.csv"

echo "Task,Checkpoint,Success_Rate,Mean_Torque" > $OUTPUT_CSV
rm -f eval_results.tmp

run_eval() {
    local task_name=$1
    local path=$2
    
    # Find the latest timestamp directory
    local latest_run=$(ls -d $path/*/ 2>/dev/null | sort -r | head -n 1)
    
    if [ -z "$latest_run" ]; then
        echo "[SKIP] No run directories found in $path"
        return
    fi

    # Find the model with the highest iteration number
    local latest_checkpoint=$(ls ${latest_run}model_*.pt 2>/dev/null | sort -V | tail -n 1)
    
    if [ -f "$latest_checkpoint" ]; then
        echo "[RUNNING] $task_name with $latest_checkpoint"
        # Using the Isaac Lab python wrapper
        $ISAAC_PYTHON $EVAL_SCRIPT --task $task_name --checkpoint $latest_checkpoint --headless
    else
        echo "[ERROR] No checkpoint found in $latest_run"
    fi
}

# --- Drawer Tasks ---
# DRAWER_TASK="Isaac-TidyBot-HandE-Drawer-v0"
# run_eval $DRAWER_TASK "$BASELINES_DIR/drawer/DR"
# run_eval $DRAWER_TASK "$BASELINES_DIR/drawer/GT"

# --- Door Tasks ---
DOOR_TASK="Isaac-TidyBot-HandE-Door-v0" 
run_eval $DOOR_TASK "$BASELINES_DIR/door/full_DR"
# run_eval $DOOR_TASK "$BASELINES_DIR/door/partial_DR"
# run_eval $DOOR_TASK "$BASELINES_DIR/door/GT"

# Final Aggregation
if [ -f "eval_results.tmp" ]; then
    cat eval_results.tmp >> $OUTPUT_CSV
    echo "Evaluation Complete. Results saved to $OUTPUT_CSV"
else
    echo "[ERROR] No results were generated. Check for Python errors above."
fi