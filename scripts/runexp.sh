#!/bin/bash
#SBATCH --job-name=peap1024   # Job name
#SBATCH --mail-type=NONE            # Mail events (NONE, BEGIN, END, FAIL, ALL)
#SBATCH --mail-user=dwt@cs.unh.edu   # Where to send mail	
#SBATCH --ntasks=1                  # Run a single task
#SBATCH --array=1-768                 # Array range
#SBATCH --time=0:10:00
#SBATCH --no-kill
#SBATCH -p compute
cd /home/aifs2/devin/Documents/pdap/scripts/
echo "$(head -${SLURM_ARRAY_TASK_ID} /home/aifs2/devin/Documents/pdap/scripts/new_partial_commands.run | tail -1)"
eval "$(head -${SLURM_ARRAY_TASK_ID} /home/aifs2/devin/Documents/pdap/scripts/new_partial_commands.run | tail -1)"
