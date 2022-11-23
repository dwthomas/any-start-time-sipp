#!/bin/bash
#SBATCH --job-name=sipptest   # Job name
#SBATCH --mail-type=NONE            # Mail events (NONE, BEGIN, END, FAIL, ALL)
#SBATCH --mail-user=dwt@cs.unh.edu   # Where to send mail	
#SBATCH --ntasks=1                  # Run a single task
#SBATCH --array=1-1120                 # Array range
#SBATCH -p compute
cd /home/aifs2/devin/Documents/pdap/scripts/
echo "$(head -${SLURM_ARRAY_TASK_ID} /home/aifs2/devin/Documents/pdap/scripts/commands.run | tail -1)"
eval "$(head -${SLURM_ARRAY_TASK_ID} /home/aifs2/devin/Documents/pdap/scripts/commands.run | tail -1)"
