#!/bin/bash
#SBATCH --job-name=sippandpdaprooms   # Job name
#SBATCH --mail-type=NONE            # Mail events (NONE, BEGIN, END, FAIL, ALL)
#SBATCH --mail-user=dwt@cs.unh.edu   # Where to send mail	
#SBATCH --ntasks=1                  # Run a single task
#SBATCH --array=1-3072                 # Array range
#SBATCH --time=0:10:00
#SBATCH --no-kill
#SBATCH -p compute
cd /home/aifs2/devin/Documents/pdap/scripts/
echo "$(head -${SLURM_ARRAY_TASK_ID} /home/aifs2/devin/Documents/pdap/scripts/commands.run | tail -1)"
eval "$(head -${SLURM_ARRAY_TASK_ID} /home/aifs2/devin/Documents/pdap/scripts/commands.run | tail -1)"
