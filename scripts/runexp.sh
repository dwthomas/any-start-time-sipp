#!/bin/bash
#SBATCH --job-name=sipptest   # Job name
#SBATCH --mail-type=NONE            # Mail events (NONE, BEGIN, END, FAIL, ALL)
#SBATCH --mail-user=dwt@cs.unh.edu   # Where to send mail	
#SBATCH --ntasks=1                  # Run a single task
#SBATCH --array=1-15                 # Array range
#SBATCH -p compute
eval "$(head -${SLURM_ARRAY_TASK_ID} $1) | tail -1"