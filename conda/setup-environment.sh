#!/bin/sh

set -e
set -x

# create the conda environment as a subdirectory
mamba env create -y -p .conda/freecad -f conda/conda-env.yaml

# add the environment subdirectory to the conda configuration
conda config --add envs_dirs $CONDA_PREFIX/envs
conda config --add envs_dirs $(pwd)/.conda
conda config --set env_prompt "({name})"

# install the FreeCAD dependencies into the environment
mamba run --live-stream -n freecad mamba-devenv -y --no-prune -f conda/environment.devenv.yml
