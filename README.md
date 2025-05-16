# ESE650_Final

This repository aggregates several upstream robotics libraries (in `vendors/`) alongside our own customized forks (in `forked/`). We won’t include all the detailed modifications for each model during training here, since there's not enough space when uploading code to Gradescope. Instead, we’ll reference the original codebase, and the specific details can be found in the report.


## Repository Layout
```
├── vendors/
│ ├── dial-mpc/ # Upstream DIAL-MPC core
│ ├── rsl_rl/ # Upstream RSL-RL framework
│ ├── unitree_rl_gym/ # Upstream Unitree RL Gym
│ ├── wococo/ # Upstream WOCOCO library
│ ├── HOVER/ # Upstream NVlabs HOVER
│ └── humanoid-bench/ # Upstream Humanoid Bench
├── forked/
│ ├── dial-mpc/ # Our fork of DIAL-MPC
│ ├── unitree_rl_gym/ # Our fork of Unitree RL Gym
│ ├── wococo/ # Our fork of WOCOCO
│ ├── HOVER/ # Our fork of HOVER
│ ├── humanoid-bench/ # Our fork of Humanoid Bench
│ ├── ProtoMotions/ # Our fork of ProtoMotions
│ └── ALMI-Open/ # Our fork of ALMI-Open
├── .gitmodules # Submodule definitions
└── README.md # ← This file
```

## Using Instruction
clone all submoudlues
```
git submodule update --init --recursive

```

Please follow each Repo installation instuction
