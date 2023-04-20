## README for the drivers

### Start ROS environment

```
roscore
```
### Start the system 

```
roslaunch pm3d_system_config system_start.launch
```

### Cleanly shutdown all nodes

```
roslaunch pm3d_system_config system_shutdown.launch
```


### BioMassEstimator module
ROS wrapper will collect the output of the segmentation module for a segmentation array and a height above ground array from a height estimation module

Inputs:
- a segmentation numpy 2D array
- a height above ground 2D array (not used in version 1)

Process:
1. count pixels for each species from segmentation label input
2. apply a linear y = mx+b correlation between species pixel count and biomass (by species if necessary)

Outputs:
- a dictionary of Biomass estimates for each possible species (floats)

Roadmap:
1. Version 1 release date: 2/10/2023
2. Version 2: change biomass estimation to include height information to calculate volume per species. Then the biomass estimate would be a correlation between species volume and biomass. No release date yet.
3. Version 3: change biomass correlation method to be either more statistically sophisticated, or introduce a neural network for biomass estimation.
p
