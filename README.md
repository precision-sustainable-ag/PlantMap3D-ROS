# PlantMap3D-ROS


## This branch acts as a beta development branch, create a new branch for feature development and create a pull request.

## Intended System Flow-diagram : 

[Note : This diagram is expected to be updated with design upgrades]
<img src="https://user-images.githubusercontent.com/71589098/223198277-275437fe-df28-4ac8-885b-ba5ca5676e2b.png" width=75%% height=75% />


## Current working pipeline : 

<img src="https://user-images.githubusercontent.com/71589098/222145418-b61845eb-078b-4856-8990-7fecf505570d.png" width=50%% height=50% />


## Some common erros

1. 

<img src="https://user-images.githubusercontent.com/71589098/218086068-499a5122-87de-429d-a2ca-c7f20ace9aa5.png" width=50%% height=50% />

Answer : Try the following command line :

```
sudo apt-get install ros-noetic-catkin-virtualenv
```

# BioMassEstimator module
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
