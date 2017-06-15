# alignmentDemo
How to line up a 3D model in ROS to the Motion capture markers. 

# files to change.
The .urdf file in this repo is hard coded to the stl Location.
The .urdf reference in the launch file is also hard coded. 
Change these to your paths or make them generic ;) 

Set the IP adresses in the launch for the motion capture. 

# To launch
Launch the launch file until fully launched. 

# Camera Location
This is roughly (-0.307,0.302,0.999) in model space (glassesOut is the model space in TF). I will update this with a better estimate
once we have a way of viewing things in a reasonable way. 
