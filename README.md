This project uses Information Gain calculation and octomap representation proposed by Stefan Isler et al. In this project, there is an example using flying stereo cameras for constructing 3D model in Gazebo. This work aims at developing and implementing IG to real youbot with 5 DoF arm for 3D object exploration in a RoboCup@Work environment.

* Params in octomap
    * use_bounding_box : this param enables the cutting of points from camera and it can only take the points within given bound, it can also be used to reduce noises from camera
* Params in iar::ros
    * cost_weight : this param enables the IG calculation
    * max_calls : the number of exploration executed by the robot
    * ig_weights : the method that we are going to test
