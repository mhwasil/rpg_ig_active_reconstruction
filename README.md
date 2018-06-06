Information Gain Based Active Reconstruction
============================================

This repository contains the code for **information gain based active reconstruction** as presented in the paper:

*An Information Gain Formulation for Active Volumetric 3D Reconstruction*  
S. Isler, R. Sabzevari, J. Delmerico and D. Scaramuzza (ICRA 2016)  
[Paper: http://rpg.ifi.uzh.ch/docs/ICRA16_Isler.pdf](http://rpg.ifi.uzh.ch/docs/ICRA16_Isler.pdf)  
[Video (Youtube)](https://www.youtube.com/watch?v=ZcJcsoGGqbA&feature=youtu.be) 

#### Disclaimer

The ig_active_reconstruction implementation in this repository is research code, any fitness for a particular purpose is disclaimed.

The code has been tested in Ubuntu 14.04 with ROS Indigo.

#### Licence

The source code is released under a GPLv3 licence.

http://www.gnu.org/licenses/

#### Citing

If you use ig_active_reconstruction in an academic context, please cite the following publication:

    @inproceedings{Isler2016ICRA,
      title={An Information Gain Formulation for Active Volumetric 3D Reconstruction},
      author={Isler, Stefan and Sabzevari, Reza and Delmerico, Jeffrey and Scaramuzza, Davide},
      booktitle={IEEE International Conference on Robotics and Automation (ICRA)},
      year={2016},
      organization={IEEE}
    }

#### Install and run ig_active_reconstruction

The wiki 

https://github.com/uzh-rpg/rpg_ig_active_reconstruction/wiki

contains instructions on how to build and run the code.

#### Acknowledgments

Thanks to Pavel Vechersky for his key contributions, as well as to Elias Mueggler, Matthias Faessler, and Junjie Zhang for their valuable feedback.
   
#### Contributing

You are very welcome to contribute to ig_active_reconstruction by opening a pull request via Github.
We try to follow the ROS C++ style guide http://wiki.ros.org/CppStyleGuide



#### Information Gain Formulation on real robot (youbot) using RGBD camera.
This work aims at developing and implementing IG to real youbot with 5 DoF arm for 3D object exploration in a RoboCup@Work environment.

* Params in octomap
    * use_bounding_box : this param enables the cutting of points from camera and it can only take the points within given bound, it can also be used to reduce noises from camera
* Params in iar::ros
    * cost_weight : this param enables the IG calculation
    * max_calls : the number of exploration executed by the robot
    * ig_weights : the method that we are going to test
    
* Note: we will update this repository
