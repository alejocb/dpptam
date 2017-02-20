#DPPTAM:

DPPTAM is a direct monocular odometry algorithm that estimates a dense reconstruction of a scene in real-time on a CPU. Highly textured image areas are mapped using standard direct mapping techniques, that minimize the photometric error across different views. We make the assumption that homogeneous-color regions belong to approximately planar areas.
Related Publication:

[1] Alejo Concha, Javier Civera. DPPTAM: Dense Piecewise Planar Tracking and Mapping from a Monocular Sequence IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS15), Hamburg, Germany, 2015

Video of the results that you should expect in the example sequences:

https://www.youtube.com/watch?v=1R3DkwKrWiI

#License

DPPTAM is licensed under the GNU General Public License Version 3 (GPLv3), please see http://www.gnu.org/licenses/gpl.html.

For commercial purposes, please contact the authors.

If you use DPPTAM in an academic work, please cite:

      @inproceedings{conchaIROS15,
      title={{Dense Piecewise Planar Tracking and Mapping  from a Monocular Sequence}},
      author={Concha, Alejo and Civera, Javier},
      booktitle={Proc. of The International Conference on Intelligent Robots and Systems (IROS)},
      year={2015}}

#Disclaimer

This site and the code provided here are under active development. Even though we try to only release working high quality code, this version might still contain some issues. Please use it with caution.

#Dependencies

ROS:

We have tested DPPTAM in Ubuntu 14.04 with ROS Indigo.

To install ROS (indigo) use the following command:

     sudo apt-get install ros-indigo-desktop
     
Or check the following link if you have any issue:

    http://wiki.ros.org/indigo/Installation/Ubuntu
     

PCL library for visualization:

     sudo apt-get install ros-indigo-pcl-ros
     
BOOST library to launch the different threads:
    
     sudo apt-get install libboost-all-dev 

#Installation

     git clone  https://github.com/alejocb/dpptam.git
    
#Compilation

     catkin_make --pkg dpptam

Third Party: SUPERPIXELS COMPILATION

Code used -> Efficient Graph-Based Image Segmentation. P. Felzenszwalb, D. Huttenlocher. International Journal of Computer Vision, Vol. 59, No. 2, September 2004

    cd root/catkin_workspace/src/dpptam/ThirdParty/segment
    make

#Usage

Launch dpptam from your 'catkin_workspace' folder:
     
    cd root/catkin_workspace 
    rosrun dpptam dpptam
    
Notice that the location of dpptam should be the following:

    root/catkin_workspace/src/dpptam

Launch the visualizer of the current frame

    rosrun image_view image_view image:=/dpptam/camera/image

Launch the visualizer of the map

    rosrun rviz rviz
    
We are working on an automatic visualizer, but for now, check the following screenshot to set up the rviz visualizer:

    http://imgur.com/OA8i3Dj      

      

Visualization of the results using '.ply' files.

You can alternatively use an offline visualizer (like 'meshlab') to see better the results. The files are saved in dpptam/src/map_and_poses

To execute the provided sequences:

    rosbag play lab_unizar.bag
    rosbag play lab_upenn.bag

There are two parameters that you have to modify (before executing a sequence) in dpptam/src/data.yml:

1-) Intrinsic parameters of the camera:

'cameraMatrix'

'distCoeffs'

2-) Camera topic

camera_path:"/image_raw"

These are the parameters and the links for the sequences that we provide: 


lab_unizar.bag -> http://webdiis.unizar.es/~jcivera/lab_unizar.bag

Update the file dpptam/src/data.yml with the follwing 'camera_path', 'cameraMatrix' and 'distCoeffs'


      camera_path:"/camera/rgb/image_color"
      cameraMatrix: !!opencv-matrix
         rows: 3
         cols: 3
         dt: d
         data: [ 520.908620, 0., 325.141442, 0., 521.007327 , 249.701764, 0., 0., 1. ]
      distCoeffs: !!opencv-matrix
         rows: 5
         cols: 1
         dt: d
         data: [ 0.231222, -0.784899, -0.003257, -0.000105, 0.917205 ]

          
          
lab_upen.bag -> http://webdiis.unizar.es/~jcivera/lab_upenn.bag

Update the file dpptam/src/data.yml with the follwing 'camera_path', 'cameraMatrix' and 'distCoeffs'
      
      
      camera_path:"/mv_25000432/image_raw"
      cameraMatrix: !!opencv-matrix
         rows: 3
         cols: 3
         dt: d
         data: [624.63,0,372.58,0,624.53,246.89,0,0,1]
      distCoeffs: !!opencv-matrix
         rows: 5
         cols: 1
         dt: d
         data: [-0.429,0.188,2.299e-06,0.00051,0]

The initialization is performed assuming that the first map is a plane parallel to the camera. It converges in a few seconds to the right solution in most of the cases. Nevertheless, we recommend to initialize in a well textured area with a relatively slow motion.

In order to exploit the benefits of the use of superpixels for low texture areas we provide the following hint: Try to 'see' the low texture area in at least 3 different views with enough parallax, so the supeprixel can be matched in multiple views. Notice that superpixels triangulation need a larger parallax than feature/photometric triangulation.

# Parameters

There are a few tuneable parameters that you can modify in dpptam/src/data.yml:

1-) Superpixel calculation

calculate_superpixels: [bool] If 1 it will calculate 3D superpixels.

2-) Number of frames for mapping

num_cameras_mapping_th: [int]. Number of frames that you want to use to estimate the depth maps. Default: 9.

3-) Minimum parallax required for mapping

translational_ratio_th_min: [double]. Minimum parallax to insert a keyframe. Default: 0.075. Typical values [0.03-0.15].

4-) Degenerated cases in 3D superpixel matching

limit_ratio_sing_val: [double]. This threshold deals with the degenerated cases in 3D superpixel calculation. Smaller values -> less outliers. Default: 100. Typical values [10-1000].

5-) Minimum normalized residual threshold required.

limit_normalized_residual: [double]. This threshold accounts for the minimum error required in superpixel calculation. Smaller values -> less outliers. Default: 0.30. Typical values [0.05-0.50].

6-) Minimum number of matches of 3D superpixels in multiple views to achieve multiview consistency.

matchings_active_search: [int]. Number of matches required of the 3D superpixel in multiple views. Larger values -> less outliers. Default: 3. Typical values [0-4].

# Contact

If you have any issue compiling/running dpptam or you would like to know anything about the code, please contact the authors:

     Alejo Concha -> aconchabelenguer@gmail.com

     Javier Civera -> jcivera@unizar.es
