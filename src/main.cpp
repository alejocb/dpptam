/**
* This file is part of DPPTAM.
*
* Copyright (C) 2015 Alejo Concha Belenguer <alejocb at unizar dot es> (University of Zaragoza)
* and Javier Civera Sancho   <jcivera at unizar dot es> (University of Zaragoza)
*
* DPPTAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DPPTAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DPPTAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>
#include <stdio.h>
#include <stack>
#include <ctime>


#include <dpptam/vo_system.h>


#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"

//#include "stdafx.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>


#define _USE_MATH_DEFINES

/////ROS IMAGE SUBSCRIBER
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>


#include <fstream>
#include <iomanip>    // Needed for stream modifiers fixed and set precision


#include <Eigen/Dense>
using namespace Eigen;
#include <opencv2/core/eigen.hpp>


//using namespace CVD;
using namespace std;

//chadir
#include <unistd.h>
// reading a text file
#include <iostream>
#include <fstream>
#include <string>
//directorio
#include <dirent.h>

#include <boost/thread/thread.hpp>
#include <iostream>
#include <stdio.h>
#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include<sensor_msgs/Imu.h>


#include <thread>


//pragma omp
#include <omp.h>
#pragma omp

//// file storage
#include <time.h>
#include "opencv2/opencv.hpp"



 int main(int argc, char** argv)
 {

     /*char cwd[1024];
        if (getcwd(cwd, sizeof(cwd)) != NULL)
            fprintf(stdout, "Current working dir: %s\n", cwd);
        else
            perror("getcwd() error");
    chdir(cwd);*/
    //chdir("/home/alejo/catkin_ws");


     ros::init(argc, argv, "camera_image");
     ros::start();

     srand ( (unsigned)time(0) );
     omp_set_dynamic(0);
     omp_set_nested(1);

    ///Launching dpptam
    vo_system vo_system_object;
    ///Launching dpptam


    ros::spin();
    ros::shutdown();
    return  0;
 }
