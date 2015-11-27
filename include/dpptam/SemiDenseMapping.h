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

#ifndef __SEMIDENSEMAPPING_H
#define __SEMIDENSEMAPPING_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>
#include <stdio.h>
#include <stack>
#include <ctime>

//chadir
#include <unistd.h>
// reading a text file
#include <iostream>
#include <fstream>
#include <string>
//directorio
#include <dirent.h>
#include <ros/ros.h>

//#include "superpixel.h"
#include <dpptam/DenseMapping.h>
#include <dpptam/SemiDenseTracking.h>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"


#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>


// TIC - TOC
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <math.h>
/*#define U_SEGS(a)\
         gettimeofday(&tv,0);\
         a = tv.tv_sec + tv.tv_usec/1000000.0*/


class SemiDenseMapping  :public Imagenes {
  public:
    SemiDenseMapping();

    float mean_depth_value;

    float convergence,convergence_total;

    int previous_images;
    int do_initialization, do_initialization_tracking;
    int do_optimization;
    int do_var_mapping;


    pcl::PointCloud<pcl::PointXYZRGB> map_pcl;


    int num_cameras_mapping;
    int init_keyframes;

    int last_frame_mapped;
    int last_frame_tracked;
    double translational_ratio_th_min,translational_ratio_th_min_aux;
    double translational_ratio_th;
    double translational_ratio;

    float overlap_tracking;

    int images_size;

    int num_cameras_mapping_th,num_cameras_mapping_th_aux;
    int num_keyframes;

    int do_init_semi;

    photometric_term X;
    photometric_term X_gradient_Magnitude;
    photometric_term X_gx_ex;
    photometric_term X_gy_ey;

    cv::Mat G_expanded;


    cv::Mat GX,GY,max_inv_depth_initial_seed, min_inv_depth_initial_seed;

    cv::Mat points_by_depth ;

    photometric_term point_limits_for_sd;

    cv::Mat points_ref_im_sd;
    cv::Mat t_r_ref;
    cv::Mat initial_inv_depth_sd;
    vector<cv::Mat>  initial_inv_depth_inEveryCamera_uncertainty;
    vector<cv::Mat>  initial_inv_depth_inEveryCamera_largeParallax,previous_or_next_frame;
    cv::Mat image_points_byFocal_sd;

    cv::Mat pixel_taken;

    int kinect_initialization;
    float limit_grad;


    float mean_value;
    float depth_step;
    cv::Mat inv_depths;

    int frames_previous_keyframe_processed,frames_previous_keyframe_used;

    cv::Mat map_points;
    cv::Mat local_map_points;


    void init_points_new_map(int);
    void set_points_new_map(vector<cv::Mat>);
    vector<cv::Mat> get_points_new_map(){return points_new_map;}

    cv::Mat points_last_keyframe;

    cv::Mat points3D_tracked, weight_tracked_points, image_coordinates_tracked_points,points3D_tracked_to_map,depth_map_points_tracked,variance_points_tracked;

    cv::Mat depth_map;
    cv::Mat B,BB;
    float overlap_tracking_th;

    void set_map_points_print(cv::Mat);
    cv::Mat get_map_points_print();

    vector<cv::Mat> points3D_toprint;


private:
    vector<cv::Mat> points_new_map;

    cv::Mat map_points_print;

 protected:
    static boost::mutex mutex_p;



};

class SemiDenseTracking;  // It is defined also here due to cross reference (#include) issues

///semidesen mapper thread
void ThreadSemiDenseMapper(Imagenes *images,Imagenes *images_previous_keyframe,SemiDenseMapping *semidense_mapper,\
                           SemiDenseTracking *semidense_tracker,DenseMapping *dense_mapper,MapShared *Map, ros::Publisher *pub_cloud);

///semidense mapper function
void semidense_mapping(DenseMapping *dense_mapper,SemiDenseMapping *semidense_mapper,SemiDenseTracking *semidense_tracker,MapShared  *Map,Imagenes *pimages,Imagenes  *pimages_previous_keyframe,ros::Publisher *pub_cloud);

void copy_first_and_last_images(Imagenes &images, Imagenes &images_map);

///Calculate the photometric reprojection error of the high gradient points
void get_photometric_errors_matrix_sd_exhaustive(Imagenes  &images,  cv::Mat &inv_depths, photometric_term &X, photometric_term &X_gradient_Magnitude,\
                                      photometric_term &X_gx_ex, photometric_term &X_gy_ey, int reference_image, cv::Mat &initial_inv_depth , int image_to_be_added, \
                                      photometric_term &points_i_todos,cv::Mat &points_ref_im_sd,int discretization, \
                                      int window_size, cv::Mat &epipolar_gradients, vector<cv::Mat> &initial_inv_depth_inEveryCamera_uncertainty,vector<cv::Mat> &initial_inv_depth_inEveryCamera_largeParallax, \
                                      cv::Mat &points_by_depth, cv::Mat &t_r_ref,float &ratio_parallax, bool optimize_previous_frame,vector<cv::Mat> &previous_or_next_frame,
                                                 cv::Mat &GX, cv::Mat &GY, int &num_cameras_mapping, cv::Mat &max_inv_depth_initial_seed, cv::Mat &min_inv_depth_initial_seed);

///Filtering points of the semidense map that are not consistent temporally
void convergence_test(SemiDenseMapping *semidense_mapper,cv::Mat &be_outlier,
                        cv::Mat &be_outlier_print,cv::Mat &deviation_inv_depth,cv::Mat &final_variances,float inv_depth_disparity_th,float inv_depth_disparity_print_th);

void find_closest_maps(SemiDenseMapping *semidense_mapper,MapShared *Map,SemiDenseTracking *semidense_tracker);


void join_last_images(Imagenes *images,Imagenes *images_previous_keyframe,\
                      DenseMapping *dense_mapper,SemiDenseMapping *semidense_mapper);

void copy_imagenes_dense(Imagenes &images, Imagenes &images_map);

template <typename T>
void filter_imagenes( T &images_dense, int num_keyframes );

void copy_imagenes(Imagenes &images, Imagenes &images_map);
#endif
