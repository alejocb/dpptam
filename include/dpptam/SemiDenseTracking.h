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

#ifndef __SEMIDENSETRACKING_H
#define __SEMIDENSETRACKING_H

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
#include <image_transport/image_transport.h>


//#include "superpixel.h"
//#include <dpptam/DenseMapping.h>
#include <dpptam/SemiDenseMapping.h>
//#include <dpptam/vo_system.h>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include <tf/transform_broadcaster.h>

class SemiDenseTracking  :public Imagenes {
  public:
    SemiDenseTracking();

    //float show_tracking;
    float mean_depth_value;

    void init_local_maps(int);
    void set_local_maps(cv::Mat,int);
    vector<cv::Mat> get_local_maps(){return local_maps;}

    void init_poses_local_maps(int);
    void set_poses_local_maps(cv::Mat,int);
    vector<cv::Mat> get_poses_local_maps(){return poses_local_maps;}

    tf::TransformBroadcaster mTfBr;


    int processed_frames;
    int processed_frames_since_keyframe;

    void init_points_last_keyframes(int);
    void set_points_last_keyframes(vector<cv::Mat>);
    vector<cv::Mat> get_points_last_keyframes(){return points_last_keyframes;}

    void init_distances_local_maps(int);
    void set_distances_local_maps(float,int);
    vector<float> get_distances_local_maps(){return distances_local_maps;}

    vector<cv::Mat> points_map_inImage;
    void init_points_map_inImage(int);

    int pyramid_levels;
    int local_maps_number, local_maps_close_number;


    void init_poses();
    void set_poses(cv::Mat);
    cv::Mat poses;
    int include_inertial;


    void init_evaluation();
    void set_evaluation(cv::Mat);
    cv::Mat evaluation;

    vector<cv::Mat> jacobian;
    void init_jacobian(int);

    vector<int> reduction_pyramid;
    void init_reduction_pyramid(int);


    vector<cv::Mat> potential_image_to_track;
    void init_potential_image_to_track(int);


    vector<cv::Mat> potential_keyframe;
    void init_potential_keyframe(int);

    vector<double> focalx;
    void init_focalx(int);

    vector<double> focaly;
    void init_focaly(int);

    vector<double> centerx;
    void init_centerx(int);

    vector<double> centery;
    void init_centery(int);

    vector<cv::Mat> image_keyframe_pyramid;
    void init_image_keyframe_pyramid(int);

    vector<cv::Mat> points_map;
    void init_points_map(int);


    int frames_processed;

    vector<cv::Mat> color;
    void init_color(int);

    vector<cv::Mat> points3D;
    void init_points3D(int);

    vector<cv::Mat> image_reduced;
    void init_image_reduced(int);

    vector<cv::Mat> error_vector;
    void init_error_vector(int);

    vector<cv::Mat> weight;
    void init_weight(int);

    vector<double> variances;
    void init_variances(int);

    double variance;

    cv::Mat R,t,R_kf,t_kf,R_prev,R_post,t_prev,t_post;


    cv::Mat image_rgb,image_to_track,image_gray,image_prev,image_keyframe;
    int image_n;

    cv::Mat *image_frame;
    int *cont_frames;
    double *stamps;
     ros::Time *stamps_ros;

    int last_cont_frames;

    cv::Mat distCoeffs,cameraMatrix;
    int distortion,reduction;

    double cx,cy,fx,fy;

    float points_projected_in_image;

    double overlap_tracking;

    int create_inv_depth_discretization, last_frame_tracked;
    double tracking_th;

    int iter_th;



    int discretization;

    float depth;

    bool init_gravity{false};

private:
    vector<cv::Mat> local_maps;
    vector<cv::Mat> poses_local_maps;
    vector<cv::Mat> points_last_keyframes;
    vector<float> distances_local_maps;
};


class SemiDenseMapping;  // It is defined also here due to cross reference (#include) issues

///Semidense tracker thread
void ThreadSemiDenseTracker(Imagenes *images,SemiDenseMapping *semidense_mapper,\
                            SemiDenseTracking *semidense_tracker,DenseMapping *dense_mapper,MapShared *Map,ros::Publisher *odom_pub,ros::Publisher *pub_poses,ros::Publisher *vis_pub,image_transport::Publisher *pub_image);
///semidense_tracking function
void  semidense_tracking(Imagenes *images,SemiDenseMapping *semidense_mapper,\
                         SemiDenseTracking *semidense_tracker,DenseMapping *dense_mapper,MapShared *Map,ros::Publisher *odom_pub,ros::Publisher *pub_poses, ros::Publisher *vis_pub,image_transport::Publisher *pub_image);
///prepare the image for tracking
void prepare_image(cv::Mat &image_frame, cv::Mat &image_rgb,cv::Mat &image_to_track,int &image_n,cv::Mat &image_gray,cv::Mat cameraMatrix,cv::Mat distCoeffs,\
                   double &fx,double &fy, double &cx, double &cy, int distortion, int reduction);
///initialize semidense map to be tracked
void initialization_sd(SemiDenseTracking *semidense_tracker,cv::Mat &R,cv::Mat &t,cv::Mat &R1,cv::Mat &t1,cv::Mat &image_rgb,cv::Mat &image_keyframe,\
                    int &pyramid_levels,vector<int> &reduction_pyramid, vector<cv::Mat> &image_keyframe_pyramid, int reduction);
///first semidense map initialization
void initialization(cv::Mat &R,cv::Mat &t,cv::Mat &R1,cv::Mat &t1,cv::Mat &image_rgb,cv::Mat &image_keyframe,\
                    int &pyramid_levels,vector<int> &reduction_pyramid, vector<double> &focalx,vector<double> &focaly,vector<cv::Mat> &image_keyframe_pyramid,\
                    vector<cv::Mat> &points_map,vector<cv::Mat> &color,vector<cv::Mat> &points3D,\
                    vector<cv::Mat> &jacobian,vector<cv::Mat> &error_vector,vector<cv::Mat> &weight, double fx,double fy, double depth,\
                    cv::Mat depth_frame, int kinect_initialization,\
                    double cx,double cy,  vector<double> &centerx,vector<double> &centery, int reduction,cv::Mat image_gray,float limit_grad  );

/// remove points whose reprojection lies outside of the image during camera pose optimization
void remove_outside_points(vector<cv::Mat> &points_map,cv::Mat R,cv::Mat t,vector<cv::Mat> &image_reduced,cv::Mat R_p,cv::Mat t_p,int pyramid_levels,\
                vector<double> &focalx,vector<double> &focaly,vector<double> &centerx,vector<double> &centery,
                           vector<cv::Mat> &image_keyframe_pyramid,vector<cv::Mat> &weight,vector<cv::Mat> &variances,double &variance,cv::Mat &points3D_cam_to_print);

void get_color (cv::Mat &points,cv::Mat &color);

/// constant velocity motion model
void motion_model(vector<cv::Mat> &points_map,cv::Mat &R,cv::Mat &t,cv::Mat R_rel,cv::Mat t_rel,\
                vector<double> &focalx, vector<double> &focaly, vector<double> &centerx, vector<double> &centery,
                  vector<cv::Mat> &image_keyframe_pyramid,int pyramid_levels, bool &good_seed);

/// function to estimate the pose of the current frame
void optimize_camera(int num_keyframes,SemiDenseTracking *semidense_tracker,SemiDenseMapping *semidense_mapper,Imagenes &images,cv::Mat &image_to_track,\
                    cv::Mat &image_rgb,cv::Mat &R,cv::Mat &t,cv::Mat &R1,cv::Mat &t1,\
                    vector<cv::Mat> &image_reduced, vector<cv::Mat> &image_keyframe_pyramid, double &variance,\
                    int &reduction, vector<int> &reduction_pyramid,
                    int &processed_frames, vector<cv::Mat> &jacobian, vector<cv::Mat> &points_map,\
                    vector<cv::Mat> &color,vector<cv::Mat> &points3D,vector<cv::Mat> &error_vector,vector<cv::Mat> &weight,vector<double> &focalx,vector<double> &focaly,\
                    vector<double> &centerx,vector<double> &centery,int &pyramid_levels, double &overlap_tracking,double &tracking_th, int iter_th,\
                    vector<double> &variances,cv::Mat &image_gray, double stamps,  ros::Time stamps_ros,
                     float mean_depth_value,ros::Publisher *odom_pub,ros::Publisher *pub_poses,ros::Publisher *vis_pub,cv::Mat &points3D_cam_to_print);




/// show the reprojection of the map into the current frame
void show_error1( cv::Mat points3D_cam, cv::Mat image,  cv::Mat image_print,int num_pixels_sd2project,image_transport::Publisher *pub_image,cv::Mat &weight);

void print_evaluation(cv::Mat points,  char buffer[]);

void print_plane(cv::Mat &points, char buffer[]);

void resize_points(cv::Mat  &points2,cv::Mat  &points, double reduction,cv::Mat &image_p, int kinect_initialization,float limit_grad);

/// compute error of the reprojection of the map into the current frame
void compute_error( cv::Mat &points3D_cam, cv::Mat image, cv::Mat &color, cv::Mat &error_vector,double variance, cv::Mat &error_vector_sqrt,cv::Mat &error_check,cv::Mat &weight);

/// compute error of the reprojection of the map into the current frame using the inverse compositional approach
void compute_error_ic( cv::Mat &points3D_cam,cv::Mat &points3D_cam_p, cv::Mat &image,cv::Mat &image_p, \
                       cv::Mat &error_vector,double &variance, cv::Mat &error_vector_sqrt,double &error, cv::Mat &weight, cv::Mat &color_p);
/// gauss newton optimization using inverse compotional approach
void gauss_newton_ic(SemiDenseTracking *semidense_tracker, cv::Mat &points3D_cam_to_print,cv::Mat &R,cv::Mat &t,\
                     cv::Mat &R_p,cv::Mat &t_p,double fx,double fy,cv::Mat &points,cv::Mat &img,cv::Mat &img_p, \
                     double &error_opt,cv::Mat &color,double variance,cv::Mat &points3D\
                    ,cv::Mat &error_vector_opt, int initial_iteration,cv::Mat &jacobian, cv::Mat &init ,\
                     cv::Mat &weight,cv::Mat &points3D_cam_p,cv::Mat &points3D_cam,\
                     cv::Mat &error_vector_inicial, cv::Mat &error_vector_sqrt_inicial, float &has_decreased,cv::Mat &color_p,\
                     int &processed_frames,cv::Mat &jacobian1k, double &overlap,double cx,double cy);
///estimation of the pose of the camera using gauss newton
void  gauss_estimation(SemiDenseTracking *semidense_tracker,cv::Mat &points3D_cam_to_print,cv::Mat &R2,cv::Mat &t2,cv::Mat &R_p,cv::Mat &t_p,double &fx,double &fy,double &cx,double &cy,cv::Mat & points, \
                       cv::Mat &img,cv::Mat &img_p,double &error_p,cv::Mat &color,int &iter, float variance,cv::Mat &points3D,\
                       cv::Mat &error_vector_opt,cv::Mat &weight, int &processed_frames,cv::Mat &jacobian1k, double &overlap_tracking,
                       double &tracking_th, int iter_th);

void compute_error_ic_ni( cv::Mat &points3D_cam,cv::Mat &points3D_cam_p, cv::Mat &image,cv::Mat &image_p, \
                       cv::Mat &error_vector,double &variance, cv::Mat &error_vector_sqrt,double &error, cv::Mat &weight, cv::Mat color_p, double &overlap);

void transformed_points_return_3Dpoints(cv::Mat &points3D_cam, cv::Mat &R,cv::Mat &t,double fx,double fy,double cx,double cy,cv::Mat &img, cv::Mat &points3D, cv::Mat &transformed_points);
void resize_points(cv::Mat  &points2,cv::Mat  &points, double reduction,cv::Mat &image_p, int kinect_initialization,float limit_grad);


void exp_SO3(cv::Mat &R, cv::Mat &w);

void  w2V(cv::Mat &V,double w1,double w2,double w3,cv::Mat &logR);

void exp_SE3 (cv::Mat &R,cv::Mat &t,cv::Mat &w,cv::Mat &v);

void log_SO3 (cv::Mat &R,cv::Mat &w);

void skew_SO3( cv::Mat &wx, cv::Mat &w);

void inv_skew_SO3( cv::Mat &wx, cv::Mat &w);

void J_r_SO3(cv::Mat &J_r,cv::Mat &w);


void euler2quaternion(float &yaw, float &pitch, float &roll, float &x, float &y, float &z, float &w);

void decomposeR(cv::Mat &R, float &yaw, float &pitch, float &roll);

void print_poses(cv::Mat &points, char buffer[]);

cv::Mat MatToQuat(cv::Mat &Rot);

/// join several 3D maps to make the tracking of the camera more robust to scale drift
void join_maps(vector<cv::Mat> &points_map,cv::Mat R,cv::Mat t,vector<cv::Mat> point_clouds,int pyramid_levels,\
                vector<double> &focalx, vector<double> &focaly, vector<double> &centerx, vector<double> &centery,   \
              vector<cv::Mat> &image_keyframe_pyramid, float  &points_projected_in_image);

void print_times(cv::Mat &points, char buffer[]);

void     publish_camera_frame(cv::Mat R,cv::Mat t,ros::Publisher *vis_pub);

#endif
