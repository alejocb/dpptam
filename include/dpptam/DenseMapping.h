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

#ifndef __DENSEMAPPING_H
#define __DENSEMAPPING_H


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

#include <dpptam/superpixel.h>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"


class DenseMapping  :public Imagenes {
  public:
    DenseMapping();

    static boost::mutex mutex;

    cv::Mat map_sup_points_total;


    void set_do_dense(int x) { images_dense_ready = x;}
    int get_do_dense(){return images_dense_ready;}

    void init_analisis();

    vector<cv::Mat> analisis;

    int calculate_superpixels, dense_kf;


    void set_superpixels3Dtracked(cv::Mat superpixels_aux) { superpixels3Dtracked = superpixels_aux.clone();}
    cv::Mat get_superpixels3Dtracked(){return superpixels3Dtracked;}

    void set_superpixels3Dprint(cv::Mat superpixels_aux) {superpixels3Dprint = superpixels_aux.clone();}
    cv::Mat get_superpixels3Dprint(){return superpixels3Dprint;}

    void set_last3Dpoints(cv::Mat last_3Dpoints_aux) { last_3Dpoints = last_3Dpoints_aux.clone();}
    cv::Mat get_last3Dpoints(){return last_3Dpoints;}

    void init_points3D4spx(int);

    vector<cv::Mat> points3D4spx;

    int num_kf_for_dense,num_kf_for_dense_limit;


    int superpixels_print_number;

    float limit_ratio_sing_val;
    float limit_normalized_residual;
    int matchings_active_search;

private:

    cv::Mat superpixels3Dtracked,last_3Dpoints;
    cv::Mat  superpixels3Dprint;
    int images_dense_ready;
};





void print_poses(cv::Mat &points, char buffer[],int color);

///dense mapping thread
void ThreadDenseMapper(DenseMapping *pdense_mapper, ros::Publisher *pub_cloud);

///dense mapping function
void fullydense_mapping(DenseMapping *pdense_mapper,ros::Publisher *pub_cloud);

void copy_from_dense2images(DenseMapping &dense, Imagenes &images);

void get_inverse_depth(Imagenes images, cv::Mat &points,cv::Mat &inv_depths, float &depth_step, int reference_image, \
                       int discretization,float &mean_value,cv::Mat &depth_map, int set_maximo, cv::Mat &variance_points_tracked);


void calculate_superpixels_and_setdata(Imagenes &images,  int reference_image);

void calculate_3D_superpixels_from_semidense(float limit_ratio_sing_val,float limit_normalized_residual,Imagenes &images, cv::Mat &points6, int reference_image, float mean_value);

///active search of superpixels
void  active_matching(Imagenes &images,vector<SuperpixelesImagen*> &supImg, int reference_image, int superpixels_index[]);

/// photometric reprojection error calculation for every pixel
void get_photometric_errors_matrix_fd(Imagenes  &images,  cv::Mat &inv_depths, photometric_term &X, \
                                      int reference_image, cv::Mat &initial_inv_depth , int image_to_be_added, \
                                     photometric_term &points_i_todos,cv::Mat &points_ref_im_sd,
                                      int discretization, \
                                      int window_size, float ph_error_limit);

///depth map regularization using variatinal methods (not used)
void variational_mapping(Imagenes &images, cv::Mat &prior_points, int reference_image, cv::Mat initial_inv_depth, \
        float &depth_step, cv::Mat &inv_depths, photometric_term &X, cv::Mat &Dtam_points_to_print,\
             int imsize_y, int imsize_x,  cv::Mat &input, cv::Mat &image_points_byFocal,\
            float lambda_factor, int discretization, int &fast_mapping,cv::Mat &t_r_ref,cv::Mat &dense_uncertainty, \
                         int &superpixels_number, cv::Mat &planar_prior);

void prepare_denseMapKeyframe(cv::Mat &points, cv::Mat &Intensity, char buffer[],cv::Mat &dense_uncertainty,cv::Mat &points_semidense_variational);


void transform_points(Imagenes images, int frame, cv::Mat points, cv::Mat &transformed_points);

void distances_interval(Imagenes images, int frame, cv::Mat transformed_points, cv::Mat &projected_points,\
                        float &maximo, float &minimo, float &mean_value, cv::Mat &depth_map,cv::Mat &real_points, int set_maximo,cv::Mat &variance_points_tracked);

/// reproject points into the image
void get_3Dpoints_inImage2(Imagenes &images, cv::Mat &points,cv::Mat &depth_map, int reference_image,cv::Mat &points_already_estimated);

///ransac to calculte 3D superpixels
void ransac_for_3Dspx(cv::Mat points_sup,cv::Mat &error_wrt_plane,cv::Mat &n, \
                      float &d, cv::Mat &singular_values, float limit_ratio_sing_val,float average_distance_btw_points, float limit_normalized_residual);

void backproject_from_plane(Imagenes &images,cv::Mat &pixels,cv::Mat &n1,float &d1,cv::Mat &inv_depth_mat_total,cv::Mat &X_total, int reference_image);

cv::Mat create_matrix(cv::Mat contour4, int limit, int imsize_x, int imsize_y);

struct DataToSend
{           cv::Mat matchings;
            float RefIm;
            float RefSup;
            bool showim;
            float percentage_limit;
            float step_size ;
            vector<SuperpixelesImagen*> SupImg;
};

float montecarlo_seed10(Imagenes &images,int a1, int b1, int c1, int d1,DataToSend DTS1 , int iterations, \
                        cv::Mat &contour3D);

void  calculate_prior(Imagenes images, int frame, cv::Mat transformed_points, cv::Mat &projected_points,cv::Mat &prior,\
                      cv::Mat &lambda, cv::Mat &dense_uncertainty, int &superpixels_number, cv::Mat &planar_prior);

///depth map regularization using variatinal methods (not used)
void inverse_depth_optimization( cv::Mat input, cv::Mat initial_inv_depth, photometric_term X, float &depth_step, \
                                 cv::Mat &estimated_inv_depth, cv::Mat &inv_depths, cv::Mat prior, cv::Mat lambda2, \
                                  float lambda_factor, int imsize_y, int imsize_x, float reducir,\
                                 float tetha,float beta,float alpha_limit, int discretization,int superpixels_number, cv::Mat &planar_prior);

/// reproject points into the image
void get_3Dpoints_inImage(Imagenes &images, cv::Mat &points,cv::Mat &depth_map, int reference_image);



void resize_data(photometric_term &X ,cv::Mat &input,\
                 cv::Mat &estimated_inv_depth,cv::Mat &initial_inv_depth,cv::Mat &prior,cv::Mat &lambda,float reducir, int imsize_y,int imsize_x, float &lambda_factor, int discretization);

cv::Mat gradientX(cv::Mat &mat, float spacing);

cv::Mat gradientY(cv::Mat &mat, float spacing);

inline void centered_gradients_d(cv::Mat &grad_dx, cv::Mat &grad_dy, cv::Mat &grad_im_x, cv::Mat &grad_im_y, cv::Mat &d, int &i, int &pixels_cols);

void centered_gradients_q(cv::Mat &grad_qx, cv::Mat &grad_qy, cv::Mat &grad_im_x, \
                          cv::Mat &GAq, cv::Mat &grad_im_y, cv::Mat &qx, cv::Mat &qy, int &i, int &pixels_cols);

inline void sampling_a_inv_ind(cv::Mat &a,cv::Mat &X_ph_error,float &inv_depth_l,\
                float &error,float &tetha,float &d_i, cv::Mat &errores,cv::Mat &depth_aux,\
                       const int &i,int &l, cv::Mat &lambda_matrix);

float reprojected_contour (Imagenes &images,DataToSend &DTS, cv::Mat matchings, int i,float &percentage,\
                           float threshold, cv::Mat &contour3D);
/// Transform the 3D points
void transformed_points(cv::Mat &points3D_cam, cv::Mat &R,cv::Mat &t,double fx,double fy,double cx,double cy,cv::Mat &img, cv::Mat &points3D);

void transformed_points_return_3Dpoints(cv::Mat &points3D_cam, cv::Mat &R,cv::Mat &t,double fx,double fy,double cx,double cy,cv::Mat &img, cv::Mat &points3D, cv::Mat &transformed_points);

#endif
