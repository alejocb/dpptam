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
///dense mapping thread

///dense mapping function
void fullydense_mapping(DenseMapping *pdense_mapper,ros::Publisher *pub_cloud);
///dense mapping function

void copy_from_dense2images(DenseMapping &dense, Imagenes &images);

void get_inverse_depth(Imagenes images, cv::Mat &points,cv::Mat &inv_depths, float &depth_step, int reference_image, \
                       int discretization,float &mean_value,cv::Mat &depth_map, int set_maximo, cv::Mat &variance_points_tracked);


void calculate_superpixels_and_setdata(Imagenes &images,  int reference_image);

void calculate_3D_superpixels_from_semidense(float limit_ratio_sing_val,float limit_normalized_residual,Imagenes &images, cv::Mat &points6, int reference_image, float mean_value);

///active search of superpixels
void  active_matching(Imagenes &images,vector<SuperpixelesImagen*> &supImg, int reference_image, int superpixels_index[]);
///active search of superpixels


void transform_points(Imagenes images, int frame, cv::Mat points, cv::Mat &transformed_points);

void distances_interval(Imagenes images, int frame, cv::Mat transformed_points, cv::Mat &projected_points,\
                        float &maximo, float &minimo, float &mean_value, cv::Mat &depth_map,cv::Mat &real_points, int set_maximo,cv::Mat &variance_points_tracked);

/// reproject points into the image
void get_3Dpoints_inImage2(Imagenes &images, cv::Mat &points,cv::Mat &depth_map, int reference_image,cv::Mat &points_already_estimated);
/// reproject points into the image

///ransac to calculte 3D superpixels
void ransac_for_3Dspx(cv::Mat points_sup,cv::Mat &error_wrt_plane,cv::Mat &n, \
                      float &d, cv::Mat &singular_values, float limit_ratio_sing_val,float average_distance_btw_points, float limit_normalized_residual);
///ransac to calculte 3D superpixels

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


/// reproject points into the image
void get_3Dpoints_inImage(Imagenes &images, cv::Mat &points,cv::Mat &depth_map, int reference_image);


//// Calculating image gradients
cv::Mat gradientX(cv::Mat &mat, float spacing);
cv::Mat gradientY(cv::Mat &mat, float spacing);
//// Calculating image gradients



/// Reproject 3D countour into the image
float reprojected_contour (Imagenes &images,DataToSend &DTS, cv::Mat matchings, int i,float &percentage,\
                           float threshold, cv::Mat &contour3D);
/// Reproject 3D countour into the image

/// Transform the 3D points
void transformed_points(cv::Mat &points3D_cam, cv::Mat &R,cv::Mat &t,double fx,double fy,double cx,double cy,cv::Mat &img, cv::Mat &points3D);
void transformed_points_return_3Dpoints(cv::Mat &points3D_cam, cv::Mat &R,cv::Mat &t,double fx,double fy,double cx,double cy,cv::Mat &img, cv::Mat &points3D, cv::Mat &transformed_points);
/// Transform the 3D points

#endif
