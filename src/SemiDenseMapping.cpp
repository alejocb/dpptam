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

#include <dpptam/SemiDenseMapping.h>
#include <dpptam/vo_system.h>
#include <ros/package.h>


#define U_SEGS(a)\
         gettimeofday(&tv,0);\
         a = tv.tv_sec + tv.tv_usec/1000000.0

double time_for_vector;
struct timeval tv;
double t1, t2,t0,t3;
void tic_init(){U_SEGS(t0);}
void toc_final(double &time){U_SEGS(t3); time =  (t3- t0)/1;}
void tic(){U_SEGS(t1);}
void toc(){U_SEGS(t2); time_for_vector = t2-t1;
           cout << (t2 - t1)/1 << endl;}


SemiDenseMapping::SemiDenseMapping():do_initialization(1),do_optimization(0), do_initialization_tracking(0), do_var_mapping(0),
    num_cameras_mapping(0), num_keyframes(0), do_init_semi(1), images_size(0), overlap_tracking(1),
    frames_previous_keyframe_processed(0),frames_previous_keyframe_used(0),convergence(1),convergence_total(0)
{
    cv::FileStorage  fs2( (ros::package::getPath("dpptam")+"/src/data.yml").c_str(), cv::FileStorage::READ);


    int pyramid_levels = 3;

    SemiDenseMapping::init_points_new_map(pyramid_levels);

    points3D_toprint.resize(5000);


    num_cameras_mapping_th = (int)fs2["num_cameras_mapping_th"];
    previous_images = num_cameras_mapping_th/2+1+1;


    translational_ratio_th_min = (float)fs2["translational_ratio_th_min"];
    translational_ratio_th_min_aux = (double)fs2["translational_ratio_th_min"];
    num_cameras_mapping_th = (int)fs2["num_cameras_mapping_th"];
    num_cameras_mapping_th_aux = (int)fs2["num_cameras_mapping_th"];

    limit_grad = (float)fs2["limit_grad"];
    kinect_initialization = 0;
    overlap_tracking_th = (double)fs2["overlap_tracking_th"];
    overlap_tracking = 1;

    int discretization = (int)fs2["discretization"];
    for (int l=0; l<discretization; l++)
    {
        X.computeError();
        X_gradient_Magnitude.computeError();
        X_gx_ex.computeError();
        X_gy_ey.computeError();
    }

    last_frame_mapped  = 0;

    cv::Mat map_points_aux(0,6,CV_64FC1);
    cv::Mat local_map_points_aux(0,6,CV_64FC1);
    local_map_points = local_map_points_aux.clone();
    map_points = map_points_aux.clone();

    cv::Mat inv_depths_aux(discretization,1, CV_32FC1);
    inv_depths = inv_depths_aux.clone();
    translational_ratio = 0;

    init_keyframes = 12;

    fs2.release();
}



void SemiDenseMapping::init_points_new_map(int pyramid_levels)
{
    points_new_map.resize(pyramid_levels);
}


void SemiDenseMapping::set_points_new_map (vector<cv::Mat> points_new_map_aux)
{
    for (int i = 0; i < points_new_map.size();i++)
    {
        points_new_map[i] = points_new_map_aux[i].clone();
    }
}



void SemiDenseMapping::set_map_points_print(cv::Mat map_points_print_aux)
{
    map_points_print = map_points_print_aux.clone();
}
cv::Mat SemiDenseMapping::get_map_points_print()
{
    return map_points_print;
}



void ThreadSemiDenseMapper(Imagenes *images,Imagenes *images_previous_keyframe,SemiDenseMapping *semidense_mapper,\
                           SemiDenseTracking *semidense_tracker,DenseMapping *dense_mapper,MapShared *Map, ros::Publisher *pub_cloud)
{
    while(ros::ok())
    {
        boost::this_thread::sleep(boost::posix_time::milliseconds(1));

        semidense_mapper->images_size = images->getNumberOfImages();

        bool insert_frame4mapping = false;


        if ( semidense_mapper->do_initialization_tracking < 0.5 &&  semidense_mapper->images_size  > 1 )
        {insert_frame4mapping = true;}


        if (semidense_mapper->num_cameras_mapping >semidense_mapper->num_cameras_mapping_th +1)
        { semidense_mapper->do_var_mapping = 1;}


        if (     (semidense_mapper->frames_previous_keyframe_used < images_previous_keyframe->getNumberOfImages()-1)   &&  \
                (semidense_mapper->frames_previous_keyframe_processed < semidense_mapper->previous_images) &&
                 images_previous_keyframe->getNumberOfImages()-1-semidense_mapper->frames_previous_keyframe_used > 0 \
              && (semidense_mapper->do_initialization_tracking < 0.5) && ( semidense_mapper->images_size  > 1))
        {insert_frame4mapping = true;}


        if (insert_frame4mapping || semidense_mapper->do_var_mapping == 1)
        {
            semidense_mapping(dense_mapper,semidense_mapper,semidense_tracker,Map,images,images_previous_keyframe,pub_cloud);
        }

    }
}



void semidense_mapping(DenseMapping *dense_mapper,SemiDenseMapping *semidense_mapper,SemiDenseTracking *semidense_tracker,MapShared  *Map,Imagenes *pimages,Imagenes  *pimages_previous_keyframe,ros::Publisher *pub_cloud)
{
                Imagenes images;
                copy_first_and_last_images(*pimages,images);

                float translational_ratio_th_min = semidense_mapper-> translational_ratio_th_min;

                cv::Mat points6;
                cv::Mat points6_aux ;
                if (semidense_mapper->points3D_toprint[semidense_mapper->num_keyframes].rows > 20)
                {points6_aux =  semidense_mapper -> points3D_toprint[semidense_mapper->num_keyframes].clone();}
                if (semidense_mapper->num_keyframes > 2 && semidense_mapper->points3D_toprint[semidense_mapper->num_keyframes-1].rows > 20)
                {points6_aux.push_back(semidense_mapper -> points3D_toprint[semidense_mapper->num_keyframes-1]);}
                if (semidense_mapper->num_keyframes > 3 &&  semidense_mapper->points3D_toprint[semidense_mapper->num_keyframes-2].rows > 20)
                {points6_aux.push_back(semidense_mapper -> points3D_toprint[semidense_mapper->num_keyframes-2]);}

                int using_close_points  = 1;
                if (points6_aux.rows > 1000)
                {
                     points6 = points6_aux.clone();
                }
                else
                {
                    using_close_points = 0;
                     points6 = semidense_mapper->points_last_keyframe.clone();
                }

                float limit_grad = semidense_mapper->limit_grad;
                int reference_image = 0;

                int num_keyframes = semidense_mapper->num_keyframes;

                int corner =  5;
                int discretization = 100;


                cv::Mat R1,R2,C1,C2,t1,t2;

                R1 = images.Im[reference_image]->R;
                R2 = images.Im[images.getNumberOfImages()-1]->R;
                t1 = images.Im[reference_image]->t;
                t2 = images.Im[images.getNumberOfImages()-1]->t;

                C1 = -R1.t()*t1;
                C2 = -R2.t()*t2;


                C2.convertTo(C2,CV_64FC1);
                C1.convertTo(C1,CV_64FC1);

                semidense_mapper -> translational_ratio =(fabs(C1.at<double>(0,0) - C2.at<double>(0,0)) + fabs(C1.at<double>(1,0) - C2.at<double>(1,0)) +
                                       fabs(C1.at<double>(2,0) - C2.at<double>(2,0)) )  / semidense_mapper-> mean_value;

                float deviation_inv_depth_print_th = 0.30;
                float neighboors_consistency_print = 0.93;
                float inv_depth_disparity_print_th = 1.0;



                float inv_depth_disparity_th = 1.0;
                float deviation_inv_depth_th = 0.20;
                float spatial_threshold = 0.8;
                spatial_threshold = 1.9;
                inv_depth_disparity_th = 1.9;
                semidense_mapper->num_cameras_mapping_th=9;

               if (semidense_mapper->num_keyframes <  semidense_mapper -> init_keyframes+1)
                {
                   semidense_mapper->num_cameras_mapping_th=7;
                   translational_ratio_th_min = 0.03;
                }
               semidense_mapper->previous_images = semidense_mapper->num_cameras_mapping_th/2+1+1;


                int alternate = 1;
                bool optimize_previous_frame  = false;

                //
                if(semidense_mapper-> num_keyframes > semidense_mapper -> init_keyframes)
                {alternate=1;}


                if ( (semidense_mapper-> num_cameras_mapping <  semidense_mapper->previous_images && ( alternate == 1) ) &&      (semidense_mapper->frames_previous_keyframe_used < pimages_previous_keyframe->getNumberOfImages()-1)   &&  \
                         pimages_previous_keyframe->getNumberOfImages()-1-semidense_mapper->frames_previous_keyframe_used > 0 )
                {
                    optimize_previous_frame = true;
                    for (int ii = 0;ii < 2;ii++)
                    {
                            images.computeImage();
                            int current_images_size = images.getNumberOfImages()-1;
                            images.Im[current_images_size]->image = pimages_previous_keyframe->Im[ pimages_previous_keyframe->getNumberOfImages()-1-semidense_mapper->frames_previous_keyframe_used]->image.clone();
                            images.Im[current_images_size]->R = pimages_previous_keyframe->Im[ pimages_previous_keyframe->getNumberOfImages()-1-semidense_mapper->frames_previous_keyframe_used]->R.clone();
                            images.Im[current_images_size]->image_gray=pimages_previous_keyframe->Im[ pimages_previous_keyframe->getNumberOfImages()-1-semidense_mapper->frames_previous_keyframe_used]->image_gray.clone();
                            images.Im[current_images_size]->t = pimages_previous_keyframe->Im[ pimages_previous_keyframe->getNumberOfImages()-1-semidense_mapper->frames_previous_keyframe_used]->t.clone();
                            images.Im[current_images_size]->t_r = pimages_previous_keyframe->Im[ pimages_previous_keyframe->getNumberOfImages()-1-semidense_mapper->frames_previous_keyframe_used]->t_r.clone();
                            images.Im[current_images_size]->fx = pimages_previous_keyframe->Im[ pimages_previous_keyframe->getNumberOfImages()-1-semidense_mapper->frames_previous_keyframe_used]->fx;
                            images.Im[current_images_size]->fy = pimages_previous_keyframe->Im[ pimages_previous_keyframe->getNumberOfImages()-1-semidense_mapper->frames_previous_keyframe_used]->fy;
                            images.Im[current_images_size]->cx = pimages_previous_keyframe->Im[ pimages_previous_keyframe->getNumberOfImages()-1-semidense_mapper->frames_previous_keyframe_used]->cx;
                            images.Im[current_images_size]->cy = pimages_previous_keyframe->Im[ pimages_previous_keyframe->getNumberOfImages()-1-semidense_mapper->frames_previous_keyframe_used]->cy;
                            images.Im[current_images_size]->error = pimages_previous_keyframe->Im[ pimages_previous_keyframe->getNumberOfImages()-1-semidense_mapper->frames_previous_keyframe_used]->error;
                            images.Im[current_images_size]->k1 = pimages_previous_keyframe->Im[ pimages_previous_keyframe->getNumberOfImages()-1-semidense_mapper->frames_previous_keyframe_used]->k1;
                            images.Im[current_images_size]->k2 = pimages_previous_keyframe->Im[ pimages_previous_keyframe->getNumberOfImages()-1-semidense_mapper->frames_previous_keyframe_used]->k2;
                    }
                    semidense_mapper->frames_previous_keyframe_used++;
                }

                int images_size = images.getNumberOfImages()-1;


                int init_mapping = images_size-1;
                int end_mapping = init_mapping+1;


                int  regularization_size = 2;
                ////////////////////////////GET PHOTOMETRIC TERM_SD term_sd ///////////////////////////////////////
                    if (semidense_mapper->do_init_semi > 0.5 )
                    {


                        cv::Mat points_ref_im_sd_init(0,3, CV_32FC1);
                        cv::Mat image_points_byFocal_sd_init(0,3, CV_32FC1);

                        semidense_mapper->points_ref_im_sd = points_ref_im_sd_init.clone();
                        semidense_mapper-> image_points_byFocal_sd = image_points_byFocal_sd_init.clone();

                        semidense_mapper->do_init_semi=0;
                        semidense_mapper->depth_map = semidense_mapper->depth_map*0;

                        cv::Mat inv_depths_borrar = semidense_mapper-> inv_depths.clone();
                        cv::Mat depth_map_points_tracked = semidense_mapper->depth_map*0;
                        cv::Mat variance_points_tracked = semidense_mapper->depth_map*0;
                        cv::Mat points3D_tracked = semidense_mapper->points3D_tracked.clone();
                        cv::Mat points_aux(0,points3D_tracked.cols,CV_32FC1);

                        for (int i = 0;i<points3D_tracked.rows;i++)
                        {
                            if ( semidense_mapper->weight_tracked_points.at<double>(i,0)<0.06)
                            {
                                points_aux.push_back(points3D_tracked.row(i));
                            }
                        }


                        points_aux.convertTo(points_aux, CV_32FC1);
                        get_inverse_depth(images,points_aux,inv_depths_borrar,semidense_mapper-> depth_step,
                                          reference_image,discretization,semidense_mapper-> mean_value,depth_map_points_tracked,1,variance_points_tracked);
                        points_aux.convertTo(points_aux, CV_64FC1);


                        semidense_mapper -> depth_map_points_tracked = depth_map_points_tracked.clone();
                        semidense_mapper -> variance_points_tracked = variance_points_tracked.clone();


                        points6.convertTo(points6, CV_32FC1);
                        get_inverse_depth(images,points6,semidense_mapper-> inv_depths,semidense_mapper-> depth_step,reference_image,
                                          discretization,semidense_mapper-> mean_value,semidense_mapper->depth_map,using_close_points,variance_points_tracked);
                        points6.convertTo(points6, CV_64FC1);


                        cv::Mat gray_image = images.Im[reference_image]->image_gray.clone();

                        gray_image.convertTo(gray_image,CV_32FC1);
                        cv::Mat GX =gradientX(gray_image,1);
                        cv::Mat GY =gradientY(gray_image,1);

                        cv::Mat G = cv::abs(GX)  + cv::abs(GY);

                        float alpha = 0.05;
                        cv::exp(-G*alpha,G);


                        cv::Mat G_reshaped = G.clone();
                        G_reshaped = G_reshaped.reshape(0,G_reshaped.rows*G_reshaped.cols);
                        cv::sort(G_reshaped,G_reshaped,CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);


                        float limit_grad_aux = G_reshaped.at<float>(25000,0);
                         if (limit_grad_aux < limit_grad){limit_grad = limit_grad_aux;}

                        cv::Mat G_expanded = G+1;
                        for (int i=corner; i<G.rows-corner ; i++)
                        {
                            for (int j=corner; j < G.cols-corner;j++)
                            {
                                if (G.at<float>(i,j) < limit_grad)
                                {
                                    G_expanded.at<float>(i,j) =   limit_grad-0.1;
                                }
                            }
                        }

                        float num_potential_points = 0;

                        //// REMOVE POINTS WHICH HAVE ONLY A FEW NEIGHBOORS
                        for (int i = corner; i < images.Im[reference_image]->image.rows-corner; i++)
                        {
                            for (int j = corner; j < images.Im[reference_image]->image.cols-corner; j++)
                            {
                                    if (G.at<float>(i,j) <  limit_grad)
                                    {
                                            int cont_neighbours = 0;
                                            for (int ii = i-1; ii <= i+1; ii++)
                                            {
                                                for (int jj = j-1; jj <= j+1; jj++)
                                                {
                                                        cont_neighbours++;
                                                        G_expanded.at<float>(ii,jj)  =  limit_grad-0.1;
                                                }
                                            }
                                            num_potential_points++;
                                      }
                            }
                        }



                        semidense_mapper->G_expanded = G_expanded.clone();

                        semidense_mapper->B = G < limit_grad;
                        semidense_mapper->BB = G < limit_grad;

                        if (semidense_mapper-> num_keyframes >  semidense_mapper -> init_keyframes)
                        {
                            semidense_mapper->BB= cv::abs(depth_map_points_tracked) > 0  ;
                        }

                        semidense_mapper->B.convertTo(semidense_mapper->B,CV_32FC1);
                        semidense_mapper->BB.convertTo(semidense_mapper->BB,CV_32FC1);


                        cv::Mat points_i_sd(0,3, CV_32FC1);
                        cv::Mat point_i_sd(1,3, CV_32FC1);
                        cv::Mat point_ref_im_sd(1,3, CV_32FC1);


                        for (int i = corner; i < images.Im[reference_image]->image.rows-corner; i++)
                        {
                            for (int j = corner; j < images.Im[reference_image]->image.cols-corner; j++)
                            {

                                if (semidense_mapper->B.at<float>(i,j) > 100 && (semidense_mapper->BB.at<float>(i,j) < 100 ||semidense_mapper->num_keyframes <  semidense_mapper -> init_keyframes+1 ))
                                {
                                    point_i_sd.at<float>(0,0) = (images.Im[reference_image]->cx-j)/images.Im[reference_image]->fx;
                                    point_i_sd.at<float>(0,1) = (i-images.Im[reference_image]->cy)/images.Im[reference_image]->fy;
                                    point_i_sd.at<float>(0,2) = 1;
                                    points_i_sd.push_back(point_i_sd);
                                    semidense_mapper-> image_points_byFocal_sd.push_back(point_i_sd);


                                    point_ref_im_sd.at<float>(0,0) = i;
                                    point_ref_im_sd.at<float>(0,1) = j;
                                    point_ref_im_sd.at<float>(0,2) = 1;
                                    semidense_mapper->points_ref_im_sd.push_back( point_ref_im_sd);
                                }

                            }
                        }


                        vector<cv::Mat>  initial_inv_depth_inEveryCamera_aux(points_i_sd.rows);
                        semidense_mapper-> initial_inv_depth_inEveryCamera_uncertainty= initial_inv_depth_inEveryCamera_aux;
                        semidense_mapper-> initial_inv_depth_inEveryCamera_largeParallax = initial_inv_depth_inEveryCamera_aux;

                        /// this variable 'previous_or_next_frame' indicates if the reprojection error is calculated against and old frame or a new frame
                        semidense_mapper-> previous_or_next_frame = initial_inv_depth_inEveryCamera_aux;


                        points_i_sd = points_i_sd.t();

                        semidense_mapper-> initial_inv_depth_sd = cv::Mat::zeros(points_i_sd.cols, 1, CV_32FC1) + semidense_mapper-> inv_depths.at<float>(0,0);
                        semidense_mapper-> initial_inv_depth_sd = cv::Mat::zeros(points_i_sd.cols, 1, CV_32FC1) + semidense_mapper-> inv_depths.at<float>(0,0);
                        semidense_mapper-> initial_inv_depth_sd = cv::Mat::zeros(points_i_sd.cols, 1, CV_32FC1) + semidense_mapper-> inv_depths.at<float>(0,0);
                        semidense_mapper-> max_inv_depth_initial_seed = cv::Mat::zeros(points_i_sd.cols, 1, CV_32FC1) + semidense_mapper-> inv_depths.at<float>(0,0) - 1000;
                        semidense_mapper-> min_inv_depth_initial_seed = cv::Mat::zeros(points_i_sd.cols, 1, CV_32FC1) + semidense_mapper-> inv_depths.at<float>(0,0) + 1000;

                        semidense_mapper-> t_r_ref = cv::repeat(images.Im[reference_image]->t, 1, points_i_sd.cols).clone();

                        semidense_mapper->points_by_depth = points_i_sd.clone();
                        cv::Mat points_i2_sd;
                        for (unsigned int l = 0; l<discretization;l++)
                        {
                            semidense_mapper->point_limits_for_sd.computeError();

                            if (l==0 || l == discretization-1)
                            {
                                points_i2_sd = points_i_sd / semidense_mapper-> inv_depths.at<float>(l,0);
                                points_i2_sd = images.Im[reference_image]->R.t() * (points_i2_sd - semidense_mapper-> t_r_ref);
                                semidense_mapper->point_limits_for_sd.ph_error[l] = points_i2_sd.clone();
                            }
                        }

                    }
                    ////////////////////////////GET PHOTOMETRIC TERM_SD term_sd ///////////////////////////////////////

                int keyframe_obj;
                int image_to_be_added = 0;

                float camera_translation;
                cv::Mat camera1;


                if (optimize_previous_frame == 0)
                {
                    init_mapping = 1;
                    end_mapping =  images.getNumberOfImages();
                }


                if (semidense_mapper->do_var_mapping < 0.5)
                {
                    for (int i=init_mapping; i< end_mapping ; i = i+1)
                    {

                                    keyframe_obj=i;

                                    if (images.Im[keyframe_obj]-> is_used_for_mapping == 0 && semidense_mapper->do_var_mapping == 0)
                                    {

                                            R1 = images.Im[reference_image]->R;
                                            t1 = images.Im[reference_image]->t;


                                            R2 = images.Im[keyframe_obj]->R;
                                            t2 = images.Im[keyframe_obj]->t;

                                            C1 = -R1.t()*t1;
                                            C2 = -R2.t()*t2;

                                            image_to_be_added=i;

                                            C2.convertTo(C2,CV_64FC1);
                                            C1.convertTo(C1,CV_64FC1);

                                            camera1 = C1.clone();


                                            float translational_ratio = (fabs(C1.at<double>(0,0) - C2.at<double>(0,0)) +
                                                                         fabs(C1.at<double>(1,0) - C2.at<double>(1,0)) + \
                                                                  fabs(C1.at<double>(2,0) - C2.at<double>(2,0)) )
                                                    / fabs(semidense_mapper-> mean_value);

                                            camera_translation = (fabs(C1.at<double>(0,0) - C2.at<double>(0,0)) + fabs(C1.at<double>(1,0) - C2.at<double>(1,0)) + \
                                                                  fabs(C1.at<double>(2,0) - C2.at<double>(2,0)) );




                                            //if (num_keyframes > semidense_mapper->init_keyframes)

                                            if (semidense_mapper->num_cameras_mapping < 2 && num_keyframes > semidense_mapper->init_keyframes)
                                            {
                                                translational_ratio_th_min = semidense_mapper-> translational_ratio_th_min / 4;
                                            }
                                            if (  num_keyframes >  semidense_mapper->init_keyframes)
                                            {


                                                if (semidense_mapper->num_cameras_mapping == 3 )
                                                {
                                                    translational_ratio_th_min = semidense_mapper-> translational_ratio_th_min *1.30;
                                                }
                                                if (semidense_mapper->num_cameras_mapping == 4)
                                                {
                                                    translational_ratio_th_min = semidense_mapper-> translational_ratio_th_min *1.70;
                                                }
                                                if (semidense_mapper->num_cameras_mapping == 5 )
                                                {
                                                    translational_ratio_th_min = semidense_mapper-> translational_ratio_th_min *2.0;
                                                }
                                            }


                                            if (translational_ratio > translational_ratio_th_min )
                                            {

                                                cv::Mat epipolar_gradients;

                                                int window_size = 3;

                                                float ratio_parallax = 1.5;

                                                get_photometric_errors_matrix_sd_exhaustive(images,  semidense_mapper-> inv_depths, semidense_mapper->X,  semidense_mapper->X_gradient_Magnitude,\
                                                                                  semidense_mapper->X_gx_ex,semidense_mapper->X_gy_ey,reference_image,  semidense_mapper-> initial_inv_depth_sd, image_to_be_added,  \
                                                                                  semidense_mapper->point_limits_for_sd,semidense_mapper->points_ref_im_sd,discretization,window_size,  \
                                                                                  epipolar_gradients,semidense_mapper-> initial_inv_depth_inEveryCamera_uncertainty, \
                                                                                  semidense_mapper-> initial_inv_depth_inEveryCamera_largeParallax,semidense_mapper->points_by_depth, \
                                                                                  semidense_mapper-> t_r_ref,ratio_parallax, optimize_previous_frame,semidense_mapper-> previous_or_next_frame,\
                                                                                  semidense_mapper-> GX, semidense_mapper-> GY, semidense_mapper->num_cameras_mapping,
                                                                                  semidense_mapper-> max_inv_depth_initial_seed,semidense_mapper-> min_inv_depth_initial_seed);

                                                if (optimize_previous_frame)
                                                {semidense_mapper->frames_previous_keyframe_processed++;}
                                                semidense_mapper->num_cameras_mapping++;
                                                if (semidense_mapper->num_cameras_mapping > semidense_mapper->num_cameras_mapping_th+1)
                                                {
                                                        semidense_mapper->do_var_mapping = 1;
                                                }
                                            }
                                    } // if used for mapping
                    }
                }



                if (semidense_mapper->do_var_mapping > 0.5)
                {
                    int print_depth_maps = 0;



                    keyframe_obj=init_mapping;

                    R1 = images.Im[reference_image]->R;
                    t1 = images.Im[reference_image]->t;


                    R2 = images.Im[keyframe_obj]->R;
                    t2 = images.Im[keyframe_obj]->t;

                    C1 = -R1.t()*t1;
                    C2 = -R2.t()*t2;


                    C2.convertTo(C2,CV_64FC1);
                    C1.convertTo(C1,CV_64FC1);

                    camera1 = C1.clone();



                    camera_translation = (fabs(C1.at<double>(0,0) - C2.at<double>(0,0)) + fabs(C1.at<double>(1,0) - C2.at<double>(1,0)) + \
                                          fabs(C1.at<double>(2,0) - C2.at<double>(2,0)) );


                    camera_translation = semidense_mapper->mean_value* semidense_mapper->translational_ratio_th_min;


                    semidense_mapper -> convergence = 0;
                    semidense_mapper->frames_previous_keyframe_processed = 0;
                    semidense_mapper->frames_previous_keyframe_used = 0;

                    cv::Mat deviation_inv_depth(semidense_mapper-> initial_inv_depth_sd.rows,1,CV_32FC1);
                    for (unsigned int l = 0; l < discretization; l = l+1)
                    {
                        semidense_mapper->X_gx_ex.ph_error[l] = semidense_mapper->X_gx_ex.ph_error[l]/semidense_mapper->num_cameras_mapping;
                        semidense_mapper->X_gy_ey.ph_error[l] = semidense_mapper->X_gy_ey.ph_error[l]/semidense_mapper->num_cameras_mapping;
                        semidense_mapper->X_gradient_Magnitude.ph_error[l] =semidense_mapper-> X_gradient_Magnitude.ph_error[l]/(semidense_mapper->num_cameras_mapping*50);;
                    }

                    cv::Mat gray_image = images.Im[reference_image]->image_gray.clone();
                    gray_image.convertTo(gray_image,CV_32FC1);

                    cv::Mat depth_map_points_tracked = semidense_mapper->depth_map*0;
                    cv::Mat variance_points_tracked = semidense_mapper->depth_map*0;

                    depth_map_points_tracked = semidense_mapper -> depth_map_points_tracked.clone();
                    variance_points_tracked = semidense_mapper -> variance_points_tracked.clone();

                    vector<cv::Mat> init_inv_dephts_maps_scale(2);
                    init_inv_dephts_maps_scale[0] = cv::abs(semidense_mapper->depth_map.clone());
                    init_inv_dephts_maps_scale[1] = init_inv_dephts_maps_scale[0].clone()*0;


                    cv::Mat be_outlier = semidense_mapper-> initial_inv_depth_sd.clone()*0;
                    cv::Mat be_outlier_print = semidense_mapper-> initial_inv_depth_sd.clone()*0;
                    cv::Mat final_variances = semidense_mapper-> initial_inv_depth_sd.clone()*0;

                    semidense_mapper->X.ph_error[0] = semidense_mapper->X.ph_error[0]/semidense_mapper->num_cameras_mapping;

                    convergence_test(semidense_mapper,be_outlier,be_outlier_print,deviation_inv_depth,final_variances,inv_depth_disparity_th,inv_depth_disparity_print_th);


                    cv::Mat G_expanded = semidense_mapper->G_expanded.clone();
                    ///////////////////// Median REGULARIZATION////////////////////////////////////////
                    cv::Mat depths2regularize =  cv::Mat::zeros(images.Im[reference_image]->image.rows,images.Im[reference_image]->image.cols,CV_32FC1);

                    cv::Mat points_ref_im_sd(0,3, CV_32FC1);
                    cv::Mat be_outlier_aux(0,1, CV_32FC1);
                    cv::Mat final_variances_aux(0,1, CV_32FC1);
                    cv::Mat be_outlier_print_aux(0,1, CV_32FC1);
                    cv::Mat initial_inv_depth_sd(0,1, CV_32FC1);
                    cv::Mat deviation_inv_depth_aux(0,1, CV_32FC1);
                    cv::Mat image_points_byFocal_sd(0,3, CV_32FC1);
                    cv::Mat point_i_sd(1,3, CV_32FC1);
                    cv::Mat point_ref_im_sd(1,3, CV_32FC1);

                    int counter_borrar = 0;

                    int cont_depths=0;

                    for (int regularization_times = 0; regularization_times<1;regularization_times++)
                    {

                        for (int i = corner; i < images.Im[reference_image]->image.rows-corner; i++)
                        {
                            for (int j = corner; j < images.Im[reference_image]->image.cols-corner; j++)
                            {

                                if (semidense_mapper->B.at<float>(i,j) > 100 && (semidense_mapper->BB.at<float>(i,j) < 100 ||semidense_mapper->num_keyframes <  semidense_mapper -> init_keyframes +1 ))
                                {
                                    point_ref_im_sd.at<float>(0,0) = i;
                                    point_ref_im_sd.at<float>(0,1) = j;
                                    point_ref_im_sd.at<float>(0,2) = 1;
                                    points_ref_im_sd.push_back( point_ref_im_sd);

                                    point_i_sd.at<float>(0,0) = (images.Im[reference_image]->cx-j)/images.Im[reference_image]->fx;
                                    point_i_sd.at<float>(0,1) = (i-images.Im[reference_image]->cy)/images.Im[reference_image]->fy;
                                    point_i_sd.at<float>(0,2) = 1;
                                    image_points_byFocal_sd.push_back(point_i_sd);

                                    be_outlier_print_aux.push_back(be_outlier_print.at<float>(cont_depths,0));
                                    be_outlier_aux.push_back(be_outlier.at<float>(cont_depths,0));
                                    deviation_inv_depth_aux.push_back(deviation_inv_depth.at<float>(cont_depths,0));

                                    initial_inv_depth_sd.push_back(semidense_mapper-> initial_inv_depth_sd.at<float>(cont_depths,0));
                                    final_variances_aux.push_back(final_variances.at<float>(cont_depths,0));

                                    depths2regularize.at<float>(i,j)= semidense_mapper-> initial_inv_depth_sd.at<float>(cont_depths,0);
                                    cont_depths++;

                                    if ( semidense_mapper -> num_keyframes > semidense_mapper -> init_keyframes && (depth_map_points_tracked.at<float>(i,j)) < 0 )
                                    {
                                       depths2regularize.at<float>(i,j)=  1/depth_map_points_tracked.at<float>(i,j);
                                       initial_inv_depth_sd.push_back( 1/depth_map_points_tracked.at<float>(i,j));
                                    }
                                }

                                 if ( semidense_mapper->BB.at<float>(i,j) > 100 && semidense_mapper->B.at<float>(i,j) > 100 &&semidense_mapper->num_keyframes >  semidense_mapper -> init_keyframes )
                                {
                                    counter_borrar++;

                                     point_ref_im_sd.at<float>(0,0) = i;
                                     point_ref_im_sd.at<float>(0,1) = j;
                                     point_ref_im_sd.at<float>(0,2) = 1;
                                     points_ref_im_sd.push_back( point_ref_im_sd);

                                     point_i_sd.at<float>(0,0) = (images.Im[reference_image]->cx-j)/images.Im[reference_image]->fx;
                                     point_i_sd.at<float>(0,1) = (i-images.Im[reference_image]->cy)/images.Im[reference_image]->fy;
                                     point_i_sd.at<float>(0,2) = 1;
                                     image_points_byFocal_sd.push_back(point_i_sd);

                                     float bb = 0;
                                     float cc = 1;

                                     final_variances_aux.push_back(variance_points_tracked.at<float>(i,j));

                                     be_outlier_print_aux.push_back(bb);
                                     be_outlier_aux.push_back(bb);
                                     deviation_inv_depth_aux.push_back(cc);

                                     depths2regularize.at<float>(i,j)=  1/depth_map_points_tracked.at<float>(i,j);
                                     initial_inv_depth_sd.push_back( depths2regularize.at<float>(i,j));
                                }

                                if ( semidense_mapper -> num_keyframes >  semidense_mapper -> init_keyframes && (depth_map_points_tracked.at<float>(i,j)) < 0 )
                                {
                                   depths2regularize.at<float>(i,j)=  1/depth_map_points_tracked.at<float>(i,j);
                                }

                            }
                        }


                        be_outlier_print = be_outlier_print_aux.clone();
                        be_outlier = be_outlier_aux.clone();
                        semidense_mapper-> initial_inv_depth_sd = initial_inv_depth_sd.clone();
                        deviation_inv_depth = deviation_inv_depth_aux.clone();
                        semidense_mapper->points_ref_im_sd = points_ref_im_sd.clone();
                        semidense_mapper->image_points_byFocal_sd = image_points_byFocal_sd.clone();
                        final_variances = final_variances_aux.clone();

                        cont_depths = 0;


                        for (cont_depths = 0; cont_depths <semidense_mapper-> initial_inv_depth_sd.rows;cont_depths++ )
                        {

                                    int j = round(semidense_mapper->points_ref_im_sd.at<float>(cont_depths,1));
                                    int i = round(semidense_mapper->points_ref_im_sd.at<float>(cont_depths,0));

                                    float max_depth = -INFINITY;
                                    float min_depth = +INFINITY;
                                    float mean_depths = 0;
                                    float cont_depths2reg = 0;

                                        for (int ii = i-regularization_size; ii < i+regularization_size+1; ii++)
                                        {
                                            for (int jj = j-regularization_size; jj < j+regularization_size+1; jj++)
                                            {
                                                if ( fabs(depths2regularize.at<float>(ii,jj)) > 0)
                                                {
                                                    if ( fabs(gray_image.at<float>(i,j)-gray_image.at<float>(ii,jj)) < 25)
                                                    {
                                                        if (depths2regularize.at<float>(ii,jj) < min_depth)
                                                        {
                                                            min_depth =  depths2regularize.at<float>(ii,jj);
                                                        }
                                                        if (depths2regularize.at<float>(ii,jj) > max_depth)
                                                        {
                                                            max_depth =  depths2regularize.at<float>(ii,jj);
                                                        }

                                                        mean_depths+=depths2regularize.at<float>(ii,jj);
                                                        cont_depths2reg ++;
                                                    }
                                                }
                                            }
                                        }


                                        if (cont_depths2reg > 0 )
                                        {

                                            if ((    ( max_depth/min_depth) < neighboors_consistency_print  ) && regularization_times < 1  )
                                            {
                                                be_outlier_print.at<float>(cont_depths,0) = 1;
                                            }


                                            if (fabs( min_depth-max_depth) /
                                                fabs(final_variances.at<float>(cont_depths,0)) >  spatial_threshold  && regularization_times < 1  )
                                            {
                                                be_outlier.at<float>(cont_depths,0) = 1;
                                                be_outlier_print.at<float>(cont_depths,0) = 1;
                                            }

                                            {
                                                semidense_mapper-> initial_inv_depth_sd.at<float>(cont_depths,0) = mean_depths / cont_depths2reg ;
                                            }


                                        } // cont_depths2reg > 2
                                        else
                                        {
                                            be_outlier.at<float>(cont_depths,0) = 1;
                                            be_outlier_print.at<float>(cont_depths,0) = 1;
                                        }

                    }
                    }


                    ////////////////////////// Median REGULARIZATION////////////////////////////////



                    float scale = 1;
                    cv::Mat inliers_matrix = cv::Mat::zeros(images.Im[reference_image]->image.rows,images.Im[reference_image]->image.cols,CV_32FC1);
                    cv::Mat pixel_taken = cv::Mat::zeros(images.Im[reference_image]->image.rows,images.Im[reference_image]->image.cols,CV_32FC1);
                    cv::Mat pixel_taken_print = cv::Mat::zeros(images.Im[reference_image]->image.rows,images.Im[reference_image]->image.cols,CV_32FC1);


                    cv::Mat points_aux2(0,7,CV_32FC1);
                    cv::Mat points_aux2_print(0,6,CV_32FC1);



                    //cv::Mat depth_image_print = images.Im[reference_image]->image_gray.clone();

                    cv::Mat channel_r= cv::Mat::zeros(images.Im[reference_image]->image.rows, images.Im[reference_image]->image.cols, CV_8UC1);
                    cv::Mat channel_g= cv::Mat::zeros(images.Im[reference_image]->image.rows, images.Im[reference_image]->image.cols, CV_8UC1);
                    cv::Mat channel_b= cv::Mat::zeros(images.Im[reference_image]->image.rows, images.Im[reference_image]->image.cols, CV_8UC1);


                    cv::Mat channel_r1= cv::Mat::zeros(images.Im[reference_image]->image.rows, images.Im[reference_image]->image.cols, CV_8UC1);
                    cv::Mat channel_g1= cv::Mat::zeros(images.Im[reference_image]->image.rows, images.Im[reference_image]->image.cols, CV_8UC1);
                    cv::Mat channel_b1= cv::Mat::zeros(images.Im[reference_image]->image.rows, images.Im[reference_image]->image.cols, CV_8UC1);


                    double min_depth,min_depth_real;
                    double max_depth,max_depth_real;


                    if (print_depth_maps > 0.5)
                    {
                        for (int i = 0; i < images.Im[reference_image]->image.rows-0; i++)
                        {
                            for (int j = 0; j < images.Im[reference_image]->image.cols-0; j++)
                            {
                                channel_r.at<unsigned char>(i,j) = gray_image.at<float>(i,j);
                                channel_g.at<unsigned char>(i,j) = gray_image.at<float>(i,j);
                                channel_b.at<unsigned char>(i,j) = gray_image.at<float>(i,j);

                                channel_r1.at<unsigned char>(i,j) = gray_image.at<float>(i,j);
                                channel_g1.at<unsigned char>(i,j) = gray_image.at<float>(i,j);
                                channel_b1.at<unsigned char>(i,j) = gray_image.at<float>(i,j);
                            }
                        }


                        //cv::minMaxLoc(cv::abs(semidense_mapper-> initial_inv_depth_sd), &min_depth, &max_depth);
                        cv::Mat sorted_inv_depths;
                        cv::sort(cv::abs(semidense_mapper-> initial_inv_depth_sd),sorted_inv_depths,CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);

                        min_depth = abs(sorted_inv_depths.at<float>(round(sorted_inv_depths.rows/3)));
                        max_depth = abs(sorted_inv_depths.at<float>(round(2*sorted_inv_depths.rows/3)));
                    }

                    float number_of_points_estimated = 0;


                    float point_to_camera;
                    cv::Mat image_i = images.Im[reference_image]->image.clone();

                    semidense_mapper-> t_r_ref = cv::repeat(images.Im[reference_image]->t, 1, semidense_mapper-> initial_inv_depth_sd.rows).clone();


                    for (int l = 0;l<2;l++)
                    {
                           cv::Mat seed_points_to_print;
                           cv::Mat initial_inv_depth1 = semidense_mapper-> initial_inv_depth_sd.clone();

                           if (l>0)
                           {
                                semidense_mapper-> image_points_byFocal_sd.copyTo(seed_points_to_print);

                                seed_points_to_print.colRange(0,1).rowRange(0,semidense_mapper-> image_points_byFocal_sd.rows) = semidense_mapper-> image_points_byFocal_sd.colRange(0,1).rowRange(0,semidense_mapper-> image_points_byFocal_sd.rows) / (initial_inv_depth1*scale);
                                seed_points_to_print.colRange(1,2).rowRange(0,semidense_mapper-> image_points_byFocal_sd.rows) = semidense_mapper-> image_points_byFocal_sd.colRange(1,2).rowRange(0,semidense_mapper-> image_points_byFocal_sd.rows) / (initial_inv_depth1*scale);
                                seed_points_to_print.colRange(2,3).rowRange(0,semidense_mapper-> image_points_byFocal_sd.rows) = semidense_mapper-> image_points_byFocal_sd.colRange(2,3).rowRange(0,semidense_mapper-> image_points_byFocal_sd.rows) / (initial_inv_depth1*scale);

                                seed_points_to_print = seed_points_to_print.t();
                                seed_points_to_print = images.Im[reference_image]->R.t() * (seed_points_to_print - semidense_mapper-> t_r_ref);

                                seed_points_to_print = seed_points_to_print.t();


                                if (print_depth_maps > 0.5)
                                {
                                    //// ONLY SELECT DEPTHS TO PRINT FOR COLOR RANGE
                                    cv::Mat depths_to_print(0,1,CV_32FC1);
                                    for (int i= 0; i<semidense_mapper->points_ref_im_sd.rows;i++)
                                    {
                                        if ((deviation_inv_depth.at<float>(i,0) > deviation_inv_depth_th  && \
                                                 be_outlier.at<float>(i,0)<0.5  ) )
                                        {
                                           depths_to_print.push_back(semidense_mapper-> initial_inv_depth_sd.at<float>(i,0));
                                        }
                                    }
                                    cv::Mat sorted_inv_depths;
                                    cv::sort(cv::abs(depths_to_print),sorted_inv_depths,CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);
                                    min_depth = abs(sorted_inv_depths.at<float>(round(sorted_inv_depths.rows/3)));
                                    max_depth = abs(sorted_inv_depths.at<float>(round(2*sorted_inv_depths.rows/3)));
                                    min_depth_real = abs(sorted_inv_depths.at<float>(round(1*sorted_inv_depths.rows/10),0) );
                                    max_depth_real = abs(sorted_inv_depths.at<float>(round(10*sorted_inv_depths.rows/10-1),0));
                                    //// ONLY SELECT DEPTHS TO PRINT FOR COLOR RANGE
                                    ///
                                }
                           }

                            for (int i= 0; i<semidense_mapper->points_ref_im_sd.rows;i++)
                            {
                                if (l > 0)
                                {
                                    float n_x_ref = semidense_mapper->points_ref_im_sd.at<float>(i,1);
                                    float n_y_ref =semidense_mapper->points_ref_im_sd.at<float>(i,0);

                                    int count_inlier_neighbours = 0;

                                    if (n_y_ref > 1 && n_y_ref < image_i.rows-2 && n_x_ref > 1 && n_x_ref < image_i.cols-2  )
                                    {
                                        for (int n_x_ref_aux = n_x_ref-1; n_x_ref_aux <= n_x_ref+1; n_x_ref_aux++)
                                        {
                                            for (int n_y_ref_aux = n_y_ref-1; n_y_ref_aux <= n_y_ref+1; n_y_ref_aux++)
                                            {
                                                if (inliers_matrix.at<float>(n_y_ref_aux,n_x_ref_aux) > 0.5 || G_expanded.at<float>(n_y_ref_aux,n_x_ref_aux) > 5)
                                                {
                                                    count_inlier_neighbours++;
                                                }
                                            }
                                        }
                                    }

                                    point_to_camera = 100000;

                                    if ((deviation_inv_depth.at<float>(i,0) > deviation_inv_depth_th  && \
                                             be_outlier.at<float>(i,0)<0.5 && count_inlier_neighbours > 3  ))
                                    {

                                        cv::Mat points_aux(1,7,CV_32FC1);
                                        points_aux.at<float>(0,0) = seed_points_to_print.at<float>(i,0);
                                        points_aux.at<float>(0,1) = seed_points_to_print.at<float>(i,1);
                                        points_aux.at<float>(0,2) = seed_points_to_print.at<float>(i,2);

                                        point_to_camera = (fabs(camera1.at<double>(0,0) - seed_points_to_print.at<float>(i,0)) + fabs(camera1.at<double>(1,0) - seed_points_to_print.at<float>(i,1)) + \
                                                              fabs(camera1.at<double>(2,0) - seed_points_to_print.at<float>(i,2)) );


                                        float n_x_ref = semidense_mapper->points_ref_im_sd.at<float>(i,1);
                                        float n_y_ref = semidense_mapper->points_ref_im_sd.at<float>(i,0);

                                        float n_y_ref_aux = n_y_ref;
                                        float n_x_ref_aux = n_x_ref;


                                        if (print_depth_maps > 0.5)
                                        {
                                            channel_b.at<unsigned char>(n_y_ref_aux,n_x_ref_aux) = 0 + 255*(fabs(semidense_mapper-> initial_inv_depth_sd.at<float>(i,0))-min_depth_real)/(max_depth_real-min_depth_real);
                                            channel_r.at<unsigned char>(n_y_ref_aux,n_x_ref_aux) = 0;
                                            channel_g.at<unsigned char>(n_y_ref_aux,n_x_ref_aux) = 0;
                                            if (fabs(semidense_mapper-> initial_inv_depth_sd.at<float>(i,0)) < min_depth_real)
                                            {
                                                channel_b.at<unsigned char>(n_y_ref_aux,n_x_ref_aux)  = 0;
                                            }
                                        }


                                        int color1 = gray_image.at<float>(n_y_ref,n_x_ref);
                                        int color2 = gray_image.at<float>(n_y_ref,n_x_ref);
                                        int color3 = gray_image.at<float>(n_y_ref,n_x_ref);

                                        points_aux.at<float>(0,3) = color1;
                                        points_aux.at<float>(0,4) = color2;
                                        points_aux.at<float>(0,5) = color3;


                                        points_aux.at<float>(0,6) = final_variances.at<float>(i,0);



                                        if (pixel_taken .at<float>(n_y_ref,n_x_ref) < 1.5)
                                        {
                                            points_aux2.push_back(points_aux);
                                            pixel_taken.at<float>(n_y_ref,n_x_ref) = 2;
                                        }
                                        semidense_mapper -> G_expanded.at<float>(round(n_y_ref),round(n_x_ref)) = 10;


                                        number_of_points_estimated++;

                                        if ((deviation_inv_depth.at<float>(i,0) > deviation_inv_depth_print_th  && \
                                               be_outlier_print.at<float>(i,0)<0.5  )
                                                && camera_translation / point_to_camera > semidense_mapper-> translational_ratio_th_min *0.6)
                                        {
                                            int color1 = image_i.at<cv::Vec3b>(n_y_ref_aux,n_x_ref_aux)[2];
                                            int color2 = image_i.at<cv::Vec3b>(n_y_ref_aux,n_x_ref_aux)[1] ;
                                            int color3 = image_i.at<cv::Vec3b>(n_y_ref_aux,n_x_ref_aux)[0];

                                            points_aux.at<float>(0,3) = color1;
                                            points_aux.at<float>(0,4) = color2;
                                            points_aux.at<float>(0,5) = color3;

                                            semidense_mapper->map_points.push_back(points_aux);
                                            semidense_mapper->local_map_points.push_back(points_aux);

                                            if (pixel_taken_print.at<float>(n_y_ref,n_x_ref) < 1.5)
                                            {
                                                points_aux2_print.push_back(points_aux);
                                                pixel_taken_print.at<float>(n_y_ref,n_x_ref) = 2;
                                            }
                                        }



                                        for (int n_x_ref_aux = n_x_ref-1; n_x_ref_aux<=n_x_ref+1;n_x_ref_aux++)
                                        {
                                            for (int n_y_ref_aux = n_y_ref-1; n_y_ref_aux<=n_y_ref+1;n_y_ref_aux++)
                                            {
                                                if (pixel_taken.at<float>(n_y_ref_aux,n_x_ref_aux)<0.5)
                                                {

                                                    if ( semidense_mapper -> G_expanded.at<float>(round(n_y_ref_aux),round(n_x_ref_aux)) <5)
                                                    {
                                                        int color1 = gray_image.at<float>(n_y_ref_aux,n_x_ref_aux);
                                                        int color2 = gray_image.at<float>(n_y_ref_aux,n_x_ref_aux);
                                                        int color3 = gray_image.at<float>(n_y_ref_aux,n_x_ref_aux);

                                                        points_aux.at<float>(0,3) = color1;
                                                        points_aux.at<float>(0,4) = color2;
                                                        points_aux.at<float>(0,5) = color3;

                                                        if (print_depth_maps > 0.5)
                                                        {

                                                            channel_b.at<unsigned char>(n_y_ref_aux,n_x_ref_aux) = 0 + 255*(fabs(semidense_mapper-> initial_inv_depth_sd.at<float>(i,0))-min_depth_real)/(max_depth_real-min_depth_real);
                                                            channel_r.at<unsigned char>(n_y_ref_aux,n_x_ref_aux) = 0;
                                                            channel_g.at<unsigned char>(n_y_ref_aux,n_x_ref_aux) = 0;

                                                            if (fabs(semidense_mapper-> initial_inv_depth_sd.at<float>(i,0)) < min_depth_real)
                                                            {
                                                              channel_b.at<unsigned char>(n_y_ref_aux,n_x_ref_aux)  = 0;
                                                            }
                                                        }

                                                        cv::Mat point_i_sd(1,3, CV_32FC1);
                                                        point_i_sd.at<float>(0,0) = ((images.Im[reference_image]->cx-n_x_ref_aux)/images.Im[reference_image]->fx)/(initial_inv_depth1.at<float>(i,0)*scale);
                                                        point_i_sd.at<float>(0,1) = ((n_y_ref_aux-images.Im[reference_image]->cy)/images.Im[reference_image]->fy)/(initial_inv_depth1.at<float>(i,0)*scale);
                                                        point_i_sd.at<float>(0,2) = 1/ (initial_inv_depth1.at<float>(i,0)*scale);
                                                        point_i_sd = images.Im[reference_image]->R.t() * (point_i_sd.t() - images.Im[reference_image]->t);
                                                        point_i_sd = point_i_sd.t();
                                                        points_aux.at<float>(0,0) = point_i_sd.at<float>(0,0);
                                                        points_aux.at<float>(0,1) = point_i_sd.at<float>(0,1);
                                                        points_aux.at<float>(0,2) = point_i_sd.at<float>(0,2);

                                                        points_aux2.push_back(points_aux);
                                                        pixel_taken.at<float>(n_y_ref_aux,n_x_ref_aux) = 1;
                                                    }



                                                    if ((deviation_inv_depth.at<float>(i,0) > deviation_inv_depth_print_th  && \
                                                           be_outlier_print.at<float>(i,0)<0.5 && count_inlier_neighbours > 3  )
                                                            && camera_translation / point_to_camera > semidense_mapper-> translational_ratio_th_min*0.6 && pixel_taken_print.at<float>(n_y_ref_aux,n_x_ref_aux)<0.5)

                                                    {
                                                        int color1 = image_i.at<cv::Vec3b>(n_y_ref_aux,n_x_ref_aux)[2];
                                                        int color2 = image_i.at<cv::Vec3b>(n_y_ref_aux,n_x_ref_aux)[1] ;
                                                        int color3 = image_i.at<cv::Vec3b>(n_y_ref_aux,n_x_ref_aux)[0];

                                                        points_aux.at<float>(0,3) = color1;
                                                        points_aux.at<float>(0,4) = color2;
                                                        points_aux.at<float>(0,5) = color3;

                                                        semidense_mapper->map_points.push_back(points_aux);
                                                        semidense_mapper->local_map_points.push_back(points_aux);
                                                        if (pixel_taken_print.at<float>(n_y_ref_aux,n_x_ref_aux) < 0.5)
                                                        {points_aux2_print.push_back(points_aux);
                                                        pixel_taken_print.at<float>(n_y_ref_aux,n_x_ref_aux) = 1;}
                                                    }
                                                }
                                            }
                                        }
                                    }



                                } // if l > 0
                                else
                                {
                                    float n_x_ref = semidense_mapper->points_ref_im_sd.at<float>(i,1);
                                    float n_y_ref = semidense_mapper->points_ref_im_sd.at<float>(i,0);
                                    if ( deviation_inv_depth.at<float>(i,0) > deviation_inv_depth_th  && \
                                         be_outlier.at<float>(i,0)<0.5)
                                    {
                                        inliers_matrix.at<float>(n_y_ref,n_x_ref) = 1;
                                    }

                                    if (deviation_inv_depth.at<float>(i,0) > deviation_inv_depth_print_th  && \
                                           be_outlier_print.at<float>(i,0)<0.5 )
                                    {
                                        init_inv_dephts_maps_scale[1].at<float>(static_cast<int>(n_y_ref),static_cast<int>(n_x_ref))=1/fabs(semidense_mapper-> initial_inv_depth_sd.at<float>(i,0));
                                    }

                                }
                        } // for i
                        pixel_taken = inliers_matrix.clone();
                        pixel_taken_print = inliers_matrix.clone();

                        if (num_keyframes > semidense_mapper->init_keyframes)
                        {
                            int size_y = init_inv_dephts_maps_scale[0].rows;
                            int size_x = init_inv_dephts_maps_scale[0].cols;
                            cv::Mat scales(0,1,CV_32FC1);
                            for (int ii = 0; ii < size_y;ii++)
                            {
                                for (int jj = 0; jj < size_x;jj++)
                                {
                                    if ( init_inv_dephts_maps_scale[1].at<float>(ii,jj) > 0 && init_inv_dephts_maps_scale[0].at<float>(ii,jj) > 0)
                                    {
                                        float scale_ratio = init_inv_dephts_maps_scale[0].at<float>(ii,jj)  / init_inv_dephts_maps_scale[1].at<float>(ii,jj);
                                        if (fabs(scale_ratio -1 ) < 0.03)
                                        {scales.push_back(scale_ratio);}
                                    }
                                }
                            }
                            if (l==0)
                            {
                                if (scales.rows > 1000)
                                {
                                    cv::Mat sorted_scales;
                                    cv::sort(cv::abs(scales),sorted_scales,CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);
                                    scale = 1/sorted_scales.at<float>(round(sorted_scales.rows/2),0);
                                    if (scale > 1.005 || scale < 0.995)
                                    {
                                        scale = 1;
                                    }

                                }

                            }
                        }
                    } // l



                    int counterdepths2 = 0;



                    if (print_depth_maps > 0.5)
                    {
                        vector<cv::Mat> channels,channels1;
                        channels.push_back(channel_r);
                        channels.push_back(channel_g);
                        channels.push_back(channel_b);

                        channels1.push_back(channel_r1);
                        channels1.push_back(channel_g1);
                        channels1.push_back(channel_b1);

                        cv::Mat depth_channel_show;
                        cv::Mat depth_channel_show1;
                        /// Merge the three channels
                        merge(channels, depth_channel_show);
                        merge(channels1, depth_channel_show1);

                        cv::resize(depth_channel_show,depth_channel_show,cv::Size(round(depth_channel_show.cols*2),round(depth_channel_show.rows*2)),0,0,cv::INTER_LINEAR);
                        cv::resize(depth_channel_show1,depth_channel_show1,cv::Size(round(depth_channel_show1.cols*2),round(depth_channel_show1.rows*2)),0,0,cv::INTER_LINEAR);

                        char write_depth_map[150];
                        sprintf (write_depth_map,(ros::package::getPath("dpptam")+"/src/results_depth_maps/depth_keyframe%d_xFiltered.png").c_str(),num_keyframes);
                        cv::imwrite(write_depth_map,depth_channel_show);
                    }



                    int pyramid_levels = semidense_mapper->get_points_new_map().size();
                    pimages->Im[reference_image]->accurate_sd_map =  semidense_mapper->local_map_points.clone();
                    ///// MAXImo 20000 POINTS
                    vector<cv::Mat> points_new_map_aux(pyramid_levels);

                    for (int i =0; i<points_aux2.rows;i++)
                    {

                        float points_limit = points_aux2.rows+2;

                        for (int j=pyramid_levels-1; j>-1;j--)
                        {
                            if (((rand() % 1000000 ) / 1000000.0) <  (points_limit / points_aux2.rows))
                            {
                                points_new_map_aux[j].push_back(points_aux2.row(i));
                            }
                            points_limit/=3;
                        }
                    }



                    for (int j=pyramid_levels-1; j>-1;j--)
                    {
                        points_new_map_aux[j].convertTo(points_new_map_aux[j],CV_64FC1);
                    }

                    semidense_mapper->set_points_new_map(points_new_map_aux);

                    for (int l=0; l<semidense_mapper->X.ph_error.size(); l++)
                    {
                        semidense_mapper->X.ph_error[l].release();
                        semidense_mapper->X_gx_ex.ph_error[l].release();
                        semidense_mapper->X_gy_ey.ph_error[l].release();
                        semidense_mapper->X_gradient_Magnitude.ph_error[l].release();
                        semidense_mapper->point_limits_for_sd.ph_error[l].release();
                    }


                    //////////////////////////////////////////////////// JOIN CLOSEST MAPS and prepare initialize dense mapper
                    find_closest_maps(semidense_mapper,Map,semidense_tracker);
                    join_last_images(pimages,pimages_previous_keyframe,dense_mapper,semidense_mapper);
                    ///////////////////////////////////////////////////////////////

                    semidense_mapper->do_initialization_tracking=1;


                    if  (semidense_mapper->do_var_mapping > 0.5)
                    {
                            semidense_mapper->point_limits_for_sd.ph_error.clear();
                            semidense_mapper->do_var_mapping =0;
                            semidense_mapper->num_cameras_mapping = 0;
                            semidense_mapper->num_keyframes++;
                            semidense_mapper->do_init_semi=1;
                    }


                    semidense_mapper -> points3D_toprint[num_keyframes]=points_aux2_print.clone();


                   if (num_keyframes %1 == 0 && num_keyframes >  semidense_mapper -> init_keyframes +1)
                   {
                        char buffer[150];
                        sprintf (buffer,(ros::package::getPath("dpptam")+"/src/map_and_poses/MAP%d.ply").c_str(),num_keyframes);
                        points_aux2_print.convertTo(points_aux2_print,CV_64FC1);

                        print_plane( points_aux2_print,buffer);
                   }


                    semidense_mapper->set_map_points_print(semidense_mapper->map_points);


                    // PUBLISH SEMIDENSE MAP


                   if (num_keyframes %1 == 0 && num_keyframes >  semidense_mapper -> init_keyframes +1)
                   {
                       // semidense_mapper->map_pcl
                        pcl::PointCloud<pcl::PointXYZRGB> map_pcl;


                            for (int i = 0; i < points_aux2_print.rows; i++)
                                {
                                    float a = points_aux2_print.at<double>(i,0);
                                    float b = points_aux2_print.at<double>(i,1);
                                    float c = points_aux2_print.at<double>(i,2);
                                    int aa = round(points_aux2_print.at<double>(i,3));
                                    int bb = round(points_aux2_print.at<double>(i,4));;
                                    int cc = round(points_aux2_print.at<double>(i,5));

                                    pcl::PointXYZRGB map_pcl_point = pcl::PointXYZRGB(aa,bb,cc);
                                    map_pcl_point.x= a;
                                    map_pcl_point.y= b;
                                    map_pcl_point.z= c;

                                    map_pcl.points.push_back(map_pcl_point);
                                }


                        sensor_msgs::PointCloud2 out_points;

                        pcl::toROSMsg(map_pcl, out_points);
                        out_points.header.frame_id = "dpptam/map";
                        out_points.header.stamp = ros::Time::now();
                        pub_cloud->publish(out_points);
                    }
                    //PUBLISH SEMIDENSE MAP

                } // do_Var_mapping > 0.5



                for (int j = 0; j< images.getNumberOfImages(); j = j+1)
                {
                    delete images.Im[j];
                    images.Im[j] = NULL;
                }

                images.Im.clear();

                semidense_mapper->last_frame_mapped =  semidense_mapper->last_frame_tracked;
}



void copy_first_and_last_images(Imagenes &images, Imagenes &images_map)
{
    int images_size  = images.getNumberOfImages()-1;
    for (int l = 0 ; l < images_size; l = l+1)
    {

        if (l < 1 || l > (images_size-6))
        {
            images_map.computeImage();
            int images_map_size = images_map.getNumberOfImages()-1;

            images_map.Im[images_map_size ]->image = images.Im[l]->image.clone();
            images_map.Im[images_map_size ]->R = images.Im[l]->R.clone();
            images_map.Im[images_map_size ]->image_gray=images.Im[l]->image_gray.clone();
            images_map.Im[images_map_size ]->t = images.Im[l]->t.clone();
            images_map.Im[images_map_size ]->t_r = images.Im[l]->t_r.clone();
            images_map.Im[images_map_size ]->fx = images.Im[l]->fx;
            images_map.Im[images_map_size ]->fy = images.Im[l]->fy;
            images_map.Im[images_map_size ]->cx = images.Im[l]->cx;
            images_map.Im[images_map_size ]->cy = images.Im[l]->cy;
            images_map.Im[images_map_size]->error = images.Im[l]->error;
            images_map.Im[images_map_size]->k1 = images.Im[l]->k1;
            images_map.Im[images_map_size]->k2 = images.Im[l]->k2;
            images_map.Im[images_map_size]->stamps = images.Im[l]->stamps;
            images_map.Im[images_map_size]->num_keyframes = images.Im[l]-> num_keyframes;

            images_map.Im[images_map_size]->is_used_for_mapping = images.Im[l]-> is_used_for_mapping;
            images.Im[l]-> is_used_for_mapping = 1;

        }
    }
}


void get_photometric_errors_matrix_sd_exhaustive(Imagenes  &images,  cv::Mat &inv_depths, photometric_term &X, photometric_term &X_gradient_Magnitude,\
                                      photometric_term &X_gx_ex, photometric_term &X_gy_ey, int reference_image, cv::Mat &initial_inv_depth , int image_to_be_added, \
                                      photometric_term &points_i_todos,cv::Mat &points_ref_im_sd,int discretization, \
                                      int window_size, cv::Mat &epipolar_gradients, vector<cv::Mat> &initial_inv_depth_inEveryCamera_uncertainty,vector<cv::Mat> &initial_inv_depth_inEveryCamera_largeParallax, \
                                      cv::Mat &points_by_depth, cv::Mat &t_r_ref,float &ratio_parallax, bool optimize_previous_frame,vector<cv::Mat> &previous_or_next_frame,
                                                 cv::Mat &GX, cv::Mat &GY, int &num_cameras_mapping, cv::Mat &max_inv_depth_initial_seed, cv::Mat &min_inv_depth_initial_seed)
{

            cv::Mat points_i2_sd;
            //-0.05
            cv::Mat inv_depths_vector = points_by_depth.clone();
            inv_depths_vector = inv_depths.at<float>(0,0);



            cv::Mat inv_depths_vector_end = points_by_depth.clone();
            inv_depths_vector_end = inv_depths.at<float>(discretization-1,0);

            float sigma_times = 3;
            for (int i = 0; i < inv_depths_vector.cols; i++)
            {
                if (initial_inv_depth_inEveryCamera_largeParallax[i].rows > 1)
                {
                    float uncertainty;
                    uncertainty = initial_inv_depth_inEveryCamera_uncertainty[i].at<float>(initial_inv_depth_inEveryCamera_uncertainty[i].rows-1,0);

                    float inv_depth_down_initial_seed = max_inv_depth_initial_seed.at<float>(i,0);
                    float inv_depth_up_initial_seed = min_inv_depth_initial_seed.at<float>(i,0);

                    inv_depths_vector.at<float>(0,i) = inv_depth_down_initial_seed+sigma_times*uncertainty;
                    inv_depths_vector.at<float>(1,i) = inv_depth_down_initial_seed+sigma_times*uncertainty;
                    inv_depths_vector.at<float>(2,i) = inv_depth_down_initial_seed+sigma_times*uncertainty;

                    inv_depths_vector_end.at<float>(0,i) = inv_depth_up_initial_seed-sigma_times*uncertainty;
                    inv_depths_vector_end.at<float>(1,i) = inv_depth_up_initial_seed-sigma_times*uncertainty;
                    inv_depths_vector_end.at<float>(2,i) = inv_depth_up_initial_seed-sigma_times*uncertainty;

                }
            }


            cv::Mat inv_depths_vector_init = inv_depths_vector.clone();

            points_i2_sd = points_by_depth / inv_depths_vector ;
            points_i2_sd = images.Im[reference_image]->R.t() * (points_i2_sd - t_r_ref);
            points_i_todos.ph_error[0] = points_i2_sd.clone();

            points_i2_sd = points_by_depth / inv_depths_vector_end;
            points_i2_sd = images.Im[reference_image]->R.t() * (points_i2_sd - t_r_ref);
            points_i_todos.ph_error[discretization-1] = points_i2_sd.clone();


            cv::Mat errores;

            for (unsigned int l = 0; l < discretization; l++)
            {
                //X.computeError();
                errores = cv::Mat::zeros(initial_inv_depth.rows, 1, CV_32FC1);
                if (X.ph_error[l].empty())
                {
                    errores.copyTo(X.ph_error[l]);
                    errores.copyTo(X_gx_ex.ph_error[l]);
                    errores.copyTo(X_gy_ey.ph_error[l]);
                    errores.copyTo(X_gradient_Magnitude.ph_error[l]);
                }
            }

            errores = cv::Mat::zeros(initial_inv_depth.rows, 1, CV_32FC1) + INFINITY;
            int  m = image_to_be_added;
            cv::Mat t_r = cv::repeat(images.Im[m]->t, 1, initial_inv_depth.rows).clone();


            cv::Mat image_o_gray = images.Im[m]->image_gray.clone();
            cv::Mat image_i_gray = images.Im[reference_image]->image_gray.clone();

            cv::Mat  gray_image = image_i_gray.clone();
            gray_image.convertTo(gray_image,CV_32FC1);

            if (num_cameras_mapping == 0)
            {
                GX =cv::abs(gradientX(gray_image,1));

                GY =cv::abs(gradientY(gray_image,1));

                cv::Mat GX2,GY2;

                cv::pow(GX,2,GX2);
                cv::pow(GY,2,GY2);
                cv::Mat G =(GX2) + (GY2);
                cv::pow(G,0.5,G);

                cv::divide(GX, G,GX,1);
                cv::divide(GY, G,GY,1);
            }


            image_i_gray.convertTo(image_i_gray,CV_32FC1);
            image_o_gray.convertTo(image_o_gray,CV_32FC1);




            cv::Mat points_i_2;
            cv::Mat points_o;

            points_i_todos.ph_error[0].copyTo(points_i_2);
            points_o  = images.Im[m]->R  * (points_i_2) + t_r;
            points_o.colRange(0,points_o.cols).rowRange(0,1) = points_o.colRange(0,points_o.cols).rowRange(0,1) * images.Im[m]->fx;
            points_o.colRange(0,points_o.cols).rowRange(1,2) = points_o.colRange(0,points_o.cols).rowRange(1,2) * images.Im[m]->fy;
            /////
            cv::Mat inv_X3_init = 1 / points_o.colRange(0,points_o.cols).rowRange(2,3);
            //////
            cv::Mat z_repeat =  cv::repeat(points_o.colRange(0,points_o.cols).rowRange(2,3),3,1);
            cv::divide(points_o, z_repeat,points_o,1);

            points_o.colRange(0,points_o.cols).rowRange(0,1) =  images.Im[reference_image]->cx-points_o.colRange(0,points_o.cols).rowRange(0,1) ;
            points_o.colRange(0,points_o.cols).rowRange(1,2) =  points_o.colRange(0,points_o.cols).rowRange(1,2) + images.Im[reference_image]->cy;

            /////
            cv::Mat xvalues_init = points_o.colRange(0,points_o.cols).rowRange(0,1).clone();
            cv::Mat yvalues_init = points_o.colRange(0,points_o.cols).rowRange(1,2).clone();
            //////
            epipolar_gradients = points_o.rowRange(0,2).clone();




            points_i_todos.ph_error[discretization-1].copyTo(points_i_2);
            points_o  = images.Im[m]->R  * (points_i_2) + t_r;
            points_o.colRange(0,points_o.cols).rowRange(0,1) = points_o.colRange(0,points_o.cols).rowRange(0,1) * images.Im[m]->fx;
            points_o.colRange(0,points_o.cols).rowRange(1,2) = points_o.colRange(0,points_o.cols).rowRange(1,2) * images.Im[m]->fy;
            /////
            cv::Mat inv_X3_end = 1 /  points_o.colRange(0,points_o.cols).rowRange(2,3);
            //////
            z_repeat =  cv::repeat(points_o.colRange(0,points_o.cols).rowRange(2,3),3,1);

            cv::divide(points_o, z_repeat,points_o,1);

            points_o.colRange(0,points_o.cols).rowRange(0,1) =  images.Im[reference_image]->cx-points_o.colRange(0,points_o.cols).rowRange(0,1) ;
            points_o.colRange(0,points_o.cols).rowRange(1,2) =  points_o.colRange(0,points_o.cols).rowRange(1,2) + images.Im[reference_image]->cy;


            /////
            cv::Mat xvalues_end = points_o.colRange(0,points_o.cols).rowRange(0,1).clone();
            cv::Mat yvalues_end = points_o.colRange(0,points_o.cols).rowRange(1,2).clone();
            //////


            epipolar_gradients = epipolar_gradients - points_o.rowRange(0,2);
            epipolar_gradients = cv::abs(epipolar_gradients);

            cv::Mat epipolar_gradientX2,epipolar_gradientY2;
            cv::Mat epipolar_gradientX = cv::abs(epipolar_gradients.colRange(0,points_o.cols).rowRange(0,1));
            cv::pow(epipolar_gradientX,2,epipolar_gradientX2);
            cv::Mat epipolar_gradientY = cv::abs(epipolar_gradients.colRange(0,points_o.cols).rowRange(1,2));
            cv::pow(epipolar_gradientY,2,epipolar_gradientY2);
            cv::Mat epipolar_gradientT = epipolar_gradientX2+epipolar_gradientY2;
            cv::pow(epipolar_gradientT,0.5,epipolar_gradientT);


            cv::divide( epipolar_gradientX, epipolar_gradientT, epipolar_gradientX,1);
            cv::divide( epipolar_gradientY, epipolar_gradientT, epipolar_gradientY,1);




            cv::Mat aaa = (1/inv_X3_end-1/inv_X3_init) / (1/inv_depths_vector_end.rowRange(0,1) - 1/inv_depths_vector_init.rowRange(0,1));
            cv::Mat bbb = 1/inv_X3_init - aaa.mul(1/inv_depths_vector_init.rowRange(0,1));

            cv::Mat ccc = (xvalues_end-xvalues_init)/ (inv_X3_end-inv_X3_init);
            cv::Mat ddd = xvalues_init - ccc.mul(inv_X3_init);

            cv::Mat eee = (yvalues_end-yvalues_init) / (inv_X3_end-inv_X3_init);
            cv::Mat fff = yvalues_init -eee.mul(inv_X3_init);

            int window_size_end =  window_size+1;

            cv::Mat xvalues_final = xvalues_init.clone();
            cv::Mat yvalues_final = yvalues_init.clone();

            cv::Mat gradient_epipolar_line= cv::Mat::zeros(initial_inv_depth.rows, 1, CV_32FC1);

            int image_cols = images.Im[reference_image]->image.cols;
            int image_rows = images.Im[reference_image]->image.rows;

            float num_wrong_epipolar_lines = 0;
            //#pragma omp parallel for num_threads(2)
            for (int i = 0; i < initial_inv_depth.rows; i=i+1)
            {
                float num_epipolar_search = 0;

                bool epipolar_inside_image = true;

                float xvalues_init1 = xvalues_init.at<float>(0,i);
                float yvalues_init1 = yvalues_init.at<float>(0,i);
                float xvalues_end1 = xvalues_end.at<float>(0,i);
                float yvalues_end1 = yvalues_end.at<float>(0,i);

                if (fabs(static_cast<int>(xvalues_init.at<float>(0,i) - xvalues_end.at<float>(0,i)) )  > fabs(static_cast<int>(yvalues_init.at<float>(0,i) - yvalues_end.at<float>(0,i)) )  )
                {
                    num_epipolar_search  = fabs(static_cast<int>(xvalues_init1 - xvalues_end1) );
                }
                else
                {
                    num_epipolar_search  =  fabs(static_cast<int>(yvalues_init1 - yvalues_end1) );
                }

                float slope_x = (xvalues_end1 - xvalues_init1)  / num_epipolar_search;
                float slope_y = (yvalues_end1 - yvalues_init1)  / num_epipolar_search;


                float l_opt = 0;
                float step_epipolar = 1;
                /*if (num_epipolar_search > 20.0)
                {
                    step_epipolar = num_epipolar_search / 20.0;
                }*/

                float X_gx_ex_aux = 0.0;
                float X_gy_ey_aux = 0.0;

                int   n_x_ref = round(points_ref_im_sd.at<float>(i,1));
                int   n_y_ref = round(points_ref_im_sd.at<float>(i,0));

                float l_init = 0;
                float l_end = num_epipolar_search+1;


                if (xvalues_init1 > 0 && xvalues_init1 < image_cols-1 && xvalues_end1 > 0 && xvalues_end1 < image_cols-1 &&
                        yvalues_init1 > 0 && yvalues_init1 < image_rows-1 && yvalues_end1 > 0 && yvalues_end1 < image_rows-1)
                {
                    l_init = 0;
                    l_end = num_epipolar_search+1;
                }
                else
                {
                    float l_init_x = l_init;
                    float l_init_y = l_init;
                    float l_end_x = l_end;
                    float l_end_y = l_end;

                    int count_intersections = 0;

                    cv::Mat intersections(0,1,CV_32FC1);

                    l_init_x = -xvalues_init1 / slope_x;
                    if (l_init_x > l_init && l_init_x < l_end)
                    {
                        count_intersections++;
                        intersections.push_back(l_init_x);
                    }

                    l_init_y = -yvalues_init1 / slope_y;
                    if (l_init_y > l_init && l_init_y < l_end)
                    {
                        count_intersections++;
                        intersections.push_back(l_init_y);
                    }

                    l_end_x = (image_rows-xvalues_init1) / slope_x;
                    if (l_end_x > l_init && l_end_x < l_end)
                    {
                        count_intersections++;
                        intersections.push_back(l_end_x);
                    }

                    l_end_y = (image_cols-yvalues_init1) / slope_y;
                    if (l_end_y > l_init && l_end_y < l_end)
                    {
                        count_intersections++;
                        intersections.push_back(l_end_y);
                    }

                    if (count_intersections == 1)
                    {
                                if (xvalues_init1 > 0 && xvalues_init1 < image_cols-1 && yvalues_init1 > 0 && yvalues_init1 < image_rows-1)
                                {
                                    intersections.push_back(l_init);
                                }
                                else
                                {
                                    intersections.push_back(l_end);
                                }
                                count_intersections++;
                    }

                    if (count_intersections == 2)
                    {
                        if (intersections.at<float>(0,0) < intersections.at<float>(1,0))
                        {
                            l_init = intersections.at<float>(0,0);
                            l_end = intersections.at<float>(1,0);
                        }
                        else
                        {
                            l_end = intersections.at<float>(0,0);
                            l_init = intersections.at<float>(1,0);
                        }
                    }

                 }


                if (epipolar_inside_image && l_end > l_init && l_end - l_init < 400)
                {
                }
                else{
                    num_wrong_epipolar_lines++;
                }

                if (epipolar_inside_image && l_end > l_init && l_end - l_init < 400)
                {
                    double error_opt = INFINITY;


                    float l = l_init-1;
                    while(l< l_end+1)
                    {
                        l++;

                        int n_x =   round(xvalues_init1 + l*slope_x);
                        int n_y =   round(yvalues_init1 + l*slope_y);

                        if (n_x > 3 && n_x < image_cols-4 && n_y > 3 \
                                && n_y < image_rows-4\
                                && n_x_ref > 3 && n_x_ref< image_cols-4 && n_y_ref >3 \
                                && n_y_ref < image_rows-4)
                        {
                                   double error = 0;

                                  for (int mm=-window_size;mm < window_size_end;mm = mm+2)
                                  {
                                      for (int nn = -window_size; nn < window_size_end; nn=nn+2)
                                      {
                                          error   += std::abs(image_i_gray.at<float>(n_y_ref+nn,n_x_ref+mm) -image_o_gray.at<float>(n_y+nn,n_x+mm));
                                      }
                                  }

                                  if (error < errores.at<float>(i,0))
                                  {
                                      error_opt = error;
                                      l_opt = l;
                                      errores.at<float>(i,0) = error;
                                  }
                       }

                    }// epipolar search



                    errores.at<float>(i,0) = INFINITY;
                    for (float l =  l_opt - 1*step_epipolar; l <= l_opt + 1*step_epipolar; l++)
                    {

                        float pos_x =    xvalues_init1 + l*slope_x;
                        float pos_y =    yvalues_init1 + l*slope_y;

                        int n_x =   round(pos_x);
                        int n_y =   round(pos_y);


                       if (n_x > 3 && n_x < image_cols-4 && n_y > 3 \
                           && n_y < image_rows-4\
                           && n_x_ref > 3 && n_x_ref< image_cols-4 && n_y_ref >3 \
                           && n_y_ref < image_rows-4)
                        {
                              double error = 0;

                              for (int mm=-window_size;mm < window_size_end;mm = mm+1)
                              {
                                for (int nn = -window_size; nn < window_size_end; nn=nn+1)
                                {
                                      error   += fabs(image_i_gray.at<float>(n_y_ref+nn,n_x_ref+mm) -image_o_gray.at<float>(n_y+nn,n_x+mm));
                                }
                              }
                              if (error < errores.at<float>(i,0))
                              {
                                  errores.at<float>(i,0) = error;

                                  X_gx_ex_aux = GX.at<float>(n_y,n_x)*epipolar_gradientX.at<float>(0,i);
                                  X_gy_ey_aux = GY.at<float>(n_y,n_x)*epipolar_gradientY.at<float>(0,i);

                                  xvalues_final.at<float>(0,i) = pos_x;
                                  yvalues_final.at<float>(0,i) = pos_y;
                              }
                        }
                    }// epipolar search
                }
                X_gx_ex.ph_error[0].at<float>(i,0) += X_gx_ex_aux;
                X_gy_ey.ph_error[0].at<float>(i,0) += X_gy_ey_aux;
                gradient_epipolar_line.at<float>(i,0) = X_gx_ex_aux+X_gy_ey_aux;
            } // points


            cv::Mat inv_depths_opt =aaa.mul(xvalues_final-ddd) / (ccc - bbb.mul(xvalues_final-ddd));
            cv::Mat inv_depths_opt_1 =aaa.mul(xvalues_final+ epipolar_gradientX-ddd) / (ccc - bbb.mul(xvalues_final+ epipolar_gradientX-ddd));
            cv::Mat inv_depths_opt_2 =aaa.mul(yvalues_final+ epipolar_gradientY-fff) / (eee - bbb.mul(yvalues_final+ epipolar_gradientY-fff));

            inv_depths_opt = inv_depths_opt.t();
            inv_depths_opt_1 = inv_depths_opt_1.t();
            inv_depths_opt_2 = inv_depths_opt_2.t();

            cv::Mat uncertainty_1 = cv::abs(inv_depths_opt - inv_depths_opt_1);
            cv::Mat uncertainty_2 = cv::abs(inv_depths_opt - inv_depths_opt_2);

            errores = errores / ((window_size*2+1)*(window_size*2+1));

            for (int i=0; i<initial_inv_depth.rows; i=i+1)
            {
                if (inv_depths_opt.at<float>(i,0) < 0 && errores.at<float>(i,0) < 50)
                {
                        if (ratio_parallax > 0   )
                        {
                            if (inv_depths_opt.at<float>(i,0) > max_inv_depth_initial_seed.at<float>(i,0))
                            {
                                 max_inv_depth_initial_seed.at<float>(i,0) = inv_depths_opt.at<float>(i,0);
                            }
                            if (inv_depths_opt.at<float>(i,0) < min_inv_depth_initial_seed.at<float>(i,0))
                            {
                                 min_inv_depth_initial_seed.at<float>(i,0) = inv_depths_opt.at<float>(i,0);
                            }

                            initial_inv_depth_inEveryCamera_largeParallax[i].push_back(inv_depths_opt.at<float>(i,0));

                            if (uncertainty_1.at<float>(i,0) > uncertainty_2.at<float>(i,0))
                            {
                                initial_inv_depth_inEveryCamera_uncertainty[i].push_back(uncertainty_1.at<float>(i,0));
                            }
                            else
                            {
                                initial_inv_depth_inEveryCamera_uncertainty[i].push_back(uncertainty_2.at<float>(i,0));
                            }


                            if (optimize_previous_frame)
                            {
                                previous_or_next_frame[i].push_back(1);
                            }
                            else
                            {
                                previous_or_next_frame[i].push_back(0);
                            }
                        }
                }
            }

}

void   convergence_test(SemiDenseMapping *semidense_mapper,cv::Mat &be_outlier,
                        cv::Mat &be_outlier_print,cv::Mat &deviation_inv_depth,cv::Mat &final_variances, float inv_depth_disparity_th,float inv_depth_disparity_print_th)
{
    float counter_converged_points = 0;

    int minim_images  = 4;
    int minim_prev_and_post_images  = 2;


    if (semidense_mapper-> frames_previous_keyframe_processed < 5 || (semidense_mapper-> num_cameras_mapping - semidense_mapper-> frames_previous_keyframe_processed) < 5)
    { minim_prev_and_post_images  = 0;}

    if (semidense_mapper -> num_keyframes < semidense_mapper->init_keyframes +1)
    {
        minim_prev_and_post_images  = 2;
    }


    float max_convergence_ratio_total = 0;
    float total_points = 0;


    #pragma omp parallel for num_threads(2)
    for (int i=0; i<semidense_mapper-> initial_inv_depth_sd.rows; i++)
    {

        if (semidense_mapper-> initial_inv_depth_inEveryCamera_largeParallax[i].rows >= minim_images && semidense_mapper -> num_keyframes > semidense_mapper->init_keyframes)
        {
            semidense_mapper -> initial_inv_depth_inEveryCamera_largeParallax[i] = semidense_mapper -> initial_inv_depth_inEveryCamera_largeParallax[i].rowRange(2, semidense_mapper -> initial_inv_depth_inEveryCamera_largeParallax[i].rows);
            semidense_mapper -> previous_or_next_frame[i] = semidense_mapper -> previous_or_next_frame[i].rowRange(2, semidense_mapper -> previous_or_next_frame[i].rows);
            semidense_mapper -> initial_inv_depth_inEveryCamera_uncertainty[i] = semidense_mapper -> initial_inv_depth_inEveryCamera_uncertainty[i].rowRange(2, semidense_mapper -> initial_inv_depth_inEveryCamera_uncertainty[i].rows);
        }

        if (semidense_mapper-> initial_inv_depth_inEveryCamera_largeParallax[i].rows >= minim_images )
        {
            cv::Mat sorted_inv_depths;
            cv::sort(semidense_mapper-> initial_inv_depth_inEveryCamera_largeParallax[i],sorted_inv_depths,CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);

            cv::Mat sorted_index ;
            cv::sortIdx(semidense_mapper-> initial_inv_depth_inEveryCamera_largeParallax[i],sorted_index,CV_SORT_EVERY_COLUMN +CV_SORT_ASCENDING);

            cv::Mat sorted_previous_or_next_frame = semidense_mapper -> previous_or_next_frame[i].clone();
            cv::Mat sorted_variances = semidense_mapper -> initial_inv_depth_inEveryCamera_uncertainty[i].clone();

            for (int l = 0; l < sorted_previous_or_next_frame.rows; l++)
            {
                sorted_previous_or_next_frame.at<float>(l,0) = semidense_mapper -> previous_or_next_frame[i].at<float>(sorted_index.at<int>(l,0),0);
                sorted_variances.at<float>(l,0) = semidense_mapper ->  initial_inv_depth_inEveryCamera_uncertainty[i].at<float>(sorted_index.at<int>(l,0),0);
            }


            deviation_inv_depth.at<float>(i,0) = (semidense_mapper->X_gy_ey.ph_error[0].at<float>(i,0)  +\
                                                semidense_mapper->X_gx_ex.ph_error[0].at<float>(i,0));

            semidense_mapper-> initial_inv_depth_sd.at<float>(i,0) = sorted_inv_depths.at<float>(sorted_inv_depths.rows/2,0);

            be_outlier.at<float>(i,0) = 1;
            be_outlier_print.at<float>(i,0) = 1;
            int leave_loop = 0;

            float max_convergence_ratio = 0;
            float cameras_consistency_minim = 0;

            int limit = sorted_inv_depths.rows;


            for (int minim_triangulations = minim_images; minim_triangulations <= limit; minim_triangulations = minim_triangulations +1)
            {

                for (int l = 0; l < sorted_inv_depths.rows-minim_triangulations;l++)
                {
                    if (leave_loop < 1)
                    {
                        int sum_prev_nex_frs = 0;
                        for (int ll = l; ll<l+minim_triangulations;ll++)
                        {
                            sum_prev_nex_frs+= sorted_previous_or_next_frame.at<int>(ll,0);
                        }

                        float cameras_consistency = fabs(mean(sorted_variances.rowRange(l,l+minim_triangulations))[0]);


                        if ( fabs(sorted_inv_depths.at<float>(l+(minim_triangulations-1),0) - sorted_inv_depths.at<float>(l,0))  / cameras_consistency < inv_depth_disparity_th)
                        {
                            if ((sum_prev_nex_frs>minim_prev_and_post_images-1 && sum_prev_nex_frs < minim_triangulations+1-minim_prev_and_post_images )||  semidense_mapper->num_keyframes <= semidense_mapper-> init_keyframes)
                            {
                                final_variances.at<float>(i,0) = cameras_consistency;
                                cameras_consistency_minim = sorted_inv_depths.at<float>(l+(minim_triangulations-1),0) / sorted_inv_depths.at<float>(l,0);
                                be_outlier.at<float>(i,0) = 0;

                                if (fabs(sorted_inv_depths.at<float>(l+(minim_triangulations-1),0) - sorted_inv_depths.at<float>(l,0))  / cameras_consistency < inv_depth_disparity_print_th && minim_triangulations > 5)
                                {
                                    be_outlier_print.at<float>(i,0) = 0;
                                    leave_loop = 1;
                                }

                                semidense_mapper-> initial_inv_depth_sd.at<float>(i,0) = sorted_inv_depths.rowRange(l,l+minim_triangulations).at<float>(sorted_inv_depths.rowRange(l,l+minim_triangulations).rows/2,0);
                            }
                        }
                    }
                }
            }

            max_convergence_ratio_total += max_convergence_ratio;
            total_points++;
        }
        else
        {
             if (semidense_mapper-> initial_inv_depth_inEveryCamera_largeParallax[i].rows > 1 )
             {
                 cv::Mat sorted_inv_depths;
                 cv::sort(semidense_mapper-> initial_inv_depth_inEveryCamera_largeParallax[i],sorted_inv_depths,CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);
                 semidense_mapper-> initial_inv_depth_sd.at<float>(i,0) = sorted_inv_depths.at<float>(round(sorted_inv_depths.rows/2),0);
             }

            be_outlier.at<float>(i,0) = 1;
            be_outlier_print.at<float>(i,0) = 1;
        }
        if (be_outlier_print.at<float>(i,0) == 0)
        counter_converged_points++;
    } // for
}


void find_closest_maps(SemiDenseMapping *semidense_mapper,MapShared *Map,SemiDenseTracking *semidense_tracker)
{
    vector<cv::Mat> points_last_keyframes(semidense_tracker->pyramid_levels);

    int c = (int)(semidense_mapper->num_keyframes) / semidense_tracker->local_maps_number;
    int pos_map = (semidense_mapper->num_keyframes) - semidense_tracker->local_maps_number * c;

    if (semidense_mapper->num_keyframes > semidense_mapper->init_keyframes-1)
    {
        for (int j = 0; j <semidense_tracker->pyramid_levels;j++)
        {
           semidense_tracker->set_local_maps(semidense_mapper->get_points_new_map()[j],pos_map*semidense_tracker->pyramid_levels+j);
        }
        //// FIX IT
        semidense_tracker->set_poses_local_maps(-Map->get_R().t()*Map->get_t(),pos_map);
    }
    int local_maps_number_real = 0;
    cv::Mat sorted_distances_local_maps(1,0,CV_32FC1);
    for ( int jj =0;jj< semidense_tracker->local_maps_number;jj=jj+1)
    {
        if ( semidense_tracker->get_poses_local_maps()[jj].rows > 0)
        {
            local_maps_number_real++;
            cv::Mat distance_local_map_aux = semidense_tracker->get_poses_local_maps()[jj]  + Map->get_R().t()*Map->get_t();
            cv::pow(distance_local_map_aux,2,distance_local_map_aux);
            semidense_tracker->set_distances_local_maps(sqrt(cv::sum(distance_local_map_aux)[0]),jj);
            sorted_distances_local_maps.push_back(semidense_tracker->get_distances_local_maps()[jj]);
        }
    }

    cv::sort(sorted_distances_local_maps,sorted_distances_local_maps,CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);

    float limit_distance=1000;
    if (semidense_mapper->num_keyframes > semidense_mapper->init_keyframes-1)
    {
        limit_distance  = 1.0001*sorted_distances_local_maps.at<float>(semidense_tracker->local_maps_close_number,0);
    }

    for ( int jj =0;jj< semidense_tracker->local_maps_number;jj=jj+1)
    {
        if (semidense_tracker->get_distances_local_maps()[jj]< limit_distance && semidense_tracker->get_distances_local_maps()[jj] > 0)
        {
            for (int ii = 0; ii<semidense_tracker->pyramid_levels;ii++)
            {
                points_last_keyframes[ii].push_back(semidense_tracker->get_local_maps()[jj*semidense_tracker->pyramid_levels+ii]);
            }
        }
    }

    semidense_tracker->set_points_last_keyframes(points_last_keyframes);
}


void join_last_images(Imagenes *images,Imagenes *images_previous_keyframe,\
                      DenseMapping *dense_mapper,SemiDenseMapping *semidense_mapper)
{
    int c = (int)(semidense_mapper->num_keyframes) / dense_mapper->points3D4spx.size();
    int pos_map = (semidense_mapper->num_keyframes) - dense_mapper->points3D4spx.size() * c;

    dense_mapper->points3D4spx[pos_map]=semidense_mapper->local_map_points.clone();
    cv::Mat local_map_points_init(0,6,CV_64FC1);
    semidense_mapper->local_map_points = local_map_points_init.clone();

    copy_imagenes_dense(*images,*dense_mapper);
    filter_imagenes(*dense_mapper,4);

    if (semidense_mapper->num_keyframes >  semidense_mapper -> init_keyframes && dense_mapper->get_do_dense() < 0.5)
    {
        cv::Mat local_map_points_aux =  dense_mapper->points3D4spx[0].clone();

        for (int i = 1 ; i < dense_mapper->points3D4spx.size();i++)
        {
            local_map_points_aux.push_back(dense_mapper->points3D4spx[i]);
        }

        dense_mapper->set_last3Dpoints(local_map_points_aux);

        if (dense_mapper->get_last3Dpoints().rows > 1000)
        {dense_mapper->set_do_dense(1);}
    }

    //images_previous_keyframe = dense_mapper;
    copy_imagenes(*images,*images_previous_keyframe);

    if( semidense_mapper->num_keyframes < 30)
    {filter_imagenes(*images_previous_keyframe,4);}
    else
    {filter_imagenes(*images_previous_keyframe,4);}

    semidense_mapper->frames_previous_keyframe_processed = 0;
    semidense_mapper->frames_previous_keyframe_used = 0;
}


void copy_imagenes_dense(Imagenes &images, Imagenes &images_map)
{
    int images_size  = images.getNumberOfImages()-1;


    for (int l = 0 ; l < images_size-1; l = l+1)
    {
        images_map.computeImage();
        int images_map_size = images_map.getNumberOfImages()-1;

        images_map.Im[images_map_size ]->image = images.Im[l]->image.clone();
        images_map.Im[images_map_size ]->R = images.Im[l]->R.clone();
        images_map.Im[images_map_size ]->image_gray=images.Im[l]->image_gray.clone();
        images_map.Im[images_map_size ]->t = images.Im[l]->t.clone();
        images_map.Im[images_map_size ]->t_r = images.Im[l]->t_r.clone();
        images_map.Im[images_map_size ]->fx = images.Im[l]->fx;
        images_map.Im[images_map_size ]->fy = images.Im[l]->fy;
        images_map.Im[images_map_size ]->cx = images.Im[l]->cx;
        images_map.Im[images_map_size ]->cy = images.Im[l]->cy;
        images_map.Im[images_map_size]->error = images.Im[l]->error;
        images_map.Im[images_map_size]->k1 = images.Im[l]->k1;
        images_map.Im[images_map_size]->k2 = images.Im[l]->k2;
        images_map.Im[images_map_size]->stamps = images.Im[l]->stamps;
        images_map.Im[images_map_size]->num_keyframes = images.Im[l]-> num_keyframes;
        images_map.Im[images_map_size]->accurate_sd_map =  images.Im[l]-> accurate_sd_map;
    }
}
template <typename T>
void filter_imagenes( T &images_dense, int num_keyframes )
//void filter_imagenes(Imagenes &images_dense, int num_keyframes)
{
    int num_images = images_dense.getNumberOfImages()-1;
    int last_keyframe = images_dense.Im[images_dense.getNumberOfImages()-2]->num_keyframes;

    for (int ii = num_images; ii> -0.5; ii--)
    {
       if (images_dense.Im[ii]->num_keyframes < (last_keyframe-num_keyframes) )
        {
          delete images_dense.Im[ii];
          images_dense.Im.erase(images_dense.Im.begin()+ii);
       }
    }


}


void copy_imagenes(Imagenes &images, Imagenes &images_map)
{
    int images_size  = images.getNumberOfImages();

    int step = 1;

    for (int l = 0 ; l < images_size-1; l = l+step)
    {
        images_map.computeImage();
        int images_map_size = images_map.getNumberOfImages()-1;

        images_map.Im[images_map_size ]->image = images.Im[l]->image.clone();
        images_map.Im[images_map_size ]->R = images.Im[l]->R.clone();
        images_map.Im[images_map_size ]->image_gray=images.Im[l]->image_gray.clone();
        images_map.Im[images_map_size ]->t = images.Im[l]->t.clone();
        images_map.Im[images_map_size ]->t_r = images.Im[l]->t_r.clone();
        images_map.Im[images_map_size ]->fx = images.Im[l]->fx;
        images_map.Im[images_map_size ]->fy = images.Im[l]->fy;
        images_map.Im[images_map_size ]->cx = images.Im[l]->cx;
        images_map.Im[images_map_size ]->cy = images.Im[l]->cy;
        images_map.Im[images_map_size]->error = images.Im[l]->error;
        images_map.Im[images_map_size]->k1 = images.Im[l]->k1;
        images_map.Im[images_map_size]->k2 = images.Im[l]->k2;
        images_map.Im[images_map_size]->stamps = images.Im[l]-> stamps;
        images_map.Im[images_map_size]->num_keyframes = images.Im[l]-> num_keyframes;
        images_map.Im[images_map_size]->accurate_sd_map =  images.Im[l]-> accurate_sd_map;
    }
}


