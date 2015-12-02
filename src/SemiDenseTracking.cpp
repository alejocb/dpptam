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

#include <dpptam/SemiDenseTracking.h>
#include <dpptam/vo_system.h>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>


SemiDenseTracking::SemiDenseTracking()
{

    /*char cwd[1024];
       if (getcwd(cwd, sizeof(cwd)) != NULL)
           fprintf(stdout, "Current working dir: %s\n", cwd);
       else
           perror("getcwd() error");
       chdir(cwd);*/
       //chdir("/home/alejo/catkin_ws");

   cv::FileStorage  fs2("src/dpptam/src/data.yml", cv::FileStorage::READ);
   fs2["cameraMatrix"] >> cameraMatrix;
   fs2["distCoeffs"] >> distCoeffs;

   last_cont_frames = 0;
   frames_processed=0 ;

   pyramid_levels = 3;


   local_maps_number = (int)fs2["local_maps_number"];


   SemiDenseTracking::init_jacobian(pyramid_levels);

   SemiDenseTracking::init_reduction_pyramid(pyramid_levels);
   SemiDenseTracking::init_potential_keyframe(pyramid_levels);
   SemiDenseTracking::init_potential_image_to_track(pyramid_levels);


   SemiDenseTracking::init_focalx(pyramid_levels);
   SemiDenseTracking::init_focaly(pyramid_levels);
   SemiDenseTracking::init_centerx(pyramid_levels);
   SemiDenseTracking::init_centery(pyramid_levels);
   SemiDenseTracking::init_image_keyframe_pyramid(pyramid_levels);
   SemiDenseTracking::init_points_map(pyramid_levels);
   SemiDenseTracking::init_color(pyramid_levels);
   SemiDenseTracking::init_points3D(pyramid_levels);
   SemiDenseTracking::init_image_reduced(pyramid_levels);
   SemiDenseTracking::init_error_vector(pyramid_levels);
   SemiDenseTracking::init_weight(pyramid_levels);
   SemiDenseTracking::init_points_map_inImage(pyramid_levels);
   SemiDenseTracking::init_local_maps(pyramid_levels*local_maps_number);
   SemiDenseTracking::init_poses_local_maps(local_maps_number);
   SemiDenseTracking::init_variances(pyramid_levels);
   SemiDenseTracking::init_evaluation();


   depth = 0.020*50;


   float distances_local_maps[local_maps_number];
   init_distances_local_maps(local_maps_number);

   for ( int jj =0;jj< local_maps_number;jj=jj+1)
   {
       distances_local_maps[jj] = INFINITY;
       set_distances_local_maps(distances_local_maps[jj],jj);
   }

   variance = (double)fs2["variance"];
   for (int j=0; j<pyramid_levels; j++)
   {variances[j] = variance;}

   image_n=0;
   last_frame_tracked = image_n;
   processed_frames = 0;
   processed_frames_since_keyframe = 0;

   create_inv_depth_discretization = 0;


   boost::filesystem::remove_all("src/dpptam/src/map_and_poses");
   boost::filesystem::create_directory("src/dpptam/src/map_and_poses");
   boost::filesystem::remove_all("src/dpptam/src/evaluation");
   boost::filesystem::create_directory("src/dpptam/src/evaluation");
   boost::filesystem::remove_all("src/dpptam/src/results_depth_maps");
   boost::filesystem::create_directory("src/dpptam/src/results_depth_maps");

   R = (cv::Mat_<double>(3, 3) <<  1,0,0,0,1,0,0,0,1);
   t = (cv::Mat_<double>(3, 1) << 0,0,0);
   cv::Mat R_kf;
   cv::Mat t_kf;


   R_kf = R.clone();
   t_kf = t.clone();
   R_prev = R.clone();
   R_post = R.clone();
   t_prev = t.clone();
   t_post = t.clone();


   tracking_th = (double)fs2["tracking_th"];

   discretization = (int)fs2["discretization"];


   local_maps_close_number = (int)fs2["local_maps_close_number"];
   local_maps_number = (int)fs2["local_maps_number"];
   SemiDenseTracking::init_points_last_keyframes(pyramid_levels);

   distortion =(int)fs2["distortion"];

   points_projected_in_image = 0;
   iter_th = (int)fs2["iter_th"];
    init_poses();



   reduction = 2;

   fs2.release();
}


void SemiDenseTracking::init_local_maps(int local_maps_number)
{
        local_maps.resize(local_maps_number);
}
void SemiDenseTracking::set_local_maps (cv::Mat local_maps_aux,int pos_map)
{
        local_maps[pos_map] = local_maps_aux.clone();
}


void SemiDenseTracking::init_poses () {
    cv::Mat poses_aux(0,3,CV_32FC1);
    poses = poses_aux.clone();
}

void SemiDenseTracking::set_poses (cv::Mat pose_aux) {
    poses.push_back(pose_aux);
}

void SemiDenseTracking::init_evaluation () {
    cv::Mat evaluation_aux(0,7,CV_64FC1);
    evaluation = evaluation_aux.clone();
}

void SemiDenseTracking::set_evaluation (cv::Mat evaluation_aux) {
    evaluation.push_back(evaluation_aux);
}


void SemiDenseTracking::init_poses_local_maps(int poses_local_maps_number)
{
    poses_local_maps.resize(poses_local_maps_number);
}
void SemiDenseTracking::set_poses_local_maps (cv::Mat poses_local_maps_aux,int pos_map) {
        poses_local_maps[pos_map] = poses_local_maps_aux.clone();
}

void SemiDenseTracking::init_points_last_keyframes(int points_last_keyframes_number)
{
    points_last_keyframes.resize(points_last_keyframes_number);
}

void SemiDenseTracking::set_points_last_keyframes(vector<cv::Mat> points_last_keyframes_aux) {
    for (int i = 0; i < points_last_keyframes_aux.size();i++)
    {
        points_last_keyframes[i] =points_last_keyframes_aux[i].clone();
    }
}

void SemiDenseTracking::init_distances_local_maps(int distances_local_maps_number)
{
    distances_local_maps.resize(distances_local_maps_number);
}

void SemiDenseTracking::set_distances_local_maps(float distances_local_maps_aux,int pos_map) {

    distances_local_maps[pos_map] =  distances_local_maps_aux;
}


void SemiDenseTracking::init_jacobian(int pyramid_levels)
{
    jacobian.resize(pyramid_levels);
}

void SemiDenseTracking::init_variances(int pyramid_levels)
{
    variances.resize(pyramid_levels);
}

void SemiDenseTracking::init_reduction_pyramid(int pyramid_levels)
{
    reduction_pyramid.resize(pyramid_levels);
}
void SemiDenseTracking::init_potential_image_to_track(int pyramid_levels)
{
    potential_image_to_track.resize(pyramid_levels);
}

void SemiDenseTracking::init_potential_keyframe(int pyramid_levels)
{
    potential_keyframe.resize(pyramid_levels);
}

void SemiDenseTracking::init_focalx(int pyramid_levels)
{
    focalx.resize(pyramid_levels);
}

void SemiDenseTracking::init_focaly(int pyramid_levels)
{
    focaly.resize(pyramid_levels);
}

void SemiDenseTracking::init_centerx(int pyramid_levels)
{
    centerx.resize(pyramid_levels);
}

void SemiDenseTracking::init_centery(int pyramid_levels)
{
   centery.resize(pyramid_levels);
}
void SemiDenseTracking::init_image_keyframe_pyramid(int pyramid_levels)
{
    image_keyframe_pyramid.resize(pyramid_levels);
}
void SemiDenseTracking::init_points_map(int pyramid_levels)
{
    points_map.resize(pyramid_levels);
}
void SemiDenseTracking::init_points_map_inImage(int pyramid_levels)
{
    points_map_inImage.resize(pyramid_levels);
}
void SemiDenseTracking::init_color(int pyramid_levels)
{
    color.resize(pyramid_levels);
}
void SemiDenseTracking::init_points3D(int pyramid_levels)
{
    points3D.resize(pyramid_levels);
}
void SemiDenseTracking::init_image_reduced(int pyramid_levels)
{
    image_reduced.resize(pyramid_levels);
}
void SemiDenseTracking::init_error_vector(int pyramid_levels)
{
    error_vector.resize(pyramid_levels);
}
void SemiDenseTracking::init_weight(int pyramid_levels)
{
    weight.resize(pyramid_levels);
}



void ThreadSemiDenseTracker(Imagenes *images,SemiDenseMapping *semidense_mapper,\
                            SemiDenseTracking *semidense_tracker,DenseMapping *dense_mapper,MapShared *Map,ros::Publisher *odom_pub,ros::Publisher *pub_poses,ros::Publisher *vis_pub,image_transport::Publisher *pub_image)
{
    while (ros::ok())
    {
           semidense_tracking(images,semidense_mapper,semidense_tracker,dense_mapper, Map,odom_pub,pub_poses,vis_pub,pub_image);
           boost::this_thread::sleep(boost::posix_time::milliseconds(1));
    }
}



void semidense_tracking(Imagenes *images,SemiDenseMapping *semidense_mapper,\
                        SemiDenseTracking *semidense_tracker,DenseMapping *dense_mapper,MapShared *Map,ros::Publisher *odom_pub,ros::Publisher *pub_poses,ros::Publisher *vis_pub,image_transport::Publisher *pub_image)
{
    if  (*semidense_tracker->cont_frames > semidense_tracker->last_cont_frames)
    {

            cv::Mat image_frame_aux  = (*semidense_tracker->image_frame).clone();
            double stamps_aux = *semidense_tracker->stamps;
             ros::Time stamps_ros= *semidense_tracker->stamps_ros;

            semidense_tracker->last_cont_frames = *semidense_tracker->cont_frames;
            prepare_image(image_frame_aux,semidense_tracker->image_rgb,semidense_tracker->image_to_track,\
                          semidense_tracker->image_n,semidense_tracker->image_gray,semidense_tracker->cameraMatrix,semidense_tracker->distCoeffs,\
                          semidense_tracker->fx,semidense_tracker->fy,semidense_tracker->cx,semidense_tracker->cy,semidense_tracker->distortion,semidense_tracker->reduction);


            if (semidense_tracker->image_n > semidense_tracker->last_frame_tracked)
            {
             if (semidense_mapper->do_initialization_tracking < 0.5 )
             {
                 semidense_tracker->image_prev = semidense_tracker->image_rgb.clone();
             }
             if (semidense_mapper->do_initialization_tracking > 0.5 )
             {

                 cv::Mat R_kf = images->Im[0]->R.clone();
                 cv::Mat t_kf = images->Im[0]->t.clone();
                 R_kf.convertTo(R_kf,CV_64FC1);
                 t_kf.convertTo(t_kf,CV_64FC1);

                 for (int i = 0;i<images->getNumberOfImages() ; i++)
                 {
                    delete images->Im[i];
                    images->Im[i] = NULL;
                 }
                 images->Im.clear();


                 for (int j=0; j<semidense_tracker->pyramid_levels ; j++)
                 {
                     semidense_tracker->points_map[j] = semidense_mapper->get_points_new_map()[j].clone();
                 }

                semidense_tracker->points_projected_in_image = 0;
                if (semidense_mapper->num_keyframes > semidense_mapper->init_keyframes  &&  semidense_tracker->get_points_last_keyframes()[0].rows > -10)
                {

                    join_maps(semidense_tracker->points_map, semidense_tracker->R,semidense_tracker->t,\
                               semidense_tracker->get_points_last_keyframes(),\
                               semidense_tracker->pyramid_levels,semidense_tracker->focalx,semidense_tracker->focaly,\
                               semidense_tracker->centerx,semidense_tracker->centery,semidense_tracker->image_reduced,\
                               semidense_tracker->points_projected_in_image);
                }


                initialization_sd(semidense_tracker,semidense_tracker->R,semidense_tracker->t,semidense_tracker->R_kf,\
                                  semidense_tracker->t_kf,semidense_tracker->image_prev,semidense_tracker->image_keyframe,\
                                  semidense_tracker->pyramid_levels, semidense_tracker->reduction_pyramid,\
                                  semidense_tracker->image_keyframe_pyramid,semidense_tracker->reduction);

                for (int j=0; j<semidense_tracker->pyramid_levels ; j++)
                {
                  semidense_tracker->points_map_inImage[j] = semidense_tracker->points_map[j].clone();
                }

                // tracked points: some 3D points may be unaccurate
                semidense_mapper->points_last_keyframe = \
                        semidense_tracker->points_map[semidense_tracker->pyramid_levels-1].clone();

                // mapped points: accure 3D points
                if (semidense_mapper->local_map_points.rows > 100)
                {semidense_mapper->points_last_keyframe = semidense_mapper->local_map_points.clone();}

                semidense_tracker->processed_frames_since_keyframe = 0;
                semidense_mapper->do_initialization_tracking = 0;
            }

            if (semidense_mapper->do_initialization > 0.5)
            {
                cv::Mat depth_frame;


                initialization(semidense_tracker->R,semidense_tracker->t,semidense_tracker->R_kf,semidense_tracker->t_kf,semidense_tracker->image_rgb,semidense_tracker->image_keyframe,semidense_tracker->pyramid_levels,semidense_tracker->reduction_pyramid,semidense_tracker->focalx,semidense_tracker->focaly,\
                             semidense_tracker->image_keyframe_pyramid,semidense_tracker->points_map,semidense_tracker->color,semidense_tracker->points3D,semidense_tracker->jacobian,semidense_tracker->error_vector,semidense_tracker->weight,semidense_tracker->fx,semidense_tracker->fy, semidense_tracker->depth,\
                               depth_frame,semidense_mapper->kinect_initialization,semidense_tracker->cx,semidense_tracker->cy,semidense_tracker->centerx,semidense_tracker->centery,semidense_tracker->reduction,semidense_tracker->image_gray,semidense_mapper->limit_grad );

                semidense_tracker->processed_frames_since_keyframe = 0;

                semidense_mapper->points_last_keyframe = semidense_tracker->points_map[semidense_tracker->pyramid_levels-1].clone();
                semidense_mapper->do_initialization = 0;

                for (int j=0; j<semidense_tracker->pyramid_levels ; j++)
                {
                    semidense_tracker->points_map_inImage[j] = semidense_tracker->points_map[j].clone();
                }
             }

             cv::Mat  points3D_cam_to_print;

             optimize_camera(semidense_mapper->num_keyframes,semidense_tracker,semidense_mapper,*images,semidense_tracker->image_to_track,\
                            semidense_tracker->image_rgb,semidense_tracker->R,semidense_tracker->t,\
                            semidense_tracker->R_kf,semidense_tracker->t_kf,semidense_tracker->image_reduced,\
                            semidense_tracker->image_keyframe_pyramid,semidense_tracker->variance,\
                            semidense_tracker->reduction, semidense_tracker->reduction_pyramid,semidense_tracker->processed_frames_since_keyframe,\
                            semidense_tracker->jacobian,semidense_tracker->points_map_inImage,semidense_tracker->color,\
                            semidense_tracker->points3D,semidense_tracker->error_vector,semidense_tracker->weight,\
                            semidense_tracker->focalx,semidense_tracker->focaly,semidense_tracker->centerx, semidense_tracker->centery,\
                            semidense_tracker->pyramid_levels,semidense_tracker->overlap_tracking,\
                            semidense_tracker->tracking_th,semidense_tracker->iter_th,semidense_tracker->variances,
                            semidense_tracker->image_gray, stamps_aux, stamps_ros,semidense_mapper->mean_value, odom_pub,pub_poses,vis_pub,points3D_cam_to_print);

              semidense_tracker->frames_processed++;
              if (images->getNumberOfImages() == 1)
              {cout << "frames_processed - >" <<100.0 * semidense_tracker->frames_processed / *semidense_tracker->cont_frames << " %" << endl;}


             Map->set_R(semidense_tracker->R);
             Map->set_t(semidense_tracker->t);

             semidense_tracker->R_post = semidense_tracker->R.clone();
             semidense_tracker->t_post = semidense_tracker->t.clone();


             if (semidense_tracker->points_projected_in_image < 5){semidense_tracker->points_projected_in_image = semidense_tracker->points_map[2].rows;}
             semidense_mapper->overlap_tracking = 1.0*semidense_tracker->points_map_inImage[2].rows / semidense_tracker->points_map[2].rows;

             if (semidense_tracker->image_n % 2 == 0)
             {
                 cv::Mat points3D_cam_show;
                 cv::Mat points3D6_print = semidense_tracker->points_map[semidense_tracker->pyramid_levels-1].colRange(0,3).t();

                 float focal_printx = semidense_tracker->focalx[semidense_tracker->pyramid_levels-1];
                 float focal_printy = semidense_tracker->focaly[semidense_tracker->pyramid_levels-1];
                 float center_printx = semidense_tracker->centerx[semidense_tracker->pyramid_levels-1];
                 float center_printy = semidense_tracker->centery[semidense_tracker->pyramid_levels-1];


                 cv::Mat image2print = semidense_tracker->image_reduced[semidense_tracker->pyramid_levels-1].clone();


                 points3D_cam_show = points3D_cam_to_print.clone();
                 int num_pixels_sd2project = points3D_cam_show.cols;


                 if (dense_mapper->get_superpixels3Dtracked().rows > 0)
                 {
                     points3D6_print = dense_mapper->get_superpixels3Dtracked().clone();
                     points3D6_print=points3D6_print.t();

                     cv::Mat points3D_superpixels;

                     cv::Mat R_prueba = semidense_tracker->R.clone();
                     R_prueba.convertTo(R_prueba,CV_64FC1);
                     cv::Mat t_prueba =semidense_tracker->t.clone();
                     transformed_points(points3D_superpixels,R_prueba ,t_prueba,focal_printx,\
                                      focal_printy,center_printx,center_printy,image2print,points3D6_print);

                     points3D_superpixels = points3D_superpixels.t();
                     points3D_cam_show = points3D_cam_show.t();
                     points3D_cam_show.push_back(points3D_superpixels);
                     points3D_cam_show = points3D_cam_show.t();
                 }
                 show_error1( points3D_cam_show,image2print,image_frame_aux,num_pixels_sd2project, pub_image,semidense_tracker->weight[semidense_tracker->pyramid_levels-1]);
             }

             cv::Mat t_print = -semidense_tracker->R.t()*semidense_tracker->t;

             cv::Mat evaluation_frame(1,7,CV_64FC1);
             if ( semidense_mapper->num_keyframes > semidense_mapper->init_keyframes && semidense_tracker->image_n%15==0)
             {
                 evaluation_frame.at<double>(0,0) =  *semidense_tracker->stamps;
                 evaluation_frame.at<double>(0,1) = (0.025*30)*t_print.at<double>(0,0);
                 evaluation_frame.at<double>(0,2) = (0.025*30)*t_print.at<double>(1,0);
                 evaluation_frame.at<double>(0,3) = (0.025*30)*t_print.at<double>(2,0);
                 evaluation_frame.at<double>(0,4) = 0;
                 evaluation_frame.at<double>(0,5) = 0;
                 evaluation_frame.at<double>(0,6) = 0;
                 evaluation_frame.at<double>(0,7) = 0;
                 semidense_tracker->set_evaluation(evaluation_frame);
             }

             if ( semidense_mapper->num_keyframes %2==0)
             {
                 char buffer_evaluation[150];
                 sprintf(buffer_evaluation,"src/dpptam/src/evaluation/evaluation%d.txt",semidense_mapper->num_keyframes);
                 print_evaluation(semidense_tracker->evaluation,buffer_evaluation);
             }

             if (semidense_tracker->create_inv_depth_discretization < 0.5)
             {
                 float depth_step=0;
                 float mean_value;
                 cv::Mat depth_map,variance_points_tracked;

                 semidense_tracker->create_inv_depth_discretization=1;
                 semidense_mapper->points_last_keyframe.convertTo(semidense_mapper->points_last_keyframe, CV_32FC1);
                 get_inverse_depth(*images,semidense_mapper->points_last_keyframe,semidense_mapper->inv_depths,depth_step,0,semidense_tracker->discretization,mean_value,depth_map,1,variance_points_tracked);
                 semidense_mapper->points_last_keyframe.convertTo(semidense_mapper->points_last_keyframe, CV_64FC1);
                 semidense_tracker->mean_depth_value=mean_value;
             }

             semidense_tracker->last_frame_tracked = semidense_tracker->image_n;

             semidense_tracker->processed_frames++;
             semidense_tracker->processed_frames_since_keyframe++;

             semidense_mapper->last_frame_tracked = semidense_tracker->last_frame_tracked;
             semidense_mapper->images_size = images->getNumberOfImages();
          }
    } // if cont_frames >
}


void prepare_image(cv::Mat &image_frame, cv::Mat &image_rgb,cv::Mat &image_to_track,int &image_n,cv::Mat &image_gray,cv::Mat cameraMatrix,cv::Mat distCoeffs,\
                   double &fx,double &fy, double &cx, double &cy, int distortion, int reduction)
{
    if (distortion > 0.5)
    {
         if (image_frame.type()==CV_8UC1) {
                //input image is grayscale
                cv::cvtColor(image_frame, image_frame, CV_GRAY2RGB);
         }
         cv::resize(image_frame,image_frame,cv::Size(image_frame.cols/reduction,image_frame.rows/reduction),0,0,cv::INTER_LINEAR);


         cv::Mat image_ff = image_frame.clone();

         cv::Size ksize,ksize1;
         ksize.width = image_frame.cols;
         ksize.height = image_frame.rows;

         cv::Mat newCameraMatrix;

         cv::Mat cameraMatrixAux = cameraMatrix.clone();
         cameraMatrixAux/=reduction;
         cameraMatrixAux.at<double>(2,2)=1;

         double alpha = 0;
         newCameraMatrix = cv::getOptimalNewCameraMatrix(cameraMatrixAux,distCoeffs,ksize,alpha,ksize1);


         cv::undistort(image_frame,image_ff,cameraMatrixAux,distCoeffs,newCameraMatrix);

         image_frame = image_ff.clone();

         fx = newCameraMatrix.at<double>(0,0);
         fy = newCameraMatrix.at<double>(1,1);
         cx = newCameraMatrix.at<double>(0,2);
         cy = newCameraMatrix.at<double>(1,2);
    }

    image_rgb = image_frame.clone();
    image_n++;

    cv::cvtColor(image_rgb,image_to_track,CV_RGB2GRAY);

    image_to_track.convertTo(image_to_track, CV_64FC1);
    image_gray = image_to_track.clone();

    image_to_track /= (255*1.0);
}

void initialization_sd(SemiDenseTracking *semidense_tracker,cv::Mat &R,cv::Mat &t,cv::Mat &R1,cv::Mat &t1,cv::Mat &image_rgb,cv::Mat &image_keyframe,\
                    int &pyramid_levels,vector<int> &reduction_pyramid, vector<cv::Mat> &image_keyframe_pyramid, int reduction)
{

   image_rgb.copyTo(image_keyframe);
   cv::Mat image_p = image_rgb.clone();

   R.copyTo(R1);
   t.copyTo(t1);

   cv::cvtColor(image_p,image_p,CV_RGB2GRAY);
   image_p.convertTo(image_p, CV_64FC1);
   image_p /= (255*1.0);


   for (int j=0; j<pyramid_levels ; j++)
   {
         cv::resize(image_p,image_keyframe_pyramid[j],cv::Size(image_p.cols/(reduction_pyramid[j]/reduction),image_p.rows/(reduction_pyramid[j]/reduction)),0,0,cv::INTER_LINEAR);
         get_color(semidense_tracker->points_map[j],semidense_tracker->color[j]);
         semidense_tracker->points3D[j] = semidense_tracker->points_map[j].colRange(0,3).t();
         semidense_tracker->error_vector[j] = cv::Mat::zeros(semidense_tracker->points_map[j].rows,1,CV_64FC1);
         semidense_tracker->weight[j] = cv::Mat::zeros(semidense_tracker->points_map[j].rows,1,CV_64FC1) + 1;
         semidense_tracker->jacobian[j] = cv::Mat::zeros(semidense_tracker->points_map[j].rows,6,CV_64FC1);
   }
}

void initialization(cv::Mat &R,cv::Mat &t,cv::Mat &R1,cv::Mat &t1,cv::Mat &image_rgb,cv::Mat &image_keyframe,\
                    int &pyramid_levels,vector<int> &reduction_pyramid, vector<double> &focalx,vector<double> &focaly,vector<cv::Mat> &image_keyframe_pyramid,\
                    vector<cv::Mat> &points_map,vector<cv::Mat> &color,vector<cv::Mat> &points3D,\
                    vector<cv::Mat> &jacobian,vector<cv::Mat> &error_vector,vector<cv::Mat> &weight, double fx,double fy, double depth,\
                    cv::Mat depth_frame, int kinect_initialization,\
                    double cx,double cy,  vector<double> &centerx,vector<double> &centery, int reduction,cv::Mat image_gray,float limit_grad)
{
   cv::Mat points_aux(0,6,CV_64FC1);

   image_rgb.copyTo(image_keyframe);

   cv::Mat image_p = image_rgb.clone();

   cv::Mat points_i(0,6, CV_32FC1);
   cv::Mat points_i_print(0,6, CV_32FC1);
   cv::Mat point_i(1,6, CV_32FC1);

   int red_im = reduction;
   cv::Mat image_red = image_rgb.clone();


   double f_x = fx;
   double f_y = fy;
   double c_x = cx;
   double c_y = cy;



   cv::Mat points = points_aux.clone();

   double depth_aux = depth;


   for (int j = 0; j < image_red.rows-0; j = j+1)
   {
   for (int i = 0; i < image_red.cols-0; i = i+1)
   {


           point_i.at<float>(0,1) =  -((c_y-j)/f_y)/(-depth);
           point_i.at<float>(0,0) =  -((i-c_x)/f_x)/(-depth);
           point_i.at<float>(0,2) =  1 / (-depth);
           //point_i.at<float>(0,2) =  point_i.at<float>(0,2) - 0.05*point_i.at<float>(0,2)  + 0.1*point_i.at<float>(0,2) * ((rand() % 1000000 ) / 1000000.0);
           point_i.at<float>(0,3) =  image_gray.at<double>(j,i);
           point_i.at<float>(0,4) =  image_gray.at<double>(j,i);
           point_i.at<float>(0,5) =  image_gray.at<double>(j,i);
           points_i.push_back(point_i);


       }
   }
   points_i.convertTo(points_i,CV_64FC1);
   points_i.copyTo(points);
   points_i_print.convertTo(points_i_print,CV_64FC1);


   R.copyTo(R1);
   t.copyTo(t1);

   cv::cvtColor(image_p,image_p,CV_RGB2GRAY);
   image_p.convertTo(image_p, CV_64FC1);
   image_p /= (255*1.0);

   int reduction_ = pow(2,pyramid_levels);

   for (int j=0; j<pyramid_levels ; j++)
   {
         reduction_pyramid[j] = reduction_;
         reduction_/=2;
         cv::resize(image_p,image_keyframe_pyramid[j],cv::Size(image_p.cols/(reduction_pyramid[j]/reduction),image_p.rows/(reduction_pyramid[j]/reduction)),0,0,cv::INTER_LINEAR);

         cv::Mat image_p1 = image_p.clone();
         resize_points(points_map[j],points,reduction_pyramid[j]/reduction,image_p1,kinect_initialization,limit_grad);

         get_color(points_map[j],color[j]);
         points3D[j] = points_map[j].colRange(0,3).t();
         error_vector[j] = cv::Mat::zeros(points_map[j].rows,1,CV_64FC1);
         weight[j] = cv::Mat::zeros(points_map[j].rows,1,CV_64FC1) + 1;
         jacobian[j] = cv::Mat::zeros(points_map[j].rows,6,CV_64FC1);
         focalx[j] = fx/(reduction_pyramid[j]/reduction);
         focaly[j] = fy/(reduction_pyramid[j]/reduction);

         centerx[j] = cx/(reduction_pyramid[j]/reduction);
         centery[j] = cy/(reduction_pyramid[j]/reduction);
   }
}

void remove_outside_points(vector<cv::Mat> &points_map,cv::Mat R,cv::Mat t,vector<cv::Mat> &image_reduced,cv::Mat R_p,cv::Mat t_p,int pyramid_levels,\
                vector<double> &focalx,vector<double> &focaly,vector<double> &centerx,vector<double> &centery,
                           vector<cv::Mat> &image_keyframe_pyramid, vector<cv::Mat> &weight, vector<double> &variances,double &variance,cv::Mat &points3D_cam_to_print )

{
    vector<cv::Mat> color(pyramid_levels);

    for (int i = 0; i < pyramid_levels;i++)
    {

            int imsize_y = image_keyframe_pyramid[i].rows - 1 ;
            int imsize_x = image_keyframe_pyramid[i].cols - 1 ;

            int cont2 =0;


            cv::Mat weight_aux1(0,1,CV_64FC1);
            cv::Mat points_joined(0,7,CV_64FC1);
            cv::Mat points_joined_previous_map(0,7,CV_64FC1);
            cv::Mat pointsClouds3Dmap_cam,pointsClouds3Dmap_cam_p,pointsClouds3Dmap_cam_transpose;


            cv::Mat error_vector(points_map[i].rows,1,CV_64FC1);
            cv::Mat error_vector_sqrt(points_map[i].rows,1,CV_64FC1);
            cv::Mat weight_aux2(points_map[i].rows,1,CV_64FC1);
            cv::Mat points3Dcam_to_print_aux(0,3,CV_64FC1);
            weight_aux2 = 1;
            cv::Mat weight_aux3;

            double error_opt;
            get_color(points_map[i],color[i]);



            cv::Mat pointsClouds3D =  points_map[i].t();
            pointsClouds3D = pointsClouds3D .rowRange(0,3);


            transformed_points(pointsClouds3Dmap_cam, R,   t,   focalx[i],focaly[i],centerx[i],centery[i],image_reduced[i], pointsClouds3D   );
            transformed_points(pointsClouds3Dmap_cam_p, R_p,   t_p,   focalx[i],focaly[i],centerx[i],centery[i],image_keyframe_pyramid[i], pointsClouds3D   );

            compute_error_ic( pointsClouds3Dmap_cam,pointsClouds3Dmap_cam_p,image_reduced[i],image_keyframe_pyramid[i],error_vector,variance,error_vector_sqrt,error_opt,weight_aux2,weight_aux2);

            pointsClouds3Dmap_cam_transpose = pointsClouds3Dmap_cam.t();


            weight_aux3 = (1 + (error_vector /(variances[i])));
            weight_aux3 = 1/weight_aux3;
            weight_aux3 = weight_aux3.mul(weight_aux3);

            int corner = 2;
            if (i == 1){corner = 4;}
            if (i == 2){corner = 8;}
            for(int k = 0; k < pointsClouds3Dmap_cam.cols;k++)
            {
                if (  pointsClouds3Dmap_cam.at<double>(1,k) > 0+corner && pointsClouds3Dmap_cam.at<double>(1,k) < imsize_y-corner &&   pointsClouds3Dmap_cam.at<double>(0,k) > 0+corner &&pointsClouds3Dmap_cam.at<double>(0,k) < imsize_x-corner)
                {
                    cont2++;
                    points_joined_previous_map.push_back(points_map[i].row(k));
                    weight_aux1.push_back(weight_aux3.at<double>(k,0));

                    if (i == pyramid_levels-1)
                    {
                        points3Dcam_to_print_aux.push_back(pointsClouds3Dmap_cam_transpose.row(k));
                    }
                }
            }
            points_map[i] = points_joined_previous_map.clone();
            weight_aux1.copyTo(weight[i]);


            if (i == pyramid_levels-1)
            {
                points3Dcam_to_print_aux.copyTo(points3D_cam_to_print);
                points3D_cam_to_print = points3D_cam_to_print.t();
            }
    }


}

void get_color (cv::Mat &points,cv::Mat &color)
{
    color = points.colRange(3,6);

    color.convertTo(color,CV_64FC1);
    color.rowRange(0,color.rows).colRange(0,1) = 0.299*color.rowRange(0,color.rows).colRange(0,1) + 0.587*color.rowRange(0,color.rows).colRange(1,2) +0.114*color.rowRange(0,color.rows).colRange(2,3);
    color.rowRange(0,color.rows).colRange(0,1).copyTo(color);

    color /=(255*1.0);
}

void motion_model(vector<cv::Mat> &points_map,cv::Mat &R,cv::Mat &t,cv::Mat R_rel,cv::Mat t_rel,\
                vector<double> &focalx, vector<double> &focaly, vector<double> &centerx, vector<double> &centery,
                  vector<cv::Mat> &image_keyframe_pyramid,int pyramid_levels, bool &good_seed)
{

    cv::Mat pointsClouds3Dmap_cam;
    vector<cv::Mat> color(pyramid_levels);

    int i = 0;
    get_color(points_map[i],color[i]);
    cv::Mat error_vector(points_map[i].rows,1,CV_64FC1);
    cv::Mat error_vector_sqrt(points_map[i].rows,1,CV_64FC1);
    cv::Mat error_check(points_map[i].rows,1,CV_64FC1);
    cv::Mat weight(points_map[i].rows,1,CV_64FC1);

    double variance=0.003;


    cv::Mat pointsClouds3D =  points_map[i].t();
    pointsClouds3D = pointsClouds3D .rowRange(0,3);

    transformed_points(pointsClouds3Dmap_cam, R,   t,   focalx[i],focaly[i],centerx[i],centery[i],image_keyframe_pyramid[i], pointsClouds3D   );
    compute_error( pointsClouds3Dmap_cam,image_keyframe_pyramid[i], color[i],error_vector,variance,error_vector_sqrt,error_check,weight);
    float error1 = mean(error_check)[0];
    cv::Mat t1 =  R*t_rel+t;
    cv::Mat R1 =  R*R_rel;

    transformed_points(pointsClouds3Dmap_cam, R1,   t1,   focalx[i],focaly[i],centerx[i],centery[i],image_keyframe_pyramid[i], pointsClouds3D   );
    compute_error( pointsClouds3Dmap_cam,image_keyframe_pyramid[i], color[i],error_vector,variance,error_vector_sqrt,error_check,weight);
    float error2 = mean(error_check)[0];


    if (error2 < error1)
    {
        good_seed=true;
        R = R1.clone();
        t = t1.clone();
    }  

}


void     publish_camera_frame(cv::Mat R,cv::Mat t,ros::Publisher *vis_pub)
{

    visualization_msgs::Marker marker;

    marker.header.frame_id = "dpptam/map";
   marker.id=4;
   marker.type = visualization_msgs::Marker::LINE_LIST;
   marker.scale.x=0.02;//0.2; 0.03
   marker.pose.orientation.w=1.0;
   marker.action=visualization_msgs::Marker::ADD;
   marker.color.r=1.0f;
   marker.color.a = 1.0;

   marker.points.clear();

    float d = 0.4;

    cv::Mat Tcw = (cv::Mat_<float>(4,4) << R.at<double>(0,0),  R.at<double>(0,1),  R.at<double>(0,2), t.at<double>(0,0),
                                         R.at<double>(1,0),  R.at<double>(1,1),  R.at<double>(1,2), t.at<double>(1,0),
                                         R.at<double>(2,0),  R.at<double>(2,1),  R.at<double>(2,2), t.at<double>(2,0),
                                         0,0,0,1 );


    //Camera is a pyramid. Define in camera coordinate system
    cv::Mat o = (cv::Mat_<float>(4,1) << 0, 0, 0, 1);
    cv::Mat p1 = (cv::Mat_<float>(4,1) << -d/2, -d*0.8/2, -d*0.5/1, 1);
    cv::Mat p2 = (cv::Mat_<float>(4,1) << -d/2, +d*0.8/2, -d*0.5/1, 1);
    cv::Mat p3 = (cv::Mat_<float>(4,1) << +d/2, +d*0.8/2, -d*0.5/1, 1);
    cv::Mat p4 = (cv::Mat_<float>(4,1) << +d/2, -d*0.8/2, -d*0.5/1, 1);

    cv::Mat Twc = Tcw.inv();
    cv::Mat ow = Twc*o;
    cv::Mat p1w = Twc*p1;
    cv::Mat p2w = Twc*p2;
    cv::Mat p3w = Twc*p3;
    cv::Mat p4w = Twc*p4;

    geometry_msgs::Point msgs_o,msgs_p1, msgs_p2, msgs_p3, msgs_p4;
    msgs_o.x=ow.at<float>(0);
    msgs_o.y=ow.at<float>(1);
    msgs_o.z=ow.at<float>(2);
    msgs_p1.x=p1w.at<float>(0);
    msgs_p1.y=p1w.at<float>(1);
    msgs_p1.z=p1w.at<float>(2);
    msgs_p2.x=p2w.at<float>(0);
    msgs_p2.y=p2w.at<float>(1);
    msgs_p2.z=p2w.at<float>(2);
    msgs_p3.x=p3w.at<float>(0);
    msgs_p3.y=p3w.at<float>(1);
    msgs_p3.z=p3w.at<float>(2);
    msgs_p4.x=p4w.at<float>(0);
    msgs_p4.y=p4w.at<float>(1);
    msgs_p4.z=p4w.at<float>(2);

    marker.points.push_back(msgs_o);
    marker.points.push_back(msgs_p1);
    marker.points.push_back(msgs_o);
    marker.points.push_back(msgs_p2);
    marker.points.push_back(msgs_o);
    marker.points.push_back(msgs_p3);
    marker.points.push_back(msgs_o);
    marker.points.push_back(msgs_p4);
    marker.points.push_back(msgs_p1);
    marker.points.push_back(msgs_p2);
    marker.points.push_back(msgs_p2);
    marker.points.push_back(msgs_p3);
    marker.points.push_back(msgs_p3);
    marker.points.push_back(msgs_p4);
    marker.points.push_back(msgs_p4);
    marker.points.push_back(msgs_p1);

    marker.header.stamp = ros::Time::now();

   vis_pub->publish(marker);

}

void optimize_camera(int num_keyframes,SemiDenseTracking *semidense_tracker,SemiDenseMapping *semidense_mapper,Imagenes &images,cv::Mat &image_to_track,\
                    cv::Mat &image_rgb,cv::Mat &R,cv::Mat &t,cv::Mat &R1,cv::Mat &t1,\
                    vector<cv::Mat> &image_reduced, vector<cv::Mat> &image_keyframe_pyramid, double &variance,\
                    int &reduction, vector<int> &reduction_pyramid,
                    int &processed_frames, vector<cv::Mat> &jacobian, vector<cv::Mat> &points_map,\
                    vector<cv::Mat> &color,vector<cv::Mat> &points3D,vector<cv::Mat> &error_vector,vector<cv::Mat> &weight,vector<double> &focalx,vector<double> &focaly,\
                    vector<double> &centerx,vector<double> &centery,int &pyramid_levels, double &overlap_tracking,double &tracking_th, int iter_th,\
                    vector<double> &variances,cv::Mat &image_gray, double stamps, ros::Time stamps_ros,float mean_depth_value,\
                     ros::Publisher *odom_pub,ros::Publisher *pub_poses,ros::Publisher *vis_pub,cv::Mat &points3D_cam_to_print)
{
    for (int j = 0 ; j < pyramid_levels; j++)
    {
      cv::resize(image_to_track,image_reduced[j],cv::Size(image_to_track.cols/(reduction_pyramid[j]/reduction),image_to_track.rows/(reduction_pyramid[j]/reduction)),0,0,cv::INTER_LINEAR);
    }


    cv::Mat R_rel =semidense_tracker->R_prev.t()*semidense_tracker->R_post;
    cv::Mat t_rel = semidense_tracker->R_prev.t()*(semidense_tracker->t_post-semidense_tracker->t_prev);
    semidense_tracker->R_prev = semidense_tracker->R.clone();
    semidense_tracker->t_prev = semidense_tracker->t.clone();


    cv::Mat t_aux2 =  R*t_rel+t;
    cv::Mat R_aux2 =  R*R_rel;

    /// MOTION MODEL
    if (num_keyframes >  0 )
    {
           t_aux2 =  R*t_rel+t;
            R_aux2 =  R*R_rel;

            bool good_seed = false;
            motion_model(semidense_tracker->points_map_inImage,semidense_tracker->R,semidense_tracker->t,R_rel,t_rel,semidense_tracker->focalx,\
            semidense_tracker->focaly,semidense_tracker->centerx,semidense_tracker->centery,\
            semidense_tracker->image_reduced,semidense_tracker->pyramid_levels,good_seed);
    }
    /// MOTION MODEL


    double error_p = 0;
    int iter = 0;


    R1.convertTo(R1,CV_64FC1);
    t1.convertTo(t1,CV_64FC1);
    cv::Mat R_p ;
    R1.copyTo(R_p);

    cv::Mat t_p;
    t1.copyTo(t_p);

    /// CAMERA POSE ESTIMATION AND UPDATING ROBUST COST FUNCTION
    for (int j = 0 ; j < pyramid_levels; j++)
    {

        gauss_estimation(semidense_tracker,points3D_cam_to_print,R,t,R_p,t_p,\
                         focalx[j],focaly[j],centerx[j],centery[j],points_map[j],image_reduced[j],\
                         image_keyframe_pyramid[j],error_p,color[j],iter,variances[j],points3D[j],error_vector[j],weight[j],processed_frames,\
                         jacobian[j],overlap_tracking,tracking_th,iter_th);

        cv::Mat sorted_variances;
        cv::sort(cv::abs(error_vector[j]),sorted_variances,CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);
        variances[j] = sqrt(sorted_variances.at<double>(round(sorted_variances.rows/2),0));

        if(variances[j] == 0)
        {
            variances[j] =variance;
        }

       weight[j] = (1 + (error_vector[j]/(variances[j])));
       weight[j] = 1/weight[j];
       weight[j] = weight[j].mul(weight[j]);
    }
    /// CAMERA POSE ESTIMATION AND UPDATING ROBUST COST FUNCTION



    /// SEND CURRENT TRACKED MAP TO THE MAPPING THREAD AS INITIAL SEED
    if (images.getNumberOfImages() == 0)
    {
                semidense_mapper->points3D_tracked  =  points_map[pyramid_levels-1].clone();
                semidense_mapper->weight_tracked_points =    weight[pyramid_levels-1].clone();
                semidense_mapper->image_coordinates_tracked_points = points3D_cam_to_print.t();

                cv::Mat error_vector_sqrt(points_map[pyramid_levels-1].rows,1,CV_64FC1);
                cv::Mat weight_aux2(points_map[pyramid_levels-1].rows,1,CV_64FC1);
                cv::Mat error_check(0,1,CV_64FC1);

                compute_error( points3D_cam_to_print,image_reduced[pyramid_levels-1], color[pyramid_levels-1],error_vector[pyramid_levels-1],variance,error_vector_sqrt,error_check,weight_aux2);


                semidense_mapper->points3D_tracked  =  points_map[pyramid_levels-1].clone();
                semidense_mapper->weight_tracked_points =    error_vector[pyramid_levels-1].clone();
                semidense_mapper->image_coordinates_tracked_points = points3D_cam_to_print.t();
    }
    /// SEND CURRENT TRACKED MAP TO THE MAPPING THREAD AS INITIAL SEED



    tf::TransformBroadcaster mTfBr;
    cv::Mat Rwc = R.t();
    cv::Mat twc = -Rwc*t;


    tf::Matrix3x3 M(Rwc.at<double>(0,0),Rwc.at<double>(0,1),Rwc.at<double>(0,2),
                    Rwc.at<double>(1,0),Rwc.at<double>(1,1),Rwc.at<double>(1,2),
                    Rwc.at<double>(2,0),Rwc.at<double>(2,1),Rwc.at<double>(2,2));
    tf::Vector3 V(twc.at<double>(0,0), twc.at<double>(1,0), twc.at<double>(2,0));
    tf::Transform tfTcw(M,V);

    mTfBr.sendTransform(tf::StampedTransform(tfTcw,ros::Time::now(), "dpptam/map", "dpptam/visualization_marker"));



    publish_camera_frame(R,t,vis_pub);



   char buffer[150];

    if (semidense_tracker->image_n % 5 == 0)
    {sprintf(buffer,"src/dpptam/src/map_and_poses/tracking.ply");
    print_poses(semidense_tracker->poses,buffer);}



    cv::Mat R_cv32fc1,t_cv32fc1;
    R.convertTo(R_cv32fc1,CV_32FC1);
    t.convertTo(t_cv32fc1,CV_32FC1);


    cv::Mat C(3,1,CV_64FC1);
    C= -R.t()*t;

    cv::Mat D_print = C.clone();

    double color_pose = 255;
    if (images.getNumberOfImages() < 5)
    {
        D_print.push_back(color_pose);
    }
    else
    {
        color_pose = 0;
        D_print.push_back(color_pose);
    }
    semidense_tracker->set_poses(D_print.t());


    float translational_ratio = 1;

    if (images.getNumberOfImages() > 0)
    {
        cv::Mat R2,t2,R1,t1;
        R2 =  images.Im[images.getNumberOfImages()-1]->R;
        t2 =  images.Im[images.getNumberOfImages()-1]->t;


        R1 =R.clone();
        t1 = t.clone();


        t2.convertTo(t2,CV_32FC1);
        R2.convertTo(R2,CV_32FC1);


        cv::Mat C1 = -R1.t()*t1;
        cv::Mat C2 = -R2.t()*t2;


        C2.convertTo(C2,CV_64FC1);
        C1.convertTo(C1,CV_64FC1);

        translational_ratio = (fabs(C1.at<double>(0,0) - C2.at<double>(0,0)) + fabs(C1.at<double>(1,0) - C2.at<double>(1,0)) + \
                               fabs(C1.at<double>(2,0) - C2.at<double>(2,0)) )  /  mean_depth_value;
    }


    if (  images.getNumberOfImages()< 0.5 || num_keyframes < 1 || translational_ratio >  0.0050)
    {
        images.computeImage();
        int cont_images = images.getNumberOfImages()-1;

        images.Im[cont_images]->R = R_cv32fc1.clone();
        images.Im[cont_images]->t = t_cv32fc1.clone();
        images.Im[cont_images]->k1 = 0;
        images.Im[cont_images]->k2 = 0;
        images.Im[cont_images]->fx = focalx[pyramid_levels-1];
        images.Im[cont_images]->fy = focaly[pyramid_levels-1];
        images.Im[cont_images]->cx = centerx[pyramid_levels-1];
        images.Im[cont_images]->cy = centery[pyramid_levels-1];
        images.Im[cont_images]->num_keyframes = num_keyframes;


        images.Im[cont_images]->stamps = stamps;
        images.Im[cont_images]->is_used_for_mapping = 0;


        cv::Mat image_gray1 = image_gray.clone();
        cv::Mat image_rgb1 = image_rgb.clone();
        cv::resize(image_gray,image_gray1,cv::Size(image_gray.cols/(reduction_pyramid[pyramid_levels-1]/reduction),image_gray.rows/(reduction_pyramid[pyramid_levels-1]/reduction)),0,0,cv::INTER_LINEAR);
        cv::resize(image_rgb,image_rgb1,cv::Size(image_rgb.cols/(reduction_pyramid[pyramid_levels-1]/reduction),image_rgb.rows/(reduction_pyramid[pyramid_levels-1]/reduction)),0,0,cv::INTER_LINEAR);

        images.Im[cont_images]->image_gray = image_gray1.clone();
        images.Im[cont_images]->image = image_rgb1.clone();
        images.Im[cont_images]->error = error_p;
    }
}

void show_error1( cv::Mat points3D_cam, cv::Mat image,  cv::Mat image_print, int num_pixels_sd2project,image_transport::Publisher *pub_image,cv::Mat &weight)

{
    int imsize_x =image.cols-1;
    int imsize_y =image.rows-1;

    for (int k =0; k< points3D_cam.cols;k++)

    {
       if (points3D_cam.at<double>(1,k) > 0 && points3D_cam.at<double>(1,k) < imsize_y && points3D_cam.at<double>(0,k) > 0 && points3D_cam.at<double>(0,k) < imsize_x)
         {
            //BILINEAR INTERPOLATION
           double x_2 = points3D_cam.at<double>(0,k);
           double y_2 = points3D_cam.at<double>(1,k);

           int xx = static_cast<int>(x_2);
           int yy = static_cast<int>(y_2);


           image_print.at<cv::Vec3b>(yy,xx)[2] = 0;
           image_print.at<cv::Vec3b>(yy,xx)[0] = 0;
           image_print.at<cv::Vec3b>(yy,xx)[1] = 0;

           if (k<num_pixels_sd2project )
           {
               image_print.at<cv::Vec3b>(yy,xx)[2] = round(190.0*weight.at<double>(k,0));
           }
           else
           {image_print.at<cv::Vec3b>(yy,xx)[0] = 255;}
        }
    }

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8",image_print).toImageMsg();
    pub_image->publish(msg);

}

void print_evaluation(cv::Mat points,  char buffer[])
{
     ofstream out(buffer);
     int num_points = points.rows;


     int val = points.rows-1;
     val = num_points;

     for (int i = 0; i<= points.rows-1;i++)
     {

         double val1 = points.at<double>(i,0);
         double val2 = points.at<double>(i,1);
         double val3 = points.at<double>(i,2);

         double val4 = points.at<double>(i,3);
         double val5 = points.at<double>(i,4);
         double val6 = points.at<double>(i,5);
         double val7 = points.at<double>(i,6);


        //if  (color4 != Intensityy.at<cv::Vec3b>(0,0)[0])
        {

            out << fixed << setprecision(5) << val1<< " " << fixed << setprecision(5) << val2 <<  " " << fixed << setprecision(5) << val3 \
            << " "<< val4 << " "<< val5 << " "<< val6  << " "<< val7 << endl;

        }
     }
     out.close();
 }


void print_plane(cv::Mat &points, char buffer[])
{
     ofstream out(buffer);
     int num_points = points.rows;
     int val = points.rows-1;

     val = num_points;


     out << "ply" << endl;out << "format ascii 1.0" << endl;out << "element face 0" << endl;out << "property list uchar int vertex_indices" << endl;
     out << "element vertex ";out << val << endl;out << "property float x" << endl;out << "property float y" << endl;out << "property float z" << endl;
     out <<  "property uchar diffuse_red"<<endl;out << "property uchar diffuse_green" << endl;out << "property uchar diffuse_blue" << endl;out << "end_header" << endl;
     for (int i = 0; i<= points.rows-1;i++)
     {

         double val1 = points.at<double>(i,0);
         double val2 = points.at<double>(i,1);
         double val3 = points.at<double>(i,2);
         double val4 = points.at<double>(i,3);
         double val5 = points.at<double>(i,4);
         double val6 = points.at<double>(i,5);

        int color1 = val4;
        int color2 =  val5 ;
        int color3 =  val6 ;

        out << fixed << setprecision(5) << val1<< " " << fixed << setprecision(5) << val2 <<  " " << fixed << setprecision(5) << val3 \
                   << " "<< color1 << " "<< color2 << " "<< color3 << endl;

     }
     out.close();
 }


void print_times(cv::Mat &points, char buffer[])
{


    ofstream out(buffer);

     int num_points = points.rows;

     int val = points.rows-1;


     val = num_points;

     for (int i = 0; i<= points.rows-1;i++)
     {
         double val1 = points.at<double>(i,0);
         out << fixed << setprecision(5) << val1 << endl;
     }
     out.close();
 }


void resize_points(cv::Mat  &points2,cv::Mat  &points, double reduction,cv::Mat &image_p, int kinect_initialization,float limit_grad)
{
    //cv::Mat color = points.colRange(3,6);
    cv::Mat points_x,points_y,points_z,points_R,points_G,points_B;
    points.colRange(0,1).copyTo(points_x);
    points.colRange(1,2).copyTo(points_y);
    points.colRange(2,3).copyTo(points_z);
    points.colRange(3,4).copyTo(points_R);
    points.colRange(4,5).copyTo(points_G);
    points.colRange(5,6).copyTo(points_B);




    //int rows = image_p.rows/2-20;
    int rows = image_p.rows;
    points_x =  points_x.reshape(0,rows);
    points_y =  points_y.reshape(0,rows);
    points_z =  points_z.reshape(0,rows);
    points_R =  points_R.reshape(0,rows);
    points_G =  points_G.reshape(0,rows);
    points_B =  points_B.reshape(0,rows);

    int reduction_r = reduction;

    cv::resize(points_x,points_x,cv::Size(points_x.cols/reduction_r,points_x.rows/reduction_r),0,0,cv::INTER_LINEAR);
    cv::resize(points_y,points_y,cv::Size(points_y.cols/reduction_r,points_y.rows/reduction_r),0,0,cv::INTER_LINEAR);
    cv::resize(points_z,points_z,cv::Size(points_z.cols/reduction_r,points_z.rows/reduction_r),0,0,cv::INTER_LINEAR);
    cv::resize(points_R,points_R,cv::Size(points_R.cols/reduction_r,points_R.rows/reduction_r),0,0,cv::INTER_LINEAR);
    cv::resize(points_G,points_G,cv::Size(points_G.cols/reduction_r,points_G.rows/reduction_r),0,0,cv::INTER_LINEAR);
    cv::resize(points_B,points_B,cv::Size(points_B.cols/reduction_r,points_B.rows/reduction_r),0,0,cv::INTER_LINEAR);



    cv::Mat points_todos3 = points_R*0.07 + points_G *0.72 + points_B*0.21;
    points_todos3 /=255;

    cv::Mat GX;
    cv::Mat GY;
    GX=gradientX(points_todos3,1);
    GY=gradientY(points_todos3,1);

    cv::Mat G = cv::abs(GX)  + cv::abs(GY);

    float alpha = 10;
    cv::exp(-G*alpha,G);

    cv::Mat sorted_gradients ;
    G.reshape(0,G.rows*G.cols).copyTo(sorted_gradients);


    cv::sort(sorted_gradients,sorted_gradients,CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);


    float limit ;


    limit = limit_grad;



    cv::Mat G_expanded = G.clone()+1;
    for (int i=5; i<G.rows-5; i++)
    {
        for (int j=5; j < G.cols-5;j++)
        {
            /*for (int ii=i-1; ii<=i+1; ii++)
            {
                for (int jj=j-1; jj<=j+1; jj++)
                {*/
                    if (G.at<float>(i,j) < limit)
                    {
                       G_expanded.at<float>(i,j) =   limit-0.1;
                    }
                /*}
            }*/
        }
    }
    G = G_expanded.clone();
    //cin >> limit;


    cv::Mat sorted_depths, points_z2;
    points_z2 = points_z.clone();
    points_z2=points_z2.reshape(0, points_z.rows* points_z.cols);

    cv::sort(cv::abs(points_z2),sorted_depths,CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);
    double variance = sorted_depths.at<double>(round(sorted_depths.rows/2),0);


    cv::Mat B;
    if (kinect_initialization > 0.5)
    { B = ((G < limit & cv::abs(points_z) < 40));}
    else
    { B = (G < limit & cv::abs(points_z) < 100* variance * 1.25);}



    B = B.reshape(0,points_x.cols*points_x.rows);

    points_x =  points_x.reshape(0,points_x.cols*points_x.rows);

    points_y =  points_y.reshape(0,points_y.cols*points_y.rows);

    points_z =  points_z.reshape(0,points_z.cols*points_z.rows);
    points_R =  points_R.reshape(0,points_R.cols*points_R.rows);
    points_G =  points_G.reshape(0,points_G.cols*points_G.rows);
    points_B =  points_B.reshape(0,points_B.cols*points_B.rows);

    cv::Mat points_x1(1,0,CV_64FC1);
    cv::Mat points_y1(1,0,CV_64FC1);
    cv::Mat points_z1(1,0,CV_64FC1);
    cv::Mat points_r1(1,0,CV_64FC1);
    cv::Mat points_g1(1,0,CV_64FC1);
    cv::Mat points_b1(1,0,CV_64FC1);

    B.convertTo(B,CV_32FC1);
    for (int i = 0; i < points_x.rows; i++)
    {
        if (B.at<float>(i,0)>100)
           {
            points_x1.push_back(points_x.at<double>(i,0));
            points_y1.push_back(points_y.at<double>(i,0));
            points_z1.push_back(points_z.at<double>(i,0));
            points_r1.push_back(points_R.at<double>(i,0));
            points_g1.push_back(points_G.at<double>(i,0));
            points_b1.push_back(points_B.at<double>(i,0));
        }
    }

    points_x1.copyTo(points_x);
    points_y1.copyTo(points_y);
    points_z1.copyTo(points_z);
    points_r1.copyTo(points_R);
    points_g1.copyTo(points_G);
    points_b1.copyTo(points_B);

    cv::Mat points_aux(points_x.rows,6,CV_64FC1);
    points_x.copyTo(points_aux.colRange(0,1));
    points_y.copyTo(points_aux.colRange(1,2));
    points_z.copyTo(points_aux.colRange(2,3));
    points_R.copyTo(points_aux.colRange(3,4));
    points_G.copyTo(points_aux.colRange(4,5));
    points_B.copyTo(points_aux.colRange(5,6));

    points_aux.copyTo(points2);
}


void compute_error( cv::Mat &points3D_cam, cv::Mat image, cv::Mat &color, cv::Mat &error_vector,double variance, cv::Mat &error_vector_sqrt,cv::Mat &error_check,cv::Mat &weight)
{
    int imsize_x =image.cols-1;
    int imsize_y =image.rows-1;

    double error_to_add = 0;
    int cont = 0;

    for (int k =0; k< points3D_cam.cols;k++)
    {
       if (points3D_cam.at<double>(1,k) > 0 && points3D_cam.at<double>(1,k) < imsize_y && points3D_cam.at<double>(0,k) > 0 && points3D_cam.at<double>(0,k) < imsize_x)
         {
           cont++;
            //BILINEAR INTERPOLATION
           double x_2 = points3D_cam.at<double>(0,k);
           double x_1 = static_cast<int>(x_2);
           double x_3 = x_1 +1;
           double y_2 = points3D_cam.at<double>(1,k);
           double y_1 = static_cast<int>(y_2);
           double y_3 = y_1 +1;

           double c1 = image.at<double>(y_1,x_1);
           double c2 = image.at<double>(y_3,x_1);

           double r1 = c1*(y_3-y_2) + c2*(y_2-y_1);

           c1 = image.at<double>(y_1,x_3);
           c2 = image.at<double>(y_3,x_3);

           double r2 = c1*(y_3-y_2) + c2*(y_2-y_1);

           double r = r1*(x_3-x_2) +  r2*(x_2-x_1);


           double error_to_add  =   (r -  color.at<double>(k,0));

           error_vector_sqrt.at<double>(k,0) = error_to_add;
           error_check.push_back(error_to_add);
        }
       else
       {
            error_to_add = 1;
            error_vector_sqrt.at<double>(k,0) = error_to_add;
       }
    }

    cv::pow(error_vector_sqrt, 2,error_vector);
    cv::pow(error_check, 2,error_check);

}

void compute_error_ic( cv::Mat &points3D_cam,cv::Mat &points3D_cam_p, cv::Mat &image,cv::Mat &image_p, \
                       cv::Mat &error_vector,double &variance, cv::Mat &error_vector_sqrt,double &error, cv::Mat &weight, cv::Mat &color_p)

{
    int imsize_x =image.cols-1;
    int imsize_y =image.rows-1;

    //error = 0;
    //float cont_errors = 0;

    for (int k = 0; k < points3D_cam.cols;k++)
    {

        if (points3D_cam.at<double>(1,k) > 0 && points3D_cam.at<double>(1,k) < imsize_y && points3D_cam.at<double>(0,k) > 0 && points3D_cam.at<double>(0,k) < imsize_x
            && points3D_cam_p.at<double>(1,k) > 0 && points3D_cam_p.at<double>(1,k) < imsize_y && points3D_cam_p.at<double>(0,k) > 0 && points3D_cam_p.at<double>(0,k) < imsize_x)
         {
           //cont++;
            //BILINEAR INTERPOLATION
           double x_2 = points3D_cam.at<double>(0,k);
           double x_1 = static_cast<int>(x_2);
           double x_3 = x_1 +1;
           double y_2 = points3D_cam.at<double>(1,k);
           double y_1 = static_cast<int>(y_2);
           double y_3 = y_1 +1;

           double c1 = image.at<double>(y_1,x_1);
           double c2 = image.at<double>(y_3,x_1);

           double r1 = c1*(y_3-y_2) + c2*(y_2-y_1);

           c1 = image.at<double>(y_1,x_3);
           c2 = image.at<double>(y_3,x_3);

           double r2 = c1*(y_3-y_2) + c2*(y_2-y_1);

           double r =  r1*(x_3-x_2) + r2*(x_2-x_1);


           //BILINEAR INTERPOLATION
           x_2 = points3D_cam_p.at<double>(0,k);
           x_1 = static_cast<int>(x_2);
           x_3 = x_1 +1;
           y_2 = points3D_cam_p.at<double>(1,k);
           y_1 = static_cast<int>(y_2);
           y_3 = y_1 +1;

           c1 = image_p.at<double>(y_1,x_1);
           c2 = image_p.at<double>(y_3,x_1);

           r1 = c1*(y_3-y_2) + c2*(y_2-y_1);

           c1 = image_p.at<double>(y_1,x_3);
           c2 = image_p.at<double>(y_3,x_3);

           r2 = c1*(y_3-y_2) + c2*(y_2-y_1);

          double r_p = r1*(x_3-x_2) +  r2*(x_2-x_1);

          color_p.at<double>(k,0) = r_p;


          error_vector_sqrt.at<double>(k,0) =(r -  r_p);

          //error = error + (r -  r_p)*(r -  r_p)*weight.at<double>(k,0);
         // cont_errors++;
        }//
       else
       {
           //BILINEAR INTERPOLATION
            if ( points3D_cam_p.at<double>(1,k) > 0 && points3D_cam_p.at<double>(1,k) < imsize_y && points3D_cam_p.at<double>(0,k) > 0 && points3D_cam_p.at<double>(0,k) < imsize_x)
            {
               double x_2 = points3D_cam_p.at<double>(0,k);
               double x_1 = static_cast<int>(x_2);
               double x_3 = x_1 +1;
               double y_2 = points3D_cam_p.at<double>(1,k);
               double y_1 = static_cast<int>(y_2);
               double y_3 = y_1 +1;



               double c1 = image_p.at<double>(y_1,x_1);
               double c2 = image_p.at<double>(y_3,x_1);

               double r1 = c1*(y_3-y_2) + c2*(y_2-y_1);

               c1 = image_p.at<double>(y_1,x_3);
               c2 = image_p.at<double>(y_3,x_3);

              double r2 = c1*(y_3-y_2) + c2*(y_2-y_1);

              double r_p = r1*(x_3-x_2) +  r2*(x_2-x_1);

              color_p.at<double>(k,0) = r_p;
            }
            else
            {
                color_p.at<double>(k,0) = sqrt(variance)*1;
            }

            error_vector_sqrt.at<double>(k,0) = sqrt(variance)*1;
       }
    }
    //cin>>cont;


    cv::pow(error_vector_sqrt, 2,error_vector);
    cv::Mat error_vector_check = error_vector.mul(weight);
    error = cv::sum(error_vector_check)[0];

    //error = error / cont_errors;

}



void gauss_newton_ic(SemiDenseTracking *semidense_tracker,cv::Mat &points3D_cam_to_print,cv::Mat &R,cv::Mat &t,\
                     cv::Mat &R_p,cv::Mat &t_p,double fx,double fy,cv::Mat &points,cv::Mat &img,cv::Mat &img_p, \
                     double &error_opt,cv::Mat &color,double variance,cv::Mat &points3D\
                    ,cv::Mat &error_vector_opt, int initial_iteration,cv::Mat &jacobian, cv::Mat &init ,\
                     cv::Mat &weight,cv::Mat &points3D_cam_p,cv::Mat &points3D_cam,\
                     cv::Mat &error_vector_inicial, cv::Mat &error_vector_sqrt_inicial, float &has_decreased,cv::Mat &color_p,\
                     int &processed_frames,cv::Mat &jacobian1k, double &overlap,double cx,double cy)
{

cv::Mat R_opt;
cv::Mat t_opt;
R.copyTo(R_opt);
t.copyTo(t_opt);


cv::Mat error_vector(points.rows,1,CV_64FC1);
cv::Mat error_vector_sqrt(points.rows,1,CV_64FC1);

//////////////////////////////////////// INVERSE
if (initial_iteration < 0.5)
{
    transformed_points( points3D_cam_p, R_p,t_p,fx,fy,cx,cy,img_p,points3D);
    transformed_points( points3D_cam  , R  ,t  ,fx,fy,cx,cy,img  ,points3D);
    compute_error_ic( points3D_cam,points3D_cam_p,img,img_p,error_vector_inicial,variance,error_vector_sqrt_inicial,error_opt,weight,color_p);

    points3D_cam_to_print = points3D_cam.clone();
}

cv::Mat w(3,1,CV_64FC1);
cv::Mat v(3,1,CV_64FC1);
cv::Mat  R1,t1,R2,t2;
if (initial_iteration <0.5)
{
    if (processed_frames < 0.5)
    {
            cv::Mat jacobian_analitic(jacobian.rows,6,CV_64FC1);

            cv::Mat points3D_cam_p2;

            cv::Mat transformed_points;
            transformed_points_return_3Dpoints(points3D_cam_p2, R_p,t_p,fx,fy,cx,cy,img_p,points3D,transformed_points);


            cv::Mat GX =(gradientX(img_p,1));
            cv::Mat GY =(gradientY(img_p,1));

            points3D_cam_p2= points3D_cam_p2.t();
            transformed_points = transformed_points.t();

            int imsize_x =img.cols-1;
            int imsize_y =img.rows-1;



            for (int ii = 0; ii < transformed_points.rows; ii++)
            {
                if (points3D_cam.at<double>(1,ii) > 2 && points3D_cam.at<double>(1,ii) < imsize_y-2 && points3D_cam.at<double>(0,ii) > 2 && points3D_cam.at<double>(0,ii) < imsize_x-2)
                {

                    float jac4 = GX.at<float>(round(points3D_cam_p2.at<double>(ii,1)),round(points3D_cam_p2.at<double>(ii,0)))*fx/transformed_points.at<double>(ii,2);
                    float jac5 = -GY.at<float>(round(points3D_cam_p2.at<double>(ii,1)),round(points3D_cam_p2.at<double>(ii,0)))*fy/transformed_points.at<double>(ii,2);
                    float jac6 = (GY.at<float>(round(points3D_cam_p2.at<double>(ii,1)),round(points3D_cam_p2.at<double>(ii,0)))*fy*transformed_points.at<double>(ii,1)-GX.at<float>(round(points3D_cam_p2.at<double>(ii,1)),round(points3D_cam_p2.at<double>(ii,0)))*fx*transformed_points.at<double>(ii,0))/(transformed_points.at<double>(ii,2)*transformed_points.at<double>(ii,2));

                    jacobian_analitic.at<double>(ii,3) =  jac4;
                    jacobian_analitic.at<double>(ii,4) =  jac5;
                    jacobian_analitic.at<double>(ii,5) =  jac6;
                    jacobian_analitic.at<double>(ii,0) =  jac6*transformed_points.at<double>(ii,1) + GY.at<float>(round(points3D_cam_p2.at<double>(ii,1)),round(points3D_cam_p2.at<double>(ii,0)))*fy ;
                    jacobian_analitic.at<double>(ii,1) =  -jac6*transformed_points.at<double>(ii,0) + GX.at<float>(round(points3D_cam_p2.at<double>(ii,1)),round(points3D_cam_p2.at<double>(ii,0)))*fx ;
                    jacobian_analitic.at<double>(ii,2) =  -jac4*transformed_points.at<double>(ii,1) + jac5*transformed_points.at<double>(ii,0);
                }
                else
                {
                    jacobian_analitic.at<double>(ii,0) = 0;
                    jacobian_analitic.at<double>(ii,1) = 0;
                    jacobian_analitic.at<double>(ii,2) = 0;
                    jacobian_analitic.at<double>(ii,3) = 0;
                    jacobian_analitic.at<double>(ii,4) = 0;
                    jacobian_analitic.at<double>(ii,5) = 0;
                    weight.at<double>(ii,0) = 0;
                }
            }

            for (int ii = 0; ii <6;ii++)
            {
                jacobian.rowRange(0,jacobian.rows).colRange(ii,ii+1) = weight.mul(jacobian_analitic.rowRange(0,jacobian_analitic.rows).colRange(ii,ii+1) );
            }
            jacobian1k = jacobian_analitic.clone();

            init = (jacobian.t()*jacobian).inv(cv::DECOMP_SVD)*jacobian.t();

    }
    else
    {
            int imsize_x =img.cols-1;
            int imsize_y =img.rows-1;

                for (int k = 0; k<points3D_cam.cols; k++)
                {
                    if (points3D_cam.at<double>(1,k) > 2 && points3D_cam.at<double>(1,k) < imsize_y-2 && points3D_cam.at<double>(0,k) > 2 && points3D_cam.at<double>(0,k) < imsize_x-2)
                    {
                        for (int z = 0; z<6; z++)
                        {
                            jacobian.at<double>(k,z)=jacobian1k.at<double>(k,z);
                        }
                    }
                    else
                    {
                        for (int z = 0; z<6; z++)
                        {
                            jacobian.at<double>(k,z) = 0;
                            weight.at<double>(k,0) = 0;

                        }
                    }
                }
                for (int z = 0; z<6; z++)
                {
                   jacobian.rowRange(0,jacobian.rows).colRange(z,z+1) = weight.mul(jacobian.rowRange(0,jacobian.rows).colRange(z,z+1));
                }

                init = (jacobian.t()*jacobian).inv(cv::DECOMP_SVD)*jacobian.t();
    }
}


cv::Mat init2;
init2 = init*(error_vector_sqrt_inicial.mul(weight)) ;
w.at<double>(0,0) = -init2.at<double>(0,0);
w.at<double>(1,0) = -init2.at<double>(1,0);
w.at<double>(2,0) = -init2.at<double>(2,0);
v.at<double>(0,0) = -init2.at<double>(3,0);
v.at<double>(1,0) = -init2.at<double>(4,0);
v.at<double>(2,0) =- init2.at<double>(5,0);



exp_SE3 (R1,t1,w,v);

cv::Mat S,U,V;
cv::SVD::compute(R1,S,U,V,cv::SVD::FULL_UV);
R1 = U*V;

R2 = R.clone();
t2 = t.clone();

t2 = -R2.t()*t2;
R2 = R2.t();

t2 = R2*t1 + t2;
R2 = R2*R1;

t2 = -R2.t()*t2;
R2 = R2.t();



double error_check;
transformed_points(points3D_cam, R2,t2,fx,fy,cx,cy,img,points3D);
compute_error_ic_ni( points3D_cam,points3D_cam_p,img,img_p,error_vector,variance,error_vector_sqrt,error_check,weight,color_p,overlap);

cv::SVD::compute(R2,S,U,V,cv::SVD::FULL_UV);
R2 = U*V;

if (error_check < error_opt )
{
    error_opt = error_check;
    R2.copyTo(R_opt);
    t2.copyTo(t_opt);
    error_vector.copyTo(error_vector_opt);
    error_vector.copyTo(error_vector_inicial);
    error_vector_sqrt.copyTo(error_vector_sqrt_inicial);
    has_decreased = 1;

    points3D_cam_to_print = points3D_cam.clone();



}
R_opt.copyTo(R);
t_opt.copyTo(t);
}

void  gauss_estimation(SemiDenseTracking *semidense_tracker,cv::Mat &points3D_cam_to_print,cv::Mat &R2,cv::Mat &t2,cv::Mat &R_p,cv::Mat &t_p,double &fx,double &fy,double &cx,double &cy,cv::Mat &points, \
                       cv::Mat &img,cv::Mat &img_p,double &error_p,cv::Mat &color,int &iter, float variance,cv::Mat &points3D,\
                       cv::Mat &error_vector_opt,cv::Mat &weight, int &processed_frames,cv::Mat &jacobian1k, double &overlap_tracking,
                       double &tracking_th, int iter_th)
{
int initial_iteration = 0;
cv::Mat jacobian(points.rows,6,CV_64FC1);
cv::Mat init;

cv::Mat points3D_cam_p, points3D_cam_last;
cv::Mat error_vector_inicial(points.rows,1,CV_64FC1);
cv::Mat error_vector_sqrt_inicial(points.rows,1,CV_64FC1);
cv::Mat color_p(points.rows,1,CV_64FC1);
float has_decreased = 0;

double overlap;
gauss_newton_ic(semidense_tracker,points3D_cam_to_print,R2,t2,R_p,t_p,fx,fy,points,img,img_p,error_p,color,variance,points3D,error_vector_opt,initial_iteration,jacobian,init,weight,points3D_cam_p,points3D_cam_last,\
                error_vector_inicial,error_vector_sqrt_inicial,has_decreased,color_p,processed_frames,jacobian1k,overlap,cx,cy);
iter ++;

initial_iteration = 1;

double error_f = error_p;
double error_f1 = 10;
double error_f2 = 0;
cv::Mat R=R2.clone();
cv::Mat t=t2.clone();
while (fabs((error_f1 - error_f2)/error_f1) > tracking_th  &&  has_decreased > 0.5 && error_f1 > error_f2 && iter < iter_th )
{

        iter ++;

        error_f1 = error_f;
        has_decreased = 0;
        gauss_newton_ic(semidense_tracker,points3D_cam_to_print,R2,t2,R_p,t_p,fx,fy,points,img,img_p,error_p,color,variance,points3D,error_vector_opt,initial_iteration,jacobian,init,weight,points3D_cam_p,points3D_cam_last,\
                        error_vector_inicial,error_vector_sqrt_inicial,has_decreased,color_p,processed_frames,jacobian1k,overlap,cx,cy);

        error_f2 = error_p;

        if (error_f1 > error_f2)
        {
            error_f =error_p;
            R=R2.clone();
            t=t2.clone();
            overlap_tracking = overlap;
        }
}
R2=R.clone();
t2=t.clone();
error_p = error_f;
}


void compute_error_ic_ni( cv::Mat &points3D_cam,cv::Mat &points3D_cam_p, cv::Mat &image,cv::Mat &image_p, \
                       cv::Mat &error_vector,double &variance, cv::Mat &error_vector_sqrt,double &error, cv::Mat &weight, cv::Mat color_p, double &overlap)

{
    int imsize_x =image.cols-1;
    int imsize_y =image.rows-1;

    double cont = 0;


    //error = 0;
    //float cont_errors = 0;

    for (int k =0; k< points3D_cam.cols;k++)
    {
        if (points3D_cam.at<double>(1,k) > 0 && points3D_cam.at<double>(1,k) < imsize_y && points3D_cam.at<double>(0,k) >0 && points3D_cam.at<double>(0,k) < imsize_x
            && points3D_cam_p.at<double>(1,k) > 0 && points3D_cam_p.at<double>(1,k) < imsize_y && points3D_cam_p.at<double>(0,k) > 0 && points3D_cam_p.at<double>(0,k) < imsize_x)
         {
           cont++;
            //BILINEAR INTERPOLATION
           double x_2 = points3D_cam.at<double>(0,k);
           double x_1 = static_cast<int>(x_2);
           double x_3 = x_1 +1;
           double y_2 = points3D_cam.at<double>(1,k);
           double y_1 = static_cast<int>(y_2);
           double y_3 = y_1 +1;

           double c1 = image.at<double>(y_1,x_1);
           double c2 = image.at<double>(y_3,x_1);

           double r1 = c1*(y_3-y_2) + c2*(y_2-y_1);

           c1 = image.at<double>(y_1,x_3);
           c2 = image.at<double>(y_3,x_3);

           double r2 = c1*(y_3-y_2) + c2*(y_2-y_1);

           double r =  r1*(x_3-x_2) + r2*(x_2-x_1);


          double r_p = color_p.at<double>(k,0);


          error_vector_sqrt.at<double>(k,0) =(r -  r_p);
          //error = error + (r -  r_p)*(r -  r_p)*weight.at<double>(k,0);
          //cont_errors++;
        }
       else
       {
            //error_to_add = sqrt(variance)*1;
            error_vector_sqrt.at<double>(k,0) = sqrt(variance)*1;
       }
    }
    //cin>>cont;


    overlap = cont / points3D_cam.cols;
    cv::pow(error_vector_sqrt, 2,error_vector);
    cv::Mat error_vector_check = error_vector.mul(weight);
    error = cv::sum(error_vector_check)[0];

    //error = error / cont_errors;

}

void exp_SO3(cv::Mat &R, cv::Mat &w)
{
    double w1 = w.at<double>(0,0);
    double w2 = w.at<double>(1,0);
    double w3 = w.at<double>(2,0);

    double data[3][3] = { {0,  -w3,   w2},
                      {w3,   0,  -w1},
                     {-w2,   w1,  0}};
    cv::Mat wx = cv::Mat(3, 3, CV_64FC1, &data);

    double data1[3] = {   w1,w2,w3};

    double data2[3][3] = { {1,0,0},
                      {0,1,0},
                     {0,0,1}};

    cv::Mat eye = cv::Mat(3, 3, CV_64FC1, &data2);

   double  tetha = sqrt(w.at<double>(0,0)*w.at<double>(0,0) + w.at<double>(1,0)*w.at<double>(1,0) + w.at<double>(2,0)*w.at<double>(2,0));

    if (tetha < 0.00015)
        {R = eye + wx +1/2*(wx*wx);}
    else
        {R = eye+ sin(tetha)/tetha*wx + (1-cos(tetha))/(tetha*tetha)*(wx*wx);}
}

void  w2V(cv::Mat &V,double w1,double w2,double w3,cv::Mat &logR)
{
    double data1[3] = {   w1,w2,w3};
    cv::Mat w = cv::Mat(3, 1, CV_64FC1, &data1);

    double  theta = sqrt(w.at<double>(0,0)*w.at<double>(0,0) + w.at<double>(1,0)*w.at<double>(1,0) + w.at<double>(2,0)*w.at<double>(2,0));

    cv::Mat eye = (cv::Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    if (theta < 0.000015)
    {
        V = eye + 0.5*logR + 1/6 * (logR * logR.t());
    }
    else
    {
        V = eye + ((1-cos(theta))/(theta*theta))*logR + (theta - sin(theta))/(theta*theta*theta) * (logR * logR.t());
    }
}


void exp_SE3 (cv::Mat &R,cv::Mat &t,cv::Mat &w,cv::Mat &v)
{
    double w1 = w.at<double>(0,0);
    double w2 = w.at<double>(1,0);
    double w3 = w.at<double>(2,0);

    exp_SO3( R,w);

    cv::Mat logR;
    double d;

    d = 1/2*(cv::trace(R)[0]-1);

    if (fabs(d) > 0.9999 &&  fabs(d)< 1.00001)
    {logR = 1/2*(R-R.t()) ;}
    else
    {logR = acos(d) * (R-R.t()) / (2*sqrt(1-d*d)) ;}

    cv::Mat V;
    w2V(V,w1,w2,w3,logR);

    t=V*v;
}


void log_SO3 (cv::Mat &R,cv::Mat &w)
{
    double d = 1/2*(cv::trace(R)[0]-1);
    cv::Mat logR;

    if (fabs(d) > 0.9999 &&  fabs(d)< 1.00001)
    {logR = 1/2*(R-R.t()) ;}
    else
    {logR = acos(d) * (R-R.t()) / (2*sqrt(1-d*d)) ;}

    w.at<double>(0,0) = logR.at<double>(2,1);
    w.at<double>(1,0) = logR.at<double>(0,2);
    w.at<double>(2,0) = logR.at<double>(1,0);
}


void skew_SO3( cv::Mat &wx, cv::Mat &w)
{
    double w1 = w.at<double>(0,0);
    double w2 = w.at<double>(1,0);
    double w3 = w.at<double>(2,0);

    wx =  (cv::Mat_<double>(3,3) << 0, -w3, w2,  w3,0, -w1,-w2, w1,0);

}
void inv_skew_SO3( cv::Mat &wx, cv::Mat &w)
{
    w.at<double>(0,0) = wx.at<double>(2,1);
    w.at<double>(1,0) = wx.at<double>(0,2);
    w.at<double>(2,0) = wx.at<double>(1,0);
}

void J_r_SO3(cv::Mat &J_r,cv::Mat &w)
{
    cv::Mat eye = (cv::Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    cv::Mat wx;
    skew_SO3(wx, w);

    double tetha = cv::norm(w);

    if (tetha > 0.0005)
    {
        J_r = eye - (1-cos(tetha))/(tetha*tetha)*wx + (tetha-sin(tetha))/(tetha*tetha*tetha)*wx*wx;
    }
    else
    {
        J_r = eye.clone();
    }
}

cv::Mat MatToQuat(cv::Mat &Rot){
    cv::Mat  Quat(4,1,CV_64FC1);
    double tr = Rot.at<double>(0,0)+ Rot.at<double>(1,1)+ Rot.at<double>(2,2);
    int ii;
    ii=0;
    if (Rot.at<double>(1,1) > Rot.at<double>(0,0)) ii=1;
    if (Rot.at<double>(2,2) > Rot.at<double>(ii,ii)) ii=2;
    double s;
    if (tr >= 0){
        s = sqrt((tr + 1));
        Quat.at<double>(0,0) = s * 0.5;
        s = 0.5 / s;
        Quat.at<double>(1,0) = (Rot.at<double>(2,1) - Rot.at<double>(1,2)) * s;
        Quat.at<double>(2,0) = (Rot.at<double>(0,2) - Rot.at<double>(2,0)) * s;
        Quat.at<double>(3,0) = (Rot.at<double>(1,0) - Rot.at<double>(0,1)) * s;
    } else {
        switch(ii) {
         case 0:
                s = sqrt((Rot.at<double>(0,0)-Rot.at<double>(1,1)-Rot.at<double>(2,2)+1));
                Quat.at<double>(1,0) = s * 0.5;
                s = 0.5 / s;

                Quat.at<double>(2,0) = (Rot.at<double>(1,0) + Rot.at<double>(0,1)) * s;//Update pose estimation

                Quat.at<double>(3,0) = (Rot.at<double>(2,0) + Rot.at<double>(0,2)) * s;
                Quat.at<double>(0,0) = (Rot.at<double>(2,1) - Rot.at<double>(1,2)) * s;
                break;
         case 1:
                s = sqrt((Rot.at<double>(1,1)-Rot.at<double>(2,2)-Rot.at<double>(0,0)+1));
                Quat.at<double>(2,0) = s * 0.5;
                s = 0.5 / s;

                Quat.at<double>(3,0) = (Rot.at<double>(2,1) + Rot.at<double>(1,2)) * s;
                Quat.at<double>(1,0) = (Rot.at<double>(0,1) + Rot.at<double>(1,0)) * s;
                Quat.at<double>(0,0) = (Rot.at<double>(0,2) - Rot.at<double>(2,0)) * s;
                break;
         case 2:
                s = sqrt((Rot.at<double>(2,2)-Rot.at<double>(0,0)-Rot.at<double>(1,1)+1));
                Quat.at<double>(3,0) = s * 0.5;
                s = 0.5 / s;

                Quat.at<double>(1,0) = (Rot.at<double>(0,2) + Rot.at<double>(2,0)) * s;
                Quat.at<double>(2,0) = (Rot.at<double>(1,2) + Rot.at<double>(2,1)) * s;
                Quat.at<double>(0,0) = (Rot.at<double>(1,0) - Rot.at<double>(0,1)) * s;
                break;
         }
    }
    return Quat;
}

void decomposeR(cv::Mat &R, float &yaw, float &pitch, float &roll)
{
    roll   = atan2(R.at<double>(2,1),R.at<double>(2,2));
    pitch = atan2(-R.at<double>(2,0), sqrt(R.at<double>(2,1)*R.at<double>(2,1)+R.at<double>(2,2)*R.at<double>(2,2))  );
    yaw  = atan2(R.at<double>(1,0),R.at<double>(0,0));
}

void euler2quaternion(float &yaw, float &pitch, float &roll, float &x, float &y, float &z, float &w)
{
    w = cos(roll/2)*cos(pitch/2)*cos(yaw/2) +  sin(roll/2)*sin(pitch/2)*sin(yaw/2) ;
    x = sin(roll/2)*cos(pitch/2)*cos(yaw/2) -  cos(roll/2)*sin(pitch/2)*sin(yaw/2) ;
    y = cos(roll/2)*sin(pitch/2)*cos(yaw/2) +  sin(roll/2)*cos(pitch/2)*sin(yaw/2) ;
    z = cos(roll/2)*cos(pitch/2)*sin(yaw/2) -  sin(roll/2)*sin(pitch/2)*cos(yaw/2) ;
}

void print_poses(cv::Mat &points, char buffer[])
{
      ofstream out(buffer);

     int num_points = points.rows;

     int val = points.rows-1;
     val = num_points;
     out << "ply" << endl;out << "format ascii 1.0" << endl;out << "element face 0" << endl;out << "property list uchar int vertex_indices" << endl;
     out << "element vertex ";out << val << endl;out << "property float x" << endl;out << "property float y" << endl;out << "property float z" << endl;
     out <<  "property uchar diffuse_red"<<endl;out << "property uchar diffuse_green" << endl;out << "property uchar diffuse_blue" << endl;out << "end_header" << endl;
     for (int i = 0; i<= points.rows-1;i++)
     {

        double val1 = points.at<double>(i,0);
        double val2 = points.at<double>(i,1);
        double val3 = points.at<double>(i,2);


        int  color1 =  0   + i*255/points.rows;
        int  color2 =  255 - i*255/points.rows;
        int  color3 =  255*0 ;
        color1 =0+points.at<double>(i,3);
        color2 = 0+points.at<double>(i,3);;color3 = 0+points.at<double>(i,3);
        {
            out << fixed << setprecision(5) << val1<< " " << fixed << setprecision(5) << val2 <<  " " << fixed << setprecision(5) << val3 \
            << " "<< color1 << " "<< color2 << " "<< color3 << endl;
        }
     }
     out.close();
 }


void join_maps(vector<cv::Mat> &points_map,cv::Mat R,cv::Mat t,vector<cv::Mat> point_clouds,int pyramid_levels,\
                vector<double> &focalx, vector<double> &focaly, vector<double> &centerx, vector<double> &centery,   \
              vector<cv::Mat> &image_keyframe_pyramid, float  &points_projected_in_image)
{

    vector<cv::Mat> color(pyramid_levels);
    for (int i = 0; i < pyramid_levels;i++)
    {


        int imsize_y = image_keyframe_pyramid[i].rows;
        int imsize_x = image_keyframe_pyramid[i].cols;

        float cont2 =0;

        float maximum_points_to_track = 0;

        if (i==pyramid_levels-1){
            maximum_points_to_track = 28000;
        }
        if (i==pyramid_levels-2){
            maximum_points_to_track = 8000;
        }
        if (i==pyramid_levels-3){
            maximum_points_to_track = 2500;
        }


        maximum_points_to_track *=1;


        cv::Mat points_joined(0,7,CV_64FC1);
        cv::Mat points_joined_previous_map(0,7,CV_64FC1);
        cv::Mat pointsClouds3Dmap_cam;




        cv::Mat points_map_aux = points_map[i].clone();
        points_map_aux.push_back(point_clouds[i]);
        point_clouds[i] = points_map_aux.clone();




        cv::Mat pointsClouds3D =  point_clouds[i].t();
        pointsClouds3D = pointsClouds3D.rowRange(0,3);

        transformed_points(pointsClouds3Dmap_cam, R,   t,   focalx[i],focaly[i],centerx[i],centery[i],image_keyframe_pyramid[i], pointsClouds3D   );




        get_color(point_clouds[i],color[i]);



        cv::Mat error_vector(point_clouds[i].rows,1,CV_64FC1);
        cv::Mat error_vector_sqrt(point_clouds[i].rows,1,CV_64FC1);
        cv::Mat error_check(0,1,CV_64FC1);
        cv::Mat weight(point_clouds[i].rows,1,CV_64FC1);

        double variance=0.03;

        double min, max;
        cv::minMaxLoc(pointsClouds3Dmap_cam, &min, &max);

        compute_error( pointsClouds3Dmap_cam,image_keyframe_pyramid[i], color[i],error_vector,variance,error_vector_sqrt,error_check,weight);


        cont2 = 0;
        float border = 40 / (4/(pow(2,i)));


        border = 0 / (4/(pow(2,i)));

        border = -1;



        cv::Mat check_repatead_points = cv::Mat::zeros(imsize_y,imsize_x,CV_64FC1);

        for(int k = 0; k < pointsClouds3Dmap_cam.cols;k++)
        {
            if (fabs(error_vector.at<double>(k,0))< 0.06 && pointsClouds3Dmap_cam.at<double>(1,k) > 0-border && pointsClouds3Dmap_cam.at<double>(1,k) < imsize_y+border && \
                    pointsClouds3Dmap_cam.at<double>(0,k) > 0-border &&pointsClouds3Dmap_cam.at<double>(0,k) < imsize_x+border)
            {
                if(check_repatead_points.at<double>(round(pointsClouds3Dmap_cam.at<double>(1,k)),
                                                    round(pointsClouds3Dmap_cam.at<double>(0,k))) < 1)
                {
                    cont2++;
                    check_repatead_points.at<double>(round(pointsClouds3Dmap_cam.at<double>(1,k)),round(pointsClouds3Dmap_cam.at<double>(0,k))) += 1;
                }
            }
        }

        cv::minMaxLoc(pointsClouds3Dmap_cam, &min, &max);



        float limit = (maximum_points_to_track) / ( cont2);
        cont2 = 0;

        check_repatead_points = cv::Mat::zeros(imsize_y,imsize_x,CV_64FC1);

        for(int k = 0; k < pointsClouds3Dmap_cam.cols;k++)
        {
            if (fabs(error_vector.at<double>(k,0))< 0.06  && pointsClouds3Dmap_cam.at<double>(1,k) > 0-border && pointsClouds3Dmap_cam.at<double>(1,k) < imsize_y+border && \
                    pointsClouds3Dmap_cam.at<double>(0,k) > 0-border &&pointsClouds3Dmap_cam.at<double>(0,k) < imsize_x+border)
            {

                if(check_repatead_points.at<double>(round(pointsClouds3Dmap_cam.at<double>(1,k)),
                                                    round(pointsClouds3Dmap_cam.at<double>(0,k))) < 1)
                {

                        points_joined_previous_map.push_back(point_clouds[i].row(k));
                        check_repatead_points.at<double>(round(pointsClouds3Dmap_cam.at<double>(1,k)),round(pointsClouds3Dmap_cam.at<double>(0,k)))+=1;


                        cont2++;
                        if (pointsClouds3Dmap_cam.at<double>(1,k) > 0-border && pointsClouds3Dmap_cam.at<double>(1,k) < imsize_y+border && \
                                pointsClouds3Dmap_cam.at<double>(0,k) > 0-border &&pointsClouds3Dmap_cam.at<double>(0,k) < imsize_x+border \
                                && i==pyramid_levels-1)
                        {
                            points_projected_in_image++;
                        }


                }
            }
        }
        limit = (maximum_points_to_track) / ( cont2);
        cv::Mat points_joined_previous_map_aux(0,7,CV_64FC1);
        for(int k = 0; k < points_joined_previous_map.rows;k++)
        {
             if  (((rand() % 1000000 ) / 1000000.0) < limit)
             {
                 points_joined_previous_map_aux.push_back(points_joined_previous_map.row(k));
             }
        }
        points_joined_previous_map = points_joined_previous_map_aux.clone();


        points_joined.push_back(points_joined_previous_map); points_map[i] = points_joined.clone();

    }
}
