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

#include <dpptam/DenseMapping.h>
#include <dpptam/SemiDenseMapping.h>
#include <dpptam/vo_system.h>

DenseMapping::DenseMapping()
{
    /*char cwd[1024];
       if (getcwd(cwd, sizeof(cwd)) != NULL)
           fprintf(stdout, "Current working dir: %s\n", cwd);
       else
           perror("getcwd() error");
   chdir(cwd);*/
   //chdir("/home/alejo/catkin_ws");

    cv::FileStorage  fs2("src/dpptam/src/data.yml", cv::FileStorage::READ);

    DenseMapping::set_do_dense(0);
    DenseMapping::init_analisis();
    calculate_superpixels = (int)fs2["calculate_superpixels"];
    dense_kf = 0;
    limit_ratio_sing_val = (float)fs2["limit_ratio_sing_val"];
    limit_normalized_residual =  (float)fs2["limit_normalized_residual"];
    matchings_active_search = (int)fs2["matchings_active_search"];

    cv::Mat map_points(0,6,CV_64FC1);
    map_sup_points_total = map_points.clone();

    int number_of_pointsKF_forSup = 1;
    DenseMapping::init_points3D4spx(number_of_pointsKF_forSup);

    fs2.release();

}

void DenseMapping::init_analisis() {
    cv::Mat aux(0,5,CV_32FC1);
    for (int i = 0; i<9;i++)
    {analisis.push_back(aux);}
}

void DenseMapping::init_points3D4spx(int num_images) {
     points3D4spx.resize(num_images);
}

void print_poses(cv::Mat &points, char buffer[],int color)
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
        if (color = 0)
        {
            color3 = 255;
        }
        {
            out << fixed  << val1<< " " << fixed  << val2 <<  " " << fixed << val3 \
            << " "<< color1 << " "<< color2 << " "<< color3 << endl;
        }
     }
     out.close();
 }



void ThreadDenseMapper(DenseMapping *pdense_mapper ,ros::Publisher *pub_cloud)
{
    while(ros::ok())
    {
        fullydense_mapping(pdense_mapper,pub_cloud);
        boost::this_thread::sleep(boost::posix_time::milliseconds(33));
    }
}



void fullydense_mapping(DenseMapping *pdense_mapper,ros::Publisher *pub_cloud)
{
    if (pdense_mapper->get_do_dense()> 0.5)
    {
        Imagenes images;
        copy_from_dense2images(*pdense_mapper,images);
        int calculate_superpixels = pdense_mapper->calculate_superpixels;

        cv::Mat points6 = pdense_mapper->get_last3Dpoints().clone();


        float   limit_ratio_sing_val = pdense_mapper->limit_ratio_sing_val;
        float limit_normalized_residual = pdense_mapper->limit_normalized_residual;
        int  matchings_active_search =pdense_mapper->matchings_active_search;


        float   ph_error_limit = 20;

        cv::Mat image_points_byFocal_sd(0,3, CV_32FC1);


        cv::Mat points_ref_im_sd;
        float translational_ratio ;

        float mean_value;
        int discretization =50;

        photometric_term X_fd;

        for (int l=0; l<discretization; l++)
        {
            X_fd.computeError();
        }

        cv::Mat inv_depths(discretization,1, CV_32FC1);

        cv::Mat t_r_ref;

        float depth_step;


        float num_images = images.getNumberOfImages();
        int reference_image = num_images / 2.0;


        for (int j = 0; j< images.getNumberOfImages()-1 ; j = j+1)
        {
            images.computeImageSup();
        }


        int window_size=0;
        int corner =  0;

        cv::Mat depth_map;

        cv::Mat variance_points_tracked = depth_map*0;
        depth_map = depth_map*0;
        points6.convertTo(points6, CV_32FC1);
        get_inverse_depth(images,points6,inv_depths,depth_step,reference_image,discretization,mean_value,depth_map,1,variance_points_tracked);
        points6.convertTo(points6, CV_64FC1);

        char buffer[150];

        int init_mapping = 0;
        int end_mapping = images.getNumberOfImages()-5;


        cv::Mat initial_inv_depth_fd;


        int keyframe_obj;
        int image_to_be_added = 0;

        int num_cameras_mapping = 0;

        int step_dense_mapping = 1;


        int fast_mapping = 0;

        int  imsize_y =images.Im[reference_image]->image.rows;
        int  imsize_x =images.Im[reference_image]->image.cols;

        cv::Mat dense_uncertainty = cv::Mat::zeros(imsize_y,imsize_x, CV_32FC1);

        cv::Mat points_semidense_variational = points6.clone();

        cv::Mat Dtam_points_to_print,input;
        cv::Mat image_points_byFocal;

        float lambda_factor = 75;


        cv::Mat points_superpixels(0,6,CV_32FC1);
        cv::Mat points_superpixels_contours(0,6,CV_32FC1);


         int cont_images_reconstructed = 0;
         for (int i = 0; i<images.getNumberOfImages()-3;i++)
         {
             if (images.Im[i]->accurate_sd_map.rows > 0)
             {
                 cont_images_reconstructed++;
             }
         }

         int superpixels_index[5];
         for (int i = 0;i<5;i++)
         {
             superpixels_index[i]=0;
         }

         cont_images_reconstructed=0;
         for (int i = 0; i<images.getNumberOfImages()-3;i++)
         {
             if (images.Im[i]->accurate_sd_map.rows > 0)
             {
                 superpixels_index[cont_images_reconstructed] = i;
                 cont_images_reconstructed++;
             }
         }

         int number_row;

         for (int i = 0; i < sizeof(superpixels_index)/sizeof(superpixels_index[0]);i++)
         {
             calculate_superpixels_and_setdata(images,superpixels_index[i]);
         }
         //////////////// INITIALIZE SUPERPIXELS  AND calculate THEM
         cv::Mat aux;
         for (int i = 0; i<images.getNumberOfImages()-1;i++)
         {
             for (int j=0; j < images.supIm[i]->getNumberOfSuperpixels();j++ )
             {
                 images.supIm[i] -> getSuperpixeles()[j] -> SetbAlreadyMatched(false);
                 images.supIm[i] -> getSuperpixeles()[j] -> matchings = aux.clone();
             }
         }


         for (int i = 0; i< sizeof(superpixels_index)/sizeof(superpixels_index[0]); i++)
         {
             number_row = superpixels_index[i];


             if(images.Im[superpixels_index[i]]->accurate_sd_map.rows>0)
             {
                 cv::Mat accurate_sd_map = images.Im[superpixels_index[i]]->accurate_sd_map.clone();
                 if (i-1 > -0.5)
                 {
                     if(images.Im[superpixels_index[i-1]]->accurate_sd_map.rows>0)
                     {
                         accurate_sd_map.push_back(images.Im[superpixels_index[i-1]]->accurate_sd_map);
                     }
                 }

                 if (i+1 < sizeof(superpixels_index)/sizeof(superpixels_index[0]) )
                 {
                     if(images.Im[superpixels_index[i+1]]->accurate_sd_map.rows>0)
                     {
                         accurate_sd_map.push_back(images.Im[superpixels_index[i+1]]->accurate_sd_map);
                     }
                 }

                 calculate_3D_superpixels_from_semidense( limit_ratio_sing_val, limit_normalized_residual,
                                                          images,accurate_sd_map,number_row,
                                                         mean_value);
              }
         }


         //////////////// INITIALIZE SUPERPIXELS AND calculate THEM
         ///
         ///
         //////////////// ACTIVE MATCHING
         ///
         ///
         cv::Mat pixel_taken = cv::Mat::zeros(imsize_y,imsize_x,CV_32FC1) ;
         for (int i = 0; i< sizeof(superpixels_index)/sizeof(superpixels_index[0]);i++)
         {
             number_row = superpixels_index[i];
             active_matching(images,images.supIm,number_row,superpixels_index);
             for (int j = 0; j<= images.supIm[number_row]->getNumberOfSuperpixels()-1  ;j++)
             {
                 cv::Mat matchings= images.supIm[number_row]->getSuperpixeles()[j]->getMatchings();
                 cv::Mat pixels3D= images.supIm[number_row]->getSuperpixeles()[j]->getPixels3D();
                 cv::Mat pixelsCoordinates= images.supIm[number_row]->getSuperpixeles()[j]->getPixels();

                 if (matchings.rows > 0)
                 {
                     cv::Mat allPairs = matchings.clone();       // Mat with duplicate values
                     cv::Mat uniqueStrides;  // Mat that will contain the unique values
                     uniqueStrides.push_back( allPairs.row(0) );
                     for (int i = 1; i < allPairs.rows; ++i) {
                         int isInside = false;
                         for (int j = 0; j < uniqueStrides.rows; ++j) {
                             int count = 0;
                             for (int k = 0; k < uniqueStrides.cols; ++k) // checks by element of
                                 if(allPairs.at<int>(i,k) == uniqueStrides.at<int>(j,k))
                                     ++count;
                             if (count == 4) {
                                 isInside = true;
                                 break;
                             }
                         }
                         if (isInside == false) uniqueStrides.push_back( allPairs.row(i) );
                     }
                     matchings = uniqueStrides.clone();
                 }

                 if(  matchings.rows >= matchings_active_search|| images.supIm[number_row]->getSuperpixeles()[j]->informative == 1)
                 {
                     if ( pixels3D.rows > 0)
                     {
                       for (int jj = 0; jj<pixels3D.rows; jj++)
                       {
                           if (pixel_taken.at<float>(pixelsCoordinates.at<float>(jj,0),pixelsCoordinates.at<float>(jj,1)) < 0.5)
                           {
                               pixel_taken.at<float>(pixelsCoordinates.at<float>(jj,0),pixelsCoordinates.at<float>(jj,1))  = 1;
                               points_superpixels.push_back(pixels3D.row(jj));

                           }
                       }
                     }

                 }
             }
         }
         //////////////// ACTIVE MATCHING

         cv::Mat points_superpixels_all = points_superpixels.clone();
         cv::Mat points_superpixels_aux(6,0,CV_32FC1);
         for (int i = 1; i<points_superpixels.rows;i++)
         {
             if ((rand() % 1000000 ) / 1000000.0 < 25000.0/points_superpixels.rows)
             {
                   points_superpixels_aux.push_back(points_superpixels.row(i));
             }
         }
         points_superpixels = points_superpixels_aux.clone();



         if ( points_superpixels.rows > 2000)
         {
             dense_uncertainty = cv::Mat::zeros(imsize_y,imsize_x, CV_32FC1);

             pdense_mapper->map_sup_points_total.push_back(points_superpixels);
             pdense_mapper->set_superpixels3Dprint(points_superpixels);
             pdense_mapper->superpixels_print_number = pdense_mapper->dense_kf;
         }

         cv::Mat superpixels_to_project_aux(0,6,CV_32FC1);
         cv::Mat superpixels_to_project;

         if (points_superpixels.rows > 100)
         {

             superpixels_to_project_aux = points_superpixels.clone();
         }


         if (superpixels_to_project_aux.rows > 10)
         {
             superpixels_to_project = superpixels_to_project_aux.clone();
             superpixels_to_project=superpixels_to_project.colRange(0,3);
             pdense_mapper->set_superpixels3Dtracked(superpixels_to_project);
         }


         // PUBLISH SUPERPIXELS

         pcl::PointCloud<pcl::PointXYZRGB> map_pcl;

         for (int i = 1; i < points_superpixels_all.rows; i++)
         {
             float a = points_superpixels_all.at<double>(i,0);
             float b = points_superpixels_all.at<double>(i,1);
             float c = points_superpixels_all.at<double>(i,2);
             int aa = round(points_superpixels_all.at<double>(i,3));
             int bb = round(points_superpixels_all.at<double>(i,4));;
             int cc = round(points_superpixels_all.at<double>(i,5));

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
         // PUBLISH SUPERPIXELS


         // PRINT SUPERPIXELS
         if (points_superpixels_all.rows > 10)
         {
             char buffer_sup[150];
             sprintf (buffer_sup,"src/dpptam/src/map_and_poses/sup%d.ply", pdense_mapper->dense_kf);
             print_plane(points_superpixels_all,buffer_sup);
         }
         // PRINT SUPERPIXELS


         pdense_mapper->dense_kf++;

         if ( calculate_superpixels < 0.5)
         {
                       ////////////////////////////GET PHOTOMETRIC TERM_SD term_sd ///////////////////////////////////////
                         cv::Mat points_i_sd(0,3, CV_32FC1);
                         cv::Mat point_i_sd(1,3, CV_32FC1);
                         cv::Mat point_ref_im_sd(1,3, CV_32FC1);

                        for (int i = corner; i < images.Im[reference_image]->image.rows-corner; i++)
                        {
                            for (int j = corner; j < images.Im[reference_image]->image.cols-corner; j++)
                            {
                                    point_i_sd.at<float>(0,0) = (images.Im[reference_image]->cx-j)/images.Im[reference_image]->fx;
                                    point_i_sd.at<float>(0,1) = (i-images.Im[reference_image]->cy)/images.Im[reference_image]->fy;
                                    point_i_sd.at<float>(0,2) = 1;

                                    points_i_sd.push_back(point_i_sd);

                                    point_ref_im_sd.at<float>(0,0) = i;
                                    point_ref_im_sd.at<float>(0,1) = j;
                                    point_ref_im_sd.at<float>(0,2) = 1;
                                    points_ref_im_sd.push_back( point_ref_im_sd);
                            }
                        }

                        points_i_sd = points_i_sd.t();

                        //photometric_term X;
                        initial_inv_depth_fd = cv::Mat::zeros(points_i_sd.cols, 1, CV_32FC1) + inv_depths.at<float>(0,0);
                        t_r_ref = cv::repeat(images.Im[reference_image]->t, 1, points_i_sd.cols).clone();




                        cv::Mat points_i2_sd;
                        photometric_term point_limits_for_sd;

                        for (unsigned int l = 0; l<discretization;l++)
                        {
                            point_limits_for_sd.computeError();

                            if (l==0 || l == discretization-1)
                            {
                                points_i2_sd = points_i_sd / inv_depths.at<float>(l,0);
                                points_i2_sd = images.Im[reference_image]->R.t() * (points_i2_sd - t_r_ref);

                                point_limits_for_sd.ph_error[l] = points_i2_sd.clone();
                            }
                        }

                        for (int i = corner; i < images.Im[reference_image]->image.rows-corner; i++)
                        {
                            for (int j = corner; j < images.Im[reference_image]->image.cols-corner; j++)
                            {
                                        point_i_sd.at<float>(0,0) = (images.Im[reference_image]->cx-j)/images.Im[reference_image]->fx;
                                        point_i_sd.at<float>(0,1) = (i-images.Im[reference_image]->cy)/images.Im[reference_image]->fy;
                                        point_i_sd.at<float>(0,2) = 1;
                                        image_points_byFocal_sd.push_back(point_i_sd);
                            }
                        }
                        ////////////////////////////GET PHOTOMETRIC TERM_SD term_sd ///////////////////////////////////////



                        step_dense_mapping = round(end_mapping / 10);

                        for (int i=init_mapping; i< end_mapping ; i = i+step_dense_mapping)
                        {
                                        keyframe_obj=i;

                                        cv::Mat R1,R2,t1,t2;

                                        R1 = images.Im[reference_image]->R;
                                        R2 = images.Im[keyframe_obj]->R;
                                        t1 = images.Im[reference_image]->t;
                                        t2 = images.Im[keyframe_obj]->t;

                                        image_to_be_added=i;

                                        t2.convertTo(t2,CV_64FC1);
                                        t1.convertTo(t1,CV_64FC1);

                                        translational_ratio = (abs(t1.at<double>(0,0) - t2.at<double>(0,0)) + abs(t1.at<double>(1,0) - t2.at<double>(1,0)) + \
                                                               abs(t1.at<double>(2,0) - t2.at<double>(2,0)) )  / mean_value;

                                         window_size = 3;
                                         get_photometric_errors_matrix_fd(images,  inv_depths, X_fd,  reference_image,  initial_inv_depth_fd, image_to_be_added,  \
                                                                       point_limits_for_sd,points_ref_im_sd,discretization,window_size,  ph_error_limit);
                                         num_cameras_mapping++;
                        }


                        if (num_cameras_mapping > 5)
                        {
                             cv::Mat sup_3Dpoints(0,3, CV_32FC1);
                             points6.convertTo(points6,CV_32FC1);

                            ////DTAM FORMULATION  BEGIN ////////////////////////////////////
                            for (int l=0; l<discretization; l++)
                            {
                                X_fd.ph_error[l] = X_fd.ph_error[l] / (num_cameras_mapping*(window_size+1)*(window_size+1));
                            }
                            ////////////////////////////////////////

                            cv::Mat min_ph_error = cv::Mat::zeros(initial_inv_depth_fd.rows, 1, CV_32FC1);
                            min_ph_error += INFINITY;

                            for (int l = 0; l < discretization; l = l+1)
                            {
                                   for (int i=0; i<initial_inv_depth_fd.rows; i++)
                                   {
                                        if ((X_fd.ph_error[l].at<float>(i,0)<min_ph_error.at<float>(i,0)) )
                                        {
                                            min_ph_error.at<float>(i,0) = X_fd.ph_error[l].at<float>(i,0);
                                            initial_inv_depth_fd.at<float>(i,0) =  inv_depths.at<float>(l,0);
                                        }
                                   }
                            }

                             points_semidense_variational.convertTo(points_semidense_variational,CV_32FC1);
                             points_superpixels.convertTo(points_superpixels,CV_32FC1);

                             if (points_superpixels.rows > 11)
                             {sup_3Dpoints.push_back(points_superpixels.colRange(0,3));}
                             int superpixels_number = points_superpixels.rows;

                             sup_3Dpoints.push_back(points6.colRange(0,3));
                             fast_mapping = 0;

                             cv::Mat planar_prior  = cv::Mat::zeros(imsize_y,imsize_x, CV_32F);

                             variational_mapping(images, sup_3Dpoints, reference_image, initial_inv_depth_fd, \
                                 depth_step, inv_depths,X_fd,Dtam_points_to_print, imsize_y, imsize_x,  input,image_points_byFocal,\
                                 lambda_factor,discretization,fast_mapping,t_r_ref, dense_uncertainty,superpixels_number,planar_prior);

                             sprintf (buffer,"src/dpptam/src/map_and_poses/dense_sup_fil%d.ply", pdense_mapper->dense_kf);
                             prepare_denseMapKeyframe(Dtam_points_to_print,input,buffer,dense_uncertainty,points_semidense_variational);

                             dense_uncertainty =1 ;
                             sprintf (buffer,"src/dpptam/src/map_and_poses/dense_sup_nofil%d.ply", pdense_mapper->dense_kf);
                             prepare_denseMapKeyframe(Dtam_points_to_print,input,buffer,dense_uncertainty,points_semidense_variational);
                        } // if num_cameras_mapping > 5
         } // if calculate_superpixels <  0.5



         pdense_mapper->set_do_dense(0);

         for (int j = 0; j< images.getNumberOfImageSuperpixels(); j = j+1)
         {
             for (int i = 0; i<images.supIm[j]->getNumberOfSuperpixels() ; i++)
             {
                 delete images.supIm[j]->sup[i];
             }
             images.supIm[j]->sup.clear();
             delete images.supIm[j];
         }
          images.supIm.clear();



         for (int j = 0; j< images.getNumberOfImages(); j = j+1)
         {
             delete images.Im[j];
         }
         images.Im.clear();

    } // if get_do_dense

}


void copy_from_dense2images(DenseMapping &dense, Imagenes &images)
{
    int images_size  = dense.getNumberOfImages()-1;


    for (int l = 0 ; l < images_size-1; l = l+1)
    {
        images.computeImage();
        int images_map_size = images.getNumberOfImages()-1;

        images.Im[images_map_size ]->image = dense.Im[l]->image.clone();
        images.Im[images_map_size ]->R = dense.Im[l]->R.clone();
        images.Im[images_map_size ]->image_gray= dense.Im[l]->image_gray.clone();
        images.Im[images_map_size ]->t =dense.Im[l]->t.clone();
        images.Im[images_map_size ]->t_r = dense.Im[l]->t_r.clone();
        images.Im[images_map_size ]->fx = dense.Im[l]->fx;
        images.Im[images_map_size ]->fy = dense.Im[l]->fy;
        images.Im[images_map_size ]->cx = dense.Im[l]->cx;
        images.Im[images_map_size ]->cy = dense.Im[l]->cy;
        images.Im[images_map_size]->error = dense.Im[l]->error;
        images.Im[images_map_size]->k1 = dense.Im[l]->k1;
        images.Im[images_map_size]->k2 = dense.Im[l]->k2;
        images.Im[images_map_size]->stamps = dense.Im[l]->stamps;
        images.Im[images_map_size]->num_keyframes = dense.Im[l]-> num_keyframes;
        images.Im[images_map_size]->accurate_sd_map =  dense.Im[l]-> accurate_sd_map;

    }
}

void get_inverse_depth(Imagenes images, cv::Mat &points,cv::Mat &inv_depths, float &depth_step, int reference_image, \
                       int discretization,float &mean_value,cv::Mat &depth_map, int set_maximo,cv::Mat &variance_points_tracked)
{
    double minVal, maxVal;

    cv::Mat transformed_points;
    transform_points(images,  reference_image,  points, transformed_points);
    cv::Mat projected_points(transformed_points.cols,transformed_points.rows,CV_32FC1);

    float maximo,minimo;
    distances_interval( images,  reference_image,  transformed_points, projected_points, maximo, minimo,mean_value,depth_map,points,set_maximo,variance_points_tracked);


    float aux = minimo;
    minimo = - maximo;
    maximo = -aux;

    minVal=1/minimo;
    maxVal=1/maximo;


    depth_step = (maxVal-minVal)/(discretization-1);

    for (int i = 0; i<discretization ; i++)
    {
        inv_depths.at<float>(i,0) = minVal + i*depth_step;
    }
}



void calculate_superpixels_and_setdata(Imagenes &images,  int reference_image)
{

    char curre_dir[150];
    getcwd(curre_dir,149);

    chdir("src/dpptam/src/images");

    char buffer[150];
    cv::Mat image_reference = images.Im[reference_image]->image.clone();
    cv::Mat image_reference_gray = images.Im[reference_image]->image_gray.clone();
    image_reference_gray.convertTo(image_reference_gray,CV_32FC1);
    sprintf (buffer,"im.ppm");
    cv::imwrite(buffer,image_reference );
    sprintf(buffer,"./segment 0.9 100 20 im.ppm out.ppm");
    system(buffer);

    images.supIm[reference_image]->image_name = "im.ppm";
    images.supIm[reference_image]->sup_name = "out.ppm";
    images.supIm[reference_image]->image = image_reference.clone();
    images.supIm[reference_image]->computeSuperpixels();

    cv::Mat R1,t1;

    float f1,ka1,kb1;

    int i = reference_image;
    f1 =  images.Im[i]->fx;
    ka1 = images.Im[i]->k1;
    kb1 = images.Im[i]->k2;
    R1 =  images.Im[i]->R;
    t1 =  images.Im[i]->t;

    images.supIm[i]   -> set_data (R1,t1,f1,ka1,kb1);
    chdir(curre_dir);
}

void calculate_3D_superpixels_from_semidense(float limit_ratio_sing_val,float limit_normalized_residual,Imagenes &images, cv::Mat &points6, int reference_image, float mean_value)
{

    cv::Mat image_reference = images.Im[reference_image]->image.clone();
    cv::Mat matrix_3Dpoints_inImage;
    points6.convertTo(points6, CV_32FC1);
    get_3Dpoints_inImage(images,points6,matrix_3Dpoints_inImage,reference_image);


    for (int i = 0; i<= images.supIm[reference_image]->getNumberOfSuperpixels()-1  ;i++)
    {
        cv::Mat pixels = images.supIm[reference_image]->getSuperpixeles()[i]->getPixels();
        cv::Mat pixels_sup = images.supIm[reference_image]-> getSuperpixeles()[i]->getContour();

        cv::Mat points_sup (0,6,CV_32FC1);
        cv::Mat points_sup_total (0,6,CV_32FC1);

        if ( pixels.rows > 1500)
        {
            for (int j = 0; j < pixels_sup.rows; j++)
            {
                    if (matrix_3Dpoints_inImage.at<float>(pixels_sup.at<float>(j,0),pixels_sup.at<float>(j,1)) > 0)
                    {
                        points_sup.push_back(points6.row(matrix_3Dpoints_inImage.at<float>(pixels_sup.at<float>(j,0),pixels_sup.at<float>(j,1))));
                        points_sup.at<float>(points_sup.rows-1,3) = -1;
                    }
            }

            for (int j = 0; j < pixels.rows; j++)
            {
                    if (matrix_3Dpoints_inImage.at<float>(pixels.at<float>(j,0),pixels.at<float>(j,1)) > 0)
                    {
                        points_sup_total.push_back(points6.row(matrix_3Dpoints_inImage.at<float>(pixels.at<float>(j,0),pixels.at<float>(j,1))));
                        points_sup_total.at<float>(points_sup_total.rows-1,3) = -1;
                    }
            }
        }


        float points_sup_rows = points_sup.rows;
        points_sup_rows = points_sup_total.rows;
        float pixels_rows = pixels_sup.rows;

        if (points_sup_rows/pixels_rows > 0.4500)
        {
            cv::Mat error_wrt_plane,n,n1;
            float d,d1;

            int be_spx_outlier = 0;
            cv::Mat singular_values;
            ///////////////////// Check if the contour cannot  be exaplained by a plane
            cv::Mat distances_btw_3dpoints(0,1,CV_32FC1);

            for (int ii = 0; ii < 20; ii++)
            {
                int random_row1 = (rand() % points_sup_total.rows);
                int random_row2 = (rand() % points_sup_total.rows);

                distances_btw_3dpoints.push_back(cv::norm(points_sup_total.colRange(0,3).row(random_row1)-points_sup_total.colRange(0,3).row(random_row2),cv::NORM_L1) / 3);
            }

            cv::Mat sorted_error_wrt_plane;
            cv::sort(abs(distances_btw_3dpoints),sorted_error_wrt_plane,CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);

            float average_distance_btw_points = mean(sorted_error_wrt_plane)[0];
            ///////////////////// Check if the contour cannot  be exaplained by a plane

            ransac_for_3Dspx(points_sup_total,error_wrt_plane,n,d,singular_values,limit_ratio_sing_val,average_distance_btw_points,limit_normalized_residual);

            if (singular_values.rows > 2)
            {
                    ///// Check if the contour can be exaplained by a line in the image
                    if(singular_values.at<float>(0,0)/singular_values.at<float>(2,0)>limit_ratio_sing_val)
                    {be_spx_outlier = 1;}
                    ///// Check if the contour can be exaplained by a line in the image


                    ////////////////////// Transform the normal and distance to image reference system
                    cv::Mat punto_plano= n.clone()*0;
                    punto_plano.at<float>(0,0) = d / n.at<float>(0,0);

                    cv::Mat R1,t1;
                    R1 = images.Im[reference_image]->R;
                    t1 = images.Im[reference_image]->t;
                    n1=R1*n;

                    cv::Mat punto_plano1 = R1 * (punto_plano) + t1;
                    cv::Mat mat_d1   = n1.t()*punto_plano1;

                    d1 = mat_d1.at<float>(0,0);
                    ////////////////////// Transform the normal and distance to image reference system


                    ///////////////////// Check if the contour cannot  be exaplained by a plane

                   if (  (mean(error_wrt_plane)[0]) / ((mean(sorted_error_wrt_plane)[0])) > limit_normalized_residual)
                   {be_spx_outlier = 1;}

                   ///////////////////////// Check if the contour cannot be exaplained by a plane

                   cv::Mat X_total(pixels.rows,3,CV_32FC1);
                   cv::Mat  inv_depth_mat_total;

                   if (  be_spx_outlier < 0.5)
                   {
                         backproject_from_plane(images,pixels,n1,d1,inv_depth_mat_total,X_total,reference_image);
                         for (int j = 0; j < pixels.rows; j++)
                         {
                             float inv_depth = inv_depth_mat_total.at<float>(j,0);
                             if ((abs(1/inv_depth)) >(2.5*abs(mean_value)))
                             {be_spx_outlier = 1;}
                         }
                   }


                   float  informative_points = 0;
                   for (int j = 0; j < pixels.rows; j++)
                   {
                       if (  be_spx_outlier < 0.5 )
                       {
                           if (matrix_3Dpoints_inImage.at<float>(pixels.at<float>(j,0),pixels.at<float>(j,1))> 0)
                           {
                               informative_points++;
                           }
                       }
                   }

                   if ( be_spx_outlier < 0.5)
                   {
                       cv::Mat points_contour_3D(pixels_sup.rows,3,CV_32FC1);
                       backproject_from_plane(images,pixels_sup,n1,d1,inv_depth_mat_total,points_contour_3D,reference_image);

                       points_contour_3D = points_contour_3D.t();
                       images.supIm[reference_image]->getSuperpixeles()[i]->set_contour3D(points_contour_3D);
                   }



                   if ( be_spx_outlier < 0.5 && (informative_points  / pixels.rows) >  0.40 &&  pixels.rows > 3700)
                   {
                       images.supIm[reference_image]->getSuperpixeles()[i]->informative = 1;
                   }

                   cv::Mat pixels3D(0,6,CV_64FC1);

                   for (int j = 0; j < pixels.rows; j++)
                   {
                       if (  be_spx_outlier < 0.5 )
                       {
                           cv::Mat points_superpixels_aux(1,6,CV_64FC1);

                           points_superpixels_aux.at<double>(0,0)=X_total.at<float>(j,0);
                           points_superpixels_aux.at<double>(0,1)=X_total.at<float>(j,1);
                           points_superpixels_aux.at<double>(0,2)=X_total.at<float>(j,2);

                           points_superpixels_aux.at<double>(0,3)=image_reference.at<cv::Vec3b>(pixels.at<float>(j,0),pixels.at<float>(j,1))[2];
                           points_superpixels_aux.at<double>(0,4)=image_reference.at<cv::Vec3b>(pixels.at<float>(j,0),pixels.at<float>(j,1))[1];
                           points_superpixels_aux.at<double>(0,5)=image_reference.at<cv::Vec3b>(pixels.at<float>(j,0),pixels.at<float>(j,1))[0];

                           pixels3D.push_back(points_superpixels_aux);
                       }
                   }

                   if (  be_spx_outlier < 0.5 )
                   {
                       images.supIm[reference_image]->getSuperpixeles()[i]->set_pixels3D(pixels3D);
                   }
            } // if at_leaset one singular value
        } // if at leas most of the contour has gradients
    } // i =number of spx
}


void  active_matching(Imagenes &images,vector<SuperpixelesImagen*> &supImg, int reference_image, int superpixels_index[])
{
    int imsize_x = images.Im[reference_image]->image.cols;
    int imsize_y = images.Im[reference_image]->image.rows;

    float  percentage =0;
    bool   exit = 0;

    float  percentage_limit2 = 0.70;
    float  percentage_limit1 = 0.50;

    cv::Mat contour11;
    cv::Mat contour22;


    for (int keyframes_aux = reference_image; keyframes_aux<=reference_image; keyframes_aux++)
    {
        for (int z = 0; z <sizeof(superpixels_index); z++)
          {
            int keyframes = superpixels_index[z];

            if (keyframes !=  keyframes_aux  )
               {
                    for (int i = 0; i <= supImg[keyframes_aux] -> getNumberOfSuperpixels()-1  ;i++)
                    {exit = 0;
                    for (int j = 0 ; j <=supImg[keyframes]->getNumberOfSuperpixels()-1 ; j++)
                    {
                        if (exit == 1){break;}
                        if (supImg[keyframes_aux]->getSuperpixeles()[i]->getContour3D().rows > 0 && supImg[keyframes_aux]->getSuperpixeles()[i]->isLarge()  \
                                && supImg[keyframes]->getSuperpixeles()[j]->isLarge() \
                                && supImg[keyframes]->getNumberOfSuperpixels() > 0 )
                        {
                                float number_contour_points1 = supImg[keyframes_aux]->getSuperpixeles()[i]->getPixels().rows;
                                float number_contour_points2 = supImg[keyframes]->getSuperpixeles()[j]->getPixels().rows;
                                if ( number_contour_points1/ number_contour_points2 < 2 \
                                        && number_contour_points1/ number_contour_points2  > 0.5)
                                {

                                         cv::Mat contour3D = supImg[keyframes_aux]-> getSuperpixeles()[i] -> getContour3D ();
                                         contour11 = supImg[keyframes_aux]->getSuperpixeles()[i]->getContour(); //
                                         contour22 = supImg[keyframes]->getSuperpixeles()[j]->getContour();


                                         if (supImg[keyframes] -> getSuperpixeles()[j] -> getMatrix().rows==0)
                                         {supImg[keyframes] -> getSuperpixeles()[j] -> set_matrix (create_matrix(contour22,5, imsize_x, imsize_y));}


                                         if (supImg[keyframes] -> getSuperpixeles()[j]-> getContourR().rows ==0)
                                          {
                                             contour22 = supImg[keyframes]->getSuperpixeles()[j]->getContour();
                                             cv::Mat contour2 = contour22.clone();
                                             supImg[keyframes] -> getSuperpixeles()[j]-> set_contourR(contour2);
                                         }


                                         DataToSend DTS;
                                         DTS.showim = 1;
                                         DTS.percentage_limit = percentage_limit2;DTS.step_size = 1;
                                         DTS.SupImg = supImg;DTS.RefIm=keyframes_aux;DTS.RefSup =i;

                                         percentage = montecarlo_seed10(images,keyframes_aux,i,keyframes,j,DTS,\
                                                                       1,contour3D);

                                         if (percentage > percentage_limit2)
                                         {
                                             supImg[keyframes_aux]-> getSuperpixeles()[i] -> set_matchings (keyframes_aux,i,keyframes,j);
                                         }
                                         if (percentage > percentage_limit1)
                                         {
                                             exit = 1;
                                             supImg[keyframes_aux]-> getSuperpixeles()[i] -> SetbAlreadyMatched(true);
                                             supImg[keyframes]-> getSuperpixeles()[j] -> SetbAlreadyMatched(true);
                                         }

                                } //if contours are similar in size
                          } //if large
                   } // for sup
                 } // for sup
            } // if !=
        } //for image
    } // for  image
}


void get_photometric_errors_matrix_fd(Imagenes  &images,  cv::Mat &inv_depths, photometric_term &X, \
                                      int reference_image, cv::Mat &initial_inv_depth , int image_to_be_added, \
                                     photometric_term &points_i_todos,cv::Mat &points_ref_im_sd,int discretization, \
                                      int window_size, float ph_error_limit)
{
            cv::Mat errores;
            for (unsigned int l = 0; l < discretization; l++)
            {
                //X.computeError();
                errores = cv::Mat::zeros(initial_inv_depth.rows, 1, CV_32F);
                if (X.ph_error[l].empty())
                {
                    errores.copyTo(X.ph_error[l]);
                }
            }

            errores = cv::Mat::zeros(initial_inv_depth.rows, 1, CV_32F) + INFINITY;

            int  m = image_to_be_added;
            cv::Mat t_r = cv::repeat(images.Im[m]->t, 1, initial_inv_depth.rows).clone();

            cv::Mat image_o_gray = images.Im[m]->image_gray;
            cv::Mat image_i_gray = images.Im[reference_image]->image_gray;

            image_i_gray.convertTo(image_i_gray,CV_32FC1);
            image_o_gray.convertTo(image_o_gray,CV_32FC1);

            cv::Mat points_i_2;
            cv::Mat points_o;

            points_i_todos.ph_error[0].copyTo(points_i_2);
            points_o  = images.Im[m]->R  * (points_i_2) + t_r;
            points_o.colRange(0,points_o.cols).rowRange(0,1) = points_o.colRange(0,points_o.cols).rowRange(0,1) * images.Im[m]->fx;
            points_o.colRange(0,points_o.cols).rowRange(1,2) = points_o.colRange(0,points_o.cols).rowRange(1,2) * images.Im[m]->fy;
            cv::Mat points_o_init = points_o.clone();
            /////
            cv::Mat inv_X3_init = 1 / points_o.colRange(0,points_o.cols).rowRange(2,3);
            //////
            float inv_depths_init = 1/inv_depths.at<float>(0,0);
            cv::Mat z_repeat =  cv::repeat(points_o.colRange(0,points_o.cols).rowRange(2,3),3,1);
            cv::divide(points_o, z_repeat,points_o,1);
            points_o.colRange(0,points_o.cols).rowRange(0,1) =  images.Im[reference_image]->cx-points_o.colRange(0,points_o.cols).rowRange(0,1) ;
            points_o.colRange(0,points_o.cols).rowRange(1,2) =  points_o.colRange(0,points_o.cols).rowRange(1,2) + images.Im[reference_image]->cy;

            /////
            cv::Mat xvalues_init = points_o.colRange(0,points_o.cols).rowRange(0,1).clone();
            cv::Mat yvalues_init = points_o.colRange(0,points_o.cols).rowRange(1,2).clone();
            //////

            points_i_todos.ph_error[discretization-1].copyTo(points_i_2);
            points_o  = images.Im[m]->R  * (points_i_2) + t_r;
            points_o.colRange(0,points_o.cols).rowRange(0,1) = points_o.colRange(0,points_o.cols).rowRange(0,1) * images.Im[m]->fx;
            points_o.colRange(0,points_o.cols).rowRange(1,2) = points_o.colRange(0,points_o.cols).rowRange(1,2) * images.Im[m]->fy;
            cv::Mat points_o_last = points_o.clone();
            /////
            cv::Mat inv_X3_end = 1 / points_o.colRange(0,points_o.cols).rowRange(2,3);
            //////
            float inv_depths_last = 1/inv_depths.at<float>(discretization-1,0);
            z_repeat =  cv::repeat(points_o.colRange(0,points_o.cols).rowRange(2,3),3,1);
            cv::divide(points_o, z_repeat,points_o,1);
            points_o.colRange(0,points_o.cols).rowRange(0,1) =  images.Im[reference_image]->cx-points_o.colRange(0,points_o.cols).rowRange(0,1) ;
            points_o.colRange(0,points_o.cols).rowRange(1,2) =  points_o.colRange(0,points_o.cols).rowRange(1,2) + images.Im[reference_image]->cy;

            /////
            cv::Mat xvalues_end = points_o.colRange(0,points_o.cols).rowRange(0,1).clone();
            cv::Mat yvalues_end = points_o.colRange(0,points_o.cols).rowRange(1,2).clone();
            //////

            cv::Mat aaa = (points_o_last-points_o_init) / (inv_depths_last-inv_depths_init);
            cv::Mat bbb = points_o_init - aaa*inv_depths_init;



            cv::Mat ccc = (xvalues_end-xvalues_init)/ (inv_X3_end-inv_X3_init);
            cv::Mat ddd = xvalues_init - ccc.mul(inv_X3_init);

            cv::Mat eee = (yvalues_end-yvalues_init) / (inv_X3_end-inv_X3_init);
            cv::Mat fff = yvalues_init -eee.mul(inv_X3_init);


            int window_size_end =  window_size+1;
            float out_error = (window_size+1)*(window_size+1)*ph_error_limit*1.1;

            for (unsigned int l = 0; l < discretization; l = l+1)
            {
                   cv::Mat points_o = aaa/inv_depths.at<float>(l,0)+bbb;
                   points_o.colRange(0,points_o.cols).rowRange(0,1) = ( 1/points_o.colRange(0,points_o.cols).rowRange(2,3)).mul(ccc) +ddd;
                   points_o.colRange(0,points_o.cols).rowRange(1,2) =   (1/points_o.colRange(0,points_o.cols).rowRange(2,3)).mul(eee)+ fff;


                   for (int i=0; i<initial_inv_depth.rows; i=i+1)
                   {
                           int n_x = round(points_o.at<float>(0,i));
                           int n_y = round(points_o.at<float>(1,i));

                           int n_x_ref = round(points_ref_im_sd.at<float>(i,1));
                           int n_y_ref = round(points_ref_im_sd.at<float>(i,0));

                           if (n_x > 3& n_x< images.Im[reference_image]->image.cols-4 & n_y > 3 \
                                   & n_y<images.Im[reference_image]->image.rows-4
                                   & n_x_ref > 3& n_x_ref< images.Im[reference_image]->image.cols-4 & n_y_ref >3 \
                                   & n_y_ref<images.Im[reference_image]->image.rows-4)
                           {

                             double error = 0;

                             for (int mm=-window_size;mm<window_size_end;mm = mm+2)
                             {
                                 for (int nn = -window_size; nn <window_size_end; nn=nn+2)
                                   {
                                       error   += fabs(image_i_gray.at<float>(n_y_ref+nn,n_x_ref+mm) -image_o_gray.at<float>(n_y+nn,n_x+mm));
                                   }
                              }
                              if (error > out_error){error=out_error;}
                              X.ph_error[l].at<float>(i,0) += error;
                           }
                           else
                           {
                               X.ph_error[l].at<float>(i,0) += out_error;
                           }
                 }
            } // for depths
}


void variational_mapping(Imagenes &images, cv::Mat &prior_points, int reference_image, cv::Mat initial_inv_depth, \
        float &depth_step, cv::Mat &inv_depths, photometric_term &X, cv::Mat &Dtam_points_to_print,\
             int imsize_y, int imsize_x,  cv::Mat &input, cv::Mat &image_points_byFocal,\
            float lambda_factor, int discretization, int &fast_mapping,cv::Mat &t_r_ref,cv::Mat &dense_uncertainty, \
                         int &superpixels_number, cv::Mat &planar_prior)
{
    cv::Mat prior  = cv::Mat::zeros(imsize_y,imsize_x, CV_32F);
    cv::Mat lambda = cv::Mat::zeros(imsize_y,imsize_x, CV_32F);

    cv::Mat transformed_points;


    if (prior_points.rows>0)
    {
        transform_points(images,  reference_image,  prior_points, transformed_points);
        cv::Mat projected_points(transformed_points.cols,transformed_points.rows,CV_32FC1);
        calculate_prior( images,  reference_image,  transformed_points, projected_points, prior,lambda, dense_uncertainty,superpixels_number,planar_prior);
    }


    cv::Mat estimated_inv_depth = cv::Mat::zeros(images.Im[reference_image]->image.cols*images.Im[reference_image]->image.rows, 1, CV_32F);

    input = images.Im[reference_image]->image;



    float reducir;
    float tetha,beta,alpha_limit;


    estimated_inv_depth = initial_inv_depth.clone();

    reducir = 1;
    tetha = 0.0150;
    beta = 0.0050;
    beta = 0.020;
    beta = 0.0010;
    alpha_limit = 0.005;


    beta = 0.20;
    beta = 0.00010;
    alpha_limit = 0.0001;
    tetha = 0.025;
    beta = 0.001;
    alpha_limit = 0.005;

    if (fast_mapping < 0.5)
    {
        inverse_depth_optimization(input, initial_inv_depth, X, depth_step, estimated_inv_depth, inv_depths, prior,\
                           lambda, lambda_factor,imsize_y,imsize_x, reducir,tetha,beta,alpha_limit,discretization,superpixels_number,planar_prior);
    }

    cv::Mat image_points_byFocal2(0,3, CV_32FC1);

    image_points_byFocal = image_points_byFocal2.clone();

    cv::Mat point_i(1,3,CV_32FC1);

    for (int i = 0; i < images.Im[reference_image]->image.rows; i++)
    {
        for (int j = 0; j < images.Im[reference_image]->image.cols; j++)
        {
            point_i.at<float>(0,0) = (images.Im[reference_image]->cx-j)/images.Im[reference_image]->fx;
            point_i.at<float>(0,1) = (i-images.Im[reference_image]->cy)/images.Im[reference_image]->fy;
            point_i.at<float>(0,2) = 1;
            image_points_byFocal.push_back(point_i);
        }
    }

    image_points_byFocal.copyTo(Dtam_points_to_print);

    Dtam_points_to_print.colRange(0,1).rowRange(0,image_points_byFocal.rows) = image_points_byFocal.colRange(0,1).rowRange(0,image_points_byFocal.rows) / estimated_inv_depth;
    Dtam_points_to_print.colRange(1,2).rowRange(0,image_points_byFocal.rows) = image_points_byFocal.colRange(1,2).rowRange(0,image_points_byFocal.rows) / estimated_inv_depth;
    Dtam_points_to_print.colRange(2,3).rowRange(0,image_points_byFocal.rows) = image_points_byFocal.colRange(2,3).rowRange(0,image_points_byFocal.rows) / estimated_inv_depth;

    Dtam_points_to_print = Dtam_points_to_print.t();

    Dtam_points_to_print = images.Im[reference_image]->R.t() * (Dtam_points_to_print - t_r_ref);
    Dtam_points_to_print = Dtam_points_to_print.t();


    input = images.Im[reference_image]->image.clone();

    int corner = 20;

    cv::Mat Dtam1 =  1*Dtam_points_to_print.rowRange(0,Dtam_points_to_print.rows).colRange(0,1);
    cv::Mat Dtam2 =  1*Dtam_points_to_print.rowRange(0,Dtam_points_to_print.rows).colRange(1,2);
    cv::Mat Dtam3 =  1*Dtam_points_to_print.rowRange(0,Dtam_points_to_print.rows).colRange(2,3);



    input =  input.reshape(0,imsize_y);
    input =  1*input.colRange(corner,imsize_x-corner).rowRange(corner,imsize_y-corner);
    input =  input.reshape(0,input.rows*input.cols);

    dense_uncertainty =   dense_uncertainty.reshape(0,imsize_y);
    dense_uncertainty =   1*dense_uncertainty.colRange(corner,imsize_x-corner).rowRange(corner,imsize_y-corner);
    dense_uncertainty =   dense_uncertainty.reshape(0, dense_uncertainty.rows* dense_uncertainty.cols);


    planar_prior =   planar_prior.reshape(0,imsize_y);
    planar_prior =   1*planar_prior.colRange(corner,imsize_x-corner).rowRange(corner,imsize_y-corner);
    planar_prior =   planar_prior.reshape(0, planar_prior.rows* planar_prior.cols);


    Dtam1 = Dtam1.reshape(0,imsize_y);
    Dtam2 = Dtam2.reshape(0,imsize_y);
    Dtam3 = Dtam3.reshape(0,imsize_y);

    Dtam1 = 1*Dtam1.colRange(corner,imsize_x-corner).rowRange(corner,imsize_y-corner);
    Dtam2 = 1*Dtam2.colRange(corner,imsize_x-corner).rowRange(corner,imsize_y-corner);
    Dtam3 = 1*Dtam3.colRange(corner,imsize_x-corner).rowRange(corner,imsize_y-corner);


    Dtam1 =  1*Dtam1.reshape(0,Dtam1.rows*Dtam1.cols);
    Dtam2 =  1*Dtam2.reshape(0,Dtam2.rows*Dtam2.cols);
    Dtam3 =  1*Dtam3.reshape(0,Dtam3.rows*Dtam3.cols);


    cv::Mat Dtam4(Dtam1.rows,3,CV_32FC1);


    Dtam4.rowRange(0,Dtam1.rows).colRange(0,1) = 1*Dtam1;
    Dtam4.rowRange(0,Dtam2.rows).colRange(1,2) = 1*Dtam2;
    Dtam4.rowRange(0,Dtam3.rows).colRange(2,3) = 1*Dtam3;

    Dtam_points_to_print = Dtam4.clone();
}


void prepare_denseMapKeyframe(cv::Mat &points, cv::Mat &Intensity, char buffer[],cv::Mat &dense_uncertainty,cv::Mat &points_semidense_variational)
{
     ofstream out(buffer);
     int val = points.rows;

     val = 0;
     for (int i = 0; i< points.rows;i++)
     {
         if (dense_uncertainty.at<float>(i,0)> 0.5)
         {
             val++;
         }
     }

     cv::Mat points_sd_var_aux(1,6,CV_32FC1);
     cv::Mat points_sd_var_aux1(0,6,CV_32FC1);

     out << "ply" << endl;out << "format ascii 1.0" << endl;out << "element face 0" << endl;out << "property list uchar int vertex_indices" << endl;
     out << "element vertex ";out << val << endl;out << "property float x" << endl;out << "property float y" << endl;out << "property float z" << endl;
     out <<  "property uchar diffuse_red"<<endl;out << "property uchar diffuse_green" << endl;out << "property uchar diffuse_blue" << endl;out << "end_header" << endl;
     for (int i = 0; i< points.rows;i++)
     {
        float val1 = points.at<float>(i,0);
        float val2 = points.at<float>(i,1);
        float val3 = points.at<float>(i,2);

        int val4 = Intensity.at<cv::Vec3b>(i,0)[2] ;
        int val5 = Intensity.at<cv::Vec3b>(i,0)[1] ;
        int val6 = Intensity.at<cv::Vec3b>(i,0)[0] ;

        if (dense_uncertainty.at<float>(i,0)> 0.5)
        {
            points_sd_var_aux.at<float>(0,0) = val1;
            points_sd_var_aux.at<float>(0,1) = val2;
            points_sd_var_aux.at<float>(0,2) = val3;
            points_sd_var_aux.at<float>(0,3) = val4;
            points_sd_var_aux.at<float>(0,3) = val5;
            points_sd_var_aux.at<float>(0,5) = val6;
            points_sd_var_aux1.push_back(points_sd_var_aux);

            out << fixed << setprecision(5) << val1<< " " << fixed << setprecision(5) << val2 <<  " " << fixed << setprecision(5) << val3 \
            << " "<< val4 << " "<< val5 << " "<< val6 << endl;
        }
     }
     out.close();
     points_semidense_variational = points_sd_var_aux1.clone();
 }



void transform_points(Imagenes images, int frame, cv::Mat points, cv::Mat &transformed_points)
{
   cv::Mat R =  images.Im[frame]->R;
   cv::Mat t =  images.Im[frame]->t;
   cv::Mat t_repeat =  cv::repeat(t,1,points.rows);

   points = points.colRange(0,3).rowRange(0,points.rows);

   transformed_points = R*points.t() +t_repeat;
}


void distances_interval(Imagenes images, int frame, cv::Mat transformed_points, cv::Mat &projected_points,\
                        float &maximo, float &minimo, float &mean_value, cv::Mat &depth_map,cv::Mat &real_points, int set_maximo,cv::Mat &variance_points_tracked)
{
    float fx =  images.Im[0]->fx;
    float fy =  images.Im[0]->fy;


    float imsize_x =images.Im[0] -> image.cols;
    float imsize_y =images.Im[0] -> image.rows;

    float cx =images.Im[0] -> cx;
    float cy =images.Im[0] -> cy;

    transformed_points = transformed_points.t();

    cv::Mat depths(0,0,CV_32FC1);
    cv::Mat real_points_in_image(0,3,CV_32FC1);

    depth_map = cv::Mat::zeros(imsize_y,imsize_x,CV_32FC1);
    variance_points_tracked = cv::Mat::zeros(imsize_y,imsize_x,CV_32FC1);
    for (int ii = 0 ; ii<=transformed_points.rows-1; ii++ )
    {
             float depth = transformed_points.at<float>(ii,2);

             transformed_points.at<float>(ii,0) /= (depth/fx);
             transformed_points.at<float>(ii,1) /= (depth/fy);


             projected_points.at<float>(ii,0) = (cy + round(transformed_points.at<float>(ii,1)));
             projected_points.at<float>(ii,1) = (-round(transformed_points.at<float>(ii,0)) + cx);
             projected_points.at<float>(ii,2) = 1.0;

             if (projected_points.at<float>(ii,0) > 0 && projected_points.at<float>(ii,0) < imsize_y-1\
                     && projected_points.at<float>(ii,1) > 0 && projected_points.at<float>(ii,1) < imsize_x-1\
                      && depth < 0)
             {
                 depth_map.at<float>(round(projected_points.at<float>(ii,0)),round(projected_points.at<float>(ii,1))) = depth;
                 variance_points_tracked.at<float>(round(projected_points.at<float>(ii,0)),round(projected_points.at<float>(ii,1))) = real_points.at<float>(ii,6);
                 depths.push_back(depth);
                 real_points_in_image.push_back(real_points.row(ii));
             }
    }



    double minVal, maxVal;
    cv::minMaxLoc(abs(depths), &minVal,&maxVal);

    cv::Mat sorted_depths;

    cv::sort(abs(depths),sorted_depths,CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);


    float low_limit = 0.05*depths.rows;
    float high_limit = 0.95*depths.rows;
    low_limit = 0;
    high_limit = depths.rows-1;

    maximo = maxVal;
    minimo = minVal;

    maximo = sorted_depths.at<float>(static_cast<int>(high_limit))*3000;
    minimo = sorted_depths.at<float>(static_cast<int>(low_limit))*0.30;
    //minimo = sorted_depths.at<float>(static_cast<int>(low_limit))*0.070;

    cv::Mat R_ = images.Im[0]->R.clone();
    cv::Mat t_ = images.Im[0]->t.clone();
    cv::Mat C = -R_.t()*t_;
    C = C.t();


    cv::Mat distances(0,1,CV_32FC1);
    for (int ii = 0 ; ii<=real_points_in_image.rows-1; ii = ii + 25 )
    {
              float distance = fabs(real_points_in_image.at<float>(ii,0)-C.at<float>(0,0))+fabs(real_points_in_image.at<float>(ii,1)-C.at<float>(0,1))+fabs(real_points_in_image.at<float>(ii,2)-C.at<float>(0,2));
              distances.push_back(distance);
    }

    mean_value = sorted_depths.at<float>(static_cast<int>(sorted_depths.rows/2));
    mean_value = mean(abs(sorted_depths.colRange(0,sorted_depths.cols).rowRange(static_cast<int>(low_limit),static_cast<int>(high_limit))))[0];

    cv::Mat sorted_distances;
    cv::sort(distances,sorted_distances,CV_SORT_ASCENDING+CV_SORT_EVERY_COLUMN);
    if (set_maximo == 1)
    { mean_value =  sorted_distances.at<float>(round(1*sorted_distances.rows/2),0);}
    else
    { mean_value =  sorted_distances.at<float>(round(1*sorted_distances.rows/10),0);}

}

void get_3Dpoints_inImage(Imagenes &images, cv::Mat &points,cv::Mat &depth_map, int reference_image)
{
    cv::Mat transformed_points;
    transform_points(images,  reference_image,  points, transformed_points);
    cv::Mat projected_points(transformed_points.cols,transformed_points.rows,CV_32FC1);

    float fx =  images.Im[0]->fx;
    float fy =  images.Im[0]->fy;


    float imsize_x =images.Im[0] -> image.cols;
    float imsize_y =images.Im[0] -> image.rows;

    int size_x =images.Im[0] -> image.cols;
    int size_y =images.Im[0] -> image.rows;

    int cx =images.Im[0] -> cx;
    int cy =images.Im[0] -> cy;

    transformed_points = transformed_points.t();


    depth_map = cv::Mat::zeros(size_y,size_x,CV_32FC1);
    //cv::Mat depth_map_show = cv::Mat::zeros(size_y,size_x,CV_32FC1);
    for (int ii = 0 ; ii<=transformed_points.rows-1; ii++ )
        {
            float depth = transformed_points.at<float>(ii,2);

            transformed_points.at<float>(ii,0) /= (depth/fx);
            transformed_points.at<float>(ii,1) /= (depth/fy);


             projected_points.at<float>(ii,0) = (cy + transformed_points.at<float>(ii,1));
             projected_points.at<float>(ii,1) = (-transformed_points.at<float>(ii,0) + cx);
             projected_points.at<float>(ii,2) = 1.0;

         if (projected_points.at<float>(ii,0) > 3 && projected_points.at<float>(ii,0) < imsize_y-3\
                 && projected_points.at<float>(ii,1) > 3 && projected_points.at<float>(ii,1) < imsize_x-3)
         {

             float y_pos =  projected_points.at<float>(ii,0);
             float x_pos =  projected_points.at<float>(ii,1);

             for (int ll = y_pos-1; ll <=y_pos+1;ll++)
             {
                 for (int mm = x_pos-1; mm <= x_pos+1;mm++)
                 {
                     depth_map.at<float>(ll,mm) = ii;
                 }
             }
         }
    }
}
void ransac_for_3Dspx(cv::Mat points_sup,cv::Mat &error_wrt_plane,cv::Mat &n, \
                      float &d, cv::Mat &singular_values, float limit_ratio_sing_val,float average_distance_btw_points, float limit_normalized_residual)
{
    float error_final = INFINITY;

    cv::Mat S,V,D;
    points_sup = points_sup.colRange(0,4);


    cv::Mat n_aux,n_final;
    float d_aux,d_final;
    for (int j = 1; j < 100 ; j++)
    {
        cv::Mat points_sup_aux (0,4,CV_32FC1);

        for (int i = 0; i < 4; i++)
        {
            int random_row = (rand() % points_sup.rows);
            points_sup_aux.push_back(points_sup.row(random_row));
        }


        cv::SVD::compute(points_sup_aux,S,V,D);
        D=D.t();

        D.colRange(3,4) = D.colRange(3,4) / cv::norm(D.colRange(3,4).rowRange(0,3));
        n_aux = D.colRange(3,4).rowRange(0,3);
        d_aux = D.at<float>(3,3);

        cv::Mat plane_parameters(4,1,CV_32FC1);
        plane_parameters.at<float>(0,0) = n_aux.at<float>(0,0);
        plane_parameters.at<float>(1,0) = n_aux.at<float>(1,0);
        plane_parameters.at<float>(2,0) = n_aux.at<float>(2,0);
        plane_parameters.at<float>(3,0) = d_aux;

        float num_inliers = 0;
        error_wrt_plane = abs(points_sup*plane_parameters);
        for (int i = 0; i < error_wrt_plane.rows;i++)
        {
            if (error_wrt_plane.at<float>(i,0)/average_distance_btw_points < (limit_normalized_residual*3)    )
            {
                num_inliers++;
            }
            if (error_wrt_plane.at<float>(i,0)/average_distance_btw_points > (limit_normalized_residual*3)    )
            {
                error_wrt_plane.at<float>(i,0)=(limit_normalized_residual*3);
            }
        }

        if (mean(error_wrt_plane)[0] < error_final && S.at<float>(0,0)/S.at<float>(2,0)<limit_ratio_sing_val\
                && num_inliers/error_wrt_plane.rows > 0.800)
        {
            error_final = mean(error_wrt_plane)[0];
            n_final = n_aux.clone();
            d_final=d_aux;

            singular_values = S.clone();
        }
    }

    n = n_final.clone();
    d = d_final;
}

void backproject_from_plane(Imagenes &images,cv::Mat &pixels,cv::Mat &n1,float &d1,cv::Mat &inv_depth_mat_total,cv::Mat &X_total, int reference_image)
{
    float fx = images.Im[reference_image]->fx;
    float fy = images.Im[reference_image]->fy;
    float cx = images.Im[reference_image]->cx;
    float cy = images.Im[reference_image]->cy;

    float data2[3][3] = {{1/fx,0,0},{0,1/fy,0},{0,0,1}};
    cv::Mat K1_inv =  cv::Mat(3, 3, CV_32FC1, &data2);

    cv::Mat R1,t1;
    R1 = images.Im[reference_image]->R;
    t1 = images.Im[reference_image]->t;

    for (int j = 0; j < pixels.rows; j++)
    {
            int A = pixels.at<float>(j,0);
            int B = pixels.at<float>(j,1);
            X_total.at<float>(j,0) = cx-B;
            X_total.at<float>(j,1) = A-cy;
            X_total.at<float>(j,2) = 1;
    }

    inv_depth_mat_total = (X_total *K1_inv * n1 / d1);



    X_total.colRange(0,1) = X_total.colRange(0,1) / (inv_depth_mat_total*fx);
    X_total.colRange(1,2) = X_total.colRange(1,2) / (inv_depth_mat_total*fy);
    X_total.colRange(2,3) = X_total.colRange(2,3) / (inv_depth_mat_total);


    cv::Mat t_r_ref = cv::repeat(images.Im[reference_image]->t, 1, pixels.rows).clone();

    X_total = R1.t() * (X_total.t() - t_r_ref);


    X_total = X_total.t();
}

cv::Mat create_matrix(cv::Mat contour4, int limit, int imsize_x, int imsize_y)
{

    cv::Mat aux1(1,2, CV_32FC1);
    cv::Mat aux2(0,2, CV_32FC1);
    aux2 = contour4;

    cv::Mat aux(0,2, CV_32FC1);
    cv::Mat aux3(0,2, CV_32FC1);

    cv::Mat m = cv::Mat( cv::Mat::zeros(imsize_y,imsize_x, CV_32F)+limit+1 );

    for (int it = 1; it <=limit ; it++)
    {
        if (it > 1) { aux2 = aux;}
        aux = aux3;
        for (int n = 0; n<= aux2.rows-1; n++)
        {
            if (it == 1){m.at<float>(aux2.at<float>(n,0),aux2.at<float>(n,1)) =0;}

            for (int l = aux2.at<float>(n,0)-1; l<= aux2.at<float>(n,0)+1; l = l+2)
            {
                  if (l>-1 && l < imsize_y  && m.at<float>(l,aux2.at<float>(n,1)) > limit)
                  {
                        m.at<float>(l,aux2.at<float>(n,1)) =it;
                        aux1.at<float>(0,0) =l;
                        aux1.at<float>(0,1) =aux2.at<float>(n,1);
                        aux.push_back(aux1.row(0));
                  }
            }
            for (int l = aux2.at<float>(n,1)-1; l<= aux2.at<float>(n,1)+1; l = l+2)
            {
                if (l>-1 && l < imsize_x  && m.at<float>(aux2.at<float>(n,0),l) > limit)
                {
                        m.at<float>(aux2.at<float>(n,0),l) =it;
                        aux1.at<float>(0,0) =aux2.at<float>(n,0);
                        aux1.at<float>(0,1) =l;
                        aux.push_back(aux1.row(0));
                }
           }
        }
    }
return m;
}



float montecarlo_seed10(Imagenes &images,int a1, int b1, int c1, int d1,DataToSend DTS1 , int iterations, \
                        cv::Mat &contour3D)
{
    DataToSend *DTS = &DTS1;
    float percentage=0;
    float percentage_total=0;

    for(int i = 0; i < iterations ; i++)
    {
        cv::Mat matchings(1,4, CV_32SC1);
        float fi [matchings.rows];
        matchings.at<int>(0,0)=c1;
        matchings.at<int>(0,1)=d1;
        matchings.at<int>(0,2)=a1;
        matchings.at<int>(0,3)=b1;

        for (int rows_num = 0 ; rows_num < matchings.rows; rows_num++)
        {
            fi [rows_num]=0;
        }

        percentage=0;
        percentage_total=0;

        int threshold = 3;

        for (int j = 0; j < matchings.rows;j++)
        {
            percentage=0;
            fi[j] = reprojected_contour (images,*DTS, matchings, j,percentage,threshold,contour3D);
            percentage_total = percentage_total + percentage/matchings.rows;
        }
    }
    return percentage_total;
}


void  calculate_prior(Imagenes images, int frame, cv::Mat transformed_points, cv::Mat &projected_points,cv::Mat &prior,\
                      cv::Mat &lambda, cv::Mat &dense_uncertainty, int &superpixels_number, cv::Mat &planar_prior)
{
    float fx =  images.Im[frame]->fx;
    float fy =  images.Im[frame]->fy;


    float imsize_x =images.Im[0] -> image.cols-1;
    float imsize_y =images.Im[0] -> image.rows-1;
    float cx =images.Im[frame]->cx;
    float cy =images.Im[frame]->cy;

    transformed_points = transformed_points.t();

    cv::Mat num_priors = prior.clone()+1;

    for (int ii = 0 ; ii<=transformed_points.rows-1; ii++ )
    {
         float depth = transformed_points.at<float>(ii,2);

         transformed_points.at<float>(ii,0) /= (depth/fy);
         transformed_points.at<float>(ii,1) /= (depth/fx);


         projected_points.at<float>(ii,0) = (cy + transformed_points.at<float>(ii,1));
         projected_points.at<float>(ii,1) = (-transformed_points.at<float>(ii,0) + cx);
         projected_points.at<float>(ii,2) = 1.0;

         if (projected_points.at<float>(ii,0) > 3 && projected_points.at<float>(ii,0) < imsize_y-4\
                 && projected_points.at<float>(ii,1) > 3 && projected_points.at<float>(ii,1) < imsize_x-4
                 && depth < 0)
         {
             int pos_y = static_cast<int>(projected_points.at<float>(ii,0)+0.5);
             int pos_x = static_cast<int>(projected_points.at<float>(ii,1)+0.5);

             int step_prior = 1;
             for (int nn = pos_y-step_prior; nn<= pos_y+step_prior; nn++)
             {
                 for (int mm = pos_x-step_prior; mm<= pos_x+step_prior; mm++)
                 {
                     dense_uncertainty.at<float>(nn,mm) = 1;

                     if (abs(prior.at<float>(nn,mm)) > 0)
                     {num_priors.at<float>(nn,mm)+=1;}

                     prior.at<float>(nn,mm)+=1/depth;
                     lambda.at<float>(nn,mm)=20;

                     if (ii < superpixels_number)
                     {
                         planar_prior.at<float>(nn,mm)=1;
                     }
                 }
             }
         }
    }
    prior = prior / num_priors;
}




void inverse_depth_optimization( cv::Mat input, cv::Mat initial_inv_depth, photometric_term X, float &depth_step, \
                                 cv::Mat &estimated_inv_depth, cv::Mat &inv_depths, cv::Mat prior, cv::Mat lambda2, \
                                  float lambda_factor, int imsize_y, int imsize_x, float reducir,\
                                 float tetha,float beta,float alpha_limit, int discretization,int superpixels_number, cv::Mat &planar_prior)
{
    if (reducir > 1.001 || reducir < 0.999)
    {resize_data(X,input,estimated_inv_depth,initial_inv_depth,prior,lambda2,reducir,imsize_y,imsize_x,lambda_factor,discretization);}

    int pixels_rows =  prior.rows;
    int pixels_cols =  prior.cols;
    int pixels_number =  pixels_rows* pixels_cols;

    float eps = 0.0001;
    float scale_change = 0;

    scale_change = 0.2 / mean(abs(initial_inv_depth))[0];

    float sigma_d = 0.1;
    float sigma_q = 100;
    float alpha = 0.01000000;

    alpha = 0.015;


    cv::Mat lambda_matrix(pixels_rows,pixels_cols, CV_32FC1);
    lambda_matrix = lambda_matrix.reshape(0,pixels_rows*pixels_cols);


    float lambda = lambda_factor*0.020/5/50;
    lambda_matrix = lambda;


    float comp = 1;
    int iteration = 1;


    cv::Mat q = cv::Mat::zeros(2*pixels_rows*pixels_cols, 1, CV_32F);

    lambda2 =  lambda2.reshape(0,pixels_rows*pixels_cols);
    prior   =  prior.reshape(0,pixels_rows*pixels_cols);
    planar_prior   =    planar_prior.reshape(0,pixels_rows*pixels_cols);



    for (int i = 0; i< prior.rows;i++)
    {
        if (abs(prior.at<float>(i,0)) > 0 && planar_prior.at<float>(i,0) > 0.5)
        {
            estimated_inv_depth.at<float>(i,0) =    prior.at<float>(i,0);
            initial_inv_depth.at<float>(i,0) =    prior.at<float>(i,0);
        }
    }

    prior = - prior * scale_change;

    initial_inv_depth = - initial_inv_depth * scale_change;
    cv::Mat a ;
    initial_inv_depth.copyTo(a);

    inv_depths *= -scale_change;
    initial_inv_depth.copyTo(estimated_inv_depth);

    cv::Mat d;
    estimated_inv_depth.copyTo(d);

    depth_step*=-scale_change;


    cv::Mat channel[3];
    // The actual splitting.

    cv::split(input, channel);

    cv::Mat grad_im;

    grad_im  = gradientX(channel[0],1);
    cv::Mat grad_im_RX;
    cv::pow(grad_im,2,grad_im_RX);

    grad_im = gradientY(channel[0],1);
    cv::Mat grad_im_RY;
    cv::pow(grad_im,2,grad_im_RY);

    grad_im = gradientX(channel[1],1);
    cv::Mat grad_im_GX;
    cv::pow(grad_im,2,grad_im_GX);

    grad_im = gradientY(channel[1],1);
    cv::Mat grad_im_GY;
    cv::pow(grad_im,2,grad_im_GY);

    grad_im = gradientX(channel[2],1);
    cv::Mat grad_im_BX;
    cv::pow(grad_im,2,grad_im_BX);

    grad_im = gradientY(channel[2],1);
    cv::Mat grad_im_BY;
    cv::pow(grad_im,2,grad_im_BY);

    cv::Mat grad_im_x ;
    cv::pow(0.666*grad_im_RX + 0.666*grad_im_GX+ 0.666*grad_im_BX,0.5,grad_im_x);

    cv::Mat grad_im_y ;
    cv::pow(0.666*grad_im_RY + 0.666*grad_im_GY+ 0.666*grad_im_BY,0.5,grad_im_y);

    cv::exp(-grad_im_x*alpha,grad_im_x);
    cv::exp(-grad_im_y*alpha,grad_im_y);

    cv::Mat AGd;
    cv::Mat qx ;
    cv::Mat qy;

    cv::Mat X_ph_error ;
    photometric_term X_inv_ind;

    cv::Mat ph_errors_per_pixel(discretization,1,CV_32FC1);
    for (int i=0; i < pixels_number ;i++)

    {    X_inv_ind.computeError();
        for (int l = 0; l < discretization;l++)
        {
            ph_errors_per_pixel.at<float>(l,0) = X.ph_error[l].at<float>(i,0);
        }
        ph_errors_per_pixel.copyTo(X_inv_ind.ph_error[i]);
    }

    cv::Mat C_max(pixels_number,1,CV_32FC1);
    cv::Mat C_min(pixels_number,1,CV_32FC1);


    for (int i=0; i < pixels_number ;i++)
    {
        float min_C = INFINITY;
        float max_C = -10;
        for (int l = 0; l < discretization;l++)
        {
            if (X.ph_error[l].at<float>(i,0) > max_C)
            {max_C = X.ph_error[l].at<float>(i,0);}
            if (X.ph_error[l].at<float>(i,0) < min_C)
            {min_C = X.ph_error[l].at<float>(i,0);}


        }
        C_max.at<float>(i,0) = max_C;
        C_min.at<float>(i,0) = min_C;
    }

    float inv_depth_l;
    float Eaux[5] ;

    cv::Mat depth_aux(pixels_rows*pixels_cols, 1, CV_32S);
    cv::Mat errores     = cv::Mat::zeros(pixels_rows*pixels_cols, 1, CV_32F) + 1000;

    float update_den;
    float update_num;

    float error;



    while(tetha > alpha_limit)
    {
          d =  d.reshape(0,pixels_rows);

         cv::Mat grad_dx(pixels_rows,pixels_cols, CV_32FC1);
         cv::Mat grad_dy(pixels_rows,pixels_cols, CV_32FC1);



         //#pragma omp parallel for num_threads(2) //
         for (int i=1 ; i < pixels_rows-1; i++)
         {
             centered_gradients_d(grad_dx, grad_dy, grad_im_x,  grad_im_y, d, i,  pixels_cols);
         }

         //#pragma omp parallel for num_threads(2) //
         for(int j=0; j<pixels_cols; j++)
         {
             grad_dy.at<float>(0,j) = (d.at<float>(1,j) - d.at<float>(0,j));
             grad_dy.at<float>(pixels_rows-1,j) = (d.at<float>(pixels_rows-1,j) - d.at<float>(pixels_rows-2,j));

             grad_dy.at<float>(0,j) *= grad_im_y.at<float>(0,j) ;
             grad_dy.at<float>(pixels_rows-1,j) *= grad_im_y.at<float>(pixels_rows-1,j) ;
         }

         //#pragma omp parallel for num_threads(2) //
         for(int i=0; i<pixels_rows; i++)
         {
             grad_dx.at<float>(i,0) = (d.at<float>(i,1) - d.at<float>(i,0));
             grad_dx.at<float>(i,pixels_cols-1) = (d.at<float>(i,pixels_cols-1) - d.at<float>(i,pixels_cols-2));

             grad_dx.at<float>(i,0) *= grad_im_x.at<float>(i,0) ;
             grad_dx.at<float>(i,pixels_cols-1) *= grad_im_x.at<float>(i,pixels_cols-1) ;
         }

         //#pragma omp parallel for num_threads(2) //
         for(int i=1; i<pixels_rows-1; i++)
         {
             grad_dy.at<float>(i,0) = (d.at<float>(i+1,0) - d.at<float>(i-1,0))/2;
             grad_dy.at<float>(i,pixels_cols-1) = (d.at<float>(i+1,pixels_cols-1) - d.at<float>(i-1,pixels_cols-1))/2;

             grad_dy.at<float>(i,0) *= grad_im_y.at<float>(i,0) ;
             grad_dy.at<float>(i,pixels_cols-1) *= grad_im_y.at<float>(i,pixels_cols-1) ;

         }
         //#pragma omp parallel for num_threads(2) //
         for(int j=1; j<pixels_cols-1; j++)
         {
             grad_dx.at<float>(0,j) = (d.at<float>(0,j+1) - d.at<float>(0,j-1))/2;
             grad_dx.at<float>(pixels_rows-1,j) = (d.at<float>(pixels_rows-1,j+1) - d.at<float>(pixels_rows-1,j-1))/2;

             grad_dx.at<float>(0,j) *= grad_im_x.at<float>(0,j) ;
             grad_dx.at<float>(pixels_rows-1,j) *= grad_im_x.at<float>(pixels_rows-1,j) ;
         }

         grad_dx=  grad_dx.reshape(0,pixels_number);
         grad_dy=  grad_dy.reshape(0,pixels_number);

         AGd = grad_dx*1;
         AGd.push_back(grad_dy);


         //#pragma omp parallel for num_threads(2) //
         for (int i=0; i< pixels_number*2;i++)
         {
             q.at<float>(i,0) = (q.at<float>(i,0) + sigma_q * AGd.at<float>(i,0)) / (1 + sigma_q*eps);
             q.at<float>(i,0) /= max(abs(q.at<float>(i,0)),comp );
         }

         qx = q.rowRange(0,pixels_number);
         qy = q.rowRange(  pixels_number ,pixels_number*2);
         qx = (qx.reshape(0,pixels_rows));
         qy = (qy.reshape(0,pixels_rows));


         cv::Mat grad_qx(pixels_rows,pixels_cols, CV_32FC1);
         cv::Mat grad_qy(pixels_rows,pixels_cols, CV_32FC1);
         cv::Mat GAq(pixels_rows,pixels_cols, CV_32FC1);

         //#pragma omp parallel for num_threads(2) //
         for (int i=1 ; i < pixels_rows-1; i++)
         {
             centered_gradients_q(grad_qx, grad_qy, grad_im_x, GAq, grad_im_y, qx, qy,i,  pixels_cols);
         }
         //#pragma omp parallel for num_threads(2) //
         for(int j=0; j<pixels_cols; j++)
         {
             grad_qy.at<float>(0,j) = (qy.at<float>(1,j) - qy.at<float>(0,j));
             grad_qy.at<float>(pixels_rows-1,j) = (qy.at<float>(pixels_rows-1,j) - qy.at<float>(pixels_rows-2,j));

             grad_qy.at<float>(0,j) *= grad_im_y.at<float>(0,j) ;
             grad_qy.at<float>(pixels_rows-1,j) *= grad_im_y.at<float>(pixels_rows-1,j) ;

             GAq.at<float>(0,j) = grad_qy.at<float>(0,j);
             GAq.at<float>(pixels_rows-1,j) = grad_qy.at<float>(pixels_rows-1,j);
         }
         //#pragma omp parallel for num_threads(2) //
         for(int i=0; i<pixels_rows; i++)
         {
             grad_qx.at<float>(i,0) = (qx.at<float>(i,1) - qx.at<float>(i,0));
             grad_qx.at<float>(i,pixels_cols-1) = (qx.at<float>(i,pixels_cols-1) - qx.at<float>(i,pixels_cols-2));

             grad_qx.at<float>(i,0) *= grad_im_x.at<float>(i,0) ;
             grad_qx.at<float>(i,pixels_cols-1) *= grad_im_x.at<float>(i,pixels_cols-1) ;

             GAq.at<float>(i,0) = grad_qx.at<float>(i,0);
             GAq.at<float>(i,pixels_cols-1) = grad_qx.at<float>(i,pixels_cols-1);
         }
        //#pragma omp parallel for num_threads(2) //
         for(int i=1; i<pixels_rows-1; i++)
         {
             grad_qy.at<float>(i,0) = (qy.at<float>(i+1,0) - qy.at<float>(i-1,0))/2;
             grad_qy.at<float>(i,pixels_cols-1) = (qy.at<float>(i+1,pixels_cols-1) - qy.at<float>(i-1,pixels_cols-1))/2;

             grad_qy.at<float>(i,0) *= grad_im_y.at<float>(i,0) ;
             grad_qy.at<float>(i,pixels_cols-1) *= grad_im_y.at<float>(i,pixels_cols-1) ;

             GAq.at<float>(i,0) += grad_qy.at<float>(i,0);
             GAq.at<float>(i,pixels_cols-1) += grad_qy.at<float>(i,pixels_cols-1);
         }
         //#pragma omp parallel for num_threads(2) //
         for(int j=1; j<pixels_cols-1; j++)
         {
             grad_qx.at<float>(0,j) = (qx.at<float>(0,j+1) - qx.at<float>(0,j-1))/2;
             grad_qx.at<float>(pixels_rows-1,j) = (qx.at<float>(pixels_rows-1,j+1) - qx.at<float>(pixels_rows-1,j-1))/2;

             grad_qx.at<float>(0,j) *= grad_im_x.at<float>(0,j) ;
             grad_qx.at<float>(pixels_rows-1,j) *= grad_im_x.at<float>(pixels_rows-1,j) ;

             GAq.at<float>(0,j) += grad_qx.at<float>(0,j);
             GAq.at<float>(pixels_rows-1,j) += grad_qx.at<float>(pixels_rows-1,j);
         }
         GAq = GAq.reshape(0,pixels_number);
         d =  d.reshape(0,pixels_number);


         float robust_prior;


        // //#pragma omp parallel for num_threads(2) //
         for (int i=0; i< pixels_number;i++)
         {
             robust_prior = 1/(1+pow(((d.at<float>(i,0)-prior.at<float>(i,0))/0.01),2));

             d.at<float>(i,0) = (d.at<float>(i,0) + sigma_d*(GAq.at<float>(i,0) + robust_prior*2*prior.at<float>(i,0)*lambda2.at<float>(i,0)\
                                      + (1/tetha)*a.at<float>(i,0)))/(1+sigma_d/tetha+   robust_prior*2*sigma_d*lambda2.at<float>(i,0));
         }

        lambda_matrix =lambda / (1 + 1 / (d));
        //lambda_matrix = lambda;
        errores = cv::Mat::zeros(pixels_rows*pixels_cols, 1, CV_32F) + 1000;

        float Cmax,Cmin,r_u,d_i;
        int ll;
         //#pragma omp parallel for private(X_ph_error,inv_depth_l,error,Cmax,Cmin,r_u,d_i \
         //        ,Eaux,update_num,update_den,ll) num_threads(4) //
         for (int i=0; i < pixels_number; i++)
         {
             X_ph_error = X_inv_ind.ph_error[i];

             Cmax = C_max.at<float>(i,0);
             Cmin = C_min.at<float>(i,0);

             r_u = 2*tetha*lambda_matrix.at<float>(i,0)*(Cmax-Cmin) ;
             d_i = d.at<float>(i,0);
             if (d_i < inv_depths.at<float>(0,0)){d_i = inv_depths.at<float>(0,0);}
             if (d_i > inv_depths.at<float>(discretization-1,0)){d_i = inv_depths.at<float>(discretization-1,0);}


             if (r_u < depth_step ) { r_u = depth_step*1.0;}

             for (int l = 0; l <discretization; l = l+1)
             {

                 //if ( (inv_depths.at<float>(l,0)-d_i)*(inv_depths.at<float>(l,0)-d_i) < r_u)
                 if ( inv_depths.at<float>(l,0) >( d_i-r_u) && inv_depths.at<float>(l,0) <  (d_i+r_u))
                 {
                         inv_depth_l = inv_depths.at<float>(l,0);
                         sampling_a_inv_ind(a,X_ph_error,inv_depth_l,error,tetha,d_i,errores,depth_aux,i,l, lambda_matrix);
                 }
             }

             ll = depth_aux.at<int>(i,0) ;
             if (ll > 1  && ll < discretization-2)
             {
                 for (int j = 0 ; j < 5; j++)
                 {
                     Eaux[j] =   lambda_matrix.at<float>(i,0) * X_ph_error.at<float>(ll-2+j,0) +  \
                             (pow((inv_depths.at<float>(ll-2+j,0) - d_i),2))/(2*tetha);
                 }
                 update_num = (Eaux[3] - Eaux[1])/(depth_step*2);
                 update_den = (Eaux[2]*2 - Eaux[0]- Eaux[4])/(pow(depth_step*2,2));

                 if (abs( update_num / update_den) < depth_step)
                 {a.at<float>(i,0) = a.at<float>(i,0) + update_num / update_den;}
             }
           }
         tetha = tetha * (1-iteration*beta);
         iteration +=1;
    }

    estimated_inv_depth = - d / scale_change;
    initial_inv_depth   = - initial_inv_depth  / scale_change;

    inv_depths = -inv_depths/scale_change;
    depth_step = -depth_step/scale_change;
    prior = - prior / scale_change;
}




void resize_data(photometric_term &X ,cv::Mat &input,\
                 cv::Mat &estimated_inv_depth,cv::Mat &initial_inv_depth,cv::Mat &prior,cv::Mat &lambda,float reducir, int imsize_y,int imsize_x, float &lambda_factor, int discretization)
{
    for (int i = 0;i < discretization;i++)
    {
        X.ph_error[i] = X.ph_error[i].reshape(0,imsize_y);
        cv::resize(X.ph_error[i],X.ph_error[i],cv::Size(X.ph_error[i].cols/reducir,X.ph_error[i].rows/reducir),0,0,cv::INTER_LINEAR);
        X.ph_error[i]=X.ph_error[i].reshape(0,X.ph_error[i].rows*X.ph_error[i].cols);
    }

    cv::resize(input,input,cv::Size(input.cols/reducir,input.rows/reducir),0,0,cv::INTER_LINEAR);
    initial_inv_depth=initial_inv_depth.reshape(0,imsize_y);
    cv::resize(initial_inv_depth,initial_inv_depth,cv::Size(initial_inv_depth.cols/reducir,initial_inv_depth.rows/reducir),0,0,cv::INTER_LINEAR);
    initial_inv_depth=initial_inv_depth.reshape(0,initial_inv_depth.rows*initial_inv_depth.cols);

    estimated_inv_depth=estimated_inv_depth.reshape(0,imsize_y);
    cv::resize(estimated_inv_depth,estimated_inv_depth,cv::Size(estimated_inv_depth.cols/reducir,estimated_inv_depth.rows/reducir),0,0,cv::INTER_LINEAR);
    estimated_inv_depth=estimated_inv_depth.reshape(0,estimated_inv_depth.rows*estimated_inv_depth.cols);

    cv::resize(prior,prior,cv::Size(prior.cols/reducir,prior.rows/reducir),0,0,cv::INTER_LINEAR);
    cv::resize(lambda,lambda,cv::Size(lambda.cols/reducir,lambda.rows/reducir),0,0,cv::INTER_LINEAR);

    lambda_factor /= (reducir);
}


cv::Mat gradientY(cv::Mat &mat, float spacing)
{
    cv::Mat grad = cv::Mat::zeros(mat.rows,mat.cols,CV_32F);

    const int maxCols = mat.cols;
    const int maxRows = mat.rows;

    /*get gradients in each border*/
    /*first row*/
    cv::Mat row = (-mat.row(0) + mat.row(1))/(float)spacing;
    row.copyTo(grad(cvRect(0,0,maxCols,1)));

    /*last row*/
    row = (-mat.row(maxRows-2) + mat.row(maxRows-1))/(float)spacing;
    row.copyTo(grad(cvRect(0,maxRows-1,maxCols,1)));

    /*centered elements*/
    cv::Mat centeredMat = mat(cvRect(0,0,maxCols,maxRows-2));
    cv::Mat offsetMat = mat(cvRect(0,2,maxCols,maxRows-2));
    cv::Mat resultCenteredMat = (-centeredMat + offsetMat)/(((float)spacing)*2.0);

    resultCenteredMat.copyTo(grad(cvRect(0,1,maxCols, maxRows-2)));
    return grad;
}

cv::Mat gradientX(cv::Mat & mat, float spacing)
{
    cv::Mat grad = cv::Mat::zeros(mat.rows,mat.cols,CV_32F);

    int maxCols = mat.cols;
    int maxRows = mat.rows;

    /* get gradients in each border */
    /* first col */
    cv::Mat col = (-mat.col(0) + mat.col(1))/(float)spacing;
    col.copyTo(grad(cvRect(0,0,1,maxRows)));

    /*  last col */
    col = (-mat.col(maxCols-2) + mat.col(maxCols-1))/(float)spacing;
    col.copyTo(grad(cvRect(maxCols-1,0,1,maxRows)));

    /* centered elements */
    cv::Mat centeredMat = mat(cvRect(0,0,maxCols-2,maxRows));
    cv::Mat offsetMat = mat(cvRect(2,0,maxCols-2,maxRows));
    cv::Mat resultCenteredMat = (-centeredMat + offsetMat)/(((float)spacing)*2.0);

    resultCenteredMat.copyTo(grad(cvRect(1,0,maxCols-2, maxRows)));
    return grad;
}


inline void centered_gradients_d(cv::Mat &grad_dx, cv::Mat &grad_dy, cv::Mat &grad_im_x, cv::Mat &grad_im_y, cv::Mat &d, int &i, int &pixels_cols)
{
    for(int j=1; j < pixels_cols-1; j++)
    {
        grad_dx.at<float>(i,j) = ((d.at<float>(i,j+1) - d.at<float>(i,j-1))/2.0);
        grad_dy.at<float>(i,j) = ((d.at<float>(i+1,j) - d.at<float>(i-1,j))/2.0);

        grad_dx.at<float>(i,j) *= grad_im_x.at<float>(i,j) ;
        grad_dy.at<float>(i,j) *= grad_im_y.at<float>(i,j) ;
    }
}

void centered_gradients_q(cv::Mat &grad_qx, cv::Mat &grad_qy, cv::Mat &grad_im_x, \
                          cv::Mat &GAq, cv::Mat &grad_im_y, cv::Mat &qx, cv::Mat &qy, int &i, int &pixels_cols)
{
    for(int j=1; j < pixels_cols-1; j++)
    {
        grad_qx.at<float>(i,j) = ((qx.at<float>(i,j+1) - qx.at<float>(i,j-1))/2.0);
        grad_qy.at<float>(i,j) = ((qy.at<float>(i+1,j) - qy.at<float>(i-1,j))/2.0);

        grad_qx.at<float>(i,j) *= grad_im_x.at<float>(i,j) ;
        grad_qy.at<float>(i,j) *= grad_im_y.at<float>(i,j) ;

        GAq.at<float>(i,j) = grad_qx.at<float>(i,j) + grad_qy.at<float>(i,j);
    }
}

inline void sampling_a_inv_ind(cv::Mat &a,cv::Mat &X_ph_error,float &inv_depth_l,\
                float &error,float &tetha,float &d_i, cv::Mat &errores,cv::Mat &depth_aux,\
                       const int &i,int &l, cv::Mat &lambda_matrix)

{
    float v = inv_depth_l-d_i;
    v *= v;
    error = lambda_matrix.at<float>(i,0) * X_ph_error.at<float>(l,0) +  1/(2*tetha) * v;

    if (errores.at<float>(i,0) > error)
    {
        errores.at<float>(i,0) = error;
        a.at<float>(i,0)       = inv_depth_l;
        depth_aux.at<int>(i,0) = l;
    }
}

float reprojected_contour (Imagenes &images,DataToSend &DTS, cv::Mat matchings, int i,float &percentage,\
                           float threshold, cv::Mat &contour3D)
{
     cv::Mat contour1_2;
     cv::Mat R1;
     cv::Mat t1 ;
     int reference_image = matchings.at<int>(i,0);

     double fx= images.Im[reference_image]-> fx;
     double fy= images.Im[reference_image]->fy;
     double cx= images.Im[reference_image]->cx;
     double cy= images.Im[reference_image]->cy;
     cv::Mat img = images.Im[reference_image]->image.clone();


     cv::Mat m;
     m = DTS.SupImg[reference_image] -> getSuperpixeles()[matchings.at<int>(i,1)]-> getMatrix();

     R1  = images.Im[reference_image]-> R;
     t1  = images.Im[reference_image]-> t;
     fx  = images.Im[reference_image]-> fx;
     fy  = images.Im[reference_image]-> fy;

     cv::Mat points3D_cam;

     transformed_points (points3D_cam, R1,t1,fx, fy, cx, cy, img, contour3D);
     points3D_cam= points3D_cam.t();
     contour1_2 = points3D_cam.clone();

     float imsize_x =DTS.SupImg[reference_image] -> image.cols;
     float imsize_y =DTS.SupImg[reference_image] -> image.rows;

     float distance, distance_total = 0, distance_opt = INFINITY;

     distance = 0;
     distance_total = 0;
     float percentage2=0;

     float d_x1_y,d_x3_y;
     float x_1,x_2,x_3,y_1,y_2,y_3;
     for (int k = 0 ; k<=contour1_2.rows-1; k++)
     {
            distance_opt = threshold;
            if (contour1_2.at<float>(k,0) > 1 && contour1_2.at<float>(k,0) < (imsize_x-2)\
                    && contour1_2.at<float>(k,1) > 1 && contour1_2.at<float>(k,1) < (imsize_y-2))
            {
                //BILINEAR INTERPOLATION
                y_2 = contour1_2.at<float>(k,1);
                y_1 = static_cast<int>(y_2);
                y_3 = y_1 +1;
                x_2 = contour1_2.at<float>(k,0);
                x_1 = static_cast<int>(x_2);
                x_3 = x_1 +1;

                d_x1_y = (y_2-y_1) * m.at<float>(y_1,x_1) + (y_3-y_2) * m.at<float>(y_3,x_1);
                d_x3_y = (y_2-y_1) * m.at<float>(y_1,x_3) + (y_3-y_2) * m.at<float>(y_3,x_3);

                distance_opt = (x_2-x_1) * d_x1_y + (x_3-x_2) *  d_x3_y;
                /////////
                //if (distance_opt > threshold){distance_opt = threshold;}
            }
            //distances.at<float>(k,0)=distance_opt;
            distance_total = distance_total + 1.0*distance_opt / contour1_2.rows;
            percentage2 = percentage2 + (distance_opt < 3) ;
     }
     percentage2 = percentage2 / contour1_2.rows;
     percentage  = percentage2;
     return distance_total;
}


void transformed_points(cv::Mat &points3D_cam, cv::Mat &R,cv::Mat &t,double fx,double fy,double cx,double cy,cv::Mat &img, cv::Mat &points3D)
{
    cv::Mat t_r =  cv::repeat(t,1,points3D.cols);
    points3D_cam = R*points3D  + t_r;

    points3D_cam.colRange(0,points3D_cam.cols).rowRange(0,1) = -points3D_cam.colRange(0,points3D_cam.cols).rowRange(0,1) * fx;
    points3D_cam.colRange(0,points3D_cam.cols).rowRange(1,2) =  points3D_cam.colRange(0,points3D_cam.cols).rowRange(1,2) * fy;

    cv::divide(points3D_cam.colRange(0,points3D_cam.cols).rowRange(0,1), points3D_cam.colRange(0,points3D_cam.cols).rowRange(2,3), points3D_cam.colRange(0,points3D_cam.cols).rowRange(0,1),1);
    cv::divide(points3D_cam.colRange(0,points3D_cam.cols).rowRange(1,2), points3D_cam.colRange(0,points3D_cam.cols).rowRange(2,3), points3D_cam.colRange(0,points3D_cam.cols).rowRange(1,2),1);
    cv::divide(points3D_cam.colRange(0,points3D_cam.cols).rowRange(2,3), points3D_cam.colRange(0,points3D_cam.cols).rowRange(2,3), points3D_cam.colRange(0,points3D_cam.cols).rowRange(2,3),1);

    points3D_cam.colRange(0,points3D_cam.cols).rowRange(0,1) =  cx + points3D_cam.colRange(0,points3D_cam.cols).rowRange(0,1) ;
    points3D_cam.colRange(0,points3D_cam.cols).rowRange(1,2) =  points3D_cam.colRange(0,points3D_cam.cols).rowRange(1,2) + cy;
}

void transformed_points_return_3Dpoints(cv::Mat &points3D_cam, cv::Mat &R,cv::Mat &t,double fx,double fy,double cx,double cy,cv::Mat &img, cv::Mat &points3D, cv::Mat &transformed_points)
{
    cv::Mat t_r =  cv::repeat(t,1,points3D.cols);
    points3D_cam = R*points3D  + t_r;

    transformed_points = points3D_cam.clone();

    points3D_cam.colRange(0,points3D_cam.cols).rowRange(0,1) = -points3D_cam.colRange(0,points3D_cam.cols).rowRange(0,1) * fx;
    points3D_cam.colRange(0,points3D_cam.cols).rowRange(1,2) = points3D_cam.colRange(0,points3D_cam.cols).rowRange(1,2) * fy;

    cv::divide(points3D_cam.colRange(0,points3D_cam.cols).rowRange(0,1), points3D_cam.colRange(0,points3D_cam.cols).rowRange(2,3), points3D_cam.colRange(0,points3D_cam.cols).rowRange(0,1),1);
    cv::divide(points3D_cam.colRange(0,points3D_cam.cols).rowRange(1,2), points3D_cam.colRange(0,points3D_cam.cols).rowRange(2,3), points3D_cam.colRange(0,points3D_cam.cols).rowRange(1,2),1);
    cv::divide(points3D_cam.colRange(0,points3D_cam.cols).rowRange(2,3), points3D_cam.colRange(0,points3D_cam.cols).rowRange(2,3), points3D_cam.colRange(0,points3D_cam.cols).rowRange(2,3),1);

    points3D_cam.colRange(0,points3D_cam.cols).rowRange(0,1) =  cx + points3D_cam.colRange(0,points3D_cam.cols).rowRange(0,1) ;
    points3D_cam.colRange(0,points3D_cam.cols).rowRange(1,2) =  points3D_cam.colRange(0,points3D_cam.cols).rowRange(1,2) + cy;
}
