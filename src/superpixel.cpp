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

#include <dpptam/superpixel.h>

void superpixel::set_matchings (int a, int b,int c, int d)
{
    cv::Mat mn(1,4, CV_32SC1);
    mn.at<int>(0,0) = a;
    mn.at<int>(0,1) = b;
    mn.at<int>(0,2) = c;
    mn.at<int>(0,3) = d;

    matchings.push_back(mn.row(0));
}

void superpixel::set_contour (float a, float b, float c)
{
    cv::Mat mn(1,3, CV_32FC1);
    mn.at<float>(0,0) = a;
    mn.at<float>(0,1) = b;
    mn.at<float>(0,2) = c;

    contour.push_back(mn.row(0));
}

void superpixel::set_plane (float a, float b, float c)
{
    cv::Mat mn(1,3, CV_32FC1);
    mn.at<float>(0,0) = a;
    mn.at<float>(0,1) = b;
    mn.at<float>(0,2) = c;
    plane.push_back(mn.row(0));
}

void superpixel::set_pixels (float a, float b,float c)
{
    cv::Mat mn(1,3, CV_32FC1);
    mn.at<float>(0,0) = a;
    mn.at<float>(0,1) = b;
    mn.at<float>(0,2) = c;
    pixels.push_back(mn.row(0));
}

int SuperpixelesImagen::computeSuperpixels()
{

    cv::Mat image_sup;
    char command[100];

    if(! image.data )
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }

    int rows = image.rows;
    int cols = image.cols;

    image_sup = cv::imread(sup_name);

    cv::Vec3b intensity = image_sup.at<cv::Vec3b>(10,10);
    cv::Mat m(1,3,CV_32SC1);
    cv::Mat B(0,3, CV_32SC1);

    float count_r,count_c;
    int step_size = 4;

    int suma1,suma2,suma3,suma4;


    for ( count_r = 1; count_r <=  rows-2; count_r=count_r+step_size)
    {
        for ( count_c = 1; count_c <=  cols-2; count_c=count_c+step_size)
        {
            intensity = image_sup.at<cv::Vec3b>(count_r,count_c);
            m.at<int>(0,0) = intensity.val[0];
            m.at<int>(0,1) = intensity.val[1];
            m.at<int>(0,2) = intensity.val[2];
            B.push_back(m.row(0));
        }
    }

    cv::Mat C(0,3, CV_32SC1);
    int i,j,Crows;
    for ( i = 0; i <=  B.rows-1; i=i+1){

        bool identical = false;
        Crows = C.rows;

        for (j=0; j<= Crows-1; j=j+1){
            if (B.at<int>(i,0)==C.at<int>(j,0) && B.at<int>(i,1)==C.at<int>(j,1) && B.at<int>(i,2)==C.at<int>(j,2))
            {identical = true;}
        }
        if ( identical == false)
        {C.push_back(B.row(i));}
    };


    step_size = 1;

    superpixel *spx;

    for (i=0; i<=C.rows-1 ; i+=1)
    {
       spx = new superpixel();
       sup.push_back(spx);
    }



    for (i=0; i<=C.rows-1; i+=1)
    {
        sup[i]->set_color(C.at<int>(i,0),C.at<int>(i,1) ,C.at<int>(i,2));
    }


    for ( count_r = 1; count_r <=  rows-2; count_r=count_r+step_size)
    {
        for ( count_c = 1; count_c <=  cols-2; count_c=count_c+step_size)
        {
            int image_sup_R = image_sup.at<cv::Vec3b>(count_r,count_c)[0];
            int image_sup_G = image_sup.at<cv::Vec3b>(count_r,count_c)[1];
            int image_sup_B = image_sup.at<cv::Vec3b>(count_r,count_c)[2];

            suma1 = image_sup.at<cv::Vec3b>(count_r+1,count_c)[0]*1 + image_sup.at<cv::Vec3b>(count_r+1,count_c)[1]*255 + image_sup.at<cv::Vec3b>(count_r+1,count_c)[2]*255^2;
            suma2 = image_sup.at<cv::Vec3b>(count_r-1,count_c)[0]*1 + image_sup.at<cv::Vec3b>(count_r-1,count_c)[1]*255 + image_sup.at<cv::Vec3b>(count_r-1,count_c)[2]*255^2;
            suma3 = image_sup.at<cv::Vec3b>(count_r,count_c-1)[0]*1 + image_sup.at<cv::Vec3b>(count_r,count_c-1)[1]*255 + image_sup.at<cv::Vec3b>(count_r,count_c-1)[2]*255^2;
            suma4 = image_sup.at<cv::Vec3b>(count_r,count_c+1)[0]*1 + image_sup.at<cv::Vec3b>(count_r,count_c+1)[1]*255 + image_sup.at<cv::Vec3b>(count_r,count_c+1)[2]*255^2;

            bool is_contour = false;
            if (suma1-suma2 != 0 || suma3-suma4!=0){is_contour = true;}

            int exit = 0;
            for (i=0; i < C.rows; i=i+1)
            {
                if (exit == 0 && C.at<int>(i,0) == image_sup_R  && C.at<int>(i,1) == image_sup_G   && C.at<int>(i,2) == image_sup_B  )
                 {
                    sup[i]->incSize();
                    sup[i]->set_pixels (count_r, count_c,image.at<cv::Vec3b>(count_r,count_c)[0]);
                    exit = 1;

                     if (is_contour)
                     {
                         sup[i]->set_contour (count_r,count_c,1);
                         exit = 1;
                     }
                }
            }

        }
    }

    return C.rows;
}

void Imagenes::computeImageSup()
{
    SuperpixelesImagen *spxIm;
    spxIm = new SuperpixelesImagen();
    supIm.push_back(spxIm);
}

void Imagenes::computeImage()
{
    Imagen *spxIm;
    spxIm = new Imagen();
    Im.push_back(spxIm);
}

void photometric_term::computeError()
{
    cv::Mat error;
    ph_error.push_back(error);
}
