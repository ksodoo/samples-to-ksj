#include </usr/include/opencv4/opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <librealsense2/rs.hpp>
#include "cv-helpers.hpp"
#include <ctime>
#include <iostream>
#include <vector>
using namespace cv;
using namespace rs2;
using namespace std;

int main()
{   
    bool is_disparity = false;
    rs2::config cfg;
    pipeline pipe;
    cfg.enable_stream(RS2_STREAM_COLOR, 848, 480, RS2_FORMAT_RGB8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 30);
    auto profile = pipe.start(cfg);
    rs2_intrinsics intrinsics = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
    rs2::align align_to(RS2_STREAM_COLOR);
    rs2::colorizer color_filter;
    clock_t prev_time  = 0/CLOCKS_PER_SEC;
    int frame100 = 0;
    float framesum = 0;
  
    while (true)
    {   
        frameset data = pipe.wait_for_frames();
        data = align_to.process(data);
        video_frame cframe = data.get_color_frame(); 
        depth_frame dframe = data.get_depth_frame();
        frame filtered;
        filtered = color_filter.process(dframe);
        Mat depth_mat, color_mat, gray_mat, blurred_mat ,hsv_mat ,mask,eroded, dilated ;
        color_mat = frame_to_mat(cframe);
        depth_mat = frame_to_mat(filtered);
        GaussianBlur(color_mat, blurred_mat, Size(11, 11), cv::BORDER_DEFAULT);
        cvtColor(blurred_mat, hsv_mat, COLOR_BGR2HSV);
        inRange(hsv_mat,
            Scalar(6, 77, 158),  
            Scalar(255, 255, 255),
            mask); 
        Mat element = getStructuringElement(0, Size( 3, 3), Point( 0, 0 ));
        erode( mask, eroded, element );
        dilate(eroded, dilated, element);
        vector<vector<Point> > contours;
        findContours( dilated, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        float distance;
        int chosenx;
        int choseny;
        if (contours.size()>0)
        {
            Point2f center;
            float radius = 0;
            for (int i = 0; i < contours.size(); i++)
            {
                minEnclosingCircle(contours[i], center, radius);
            }
            Moments m = moments(dilated, true);
            chosenx = int(m.m10/m.m00);
            choseny = int(m.m01/m.m00);
            distance = dframe.get_distance(chosenx, choseny)*100;
        }
        Mat drawing = Mat::zeros(dilated.size(), CV_8UC3 );
        

        // float distance = dframe.get_distance(chosen[0], chosen[1])*100;
        // // assert(intrinsics.model != RS2_DISTORTION_MODIFIED_BROWN_CONRADY);
        // // assert(intrinsics.model != RS2_DISTORTION_FTHETA);

        // // if(intrinsics.model == RS2_DISTORTION_INVERSE_BROWN_CONRADY)
        // // {
        // //     float r2  = x*x + y*y;
        // //     float f = 1 + intrinsics.coeffs[0]*r2 + intrinsics.coeffs[1]*r2*r2 + intrinsics.coeffs[4]*r2*r2*r2;
        // //     float ux = x*f + 2*intrinsics.coeffs[2]*x*y + intrinsics.coeffs[3]*(r2 + 2*x*x);
        // //     float uy = y*f + 2*intrinsics.coeffs[3]*x*y + intrinsics.coeffs[2]*(r2 + 2*y*y);
        // //     x = ux;
        // //     y = uy;
        // // }
        // // auto point1 = distance * x;
        // // auto point2= distance * y;
        // // auto point3 = distance ;
        float xaxis = distance*(chosenx-intrinsics.ppx)/intrinsics.fx;
        float yaxis = distance*(choseny-intrinsics.ppy)/intrinsics.fy;
        float zaxis = distance;


        std::clock_t start;
        start = std::clock();
        double elapsed_secs = (start - prev_time)/(double) CLOCKS_PER_SEC;
        prev_time = start;
        float fps = float(1/elapsed_secs);
        
        printf("\n x: %.2f  y: %.2f z: %.2f ", xaxis, yaxis, zaxis);
        printf("\n px: %i, py: %i", chosenx, choseny);
        printf("\n %f", fps);
        
        if (waitKey(1) >= 0)
        {
            break;
        }
    }

// rs2::depth_frame dpt_frame = frame.as<rs2::depth_frame>();
// float pixel_distance_in_meters = dpt_frame.get_distance(x,y);

}