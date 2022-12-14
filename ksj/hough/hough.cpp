#include <opencv2/opencv.hpp>
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
    rs2::config cfg;
    pipeline pipe;
    cfg.enable_stream(RS2_STREAM_COLOR, 848, 480, RS2_FORMAT_RGB8, 60);
    cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 60);
    auto profile = pipe.start(cfg);
    rs2_intrinsics intrinsics = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
    rs2::align align_to(RS2_STREAM_COLOR);
    clock_t prev_time  = 0/CLOCKS_PER_SEC;
    // int frame100 = 0;
    // float framesum = 0;
  
    while (true)
    {   
        frameset data = pipe.wait_for_frames();
        data = align_to.process(data);
        video_frame cframe = data.get_color_frame(); 
        depth_frame dframe = data.get_depth_frame();
        Mat depth_mat; 
        Mat color_mat;
        color_mat = frame_to_mat(cframe);
        depth_mat = frame_to_mat(dframe);
        Mat gray_mat;
        Mat blurred_mat;
        cvtColor(color_mat, gray_mat, COLOR_BGR2GRAY);
        GaussianBlur(gray_mat, blurred_mat, Size(15, 15), cv::BORDER_DEFAULT);
        vector<Vec3f> circles;
        Vec3i chosen;
        HoughCircles(blurred_mat, circles, HOUGH_GRADIENT, 1, 1000 , 100, 15, 5 , 100);
        for( size_t i = 0; i < circles.size(); i++ )
        {
            chosen = circles[i];
            Point center = Point(chosen[0], chosen[1]);
            // circle center
            circle( color_mat, center, 1, Scalar(0,100,100), 3);
            // circle outline
            int radius = chosen[2];
            circle( color_mat, center, radius, Scalar(255,0,255), 3);
        }
        float distance = dframe.get_distance(chosen[0], chosen[1])*100;
        // assert(intrinsics.model != RS2_DISTORTION_MODIFIED_BROWN_CONRADY);
        // assert(intrinsics.model != RS2_DISTORTION_FTHETA);

        // if(intrinsics.model == RS2_DISTORTION_INVERSE_BROWN_CONRADY)
        // {
        //     float r2  = x*x + y*y;
        //     float f = 1 + intrinsics.coeffs[0]*r2 + intrinsics.coeffs[1]*r2*r2 + intrinsics.coeffs[4]*r2*r2*r2;
        //     float ux = x*f + 2*intrinsics.coeffs[2]*x*y + intrinsics.coeffs[3]*(r2 + 2*x*x);
        //     float uy = y*f + 2*intrinsics.coeffs[3]*x*y + intrinsics.coeffs[2]*(r2 + 2*y*y);
        //     x = ux;
        //     y = uy;
        // }
        // auto point1 = distance * x;
        // auto point2= distance * y;
        // auto point3 = distance ;
        float xaxis = distance*(chosen[0]-intrinsics.ppx)/intrinsics.fx;
        float yaxis = distance*(chosen[1]-intrinsics.ppy)/intrinsics.fy;
        float zaxis = distance;

        // imshow("Hough detector", color_mat);
        std::clock_t start;
        start = std::clock();
        double elapsed_secs = (start - prev_time)/(double) CLOCKS_PER_SEC;
        prev_time = start;
        float fps = float(1/elapsed_secs);
        
        printf("\n x: %.2f  y: %.2f z: %.2f ", xaxis, yaxis, zaxis);
        printf("\n px: %i, py: %i", chosen[0],chosen[1]);
        printf("\n %f", fps);
            // printf("\n%d", frame100);
            // printf("%f", framesum/100);
        
        if (waitKey(1) >= 0)
        {
            break;
        }
    }

// rs2::depth_frame dpt_frame = frame.as<rs2::depth_frame>();
// float pixel_distance_in_meters = dpt_frame.get_distance(x,y);

}