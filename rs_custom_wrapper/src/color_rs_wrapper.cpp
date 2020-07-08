///////////////////////////////////////////////////////////////////////////////
// Function Name : depth
// Purpose : ROS publisher for depth, infra1, and infra2 images
///////////////////////////////////////////////////////////////////////////////
// Changelog :
// Date           % Name       %   Reason
// 03 / 04 / 2019 % Gene Rush  %   Created code
// 03 / 06 / 2019 % Gene Rush  %   Added this comment block
///////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <iostream>
#include <cv_bridge/cv_bridge.h> // Bridge between OpenCV and ROS
#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
//#include "color_rs_wrapper/color_rs_wrapper.h"

// https://stackoverflow.com/questions/19331575/cout-and-endl-errors
using std::cout;
using std::endl;

int main(int argc, char * argv[]) try
{
    ros::init(argc, argv, "color_rs_node");
    ros::NodeHandle nh_("~"); // the "~" is required to get parameters

    ////////////////////
    // GET PARAMETERS //
    ////////////////////

    bool IMSHOW_COLOR = 0;
    bool COLOR_FLAG = 0;
    int COLOR_WIDTH = 848;
    int COLOR_HEIGHT = 480;
    int COLOR_FPS = 30;

    nh_.param("IMSHOW_COLOR", IMSHOW_COLOR, IMSHOW_COLOR);
    nh_.param("COLOR_FLAG", COLOR_FLAG, COLOR_FLAG);
    nh_.param("COLOR_WIDTH", COLOR_WIDTH, COLOR_WIDTH);
    nh_.param("COLOR_HEIGTH", COLOR_HEIGHT, COLOR_HEIGHT);
    nh_.param("COLOR_FPS", COLOR_FPS, COLOR_FPS);

    //////////////////////////
    // INITIALIZE REALSENSE //
    //////////////////////////

    // Declare first loop flag
    bool first_loop_flag = 1;

    // Declare counter
    unsigned int cnt = 0;

    // Declare depth colorizer for pretty visualization of depth frame
    rs2::colorizer color_map;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    /////////////////////////////////
    // SET REALSENSE STREAM PARAMS //
    /////////////////////////////////

   // Configured color stream
    if (COLOR_FLAG)
    {
        cfg.enable_stream(RS2_STREAM_COLOR, COLOR_WIDTH, COLOR_HEIGHT, RS2_FORMAT_BGR8, COLOR_FPS);
    }
    
    // Instruct pipeline to start streaming with the requested configuration
    cout << "Starting RealSense (COLOR)..." << endl;
    rs2::pipeline_profile profile = pipe.start(cfg);

    /////////////////////////////////
    // GET REALSENSE DEPTH SCALING //
    /////////////////////////////////

    // Find device
    // https://github.com/IntelRealSense/librealsense/wiki/API-How-To
    auto dev = profile.get_device();

    
    ////////////////////////////////////////////
    // CONFIGURE IMSHOW VISUALIZATION WINDOWS //
    ////////////////////////////////////////////
        
    using namespace cv;
    const auto window_name_color = "Color";

    if (IMSHOW_COLOR)
    {
        namedWindow(window_name_color, WINDOW_AUTOSIZE);
    }

    //////////////////////////////
    // CONFIGURE ROS PUBLISHERS //
    //////////////////////////////

    // Publish sensor_msgs::Image
    // https://answers.ros.org/question/99831/publish-file-to-image-topic/

    // Setup color publisher
    ros::Publisher pub_image_color = nh_.advertise<sensor_msgs::Image>("/camera/color/image_rect_raw", 1);

    cv::Mat mat_color( Size( COLOR_WIDTH , COLOR_HEIGHT ) , CV_8UC3 ); 
    
    // ~~ Declare vars ~~
    // Bridge
    cv_bridge::CvImage cv_image_color;
    cv_image_color.encoding = "bgr8";
    // Image Msg
    sensor_msgs::Image ros_image_color;
    bool INDIRECT = false; // Why 'mat_color' as an intermediary? 'cv_image_color.image' is already 'cv::Mat'
    
    while (nh_.ok() && waitKey(1) < 0) 
    {
        cnt++;

        //////////////////////////////////
        // PRINT PARAMETERS TO TERMINAL //
        //////////////////////////////////

        if(first_loop_flag == 1)
        {
            cout << "Collecting data (COLOR)..." << endl;
            first_loop_flag = 0;
        }

        //////////////////////////////
        // WAIT FOR REALSENSE FRAME //
        //////////////////////////////
        
        // Wait for next set of frames from the camera
        rs2::frameset frame_set = pipe.wait_for_frames();

        if(COLOR_FLAG)
        {
            rs2::frame frame_color = frame_set.get_color_frame();

            if( INDIRECT ){
                //////////////////////////////////////////////
                // SEND FRAME DATA TO 8-BIT OPENCV MATRICES //
                //////////////////////////////////////////////
                mat_color = cv::Mat( Size( COLOR_WIDTH , COLOR_HEIGHT ) , CV_8UC3 , (void*)frame_color.get_data() , Mat::AUTO_STEP );

                //////////////////////////////////////////
                // CONVERT CV:MAT to SENSOR_MSGS::IMAGE //
                //////////////////////////////////////////
                cv_image_color.image = mat_color; // ----------- Save the frame from the realsense
            }else{
                cv_image_color.image = cv::Mat( Size( COLOR_WIDTH , COLOR_HEIGHT ) , CV_8UC3 , (void*)frame_color.get_data() , Mat::AUTO_STEP ); // Save the frame from the realsense
            }
            cv_image_color.toImageMsg( ros_image_color ); // Convert to the ROS format and load into 'ros_image_color'
            

            if(IMSHOW_COLOR)
            {
                /////////////////////////////////////////
                // UPDATE IMSHOW VISUALIZATION WINDOWS //
                /////////////////////////////////////////
                cv::imshow( window_name_color , mat_color );
            }

            ////////////////////////////////////////
            // PUBLISH ROS MESSAGES TO ROS TOPICS //
            ////////////////////////////////////////
            pub_image_color.publish( ros_image_color );
        }
        
		char c = cv::waitKey(1);
		if (c == 's')
		{

		}
		else if (c == 'q')
		{
            break;
        }
    }
      
    return EXIT_SUCCESS;
}
/////////////////
// END OF MAIN //
/////////////////

catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}

catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
