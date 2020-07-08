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
#include "rs_custom_wrapper/rs_custom_wrapper.h"
#include <sensor_msgs/Imu.h>

// https://stackoverflow.com/questions/19331575/cout-and-endl-errors
using std::cout;
using std::endl;

int main(int argc, char * argv[]) try
{
    ros::init(argc, argv, "rs_custom_node");
    ros::NodeHandle nh_("~"); // the "~" is required to get parameters

    ////////////////////
    // GET PARAMETERS //
    ////////////////////

    bool IMSHOW_DEPTH = 0;
    bool IMSHOW_INFRARED = 0;
    bool DEPTH_RGB_FLAG = 0;
    bool DEPTH_METERS_FLAG = 1;
    bool LASER_FLAG = 0;

    bool DEPTH_FLAG = 0;
    int DEPTH_WIDTH = 848;
    int DEPTH_HEIGHT = 480;
    int DEPTH_FPS = 30;

    bool INFRARED_FLAG = 0;
    int INFRARED_WIDTH = 848;
    int INFRARED_HEIGHT = 480;
    int INFRARED_FPS = 30;

    bool IMU_FLAG = 0;

    nh_.param("IMSHOW_DEPTH", IMSHOW_DEPTH, IMSHOW_DEPTH);
    nh_.param("IMSHOW_INFRARED", IMSHOW_INFRARED, IMSHOW_INFRARED);
    //nh_.param("IMSHOW_COLOR", IMSHOW_COLOR, IMSHOW_COLOR);
    nh_.param("DEPTH_RGB_FLAG", DEPTH_RGB_FLAG, DEPTH_RGB_FLAG);
    nh_.param("DEPTH_METERS_FLAG", DEPTH_METERS_FLAG, DEPTH_METERS_FLAG);
    nh_.param("LASER_FLAG", LASER_FLAG, LASER_FLAG);
    
    nh_.param("DEPTH_FLAG", DEPTH_FLAG, DEPTH_FLAG);
    nh_.param("DEPTH_WIDTH", DEPTH_WIDTH, DEPTH_WIDTH);
    nh_.param("DEPTH_HEIGHT", DEPTH_HEIGHT, DEPTH_HEIGHT);
    nh_.param("DEPTH_FPS", DEPTH_FPS, DEPTH_FPS);

    nh_.param("INFRARED_FLAG", INFRARED_FLAG, INFRARED_FLAG);
    nh_.param("INFRARED_WIDTH", INFRARED_WIDTH, INFRARED_WIDTH);
    nh_.param("INFRARED_HEIGHT", INFRARED_HEIGHT, INFRARED_HEIGHT);
    nh_.param("INFRARED_FPS", INFRARED_FPS, INFRARED_FPS);

    nh_.param("IMU_FLAG", IMU_FLAG, IMU_FLAG);

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

    if (DEPTH_FLAG)
    {
        // Configured depth stream
        cfg.enable_stream(RS2_STREAM_DEPTH, DEPTH_WIDTH, DEPTH_HEIGHT, RS2_FORMAT_Z16, DEPTH_FPS);
    }

    // Configured IMU stream
    if (IMU_FLAG)
    {
        cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
        cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    }

    // Configured left infrared stream
    // https://github.com/IntelRealSense/librealsense/issues/1140
    if (INFRARED_FLAG)
    {   // https://github.com/IntelRealSense/librealsense/issues/1140#issuecomment-364737911
        cfg.enable_stream(RS2_STREAM_INFRARED, 1, INFRARED_WIDTH, INFRARED_HEIGHT, RS2_FORMAT_Y8, INFRARED_FPS); // left
        cfg.enable_stream(RS2_STREAM_INFRARED, 2, INFRARED_WIDTH, INFRARED_HEIGHT, RS2_FORMAT_Y8, INFRARED_FPS); // right
    }
    
    // Instruct pipeline to start streaming with the requested configuration
    cout << "Starting RealSense..." << endl;
    rs2::pipeline_profile profile = pipe.start(cfg);

    /////////////////////////////////
    // GET REALSENSE DEPTH SCALING //
    /////////////////////////////////

    // Find device
    // https://github.com/IntelRealSense/librealsense/wiki/API-How-To
    auto dev = profile.get_device();

    // Find first depth sensor (device_list can have zero or more then one)
    // https://github.com/IntelRealSense/librealsense/wiki/API-How-To
    auto depth_sensor = dev.first<rs2::depth_sensor>();

    // Find depth scale (to meters)
    auto scale =  depth_sensor.get_depth_scale();

    //////////////////////////
    // TURN LASER ON OR OFF //
    //////////////////////////

    // https://github.com/IntelRealSense/librealsense/wiki/API-How-To
    if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
    {
        if(LASER_FLAG)
        {
            depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f); // Enable emitter
        }
        else
        {
            depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f); // Disable emitter
        }
    }
    if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
    {
        // Query min and max values:
        auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
        if(LASER_FLAG)
        {
            depth_sensor.set_option(RS2_OPTION_LASER_POWER, range.max); // Set max power
        }
        else
        {
            depth_sensor.set_option(RS2_OPTION_LASER_POWER, 0.f); // Disable laser
        }
    }
    
    ////////////////////////////////////////////
    // CONFIGURE IMSHOW VISUALIZATION WINDOWS //
    ////////////////////////////////////////////
        
    using namespace cv;
    const auto window_name_depth = "Depth";
    const auto window_name_infrared_left = "IR (Left)";
    const auto window_name_infrared_right = "IR (Right)";

    if (IMSHOW_DEPTH)
    {
        namedWindow(window_name_depth, WINDOW_AUTOSIZE);
    }

    if (IMSHOW_INFRARED)
    {
        namedWindow(window_name_infrared_left, WINDOW_AUTOSIZE);        
        namedWindow(window_name_infrared_right, WINDOW_AUTOSIZE);
    }

    //////////////////////////////
    // CONFIGURE ROS PUBLISHERS //
    //////////////////////////////

    // Publish sensor_msgs::Image
    // https://answers.ros.org/question/99831/publish-file-to-image-topic/

    ros::Publisher pub_image_depth_RGB = nh_.advertise<sensor_msgs::Image>("/camera/depth_RGB/image_rect_raw", 1);
    ros::Publisher pub_image_depth_meters = nh_.advertise<std_msgs::Float32MultiArray>("/camera/depth/image_rect_raw", 1);
    ros::Publisher pub_image_infrared_left = nh_.advertise<sensor_msgs::Image>("/camera/infra1/image_rect_raw", 1);
    ros::Publisher pub_image_infrared_right = nh_.advertise<sensor_msgs::Image>("/camera/infra2/image_rect_raw", 1);

    // Setup IMU publisher
    ros::Publisher pub_imu = nh_.advertise<sensor_msgs::Imu>("/imu/raw", 1);

    // Pre-allocate the depth matrix and message
    cv::Mat mat_depth_meters(Size(DEPTH_WIDTH, DEPTH_HEIGHT), CV_32FC1);

    std_msgs::Float32MultiArray depth_msg;
    depth_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    depth_msg.layout.dim[0].label = "height";
    depth_msg.layout.dim[0].size = DEPTH_HEIGHT;
    depth_msg.layout.dim[0].stride =  DEPTH_HEIGHT * DEPTH_WIDTH;
    depth_msg.layout.dim[0].label = "width";
    depth_msg.layout.dim[0].size = DEPTH_WIDTH;
    depth_msg.layout.dim[0].stride =  DEPTH_WIDTH;

    while (nh_.ok() && waitKey(1) < 0) 
    {
        cnt++;

        //////////////////////////////////
        // PRINT PARAMETERS TO TERMINAL //
        //////////////////////////////////

        if(first_loop_flag == 1)
        {
            cout << "Collecting data..." << endl;
            first_loop_flag = 0;
        }

        //////////////////////////////
        // WAIT FOR REALSENSE FRAME //
        //////////////////////////////
        
        // Wait for next set of frames from the camera
        rs2::frameset frame_set = pipe.wait_for_frames();

        // Find and retrieve IMU and/or tracking data
        if(IMU_FLAG)
        {
            sensor_msgs::Imu imu_msg;

            if (rs2::motion_frame accel_frame = frame_set.first_or_default(RS2_STREAM_ACCEL))
            {
                // linear acceleration
                rs2_vector accel_sample = accel_frame.get_motion_data();
                imu_msg.linear_acceleration.x = accel_sample.x;
                imu_msg.linear_acceleration.y = accel_sample.y;
                imu_msg.linear_acceleration.z = accel_sample.z;
            }
        
            if (rs2::motion_frame gyro_frame = frame_set.first_or_default(RS2_STREAM_GYRO))
            {
                // angular velocity
                rs2_vector gyro_sample = gyro_frame.get_motion_data();
                imu_msg.angular_velocity.x = gyro_sample.x;
                imu_msg.angular_velocity.y = gyro_sample.y;
                imu_msg.angular_velocity.z = gyro_sample.z;
            }

            pub_imu.publish(imu_msg);
        }

        if(DEPTH_FLAG)
        {

            //////////////////////////////
            // GET IMAGES FROM FRAMESET //
            //////////////////////////////
            rs2::frame frame_depth = frame_set.get_depth_frame();
 
            ////////////////////////////////////////
            // SEND FRAME DATA TO OPENCV MATRICES //
            ////////////////////////////////////////

            // Obtain 16-bit depth matrix
            cv::Mat mat_depth_16b(Size(DEPTH_WIDTH, DEPTH_HEIGHT), CV_16U, (void*)(frame_depth.get_data()), Mat::AUTO_STEP);

            ////////////////////////////////////////
            // PUBLISH ROS MESSAGES TO ROS TOPICS //
            ////////////////////////////////////////

            if(DEPTH_RGB_FLAG)
            {
                ////////////////////////
                // APPLY RGB COLORMAP //
                ////////////////////////
                rs2::frame frame_depth_RGB = frame_depth.apply_filter(color_map);
                
                //////////////////////////////////////////////
                // SEND FRAME DATA TO 8-BIT OPENCV MATRICES //
                //////////////////////////////////////////////
                cv::Mat mat_depth_RGB(Size(DEPTH_WIDTH, DEPTH_HEIGHT), CV_8UC3, (void*)frame_depth_RGB.get_data(), Mat::AUTO_STEP);
    
                //////////////////////////////////////////
                // CONVERT CV:MAT to SENSOR_MSGS::IMAGE //
                //////////////////////////////////////////
                cv_bridge::CvImage cv_image_depth_RGB;
                cv_image_depth_RGB.image = mat_depth_RGB;
                cv_image_depth_RGB.encoding = "bgr8";
                sensor_msgs::Image ros_image_depth_RGB;
                cv_image_depth_RGB.toImageMsg(ros_image_depth_RGB);

                if(IMSHOW_DEPTH)
                {
                    /////////////////////////////////////////
                    // UPDATE IMSHOW VISUALIZATION WINDOWS //
                    /////////////////////////////////////////
                    cv::imshow(window_name_depth, mat_depth_RGB);
                }

                ////////////////////////////////////////
                // PUBLISH ROS MESSAGES TO ROS TOPICS //
                ////////////////////////////////////////
                pub_image_depth_RGB.publish(ros_image_depth_RGB);
            }
            
            if(DEPTH_METERS_FLAG)
            {
                // Obtain depth image (meters) for calculations
                mat_depth_16b.convertTo(mat_depth_meters, CV_32F, scale);
                
                // copy in the data
                depth_msg.data.clear();
                depth_msg.data.insert(depth_msg.data.end(), (float*)mat_depth_meters.datastart, (float*)mat_depth_meters.dataend);

                ////////////////////////////////////////
                // PUBLISH ROS MESSAGES TO ROS TOPICS //
                ////////////////////////////////////////
                pub_image_depth_meters.publish(depth_msg);
            }
        }

        if(INFRARED_FLAG)
        {
            //////////////////////////////
            // GET IMAGES FROM FRAMESET //
            //////////////////////////////
            rs2::video_frame frame_infrared_left = frame_set.get_infrared_frame(1);
            rs2::video_frame frame_infrared_right = frame_set.get_infrared_frame(2);

            ////////////////////////////////////////
            // SEND FRAME DATA TO OPENCV MATRICES //
            ////////////////////////////////////////
            cv::Mat mat_infrared_left(Size(INFRARED_WIDTH, INFRARED_HEIGHT), CV_8UC1, (void*)frame_infrared_left.get_data(), Mat::AUTO_STEP);
            cv::Mat mat_infrared_right(Size(INFRARED_WIDTH, INFRARED_HEIGHT), CV_8UC1, (void*)frame_infrared_right.get_data(), Mat::AUTO_STEP); 
            
            //////////////////////////////////////////
            // CONVERT CV:MAT to SENSOR_MSGS::IMAGE //
            //////////////////////////////////////////
            cv_bridge::CvImage cv_image_infrared_left;
            cv_image_infrared_left.image = mat_infrared_left;
            cv_image_infrared_left.encoding = "mono8";
            sensor_msgs::Image ros_image_infrared_left;

            cv_bridge::CvImage cv_image_infrared_right;
            cv_image_infrared_right.image = mat_infrared_right;
            cv_image_infrared_right.encoding = "mono8";
            sensor_msgs::Image ros_image_infrared_right;

            cv_image_infrared_left.toImageMsg(ros_image_infrared_left);
            cv_image_infrared_right.toImageMsg(ros_image_infrared_right);

            if( IMSHOW_INFRARED )
            {
                /////////////////////////////////////////
                // UPDATE IMSHOW VISUALIZATION WINDOWS //
                /////////////////////////////////////////
                cv::imshow(window_name_infrared_left, mat_infrared_left);
                cv::imshow(window_name_infrared_right, mat_infrared_right);
            }
            
            ////////////////////////////////////////
            // PUBLISH ROS MESSAGES TO ROS TOPICS //
            ////////////////////////////////////////
            pub_image_infrared_left.publish(ros_image_infrared_left);
            pub_image_infrared_right.publish(ros_image_infrared_right);

            // POINT CLOUD CREATION: HAVE NOT IMPLEMENTED
            // https://stackoverflow.com/questions/32521043/how-to-convert-cvmat-to-pclpointcloud
            // {
            //     pcl::PointCloud<pcl::PointXYZ>::Ptr MatToPoinXYZ(cv::Mat depthMat)
            // {
            //     pcl::PointCloud<pcl::PointXYZ>::Ptr ptCloud (new pcl::PointCloud<pcl::PointXYZ>);
            // pcl::PointCLoud
            // // calibration parameters
            //     float const fx_d = 5.9421434211923247e+02;
            //     float const fy_d = 5.9104053696870778e+02;
            //     float const cx_d = 3.3930780975300314e+02;
            //     float const cy_d = 2.4273913761751615e+02;

            //     unsigned char* p = depthMat.data;
            //     for (int i = 0; i<depthMat.rows; i++)
            //     {
            //         for (int j = 0; j < depthMat.cols; j++)
            //         {
            //             float z = static_cast<float>(*p);
            //             pcl::PointXYZ point;
            //             point.z = 0.001 * z;
            //             point.x = point.z*(j - cx_d)  / fx_d;
            //             point.y = point.z *(cy_d - i) / fy_d;
            //             ptCloud->points.push_back(point);
            //             ++p;
            //         }
            //     }
            //     ptCloud->width = (int)depthMat.cols; 
            //     ptCloud->height = (int)depthMat.rows; 

            //     return ptCloud;

            // }
            // }

        }

        ////////////////////////////////////
        // EXTRACT ROW DATA FROM MATRICES //
        ////////////////////////////////////

        // Obtain scaled 16-bit matrix for calculations
        // https://stackoverflow.com/questions/6909464/convert-16-bit-depth-cvmat-to-8-bit-depth
        //cv::Mat depth_image_16bit(Size(w, h), CV_16U, (void*)(depth.get_data()), Mat::AUTO_STEP);

        // https://stackoverflow.com/questions/17892840/opencv-multiply-scalar-and-matrix
        // depth_image_16bit *= scale;

        // Obtain depth image (meters) for calculations
        //https://stackoverflow.com/questions/6302171/convert-uchar-mat-to-float-mat-in-opencv
        //cv::Mat depth_image_float_m;
        //depth_image_16bit.convertTo(depth_image_float_m, CV_32F, scale);//0.00390625); // Note: 1/256 = 0.00390625

        // Copy one row of depth image to a new matrix
        //cv::Mat depth_vector_float_m(Size(w, 0), CV_32F);
        //depth_vector_float_m.push_back(depth_image_float_m.row(h-1));

        
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
