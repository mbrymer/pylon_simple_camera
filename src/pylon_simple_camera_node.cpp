/*
Simple ROS Driver Node for Basler daA-2500-60mci camera
Mostly copy-pasta from Basler pylon-ros-camera repo
Nearly zero input/error checking, use at your own risk
*/

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>
#include <image_transport/camera_common.h>

// Basler Pylon C++ API
#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>

// std lib includes
#include <string.h>

using namespace Pylon;

// Declarations
bool genAPI2Ros(const std::string& gen_api_enc, std::string& ros_enc);

int main (int argc, char **argv)
{   
    // Initialize node
    ros::init(argc, argv, "pylon_simple_camera");
    ros::NodeHandle node_pylon_camera;

    // Grab parameters
    std::string camera_info_topic;
    std::string camera_topic;
    int grab_timeout; // ms
    double run_rate; // Hz

    std::vector<double> K; // Intrinsics
    std::vector<double> D; // Distortion coefficients

    int image_width;
    int image_height;
    int image_offsetX;
    int image_offsetY;

    bool use_scaling;

    node_pylon_camera.param<std::string>("camera_topic",camera_topic,"/basler_camera/image_raw");
    node_pylon_camera.param<std::string>("camera_info_topic",camera_info_topic,"/basler_camera/camera_info");

    node_pylon_camera.param<double>("run_rate",run_rate,20.0);
    node_pylon_camera.param<int>("grab_timeout",grab_timeout,5000);
    node_pylon_camera.param<int>("image_width",image_width,640);
    node_pylon_camera.param<int>("image_height",image_height,480);
    node_pylon_camera.param<int>("image_offsetX",image_offsetX,0);
    node_pylon_camera.param<int>("image_offsetY",image_offsetY,0);

    node_pylon_camera.param<bool>("use_scaling",use_scaling,true);

    node_pylon_camera.getParam("K",K);
    node_pylon_camera.getParam("D",D);

    // Run Rate
    ros::Rate node_rate(run_rate);

    // Define publishers
    image_transport::ImageTransport it_pylon_camera(node_pylon_camera);
    image_transport::CameraPublisher camera_pub = it_pylon_camera.advertiseCamera(camera_topic,1);

    // Start up Pylon and camera
    Pylon::PylonAutoInitTerm pylon_autoterm;
    Pylon::CBaslerUniversalInstantCamera myCamera(Pylon::CTlFactory::GetInstance().CreateFirstDevice() );

    // Clear the default camera configuration, use values set on camera itself
    myCamera.RegisterConfiguration( (Pylon::CConfigurationEventHandler*) NULL, Pylon::RegistrationMode_ReplaceAll, Pylon::Cleanup_None);
    myCamera.Open();

    // Set ROI
    myCamera.Height.SetValue(image_height);
    myCamera.Width.SetValue(image_width);
    myCamera.OffsetX.SetValue(image_offsetX);
    myCamera.OffsetY.SetValue(image_offsetY);

    // Enable downsampling
    GenApi::INodeMap& nodemap = myCamera.GetNodeMap();
    CBooleanParameter(nodemap, "BslScalingEnable").SetValue(use_scaling);
    
    myCamera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);

    // Check pixel format is valid
    std::string gen_api_encoding(myCamera.PixelFormat.ToString().c_str());
    std::string ros_encoding("");
    std::cout << "Pixel Format: " << myCamera.PixelFormat.ToString() << std::endl;
    if ( !genAPI2Ros(gen_api_encoding, ros_encoding) )
    {
        std::cout << "Invalid pixel encoding format" << std::endl;
    }
    else
    {
        std::cout << "Camera initialized, beginning to take pictures:" << std::endl;
    }

    while (ros::ok())
    {
        // Initialize image message object
        sensor_msgs::Image image_msg;
        sensor_msgs::CameraInfo camera_info_msg;

        // Create pointer to store captured image and grab image
        Pylon::CBaslerUniversalGrabResultPtr ptrGrabResult;
        bool grab_successful = myCamera.RetrieveResult(grab_timeout, ptrGrabResult, Pylon::TimeoutHandling_Return);

        if (grab_successful)
        {
            // Grab successful, populate image message and send it out
            const uint8_t* pImageBuffer = reinterpret_cast<uint8_t*>(ptrGrabResult->GetBuffer());
            
            // Grab image/pixel dimensions
            int n_cols = ptrGrabResult->GetWidth();
            int n_rows = ptrGrabResult->GetHeight();
            int pixel_depth = 2; // Fixed at UYVY -> 2 bytes per pixel

            std::cout << "Pixel Depth: " << pixel_depth << std::endl;

            std::cout << "First pixel values:" << std::to_string(pImageBuffer[0]) << "," << std::to_string(pImageBuffer[1])
             << "," << std::to_string(pImageBuffer[2]) << "," << std::to_string(pImageBuffer[3]) << std::endl;

            // Populate image data
            image_msg.data.assign(pImageBuffer,pImageBuffer+n_cols*n_rows*pixel_depth);

            // Populate image parameters
            auto stamp_now = ros::Time::now();
            image_msg.header.stamp = stamp_now;
            image_msg.height = n_rows;
            image_msg.width = n_cols;
            image_msg.step = n_cols*pixel_depth;
            image_msg.encoding = ros_encoding;

            // Populate camera info
            camera_info_msg.header.stamp = stamp_now;
            camera_info_msg.height = image_height;
            camera_info_msg.width = image_width;
            camera_info_msg.K = {K[0], K[1], K[2],
                                K[3], K[4], K[5],
                                K[6], K[7], K[8]}; // Intrinsics
            camera_info_msg.P = {K[0], K[1], K[2],0,
                                K[3], K[4], K[5],0,
                                K[6], K[7], K[8],0}; 
            camera_info_msg.R = {1,0,0,
                                 0,1,0,
                                 0,0,1}; // Rectification
            camera_info_msg.D = D; // Distortion coefficients
            
            camera_info_msg.distortion_model = "plumb_bob";

            // Publish image
            camera_pub.publish(image_msg,camera_info_msg);
        }
        else
        {
            ROS_INFO("Camera image grab unsuccessful");
        }
        
        node_rate.sleep(); // Wait for next cycle
    }

    // Clean up camera
    myCamera.StopGrabbing();
    myCamera.Close();

}

bool genAPI2Ros(const std::string& gen_api_enc, std::string& ros_enc)
{
    if ( gen_api_enc == "Mono8" )
    {
        ros_enc = sensor_msgs::image_encodings::MONO8;
    }
    else if ( gen_api_enc == "Mono12" )
    {
        ros_enc = sensor_msgs::image_encodings::MONO16;
    }
    else if ( gen_api_enc == "Mono16" )
    {
        ros_enc = sensor_msgs::image_encodings::MONO16;
    }
    else if ( gen_api_enc == "BGR8" )
    {
        ros_enc = sensor_msgs::image_encodings::BGR8;
    }
    else if ( gen_api_enc == "RGB8" )
    {
        ros_enc = sensor_msgs::image_encodings::RGB8;
    }
    else if ( gen_api_enc == "BayerBG8" )
    {
        ros_enc = sensor_msgs::image_encodings::BAYER_BGGR8;
    }
    else if ( gen_api_enc == "BayerGB8" )
    {
        ros_enc = sensor_msgs::image_encodings::BAYER_GBRG8;
    }
    else if ( gen_api_enc == "BayerRG8" )
    {
        ros_enc = sensor_msgs::image_encodings::BAYER_RGGB8;
    }
    else if ( gen_api_enc == "BayerGR8" )
    {
        ros_enc = sensor_msgs::image_encodings::BAYER_GRBG8;
    }
    else if ( gen_api_enc == "BayerRG12" )
    {
        ros_enc = sensor_msgs::image_encodings::BAYER_RGGB16;
    }
    else if ( gen_api_enc == "BayerBG12" )
    {
        ros_enc = sensor_msgs::image_encodings::BAYER_BGGR16;
    }
    else if ( gen_api_enc == "BayerGB12" )
    {
        ros_enc = sensor_msgs::image_encodings::BAYER_GBRG16;
    }
    else if ( gen_api_enc == "BayerGR12" )
    {
        ros_enc = sensor_msgs::image_encodings::BAYER_GRBG16;
    }
    else if ( gen_api_enc == "BayerRG16" )
    {
        ros_enc = sensor_msgs::image_encodings::BAYER_RGGB16;
    }
    else if ( gen_api_enc == "BayerBG16" )
    {
        ros_enc = sensor_msgs::image_encodings::BAYER_BGGR16;
    }
    else if ( gen_api_enc == "BayerGB16" )
    {
        ros_enc = sensor_msgs::image_encodings::BAYER_GBRG16;
    }
    else if ( gen_api_enc == "BayerGR16" )
    {
        ros_enc = sensor_msgs::image_encodings::BAYER_GRBG16;
    }
    else if ( gen_api_enc == "YUV422Packed" || gen_api_enc == "YUV422_8_UYVY")
    {
        ros_enc = sensor_msgs::image_encodings::YUV422;
    }

    /*
        // Currently no ROS equivalents:
        else if ( gen_api_enc == "YUV422_YUYV_Packed" )
        {
            ros_enc = sensor_msgs::image_encodings::YUV422;
        }
        else if ( gen_api_enc == "YCbCr422_8" )
        {
            ros_enc = sensor_msgs::image_encodings::YUV422;
        }
    */


    else
    {
        /* Unsupported are:
         * - Mono10
         * - Mono10p
         * - Mono12p
         * - BayerGR10
         * - BayerGR10p
         * - BayerRG10
         * - BayerRG10p
         * - BayerGB10
         * - BayerGB10p
         * - BayerBG10
         * - BayerBG10p
         * - BayerGR12p
         * - BayerRG12p
         * - BayerGB12p
         * - BayerBG12p
         * - YCbCr422_8
         * - YUV422_YUYV_Packed
         */
        return false;
    }
    return true;
}
