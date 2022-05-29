/*
Simple ROS Driver Node for Basler daA-2500-60mci camera
Mostly copy-pasta from Basler pylon-ros-camera repo
Nearly zero input/error checking, use at your own risk
*/

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

// Basler Pylon C++ API
#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>

// std lib includes
#include <string.h>

// Parameters
std::string image_topic = "image_raw";
int grab_timeout = 5000; // ms

// Declarations
// bool grab_image(sensor_msgs::Image& image_msg);
bool genAPI2Ros(const std::string& gen_api_enc, std::string& ros_enc);

int main (int argc, char **argv)
{   
    // Initialize node
    ros::init(argc, argv, "pylon_simple_camera");
    ros::NodeHandle node_pylon_camera;

    // Define publisher
    ros::Publisher image_pub = node_pylon_camera.advertise<sensor_msgs::Image>(image_topic,1);

    // Start up Pylon and camera
    Pylon::PylonAutoInitTerm pylon_autoterm;
    Pylon::CBaslerUniversalInstantCamera myCamera(Pylon::CTlFactory::GetInstance().CreateFirstDevice() );
    // Clear the default camera configuration, use values set on camera itself
    myCamera.RegisterConfiguration( (Pylon::CConfigurationEventHandler*) NULL, Pylon::RegistrationMode_ReplaceAll, Pylon::Cleanup_None);
    myCamera.Open();
    myCamera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);

    // Check pixel format is valid
    std::string gen_api_encoding(myCamera.PixelFormat.ToString().c_str());
    std::string ros_encoding("");
    std::cout << "Pixel Format: " << myCamera.PixelFormat.ToString();
    if ( !genAPI2Ros(gen_api_encoding, ros_encoding) )
    {
        std::cout << "Invalid pixel encoding format";
    }
    else
    {
        std::cout << "Camera initialized, beginning to take pictures:\n";
    }

    while (ros::ok())
    {
        // Initialize image message object
        sensor_msgs::Image image_msg;

        // Grab image and put in .data field
        // bool grab_successful = grab_image(image_msg);

        // Create pointer to store captured image and grab image
        Pylon::CBaslerUniversalGrabResultPtr ptrGrabResult;
        bool grab_successful = myCamera.RetrieveResult(grab_timeout, ptrGrabResult, Pylon::TimeoutHandling_Return);

        if (grab_successful)
        {
            // Grab successful, populate image message and send it out
            const uint8_t *pImageBuffer = reinterpret_cast<uint8_t*>(ptrGrabResult->GetBuffer());
            
            // Grab image/pixel dimensions
            int n_cols = ptrGrabResult->GetWidth();
            int n_rows = ptrGrabResult->GetHeight();
            int pixel_depth = myCamera.PixelSize.GetIntValue()/8; // Pixel size is in bits, convert to bytes

            // Populate image data
            image_msg.data.assign(pImageBuffer,pImageBuffer+n_cols*n_rows*pixel_depth);

            // Populate image parameters
            image_msg.header.stamp = ros::Time::now();
            image_msg.height = n_rows;
            image_msg.width = n_cols;
            image_msg.step = n_cols*pixel_depth;
            image_msg.encoding = ros_encoding;

            // Publish image
            image_pub.publish(image_msg);
        }
        else
        {
            ROS_INFO("Camera image grab unsuccessful");
        }
        
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
    else if ( gen_api_enc == "YUV422Packed" )
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