#include <ros/ros.h>
#include <heimdall_image_handler/image_handler.h>
#include <thread>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_handler");

    ros::NodeHandle nh("en");
    Heimdall::ImageHandler acquirer(nh, "/usb_cam/image_raw", "/camera/depth/image_raw");

    acquirer.setDepthRate(0.1);
    acquirer.setRGBRate(0.1);

    ros::AsyncSpinner spinner(5);
    spinner.start();

    while (ros::ok())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        acquirer.check();
    }
}
