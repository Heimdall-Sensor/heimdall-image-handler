#include <ros/ros.h>
#include <heimdall_msgs/CommandSrv.h>

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cerr << "Syntax: " << argv[0] << " <command>\n";
        return 1;
    }

    std::string command = argv[1];
    std::string json;

    if (command == "enable_mask" || command == "disable_mask")
    {
        if (argc < 3)
            return 1;

        std::string label = argv[2]; 
        json = "{\"command\": \"" + command + "\", \"label\": \"" + label + "\"}";
    }
    else
    {
        json = "{\"command\": \"" + command + "\"}";
    }

    ros::init(argc, argv, "heimdall_test");
    heimdall_msgs::CommandSrv srv;
    srv.request.command = json;
    std::cout << srv.request.command << std::endl;

    if (ros::service::call("/heimdall/image_handler", srv))
    {
        std::cout << "Successfully called service: " << srv.response.result << "\n";
    }
    else
        std::cerr << "Failed to call service\n";
    return 0;
}
