/*
	Heimdall Image Handler grabs images, stores them and generates videos on request.
    Copyright (C) 2017 Christof Oost, Amir Shantia, Ron Snijders, Egbert van der Wal

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <heimdall_image_handler/image_handler.h>
#include <opencv2/highgui/highgui.hpp>

#include <unistd.h>

#include <std_msgs/String.h>

namespace Heimdall
{
    /**
     * The constructor requires a ROS Node handle, a topic for RGB and a topic for depth
     *
     * @param nh The ROS node handle
     * @param rgb_topic The RGB topic to subscribe to
     * @param depth_topic the depth topic to subscribe to
     */
    ImageHandler::ImageHandler(ros::NodeHandle &nh, std::string const &rgb_topic, std::string const &depth_topic)
    :
        d_node_handle(nh),
        d_image_transport(nh),
        d_last_activity(-1),
        d_last_activity_time(std::chrono::steady_clock::now()),
        d_activity_subscriber(nh.subscribe("/rv/activity", 1, &ImageHandler::activityCallback, this)),
        d_volume_subscriber(nh.subscribe("/dd/volumes", 1, &ImageHandler::volumeCallback, this)),
        d_command_service(nh.advertiseService("image_handler", &ImageHandler::commandService, this)),
        d_image_path("/opt/enacer/camera"),
        d_image_prefix("frame_"),
        d_active_type(RGB),
        d_auto_select_type(true),
        d_active_lock_until(std::chrono::steady_clock::now()),
        d_rgb_capture(nullptr),
        d_rgb_enabled(false),
        d_rgb_topic(rgb_topic),
        d_rgb_target_interval(5),
        d_depth_enabled(false),
        d_depth_topic(depth_topic),
        d_depth_target_interval(5),
        d_min_hz(4),
        d_max_hz(10),
        d_image_history_size(3600),
        d_last_gc(std::chrono::steady_clock::now())
    {
        checkPath();
        setRGBEnabled(true);

        d_cv_imwrite_parameters.push_back(CV_IMWRITE_JPEG_QUALITY);
        d_cv_imwrite_parameters.push_back(85);
        d_img_update_publisher = d_node_handle.advertise<std_msgs::String>("/en/image_updated", 1);
    }
}
