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
#include <heimdall_image_handler/datetime.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

namespace Heimdall
{
    /**
     * Callback for incoming depth messages. If the interval since the previous
     * message has passed, the image will be converted from 16UC1 to BGR8.
     *
     * @param msg The received message
     */
    void ImageHandler::depthCallback(sensor_msgs::ImageConstPtr const &msg)
    {
        // Lock the depth mutex
        std::unique_lock<std::mutex> lock(d_depth_mutex);

        // Ignore message if disabled
        if (!d_depth_enabled)
        {
            ROS_DEBUG("Dropping depth image because depth is disabled");
            return;
        }

        using namespace std::chrono;
        time_point<steady_clock> now_st = steady_clock::now();
        time_point<system_clock> now_r = system_clock::now();

        milliseconds elapsed = duration_cast<milliseconds>(now_st - d_last_depth_received);

        if (elapsed < d_depth_target_interval)
        {
            ROS_DEBUG("Dropping depth image because it exceeds the targer framerate");
            return;
        }

        // Store the received time
        d_last_depth_received = now_st;
        d_last_depth_time = now_r;

        // Convert the mono16 image to a bgr8 image
        cv_bridge::CvImageConstPtr cv_ptr;

        try
        {
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("Error converting image: %s", e.what());
            return;
        }

        cv::Mat depth = cv_ptr->image;

        // The input image has range data from ~400 - ~8000
        // So to optimally visualize the image in 8 bits, the
        // values are scaled by a factor 255.0 / 8192 = 1/32 and
        // afterwards, a value of 400/32 = 12.5 is subtracted
        cv::Mat depth8;
        depth.convertTo(depth8, CV_8UC1, 255.0 / 8192.0, -12.5);

        // Convert the 8 bit depth image to 3 channel 8 bit color
        cv::cvtColor(depth8, d_last_depth, cv::COLOR_GRAY2BGR, 3);

        cv::Scalar mean, stddev;
        cv::meanStdDev(d_last_depth, mean, stddev);

        // Add a timestamp
        stampImage(d_last_depth);

        // Store the image
        std::string im_file = d_image_path + "/" + d_image_prefix + format(d_last_depth_time) + "_depth.jpg";
        ROS_DEBUG("Wrote depth image to: %s", im_file.c_str());
        cv::imwrite(im_file, d_last_depth, d_cv_imwrite_parameters);

        updateLastFrame(DEPTH);
    }
}
