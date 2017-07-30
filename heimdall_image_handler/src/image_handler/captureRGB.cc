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

using namespace std::chrono;

namespace Heimdall
{
    /**
     * Callback for incoming rgb messages. If the interval since the previous
     * message has passed, the image will be stored.
     *
     * @param msg The received message
     */
    void ImageHandler::captureRGB()
    {
        // Lock the RGB mutex
        std::unique_lock<std::mutex> lock(d_rgb_mutex);

        if (d_rgb_capture == nullptr)
        {
            ROS_DEBUG("Request to capture RGB image cannot be executed because the VideoCapture is not available");
            return;
        }

        cv::Mat image;
        if (!d_rgb_capture->read(image))
        {
            ROS_WARN("Failed to grab image");
            return;
        }

        // Store the received time
        d_last_rgb_received = steady_clock::now();
        d_last_rgb_time = system_clock::now();

        // Determine the intensity
        if (not d_auto_select_type && std::chrono::steady_clock::now() > d_active_lock_until)
            d_auto_select_type = true;

        if (d_auto_select_type)
        {
            cv::Scalar mean, stddev;
            cv::meanStdDev(image, mean, stddev);
            ROS_DEBUG("Autoselect is on - intensity: %lf", mean[0]);

            if (d_active_type == RGB && mean[0] <= 32)
            {
                ROS_INFO("Auto-switching to depth");
                d_active_type = DEPTH;
                setDepthEnabled(true);
            }
            else if (d_active_type == DEPTH && mean[0] >= 40)
            {
                ROS_INFO("Auto-switching to RGB");
                d_active_type = RGB;
                setDepthEnabled(false);
            }
        }
        else
            ROS_DEBUG("Autoselect is off");

        d_last_rgb = image;

        // Add a timestamp
        stampImage(d_last_rgb);

        // Store the image if it is the active type
        if (RGB != d_active_type)
            return;

        std::string im_file = d_image_path + "/" + d_image_prefix + format(d_last_rgb_time) + "_rgb.jpg";
        ROS_DEBUG("Wrote RGB image to: %s", im_file.c_str());
        cv::imwrite(im_file, d_last_rgb, d_cv_imwrite_parameters);

        updateLastFrame(RGB);
    }
}
