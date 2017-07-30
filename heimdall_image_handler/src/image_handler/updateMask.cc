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
#include <heimdall_msgs/GetMaskSrv.h>
#include <boost/thread.hpp>

namespace Heimdall
{
    /**
     * Update the mask applied to output files.
     *
     * @param reference The reference image which dimensions should match the mask
     */
    void ImageHandler::updateMask()
    {
        // Don't update when not needed
        if (d_mask_ok)
            return;

        boost::thread(boost::bind(&Heimdall::ImageHandler::updateMaskThread, this));
    }

    void ImageHandler::updateMaskThread()
    {
        // Try to lock
        std::unique_lock<std::mutex> update_lock(d_mask_update_mutex, std::try_to_lock);
        if (not update_lock.owns_lock())
        {
            ROS_WARN("Multiple simultanuous mask updates requested");
            return;
        }

        cv::Mat const &reference = d_active_type == RGB ? d_last_rgb : d_last_depth;
        if (d_masks.empty())
        {
            // When no masks are defined, show entire image
            std::unique_lock<std::mutex> mask_lock(d_mask_mutex);
            d_mask = cv::Mat(reference.size(), CV_8UC3, CV_RGB(1, 1, 1));
            d_mask_ok = true;
            return;
        }

        if (not ros::service::exists("/dd/get_mask", false))
        {
            ROS_ERROR("get_mask service is not available. Cannot apply masks");
            d_mask_ok = false;
            return;
        }

        // Start with an empty image
        cv::Mat new_mask(cv::Mat(reference.size(), CV_8UC3, CV_RGB(0, 0, 0)));

        // Add all pixels belonging to one of the activated masks
        auto iterator = d_masks.begin();
        while (iterator != d_masks.end())
        {
            en_msgs::GetMaskSrv req;
            req.request.label = *iterator;

            if (ros::service::call("/dd/get_mask", req))
            {
                cv::Mat mask(req.response.height, req.response.width, CV_8UC3);
                size_t length = req.response.width * req.response.height;
                for (size_t i = 0; i < length; ++i)
                    mask.at<uint8_t>(3 * i) = mask.at<uint8_t>(3 * i + 1) = mask.at<uint8_t>(3 * i + 2) = req.response.pixels[i] ? 1 : 0;

                new_mask |= mask;
            }
            else
            {
                auto to_remove = iterator++;
                ROS_ERROR("Could not obtain mask with label %s. Removing from list", to_remove->c_str());
                d_masks.erase(to_remove);
                continue;
            }
            ++iterator;
        }
        
        // d_masks may be empty as a result of removing unavailable masks
        if (d_masks.empty())
            new_mask = cv::Mat(reference.size(), CV_8UC3, CV_RGB(1, 1, 1));

        // Mask is accurately reflecting d_masks
        std::unique_lock<std::mutex> mask_lock(d_mask_mutex);
        new_mask.copyTo(d_mask);
        d_mask_ok = true;
    }
}
