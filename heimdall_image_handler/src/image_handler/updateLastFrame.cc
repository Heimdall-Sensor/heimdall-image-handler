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

#include <boost/filesystem.hpp>
#include <unistd.h>

#include <std_msgs/String.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace Heimdall
{
    /**
     * Write the last frame to the disk, if the type matches the currently active output type
     * @param update_type The type of image supplied, either RGB or Depth
     * @param img The image that should be written, after applying the masks
     */
    void ImageHandler::updateLastFrame(IMGType update_type)
    {
        if (update_type == d_active_type)
        {
            std::unique_lock<std::mutex> mask_lock(d_mask_mutex);
            cv::Mat const &img = d_active_type == RGB ? d_last_rgb : d_last_depth;

            // Make sure the mask has the correct dimensions
            validateMask();

            // Create a new image, masked from the active image
            cv::Mat masked(img.size(), CV_8UC3, CV_RGB(0, 0, 0));
            img.copyTo(masked, d_mask);

            // If masks are enabled, the timestamp may have been masked away, so add it again
            if (not d_masks.empty())
                stampImage(masked);
        
            // Write it to the last frame file
            std::string live_file = d_image_path + "/" + d_image_prefix + "last.jpg";
            std::string current_file;
            std::string new_file;

            char filebuf[256];
            int read = readlink(live_file.c_str(), filebuf, 256);
            int seq = 0;
            if (read > 0)
            {
                current_file = std::string(filebuf, read);
                if (current_file.length() > 5)
                    seq = (current_file[current_file.length() - 5] - 47) % 10;
            }

            std::ostringstream new_file_buf;
            new_file_buf << d_image_path << '/' << d_image_prefix << "last." << seq << ".jpg";
            new_file = new_file_buf.str();

            ROS_DEBUG("Saved frame to %s", new_file.c_str());
            cv::imwrite(new_file, masked);

            // Remove existing symlink
            if (boost::filesystem::exists(live_file))
                unlink(live_file.c_str());
            
            // Create new symlink
            if (symlink(new_file.c_str(), live_file.c_str()) < 0)
            {
                ROS_ERROR("Error while creating symlink at %s: %d", new_file.c_str(), errno);
                return;
            }

            // Publish update
            boost::property_tree::ptree op;
            boost::property_tree::ptree mask_list;
            for (auto ptr = d_masks.begin(); ptr != d_masks.end(); ++ptr)
            {
                boost::property_tree::ptree child;
                child.put("", *ptr);
                mask_list.push_back(std::make_pair("", child));
            }

            op.add_child("enabled_masks", mask_list);
            op.put<std::string>("type", d_active_type == RGB ? "rgb" : "depth");
            op.put<size_t>("interval", d_active_type == RGB ? d_rgb_target_interval.count() : d_depth_target_interval.count());

            std_msgs::String msg;
            std::ostringstream json_buf;
            write_json(json_buf, op);
            msg.data = json_buf.str();
            d_img_update_publisher.publish(msg);
        }
    }
} 
