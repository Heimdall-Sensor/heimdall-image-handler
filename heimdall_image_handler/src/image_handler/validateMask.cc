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

namespace Heimdall
{
    /**
     * Validate if the mask has appropriate dimensions
     *
     * @param reference The reference image that should match the mask
     */
    void ImageHandler::validateMask()
    {
        cv::Mat const &reference = d_active_type == RGB ? d_last_rgb : d_last_depth;
        if (reference.size() != d_mask.size())
        {
            d_mask = cv::Mat(reference.size(), CV_8UC3, CV_RGB(1, 1, 1));
            ROS_INFO("Overwriting d_mask with a new mask of the correct size!");
            d_mask_ok = false;
        }

        if (not d_mask_ok)
        {
            ROS_INFO("validateMask issues updateMask request");
            updateMask();
        }
    }
}
