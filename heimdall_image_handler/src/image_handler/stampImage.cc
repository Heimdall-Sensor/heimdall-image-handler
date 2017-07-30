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
     * Callback for incoming rgb messages. If the interval since the previous
     * message has passed, the image will be stored.
     *
     * @param msg The received message
     */
    void ImageHandler::stampImage(cv::Mat &mat) const
    {
        // Add a timestamp
        datetime_t last = d_active_type == RGB ? d_last_rgb_time : d_last_depth_time;
        DateTime dt(last);
        std::string txt = dt.format("%Y-%m-%d %H:%M:%S");
        cv::putText(mat, txt, cv::Point(20, 20), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 0), 3);
        cv::putText(mat, txt, cv::Point(20, 20), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 255, 255));
    }
}
