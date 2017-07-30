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

namespace Heimdall
{
    /**
     * Set the state of the RGB subscriber.
     *
     * @param state The new state, enabled or disabled
     */
    void ImageHandler::setRGBEnabled(bool state)
    {
        std::unique_lock<std::mutex> lock(d_rgb_mutex);
        if (d_rgb_enabled != state)
        {
            if (d_rgb_capture != nullptr)
            {
                delete d_rgb_capture;
                d_rgb_capture = nullptr;
            }

            if (state)
            {
                ROS_INFO("Constructing the Video Capture");
                d_rgb_capture = new cv::VideoCapture(0);
                {
                    if (!d_rgb_capture->isOpened())
                    {
                        ROS_WARN("Failed to construct RGB Capture - relying on ROS Topic for RGB image");
                        d_rgb_subscriber = d_image_transport.subscribe(d_rgb_topic, 1, &ImageHandler::rgbCallback, this);
                        delete d_rgb_capture;
                        d_rgb_capture = nullptr;
                    }
                }
                d_rgb_enabled = true;
            }
            else
            {
                d_rgb_subscriber.shutdown();
                d_rgb_enabled = false;
            }
        }
    }
}
