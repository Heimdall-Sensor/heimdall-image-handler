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
     * Set the state of the depth subscriber.
     *
     * @param state The new state, enabled or disabled
     */
    void ImageHandler::setDepthEnabled(bool state)
    {
        std::unique_lock<std::mutex> lock(d_depth_mutex);
        if (d_depth_enabled != state)
        {
            if (state)
            {
                d_depth_subscriber = d_image_transport.subscribe(d_depth_topic, 1, &ImageHandler::depthCallback, this);
                d_depth_enabled = true;
                ROS_INFO("Enabling depth");
            }
            else
            {
                d_depth_subscriber.shutdown();
                d_depth_enabled = false;
                ROS_INFO("Disabling depth");
            }
        }
    }
}
