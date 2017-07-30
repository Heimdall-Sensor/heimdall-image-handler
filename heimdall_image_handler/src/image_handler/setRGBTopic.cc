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
     * Change the RGB topic. Will restart the subscriber if the RGB
     * topic is enabled.
     *
     * @param topic The new topic to subscribe to
     */
    void ImageHandler::setRGBTopic(std::string topic)
    {
        std::unique_lock<std::mutex> lock(d_rgb_mutex);
        if (d_rgb_topic == topic)
            return;

        if (d_rgb_enabled)
        {
            setRGBEnabled(false);
            d_rgb_topic = topic;
            setRGBEnabled(true);
        }
        else
            d_rgb_topic = topic;
    }
}
