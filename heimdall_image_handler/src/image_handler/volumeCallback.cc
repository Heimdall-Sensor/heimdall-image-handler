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
#include <cmath>

using namespace std::chrono;

namespace Heimdall
{
    void ImageHandler::volumeCallback(std_msgs::Float64MultiArray const &msg)
    {
        static const milliseconds threshold(5000);

        VolumeMeasurement measurement;
        measurement.received = steady_clock::now();
        measurement.data = std::move(msg.data);

        size_t mask_count = measurement.data.size();

        while (not d_recent_volumes.empty())
        {
            auto begin = d_recent_volumes.begin();
            if (begin->data.size() != mask_count)
            {
                ROS_INFO("Change in number of masks from %lu to %lu. Clearing volume log", begin->data.size(), mask_count);
                d_recent_volumes.clear();
                break;
            }

            // Remove outdated readings from the front
            timestamp_t &recv = begin->received;
            milliseconds elapsed = duration_cast<milliseconds>(measurement.received - recv);
            if (elapsed <= threshold)
                break;
            
            // Outdated, so remove
            d_recent_volumes.pop_front();
        }

        d_recent_volumes.push_back(std::move(measurement));
        ROS_DEBUG("Received volume reading - size of history: %lu", d_recent_volumes.size());

        setRateAdaptive();
    }
}
