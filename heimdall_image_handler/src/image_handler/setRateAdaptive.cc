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
    void ImageHandler::setRateAdaptive()
    {
        static const milliseconds threshold(5000);

        timestamp_t now_st = steady_clock::now();

        // Don't change the rate while locked
        if (not d_auto_select_type)
            return;

        milliseconds activity_elapsed = duration_cast<milliseconds>(now_st - d_last_activity_time);
        double activity_hz = d_min_hz;
        bool recent_data = false;
        if (activity_elapsed <= threshold)
        {
            recent_data = true;
            activity_hz = 0.25 + 10 / (1 + exp(-(10 * d_last_activity) + 5));
        }

        // Suggest a rate based on recent change in volume
        double volume_hz = d_min_hz;
        if (not d_recent_volumes.empty())
        {
            size_t mask_count = d_recent_volumes.begin()->data.size();
            size_t history_size = d_recent_volumes.size();

            double *change = new double[mask_count];
            double *prev_value = new double[mask_count];
            for (size_t i = 0; i < mask_count; ++i)
            {
                change[i] = 0;
                prev_value[i] = 0;
            }

            size_t count = 0;
            for (auto ptr = d_recent_volumes.rbegin(); ptr != d_recent_volumes.rend(); ++ptr)
            {
                // Don't consider old changes
                milliseconds elapsed = duration_cast<milliseconds>(now_st - ptr->received);
                if (elapsed > threshold)
                    break;

                for (size_t mask_idx = 0; mask_idx < mask_count; ++mask_idx)
                {
                    double cur_val = ptr->data[mask_idx];
                    double prev_val = count == 0 ? ptr->data[mask_idx] : prev_value[mask_idx];
                    prev_value[mask_idx] = cur_val;
                    change[mask_idx] += (cur_val - prev_val);
                }
                ++count;
            }

            // Find the most active mask
            double max = 0;
            if (count > 0)
            {
                recent_data = true;
                for (size_t i = 0; i < mask_count; ++i)
                    max = change[i] > max ? change[i] : max;
    
                // Boost the rate when there's reasonable amount of movement
                if (max > 1)
                    max *= 2;
            }

            // Clean up
            delete [] change;
            delete [] prev_value;

            volume_hz = max;
        }

        double target_hz = std::max(activity_hz, volume_hz);

        // Bump up the rate a bit if there was no activity data received recently
        if (not recent_data)
        {
            target_hz = (d_max_hz - d_min_hz) / 2.0;
            ROS_WARN("No activity data has been received!");
        }

        target_hz = target_hz > d_max_hz ? d_max_hz : (target_hz < d_min_hz ? d_min_hz : target_hz);
        double cur_hz = 1000.0 / d_rgb_target_interval.count();

        milliseconds target_interval(static_cast<int>(round(1000.0 / target_hz)));
        if (abs(cur_hz - target_hz) > 0.25)
            ROS_INFO("Setting rate to %lf hz ", target_hz);
        else
            ROS_DEBUG("Setting rate to %lf hz", target_hz);

        setDepthInterval(target_interval);
        setRGBInterval(target_interval);
    }
}
