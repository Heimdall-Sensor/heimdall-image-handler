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
#include <sstream>
#include <boost/format.hpp>
#include <heimdall_image_handler/datetime.h>
#include <glob.h>

#include <unistd.h>
#include <boost/thread.hpp>
using namespace std::chrono;

namespace Heimdall
{
    static bool isDigit(char ch)
    {
        return ch >= 48 && ch <= 57; 
    }

    /**
     * Check the frames received and garbage collect images
     */
    void ImageHandler::check()
    {
        timestamp_t now_st(steady_clock::now());

        // Check if a new image should be fetched
        if (d_rgb_enabled)
        {
            milliseconds since_last_img = duration_cast<milliseconds>(now_st - d_last_rgb_received);
            if (since_last_img >= d_rgb_target_interval)
                captureRGB();
        }

        // Check if a video should be generated
        if (not d_generate_video_requests.empty())
        {
            auto ptr = d_generate_video_requests.begin();
            milliseconds time_since_last_modified = duration_cast<milliseconds>(now_st - ptr->modified);
            if (time_since_last_modified.count() > 5000)
            {
                VideoRequest req = *ptr;
                d_generate_video_requests.erase(ptr);
                boost::thread myThread = boost::thread(boost::bind(&Heimdall::ImageHandler::generateVideo, this, req));
                myThread.detach();
                //generateVideo(req);
            }
        }

        // Limit execution
        milliseconds since_last = duration_cast<milliseconds>(now_st - d_last_gc);
        if (since_last.count() < 2000)
            return;

        // Update last GC action
        d_last_gc = now_st;

        // Check if a last RGB frame was received
        if (d_rgb_enabled)
        {
            milliseconds elapsed = duration_cast<milliseconds>(now_st - d_last_rgb_received);
            if (elapsed > 4 * d_rgb_target_interval)
                ROS_WARN("Last RGB image was received %5.3lf seconds ago!", elapsed.count() / 1000.0);
        }

        // Check if a last depth frame was received
        if (d_depth_enabled)
        {
            milliseconds elapsed = duration_cast<milliseconds>(now_st - d_last_rgb_received);
            if (elapsed > 4 * d_depth_target_interval)
                ROS_WARN("Last depth image was received %5.3lf seconds ago!", elapsed.count() / 1000.0);
        }

        // Check activity detection
        milliseconds since_last_activity = duration_cast<milliseconds>(now_st - d_last_activity_time);
        if (not d_recent_volumes.empty())
        {
            auto first = d_recent_volumes.rbegin();
            milliseconds since_last_volume = duration_cast<milliseconds>(now_st - first->received);
            if (since_last_volume < since_last_activity)
                since_last_activity = since_last_volume;
        }

        if (not d_auto_select_type && since_last_activity.count() > 10000)
        {
            milliseconds interval(250);
            int cur_ms = d_rgb_target_interval.count();
            if (interval < d_rgb_target_interval)
            {
                int ms = interval.count();
                ROS_WARN("No activity or volume reading received in the past 5 seconds. Increasing image interval to %d", ms);
                setRGBInterval(interval);
                setDepthInterval(interval);
            }
        }

        // Do garbage collection
        std::string full_prefix = d_image_path + "/" + d_image_prefix;
        std::string img_glob = full_prefix + "*.jpg";

        datetime_t now_r(system_clock::now());
        glob_t globbuf;
        int ret = glob(img_glob.c_str(), 0, NULL, &globbuf);
        if (ret == 0)
        {
            for (int i = 0; i < globbuf.gl_pathc; ++i)
            {
                std::string filename(globbuf.gl_pathv[i]);
                std::string suffix = filename.substr(full_prefix.length());
                if (suffix.length() < 11 || suffix[10] != '_')
                    continue;

                bool isNumeric = true;
                for (size_t strpos = 0; strpos < 10; ++strpos)
                {
                    if (!isDigit(suffix[strpos]))
                    {
                        isNumeric = false;
                        break;
                    }
                }

                if (not isNumeric)
                    continue;
                    
                // 10 digits followed by an underscore, must/should be a unix timestamp
                std::string epoch_str = suffix.substr(0, 10);
                std::tm tm = {};
                strptime(epoch_str.c_str(), "%s", &tm);
                datetime_t created = std::chrono::system_clock::from_time_t(std::mktime(&tm));

                std::string fmt = format(created);
                seconds elapsed = duration_cast<seconds>(now_r - created);
                if (elapsed > d_image_history_size)
                {
                    if (unlink(globbuf.gl_pathv[i]))
                        ROS_ERROR("Failed to remove file: %s", globbuf.gl_pathv[i]);
                    else
                        ROS_INFO("Removed file: %s", globbuf.gl_pathv[i]);
                }
            }
        }
        else if (ret != GLOB_NOMATCH)
            ROS_WARN("Glob returned %d", ret);

        globfree(&globbuf);

        // Force a mask regeneration
        d_mask_ok = false;
        updateMask();
    }
}
