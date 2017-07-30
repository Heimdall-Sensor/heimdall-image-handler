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

namespace Heimdall
{
    void ImageHandler::storeVideoRequest(VideoRequest req)
    {
        for (auto ptr = d_generate_video_requests.begin(); ptr != d_generate_video_requests.end(); ++ptr)
        {
            uint64_t st_diff = abs(req.start - ptr->start);
            uint64_t e_diff = abs(req.end - ptr->end);

            if (st_diff < 5 && e_diff < 5)
            {
                req.start = req.start < ptr->start ? req.start : ptr->start;
                req.end = req.end > ptr->end ? req.end : ptr->end;
                req.thumbnail = ptr->thumbnail;
                req.name = ptr->name;
                d_generate_video_requests.erase(ptr);
                ROS_INFO("Updated video request %lf - %lf", req.start, req.end);
                break;
            }
        }

        // Insert - the object has already been modified if it overlaps with another request
        d_generate_video_requests.insert(req);
    }
}
