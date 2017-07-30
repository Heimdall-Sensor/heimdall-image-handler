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
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <heimdall_image_handler/datetime.h>

namespace Heimdall
{
    /**
     * Format a date string to use in the image file
     *
     * @param time The time to format
     * @param add_ms Whether to add the milliseconds or not
     */
    std::string ImageHandler::format(std::chrono::time_point<std::chrono::system_clock> const &time, bool add_ms)
    {
        Heimdall::DateTime dt(time);
        std::string fmt = dt.format("_%Y%m%d%H%M%S");

        if (not add_ms)
            return fmt;

        unsigned long epoch = std::chrono::duration_cast<std::chrono::seconds>(time.time_since_epoch()).count();
        std::ostringstream out;
        out << epoch << fmt << (boost::format("%03d") % dt.milliseconds());


        return out.str();
    }
}
