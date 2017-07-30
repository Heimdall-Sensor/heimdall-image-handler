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
#include <boost/filesystem.hpp>
#include <fstream>
#include <stdexcept>

namespace Heimdall
{
    /**
     * Validate that the configured image path exists or can be created, and
     * is writable. If it is not writable, an exception will be thrown.
     */
    void ImageHandler::checkPath()
    {
        boost::filesystem::path img_path(d_image_path);

        // Check if the path exists
        if (!boost::filesystem::exists(img_path))
        {
            try
            {
                boost::filesystem::create_directories(img_path);
                ROS_INFO("Created image path: %s", d_image_path.c_str());
            }
            catch (boost::filesystem::filesystem_error const &e)
            {
                throw std::runtime_error("Image path does not exist and could not be created: " + d_image_path);
            }
        }

        // Check if the target is a directory
        if (!boost::filesystem::is_directory(img_path))
            throw std::runtime_error("Image path is not a directory: " + d_image_path);

        // Check if the path is writable by attempting to write a file
        boost::filesystem::path test_file(img_path);
        test_file += boost::filesystem::path("/" + d_image_prefix + "test.jpg");

        std::ofstream test(test_file.native());
        if (!test.is_open())
            throw std::runtime_error("Image path is not writable: " + d_image_path);

        test << "test\n";
        test.flush();
        test.close();

        if (test.bad())
            throw std::runtime_error("Image path is not writable: " + d_image_path);

        unlink(test_file.c_str());
    }
}
