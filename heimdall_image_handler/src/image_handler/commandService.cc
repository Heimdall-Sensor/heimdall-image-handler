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
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace Heimdall
{
    bool ImageHandler::commandService(heimdall_msgs::CommandSrv::Request &request, heimdall_msgs::CommandSrv::Response &response)
    {
        if (request.command == "switch_rgb")
            request.command = "{\"command\": \"switch_rgb\"}";
        else if (request.command == "switch_depth")
            request.command = "{\"command\": \"switch_depth\"}";

        ROS_INFO("Received command: %s", request.command.c_str());
        std::istringstream json_str(request.command);
        boost::property_tree::ptree json;
        boost::property_tree::json_parser::read_json(json_str, json);

        std::string cmd = json.get<std::string>("command");
        if (cmd == "switch_rgb")
        {
            size_t duration = json.get<size_t>("lock_duration", 15);
            ROS_INFO("Switching to RGB view, locking auto switch for %lu seconds", duration);
            d_auto_select_type = false;
            d_active_type = RGB;
            setDepthEnabled(false);
            setRGBRate(d_max_hz);
            d_active_lock_until = std::chrono::steady_clock::now() + std::chrono::seconds(duration);
        }
        else if (cmd == "switch_depth")
        {
            size_t duration = json.get<size_t>("lock_duration", 15);
            ROS_INFO("Switching to depth view, locking auto switch for %lu seconds", duration);
            d_auto_select_type = false;
            d_active_type = DEPTH;
            setDepthEnabled(true);
            setDepthRate(d_max_hz);
            d_active_lock_until = std::chrono::steady_clock::now() + std::chrono::seconds(duration);
        }
        else if (cmd == "enable_mask")
        {
            std::string mask = json.get<std::string>("label");
            enableMask(mask);
        }
        else if (cmd == "disable_mask")
        {
            std::string mask = json.get<std::string>("label");
            disableMask(mask);
        }
        else if (cmd == "get_enabled_masks")
        {
            boost::property_tree::ptree arr;
            for (auto ptr = d_masks.begin(); ptr != d_masks.end(); ++ptr)
            {
                boost::property_tree::ptree child;
                child.put("", *ptr);
                arr.push_back(std::make_pair("", child));
            }

            boost::property_tree::ptree op;
            op.add_child("enabled_masks", arr);

            std::ostringstream out;
            write_json(out, op);
            response.result = out.str();
        }
        else if (cmd == "set_rate")
        {
            double rate = json.get<double>("rate", 4.0);
            size_t duration = json.get<size_t>("lock_duration", 15);
            setRGBRate(rate);
            setDepthRate(rate);
            d_auto_select_type = true;
            d_active_lock_until = std::chrono::steady_clock::now() + std::chrono::seconds(duration);
            ROS_INFO("Set framerate to %lf Hz - locking for %lu seconds", rate, duration);
        }
        else if (cmd == "generate_video")
        {
            VideoRequest req;
            req.start = json.get<uint64_t>("start");
            req.end = json.get<uint64_t>("end");
            req.thumbnail = json.get<uint64_t>("thumbnail");                    
            req.name = json.get<std::string>("name", "notification");
            req.modified = std::chrono::steady_clock::now();

            storeVideoRequest(req);
            ROS_INFO("Stored video request");
        }
        else
            return false;

        return true;
    }
}
