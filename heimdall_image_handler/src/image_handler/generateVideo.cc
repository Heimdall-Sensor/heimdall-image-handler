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
#include <cmath>
#include <glob.h>

namespace Heimdall
{
    static bool isDigit(char ch)
    {
        return ch >= 48 && ch <= 57; 
    }

    void ImageHandler::generateVideo(VideoRequest req)
    {
        ROS_INFO("Generating video from %lf to %lf", req.start, req.end);

        // Find images
        std::string full_prefix = d_image_path + "/" + d_image_prefix;
        std::string img_glob = full_prefix + "*.jpg";

        // Filename
        std::string video_file = (boost::format("%s/%s_%lu_%lu.avi") % d_image_path % req.name % req.start % req.end).str();
        std::string mp4_file = video_file;
        mp4_file.replace(video_file.length() - 3, 3, "mp4");

        ROS_INFO("Writing video to %s", video_file.c_str());

        // Create the writer
        cv::VideoWriter writer;
        auto fourcc = CV_FOURCC('P', 'I', 'M', '1');

        writer.open(video_file, fourcc, 25.0, cv::Size(640, 480), true);

        glob_t globbuf;
        int ret = glob(img_glob.c_str(), 0, NULL, &globbuf);
        std::string last_file;
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
                uint64_t epoch = atol(epoch_str.c_str());

                if (epoch >= req.start && epoch <= req.end)
                {
                    cv::Mat img = cv::imread(filename);
                    if (img.data == nullptr)
                        ROS_ERROR("Failed to load image %s", filename.c_str());
                    last_file = filename;

                    // write it 5 times to prolong the duration a bit
                    writer << img;
                    writer << img;
                    writer << img;
                    writer << img;
                    writer << img;
                    ROS_INFO("Added image %s", filename.c_str());
                }
            }
        }

        cv::Mat thumb_img = cv::imread(last_file);
        std::string thumb_file = video_file;
        thumb_file.replace(video_file.length() - 3, 3, "jpg");
        ROS_INFO("Wrote thumbnail to: %s", thumb_file.c_str());
        cv::imwrite(thumb_file, thumb_img, d_cv_imwrite_parameters);

        //std::string cmd = "/usr/bin/ffmpeg -i " + video_file + " -profile:v baseline -level 3.0 " + mp4_file;
        std::string cmd = "nice -n 19 /usr/bin/ffmpeg -y -i " + video_file
                        + " -c:v libx264 -crf 19 -preset slow -c:a libfdk_aac -b:a 192k -ac 2 -threads 1 "
                        + mp4_file;
        ROS_INFO_STREAM("command is \n %s" << cmd.c_str() << '\n');
        std::cout << cmd << '\n';
        system(cmd.c_str());
        ROS_INFO("Converted video file to MP4 in %s", mp4_file.c_str());
    }
}
