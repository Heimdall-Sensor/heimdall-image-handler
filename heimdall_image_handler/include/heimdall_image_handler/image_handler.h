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
#ifndef __INCLUDED_HEIMDALL_IMAGE_HANDLER_H_
#define __INCLUDED_HEIMDALL_IMAGE_HANDLER_H_
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <chrono>
#include <mutex>
#include <deque>
#include <set>

#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <heimdall_msgs/CommandSrv.h>

// Forward declaration
namespace cv
{
    class VideoCapture;
}

namespace Heimdall
{ 
    class ImageHandler
    {
        private:
            typedef std::chrono::time_point<std::chrono::steady_clock> timestamp_t;
            typedef std::chrono::time_point<std::chrono::system_clock> datetime_t;

            enum IMGType 
            {
                RGB,
                DEPTH
            };

            struct VideoRequest
            {
                uint64_t start;
                uint64_t end;
                uint64_t thumbnail;
                timestamp_t modified;
                std::string name;

                bool operator<(VideoRequest const &rhs) const
                {
                    if (start == rhs.start)
                        return end < rhs.end;
                    return start < rhs.start;
                }
            };

            struct VolumeMeasurement
            {
                std::vector<double> data;
                timestamp_t received;
            };

            // General data members
            ros::NodeHandle d_node_handle;
            image_transport::ImageTransport d_image_transport;
            ros::Subscriber d_activity_subscriber;
            ros::Subscriber d_volume_subscriber;
            ros::ServiceServer d_command_service;
            std::string d_image_path;
            std::string d_image_prefix;

            ros::Publisher d_img_update_publisher;
            
            // Activity measurement
            double d_last_activity;
            timestamp_t d_last_activity_time;
            std::deque<VolumeMeasurement> d_recent_volumes;
            
            // Determine the type of the active vision, saved to frame "last"
            IMGType d_active_type;
            bool d_mask_ok;
            cv::Mat d_mask;
            std::mutex d_mask_mutex;
            std::mutex d_mask_update_mutex;
            std::set<std::string> d_masks;
            bool d_auto_select_type;
            timestamp_t d_active_lock_until;

            // Last received RGB image
            cv::Mat d_last_rgb;

            // RGB image adminstration
            cv::VideoCapture *d_rgb_capture;
            image_transport::Subscriber d_rgb_subscriber;
            bool d_rgb_enabled;
            std::string d_rgb_topic;
            std::mutex d_rgb_mutex;
            timestamp_t d_last_rgb_received;
            datetime_t d_last_rgb_time;
            std::chrono::milliseconds d_rgb_target_interval;

            // Last received depth image
            cv::Mat d_last_depth;

            // Depth image adminstration
            image_transport::Subscriber d_depth_subscriber;
            bool d_depth_enabled;
            std::string d_depth_topic;
            std::mutex d_depth_mutex;
            timestamp_t d_last_depth_received;
            datetime_t d_last_depth_time;
            std::chrono::milliseconds d_depth_target_interval;

            // Video generation
            std::set<VideoRequest> d_generate_video_requests;

            // Parameters
            double d_min_hz;
            double d_max_hz;
            std::vector<int> d_cv_imwrite_parameters;
            std::chrono::seconds d_image_history_size;

            timestamp_t d_last_gc;

        public:
            /**
             * The constructor requires a ROS Node handle, a topic for RGB and a topic for depth
             *
             * @param nh The ROS node handle
             * @param rgb_topic The RGB topic to subscribe to
             * @param depth_topic the depth topic to subscribe to
             */
            ImageHandler(ros::NodeHandle &nh, std::string const &rgb_topic, std::string const &depth_topic);

            /**
             * Set the state of the RGB subscriber.
             *
             * @param state The new state, enabled or disabled
             */
            void setRGBEnabled(bool state);

            /**
             * Set the state of the depth subscriber.
             *
             * @param state The new state, enabled or disabled
             */
            void setDepthEnabled(bool state);

            /**
             * Destructor
             */
            ~ImageHandler();

            /**
             * Validate that the configured image path exists or can be created, and
             * is writable. If it is not writable, an exception will be thrown.
             */
            void checkPath();

            /**
             * Change the RGB topic. Will restart the subscriber if the RGB
             * topic is enabled.
             *
             * @param topic The new topic to subscribe to
             */
            void setRGBTopic(std::string topic);

            /**
             * Change the depth topic. Will restart the subscriber if the depth
             * topic is enabled.
             *
             * @param topic The new topic to subscribe to
             */
            void setDepthTopic(std::string topic);

            /**
             * Callback for incoming rgb messages. If the interval since the previous
             * message has passed, the image will be stored.
             *
             * @param msg The received message
             */
            void rgbCallback(sensor_msgs::ImageConstPtr const &msg);

            /**
             * Capture a RGB image from a USB camera
             */
            void captureRGB();

            /**
             * Callback for incoming depth messages. If the interval since the previous
             * message has passed, the image will be converted from 16UC1 to BGR8.
             *
             * @param msg The received message
             */
            void depthCallback(sensor_msgs::ImageConstPtr const &msg);

            /**
             * Callback for incoming activity measurement
             */
            void activityCallback(std_msgs::Float64 const &msg);

            /**
             * Callback for incoming volume measurements
             */
            void volumeCallback(std_msgs::Float64MultiArray const &msg);

            /**
             * Service for changing the behaviour of the node
             */
            bool commandService(heimdall_msgs::CommandSrv::Request &request, heimdall_msgs::CommandSrv::Response &response);

            /**
             * Format a date string to use in the image file
             *
             * @param time The time to format
             * @param add_ms Whether to add the milliseconds or not
             */
            std::string format(datetime_t const &time, bool add_ms = true);


            /**
             * Set the rate based on recent activity. Should be called after
             * each incoming activity measurement
             */
            void setRateAdaptive();

            /**
             * Set the RGB interval in milliseconds
             *
             * @param interval The number of milliseconds between each RGB frame
             */
            void setRGBInterval(std::chrono::milliseconds interval);

            /**
             * Set the RGB interval by a rate in Hz
             *
             * @param rate The number of frames per second. Can be less than 1.
             */
            void setRGBRate(double rate);

            /**
             * Get the RGB interval in milliseconds
             *
             * @return The number of milliseconds between each RGB frame
             */
            std::chrono::milliseconds getRGBInterval();

            /**
             * Get the RGB interval as a rate in Hz
             *
             * @return The number of RGB frames per second. Can be less than 1
             */
            double getRGBRate();

            /**
             * Set the depth interval in milliseconds
             *
             * @param interval The number of milliseconds between each frame
             */
            void setDepthInterval(std::chrono::milliseconds interval);

            /**
             * Set the depth interval as a rate in Hz
             *
             * @param rate The number of depth frames per seond. Can be less than 1
             */
            void setDepthRate(double rate);

            /**
             * Get the depth interval in milliseconds
             *
             * @return The number of milliseonds between each frame
             */
            std::chrono::milliseconds getDepthInterval();

            /**
             * Get the depth interval as a rate in Hz
             *
             * @return The number of depth frames per second
             */
            double getDepthRate();

            /**
             * Validate if the mask has appropriate dimensions
             *
             * @param reference The reference image that should match the mask
             */
            void validateMask();

            /**
             * Add a mask to the lsit of applied / enabled masks
             *
             * @param name The mask to add
             */
            void enableMask(std::string const &name);

            /**
             * Remove a mask from the list of applied / enabled masks
             *
             * @param name The mask to remove
             */
            void disableMask(std::string const &name);

            /**
             * Update the mask applied to output files.
             *
             * @param reference The reference image which dimensions should match the mask
             */
            void updateMask();

            /**
             * Perform the mask update asynchronously. Should not be called
             * directly - updateMask calls this in a new thread
             */
            void updateMaskThread();

            /**
             * Change the active view of the camera, determining if either the RGB
             * or the depth stream is visible
             */
            void setActiveType(IMGType type);

            /**
             * Write the last frame to the disk, if the type matches the currently active output type
             * @param update_type The type of image supplied, either RGB or Depth
             * @param img The image that should be written, after applying the masks
             */
            void updateLastFrame(IMGType update_type);

            /**
             * Store the video request, or update it
             */
            void storeVideoRequest(VideoRequest req);

            /**
             * Generate a video based on the request
             * @param req The request to handle
             */
            void generateVideo(VideoRequest req);

            /**
             * Should be called periodically (once a second) to check status
             */
            void check();

            /**
             * Clamp the value
             */
            double clamp(double val, double min, double max) const;

            // Add a timestamp to the image
            void stampImage(cv::Mat &image) const;
    };

    /**
     * Change the active view of the camera, determining if either the RGB
     * or the depth stream is visible
     */
    inline void ImageHandler::setActiveType(IMGType type)
    {
        d_active_type = type;
    }

    /**
     * Get the depth interval in milliseconds
     *
     * @return The number of milliseonds between each frame
     */
    inline std::chrono::milliseconds ImageHandler::getDepthInterval()
    {
        return d_depth_target_interval;
    }

    /**
     * Get the depth interval as a rate in Hz
     *
     * @return The number of depth frames per second
     */
    inline double ImageHandler::getDepthRate()
    {
        return 1000.0 / d_depth_target_interval.count();
    }

    /**
     * Set the depth interval in milliseconds
     *
     * @param interval The number of milliseconds between each frame
     */
    inline void ImageHandler::setDepthInterval(std::chrono::milliseconds interval)
    {
        d_depth_target_interval = interval;
    }

    /**
     * Clamp the value to a valid range
     * @param val The value to clamp
     * @param min The minimum value
     * @param max The maximum value
     */
    inline double ImageHandler::clamp(double val, double min, double max) const
    {
        return val < min ? min : (val > max ? max : val);
    }

    /**
     * Set the depth interval as a rate in Hz
     *
     * @param rate The number of depth frames per seond. Can be less than 1
     */
    inline void ImageHandler::setDepthRate(double rate)
    {
        rate = clamp(rate, d_min_hz, d_max_hz);
        ROS_INFO("Setting depth rate to %lf Hz", rate);
        d_depth_target_interval = std::chrono::milliseconds(static_cast<int>(round(1000.0 / rate)));
    }

    /**
     * Set the RGB interval in milliseconds
     *
     * @param interval The number of milliseconds between each RGB frame
     */
    inline void ImageHandler::setRGBInterval(std::chrono::milliseconds interval)
    {
        d_rgb_target_interval = interval;
    }

    /**
     * Set the RGB interval by a rate in Hz
     *
     * @param rate The number of frames per second. Can be less than 1.
     */
    inline void ImageHandler::setRGBRate(double rate)
    {
        rate = clamp(rate, d_min_hz, d_max_hz);
        ROS_INFO("Setting RGB rate to %lf Hz", rate);
        d_rgb_target_interval = std::chrono::milliseconds(static_cast<int>(round(1000.0 / rate)));
    }

    /**
     * Get the RGB interval in milliseconds
     *
     * @return The number of milliseconds between each RGB frame
     */
    inline std::chrono::milliseconds ImageHandler::getRGBInterval()
    {
        return d_rgb_target_interval;
    }

    /**
     * Get the RGB interval as a rate in Hz
     *
     * @return The number of RGB frames per second. Can be less than 1
     */
    inline double ImageHandler::getRGBRate()
    {
        return 1000.0 / d_rgb_target_interval.count();
    }
}
#endif
