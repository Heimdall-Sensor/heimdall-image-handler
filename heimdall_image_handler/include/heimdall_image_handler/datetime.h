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
#ifndef __INCLUDED_HEIMDALL_DATETIME_H_
#define __INCLUDED_HEIMDALL_DATETIME_H_

#include <chrono>
#include <ctime>
#include <string>

namespace Heimdall
{
    class DateTime
    {
        public:
            typedef std::chrono::system_clock Clock;
            typedef std::chrono::time_point<Clock> TimePoint;
            typedef std::chrono::seconds Second;

            struct TimeData
            {
                uint32_t nanoseconds;
                uint16_t second;
                uint16_t minute;
                uint16_t hour;
                uint16_t day;
                uint16_t month;
                uint16_t year;
            };

        private:
            TimePoint d_time;
            Second d_seconds;
            std::time_t d_time_t;
            std::tm d_tm;

        public:
            DateTime(TimePoint const &time);

            uint32_t nanoseconds() const;
            uint32_t microseconds() const;
            uint16_t milliseconds() const;
            uint16_t second() const;
            uint16_t minute() const;
            uint16_t hour() const;
            uint16_t day() const;
            uint16_t month() const;
            uint16_t year() const; 

            TimeData disect() const;
            std::string format(std::string const &fmt) const;
    };

    inline DateTime::DateTime(DateTime::TimePoint const &time)
    :
        d_time(time),
        d_seconds(std::chrono::duration_cast<Second>(d_time.time_since_epoch())),
        d_time_t(Clock::to_time_t(d_time)),
        d_tm(*std::localtime(&d_time_t))
    {}

    inline uint32_t DateTime::nanoseconds() const
    {
        return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(d_time.time_since_epoch() - d_seconds).count());
    }

    inline uint32_t DateTime::microseconds() const
    {
        return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(d_time.time_since_epoch() - d_seconds).count());
    }

    inline uint16_t DateTime::milliseconds() const
    {
        return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(d_time.time_since_epoch() - d_seconds).count());
    }

    inline uint16_t DateTime::second() const
    {
        char buf[3];
        std::strftime(buf, sizeof(buf), "%S", &d_tm);
        return static_cast<uint16_t>(std::stoi(buf));
    }

    inline uint16_t DateTime::minute() const
    {
        char buf[3];
        std::strftime(buf, sizeof(buf), "%M", &d_tm);
        return static_cast<uint16_t>(std::stoi(buf));
    }

    inline uint16_t DateTime::hour() const
    {
        char buf[3];
        std::strftime(buf, sizeof(buf), "%H", &d_tm);
        return static_cast<uint16_t>(std::stoi(buf));
    }

    inline uint16_t DateTime::day() const
    {
        char buf[3];
        std::strftime(buf, sizeof(buf), "%d", &d_tm);
        return static_cast<uint16_t>(std::stoi(buf));
    }

    inline uint16_t DateTime::month() const
    {
        char buf[3];
        std::strftime(buf, sizeof(buf), "%m", &d_tm);
        return static_cast<uint16_t>(std::stoi(buf));
    }

    inline uint16_t DateTime::year() const
    {
        char buf[5];
        std::strftime(buf, sizeof(buf), "%Y", &d_tm);
        return static_cast<uint16_t>(std::stoi(buf));
    }

    inline DateTime::TimeData DateTime::disect() const
    {
        uint32_t ns = nanoseconds();
        uint16_t s = second();
        uint16_t mi = minute();
        uint16_t h = hour();
        uint16_t d = day();
        uint16_t mo = month();
        uint16_t y = year();
        return TimeData{ns, s, mi, h, d, mo, y};
    }

    inline std::string DateTime::format(std::string const &fmt) const
    {
        size_t length = fmt.length() * 2;
        char *buf = new char[length];

        std::strftime(buf, length, fmt.c_str(), &d_tm);
        std::string formatted(buf);
        delete [] buf;

        return formatted;
    }
}
#endif
