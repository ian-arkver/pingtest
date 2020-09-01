/*
 * Copyright (C) 2020 Ian Jamison <ian.arkver.dev@gmail.com>
 *
 * Ping test for blue-pill USB Serial Comms
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the GNU
 * General Public License as published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * For a copy of the GNU General Public License see <https://www.gnu.org/licenses/>.
 */

#include <algorithm>
#include <chrono>
#include <iostream>

#include "asio.hpp"

/*
 * Set this to 1 for non-handshake comms test.
 * Set it to 0 to make the next command wait for the previous OK.
 */
#define ASYNC_CMD_OK 1

class pingtest
{
public:
    using clock = std::chrono::high_resolution_clock;

    pingtest(const char *_p)
        : sp(ioc)
    {
        std::error_code ec;

        cmd_count = 0;
        ok_count = 0;
        total_time = 0.0;
        max_time = 0.0;
        min_time = 1000.0;

        sp.open(_p, ec);
        if (ec || !sp.is_open())
        {
            std::cerr << "Can't open port " << _p << std::endl;
            return;
        }

        sp.set_option(asio::serial_port_base::baud_rate(1000000));
        sp.set_option(asio::serial_port_base::character_size()); // default 8
        sp.set_option(asio::serial_port_base::parity());         // default none
        sp.set_option(asio::serial_port_base::flow_control(
            asio::serial_port_base::flow_control::type::hardware));

        connected = true;
        do_write();
#if ASYNC_CMD_OK
        do_read();
#endif
    }

    ~pingtest()
    {
    }

    asio::io_context &get_ioc() { return ioc; }

    void do_write()
    {
        if (!connected)
            return;

        std::string cmd("G92 X0 Y0 Z0\r\n");

        asio::async_write(sp,
                          asio::buffer(cmd.data(), cmd.length()),
                          [this](std::error_code ec, std::size_t length) {
                              if (!ec)
                              {
                                  if (length != 14)
                                      std::cerr << "Got length " << length << std::endl;
#if ASYNC_CMD_OK
                                  if (++cmd_count < 10000)
                                  {
                                      // XXX: Slow down the writes here to allow the STM to keep up?
                                      std::this_thread::sleep_for(std::chrono::microseconds(500));
                                      do_write();
                                  }
                                  else
                                  {
                                      std::cout << "All commands sent" << std::endl;
                                  }

#else
                    wrt_time = clock::now();
                    do_read();
#endif
                              }
                              else
                              {
                                  std::cout << "Write error: " << ec << std::endl;
                                  asio::error_code ec;
                                  sp.close(ec);
                                  connected = false;
                              }
                          });
    }

    void do_read()
    {
        if (!connected)
            return;

        asio::async_read_until(
            sp,
            asio::dynamic_buffer(rbuf),
            '\n',
            [this](std::error_code ec, std::size_t length) {
                if (!ec)
                {
#if !ASYNC_CMD_OK
                    clock::time_point rd_time = clock::now();
                    std::chrono::duration<double> round_trip = std::chrono::duration_cast<std::chrono::duration<double>>(rd_time - wrt_time);
                    // std::cout
                    //     << "resp in " << round_trip.count() << " ns: " << rbuf << std::endl;
                    total_time += round_trip.count();
                    max_time = std::max(max_time, round_trip.count());
                    if (round_trip.count() < min_time)
                        min_time = round_trip.count();
#endif
                    // std::cout << "resp: " << rbuf << std::endl;
                    int okstate = 0;
                    for (const auto &ch : rbuf)
                    {
                        switch (okstate)
                        {
                        case 0:
                            if (ch == 'o')
                                okstate = 1;
                            break;

                        case 1:
                            if (ch == 'k')
                                okstate = 2;
                            else
                                okstate = 0;
                            break;

                        case 2:
                            if (ch == '\r')
                                ok_count++;
                            okstate = 0;
                            break;

                        default:
                            okstate = 0;
                            break;
                        }
                    }

                    rbuf.erase(0, length);
                    if (ok_count < 10000)
                    {
#if ASYNC_CMD_OK
                        do_read();
#else
                        do_write();
#endif
                    }
                    else
                    {
#if !ASYNC_CMD_OK
                        std::cout << "Sent " << ok_count << " commands in " << total_time << "s\n"
                                  << "Average: " << (total_time / (double)ok_count)
                                  << "  Min: " << min_time << "  Max: " << max_time << std::endl;
#endif

                        asio::error_code ec;
                        sp.close(ec);
                        connected = false;
                    }
                }
                else
                {
                    asio::error_code ec;
                    sp.close(ec);
                    connected = false;
                }
            });
    }

    int cmd_count, ok_count;

private:
    std::string port;

    asio::io_context ioc;
    asio::serial_port sp;
    bool connected;

    clock::time_point wrt_time;
    std::string rbuf;

    double total_time;
    double min_time, max_time;
};

int main(int argc, char *argv[])
{
    pingtest ping("/dev/ttyACM0");
    ping.get_ioc().run();
#if ASYNC_CMD_OK
    std::cout << "Sent " << ping.cmd_count << ", received " << ping.ok_count << std::endl;
#endif
    return 0;
}
