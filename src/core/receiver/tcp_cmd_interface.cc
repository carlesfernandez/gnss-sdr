/*!
 * \file tcp_cmd_interface.cc
 * \brief Class that implements a TCP/IP telecommand command line interface
 * for GNSS-SDR
 * \author Javier Arribas jarribas (at) cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "tcp_cmd_interface.h"
#include "command_event.h"
#include "pvt_interface.h"
#include <boost/asio.hpp>
#include <cmath>      // for isnan
#include <exception>  // for exception
#include <iomanip>    // for setprecision
#include <sstream>    // for stringstream
#include <utility>    // for move

#if USE_BOOST_ASIO_IO_CONTEXT
using b_io_context = boost::asio::io_context;
#else
using b_io_context = boost::asio::io_service;
#endif

TcpCmdInterface::TcpCmdInterface()
    : rx_latitude_(0.0),
      rx_longitude_(0.0),
      rx_altitude_(0.0),
      receiver_utc_time_(0),
      keep_running_(true)
{
    register_functions();
    control_queue_ = nullptr;
}


void TcpCmdInterface::register_functions()
{
#if HAS_GENERIC_LAMBDA
    functions_["status"] = [&](auto &s) { return TcpCmdInterface::status(s); };
    functions_["standby"] = [&](auto &s) { return TcpCmdInterface::standby(s); };
    functions_["reset"] = [&](auto &s) { return TcpCmdInterface::reset(s); };
    functions_["hotstart"] = [&](auto &s) { return TcpCmdInterface::hotstart(s); };
    functions_["warmstart"] = [&](auto &s) { return TcpCmdInterface::warmstart(s); };
    functions_["coldstart"] = [&](auto &s) { return TcpCmdInterface::coldstart(s); };
    functions_["set_ch_satellite"] = [&](auto &s) { return TcpCmdInterface::set_ch_satellite(s); };
#else
    functions_["status"] = std::bind(&TcpCmdInterface::status, this, std::placeholders::_1);
    functions_["standby"] = std::bind(&TcpCmdInterface::standby, this, std::placeholders::_1);
    functions_["reset"] = std::bind(&TcpCmdInterface::reset, this, std::placeholders::_1);
    functions_["hotstart"] = std::bind(&TcpCmdInterface::hotstart, this, std::placeholders::_1);
    functions_["warmstart"] = std::bind(&TcpCmdInterface::warmstart, this, std::placeholders::_1);
    functions_["coldstart"] = std::bind(&TcpCmdInterface::coldstart, this, std::placeholders::_1);
    functions_["set_ch_satellite"] = std::bind(&TcpCmdInterface::set_ch_satellite, this, std::placeholders::_1);
#endif
}


void TcpCmdInterface::set_pvt(std::shared_ptr<PvtInterface> PVT_sptr)
{
    PVT_sptr_ = std::move(PVT_sptr);
}


time_t TcpCmdInterface::get_utc_time() const
{
    return receiver_utc_time_;
}


std::array<float, 3> TcpCmdInterface::get_LLH() const
{
    return std::array<float, 3>{rx_latitude_, rx_longitude_, rx_altitude_};
}


std::string TcpCmdInterface::reset(const std::vector<std::string> &commandLine __attribute__((unused)))
{
    std::string response;
    if (control_queue_ != nullptr)
        {
            const command_event_sptr new_evnt = command_event_make(200, 1);  // send the restart message (who=200,what=1)
            control_queue_->push(pmt::make_any(new_evnt));
            response = "OK\n";
        }
    else
        {
            response = "ERROR\n";
        }
    return response;
}


std::string TcpCmdInterface::standby(const std::vector<std::string> &commandLine __attribute__((unused)))
{
    std::string response;
    if (control_queue_ != nullptr)
        {
            const command_event_sptr new_evnt = command_event_make(300, 10);  // send the standby message (who=300,what=10)
            control_queue_->push(pmt::make_any(new_evnt));
            response = "OK\n";
        }
    else
        {
            response = "ERROR\n";
        }
    return response;
}


std::string TcpCmdInterface::status(const std::vector<std::string> &commandLine __attribute__((unused)))
{
    std::stringstream str_stream;
    // todo: implement the receiver status report

    //    str_stream << "-------------------------------------------------------\n";
    //    str_stream << "ch | sys | sig | mode | Tlm | Eph | Doppler | CN0 |\n";
    //    str_stream << "   |     |     |      |     |     |  [Hz]   | [dB - Hz] |\n";
    //    str_stream << "-------------------------------------------------------\n";
    //    int n_ch = 10;
    //    for (int n = 0; n < n_ch; n++)
    //        {
    //            str_stream << n << "GPS | L1CA | TRK | YES | YES | 23412.4 | 44.3 |\n";
    //        }
    //    str_stream << "--------------------------------------------------------\n";

    double longitude_deg;
    double latitude_deg;
    double height_m;
    double ground_speed_kmh;
    double course_over_ground_deg;
    time_t UTC_time;
    if (PVT_sptr_->get_latest_PVT(&longitude_deg,
            &latitude_deg,
            &height_m,
            &ground_speed_kmh,
            &course_over_ground_deg,
            &UTC_time) == true)
        {
            struct tm tstruct{};
            std::array<char, 80> buf1{};
            tstruct = *gmtime(&UTC_time);
            strftime(buf1.data(), buf1.size(), "%d/%m/%Y %H:%M:%S", &tstruct);
            const std::string str_time = std::string(buf1.data());
            str_stream << "- Receiver UTC Time: " << str_time << '\n';
            str_stream << std::setprecision(9);
            str_stream << "- Receiver Position WGS84 [Lat, Long, H]: "
                       << latitude_deg << ", "
                       << longitude_deg << ", ";
            str_stream << std::setprecision(3);
            str_stream << height_m << '\n';
            str_stream << std::setprecision(1);
            str_stream << "- Receiver Speed over Ground [km/h]: " << ground_speed_kmh << '\n';
            str_stream << "- Receiver Course over ground [deg]: " << course_over_ground_deg << '\n';
        }
    else
        {
            str_stream << "No PVT information available.\n";
        }

    return str_stream.str();
}


std::string TcpCmdInterface::hotstart(const std::vector<std::string> &commandLine)
{
    std::string response;
    if (commandLine.size() > 5)
        {
            // Read commandline time parameter
            struct tm tm{};
            const std::string tmp_str = commandLine.at(1) + commandLine.at(2);
            if (strptime(tmp_str.c_str(), "%d/%m/%Y %H:%M:%S", &tm) == nullptr)
                {
                    response = "ERROR: time parameter malformed\n";
                    return response;
                }
            receiver_utc_time_ = timegm(&tm);

            // Read latitude, longitude, and height
            rx_latitude_ = std::stof(commandLine.at(3).c_str());
            rx_longitude_ = std::stof(commandLine.at(4).c_str());
            rx_altitude_ = std::stof(commandLine.at(5).c_str());

            if (std::isnan(rx_latitude_) || std::isnan(rx_longitude_) || std::isnan(rx_altitude_))
                {
                    response = "ERROR: position malformed\n";
                }
            else
                {
                    if (control_queue_ != nullptr)
                        {
                            const command_event_sptr new_evnt = command_event_make(300, 12);  // send the standby message (who=300,what=12)
                            control_queue_->push(pmt::make_any(new_evnt));
                            response = "OK\n";
                        }
                    else
                        {
                            response = "ERROR\n";
                        }
                }
        }
    else
        {
            response = "ERROR: time parameter not found, please use hotstart %d/%m/%Y %H:%M:%S Lat Long H\n";
        }
    return response;
}


std::string TcpCmdInterface::warmstart(const std::vector<std::string> &commandLine)
{
    std::string response;
    if (commandLine.size() > 5)
        {
            // Read commandline time parameter
            struct tm tm{};
            const std::string tmp_str = commandLine.at(1) + commandLine.at(2);
            if (strptime(tmp_str.c_str(), "%d/%m/%Y %H:%M:%S", &tm) == nullptr)
                {
                    response = "ERROR: time parameter malformed\n";
                    return response;
                }
            receiver_utc_time_ = timegm(&tm);

            // Read latitude, longitude, and height
            rx_latitude_ = std::stof(commandLine.at(3).c_str());
            rx_longitude_ = std::stof(commandLine.at(4).c_str());
            rx_altitude_ = std::stof(commandLine.at(5).c_str());

            if (std::isnan(rx_latitude_) || std::isnan(rx_longitude_) || std::isnan(rx_altitude_))
                {
                    response = "ERROR: position malformed\n";
                }
            else
                {
                    if (control_queue_ != nullptr)
                        {
                            const command_event_sptr new_evnt = command_event_make(300, 13);  // send the warmstart message (who=300,what=13)
                            control_queue_->push(pmt::make_any(new_evnt));
                            response = "OK\n";
                        }
                    else
                        {
                            response = "ERROR\n";
                        }
                }
        }
    else
        {
            response = "ERROR: time parameter not found, please use warmstart %d/%m/%Y %H:%M:%S Lat Long H\n";
        }
    return response;
}


std::string TcpCmdInterface::coldstart(const std::vector<std::string> &commandLine __attribute__((unused)))
{
    std::string response;
    if (control_queue_ != nullptr)
        {
            const command_event_sptr new_evnt = command_event_make(300, 11);  // send the coldstart message (who=300,what=11)
            control_queue_->push(pmt::make_any(new_evnt));
            response = "OK\n";
        }
    else
        {
            response = "ERROR\n";
        }

    return response;
}


std::string TcpCmdInterface::set_ch_satellite(const std::vector<std::string> &commandLine __attribute__((unused)))
{
    std::string response;
    // todo: implement the set satellite command
    response = "Not implemented\n";
    return response;
}


void TcpCmdInterface::set_msg_queue(std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> control_queue)
{
    control_queue_ = std::move(control_queue);
}


void TcpCmdInterface::run_cmd_server(int tcp_port)
{
    // Get the port from the parameters
    const uint16_t port = tcp_port;

    // Error to not throw exception
    boost::system::error_code not_throw;

    // Socket and acceptor
    b_io_context context;
    try
        {
            boost::asio::ip::tcp::acceptor acceptor(context, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port));

            while (keep_running_)
                {
                    try
                        {
                            std::cout << "TcpCmdInterface: Telecommand TCP interface listening on port " << tcp_port << '\n';

                            boost::asio::ip::tcp::socket socket(context);
                            acceptor.accept(socket, not_throw);  // NOLINT(bugprone-unused-return-value)
                            if (not_throw)
                                {
                                    std::cerr << "TcpCmdInterface: Error when binding the port to the socket: " << not_throw.message() << '\n';
                                    continue;
                                }

                            // Read a message
                            boost::system::error_code error = boost::asio::error::eof;
                            do
                                {
                                    std::string response;
                                    boost::asio::streambuf b;
                                    if (boost::asio::read_until(socket, b, '\n', error) == 0)
                                        {
                                            std::cerr << "TcpCmdInterface: Error reading messages: " << error.message() << '\n';
                                        }
                                    std::istream is(&b);
                                    std::string line;
                                    std::getline(is, line);
                                    std::istringstream iss(line);
                                    const std::vector<std::string> cmd_vector(std::istream_iterator<std::string>{iss},
                                        std::istream_iterator<std::string>());

                                    if (!cmd_vector.empty())
                                        {
                                            try
                                                {
                                                    if (cmd_vector.at(0) == "exit")
                                                        {
                                                            error = boost::asio::error::eof;
                                                            // send cmd response
                                                            if (socket.write_some(boost::asio::buffer("OK\n"), not_throw) == 0)
                                                                {
                                                                    std::cerr << "Error: 0 bytes sent in cmd response\n";
                                                                }
                                                        }
                                                    else
                                                        {
                                                            response = functions_[cmd_vector.at(0)](cmd_vector);
                                                        }
                                                }
                                            catch (const std::bad_function_call &ex)
                                                {
                                                    response = "ERROR: command not found \n ";
                                                }
                                            catch (const std::exception &ex)
                                                {
                                                    response = "ERROR: command execution error: " + std::string(ex.what()) + "\n";
                                                }
                                        }
                                    else
                                        {
                                            response = "ERROR: empty command\n";
                                        }

                                    // send cmd response
                                    if (socket.write_some(boost::asio::buffer(response), not_throw) == 0)
                                        {
                                            std::cerr << "Error: 0 bytes sent in cmd response\n";
                                        }
                                    if (not_throw)
                                        {
                                            std::cerr << "Error sending(" << not_throw.value() << "): " << not_throw.message() << '\n';
                                            break;
                                        }
                                }
                            while (error != boost::asio::error::eof);

                            if (error == boost::asio::error::eof)
                                {
                                    std::cerr << "TcpCmdInterface: EOF detected\n";
                                }
                            else
                                {
                                    std::cerr << "TcpCmdInterface unexpected error: " << error << '\n';
                                }

                            // Close socket
                            socket.close();
                        }
                    catch (const boost::exception &e)
                        {
                            std::cerr << "TcpCmdInterface: Boost exception\n";
                        }
                    catch (const std::exception &ex)
                        {
                            std::cerr << "TcpCmdInterface: Exception " << ex.what() << '\n';
                        }
                }
        }
    catch (const boost::exception &e)
        {
            std::cerr << "TCP Command Interface exception: address already in use\n";
        }
}
