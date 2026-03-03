/* kiko@idiospace.com 2021.01 */

/* Serial API
 *  1. int send(const std::string& s);
 *  @Brief: Send 'send_msg' to serial port
 *  @return: bytes sent to serial port (int Serial::serial_port)
 *
 *
 *  2. void recv(std::string& s);
 *  @Brief: Retrieve a at least one byte and store in string s
 *  @return: none
 *  @NOTICE: This method will block the process if reach no data!
 *
 *
 *  3. bool try_recv_for(std::string& s, int milli_secs = 3);
 *  @Brief:  wait up to mill_secs milliseconds (default 3 ms)
 *  @return: if retrieved at least one byte, then return true, otherwise return false.
 *  @Notice: This method does not block, good feature for games like RM.
 */
#pragma once

// toggle between linux and windows
#define LINUX_PORT  // WINDOWS_PORT LINUX_PORT
// #define SERIAL_DEBUG

// C library
// #include <stdio.h>
// #include <string.h>
// #include <iostream>
#include <chrono>

//-------------------------------------------------------------------------

#ifdef LINUX_PORT  // Linux headers
#include <string>
// #include <errno.h>    // Error integer and strerror() function
#include <fcntl.h>    // Contains file controls like open(), fcntl(), O_RDWR
#include <termios.h>  // Contains POSIX terminal control definitions
#include <unistd.h>   // write(), read(), close()
#endif                // LINUX_PORT

#ifdef WINDOWS_PORT  // Windows headers
#include <windows.h>
#endif  // WINDOWS_PORT

namespace hnurm
{

class Serial
{
protected:
    using Ms  = std::chrono::milliseconds;           // ms
    using clk = std::chrono::high_resolution_clock;  // clk

public:
    Serial()
    {
    }

    ~Serial();

    /**
     * @brief Set baud rate, parity, RTS/CTS, etc
     *        None block by default
     * @return true
     * @return false
     */
    bool init();

    /**
     * @brief Send 'send_msg' to serial port
     *
     * @param s
     * @return int
     */
    int send(const std::string &s) const;

    /**
     * @brief Retrieve at least one byte store in string s
     *
     * @param s
     */
    void recv(std::string &s);

    /**
     * @brief Try to retrive str from serial port and store in s for up to milli_secs
     * return true if at least one byte has been retrieved, otherwise return false
     *
     * @param s
     * @param milli_secs
     * @return true
     * @return false
     */
    bool try_recv_for(std::string &s, int milli_secs);

    /**
     * @brief
     *
     * @param dev_name
     * @return true
     * @return false
     */
    bool open_port(std::string dev_name);

private:
#ifdef LINUX_PORT
    struct termios tty;          // Create new termios struct, call it 'tty' for convention
    int            serial_port;  // A file descriptor for the opened serial port
#endif                           // LINUX_PORT

#ifdef WINDOWS_PORT
    HANDLE hcom;
#endif  // WINDOWS_PORT

    /**
     * @brief
     *
     */
    void close_port();

    /**
     * @brief
     *
     */
    void clear_input_buffer() const;  // Clear read buffer

    /**
     * @brief
     *
     */
    void clear_output_buffer() const;  // Clear write buffer

    /**
     * @brief
     *
     */
    void clear_input_output_buffer() const;  // Clear input & output buffer
};

}
