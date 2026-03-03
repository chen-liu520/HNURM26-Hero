#include "hnurm_uart/serial.h"
#include <cstring>

namespace hnurm
{

#ifdef LINUX_PORT

void Serial::clear_input_buffer() const
{
    tcflush(serial_port, TCIFLUSH);
}

void Serial::clear_output_buffer() const
{
    tcflush(serial_port, TCOFLUSH);
}

void Serial::clear_input_output_buffer() const
{
    tcflush(serial_port, TCIOFLUSH);
}

Serial::~Serial()
{
    close_port();
}

int Serial::send(const std::string &s) const
{
    // clear_output_buffer();  lose data, don't do this.
    return write(serial_port, s.c_str(), s.size());
}

void Serial::recv(std::string &s)
{
    s = try_recv_for(s, 0);
}

bool Serial::init()
{
    tty.c_cflag &= ~PARENB;   // Clear parity bit, disableing parity
    tty.c_cflag &= ~CSTOPB;   // Clear stop field, only use one stop bit, Otherwise two bits are used
    tty.c_cflag &= ~CSIZE;    // Clear all the size bits, then use one of the statements below
    tty.c_cflag |= CS8;       // Transmit 8 bits per byte across the serial port
    tty.c_cflag &= ~CRTSCTS;  // Disable RTS/CTS hardware flow control to avoid infinite waiting, RTS - request transmit
    tty.c_cflag |= CREAD | CLOCAL;           // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    tty.c_lflag &= ~ICANON;                  // Disable canonical mode
    tty.c_lflag &= ~ECHO;                    // Disable echo
    tty.c_lflag &= ~ECHOE;                   // Disable erasure
    tty.c_lflag &= ~ECHONL;                  // Disable new-line echo
    tty.c_lflag &= ~ISIG;                    // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // Turn off s/w flow ctrl
    tty.c_iflag &= ~(
        IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL
    );                      // Disable any special handling of received bytes
    tty.c_oflag &= ~OPOST;  // Prevent special interpretation of output bytes (eg: newline chars)
    tty.c_oflag &= ~ONLCR;  // Prevent conversion of newline to carriage return / line feed

    // READ SETTINGS: Only work at block mode
    tty.c_cc[VTIME] = 1;  // wait for up to 100ms between characters since first char has been reached
    tty.c_cc[VMIN]  = 1;  // wait for at least one character

    // Set in/out baud rate to be 115200
    // darwin 119200
    // otherwise 921600
#ifdef __APPLE__
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
#elif __linux__
    cfsetispeed(&tty, B921600);
    cfsetospeed(&tty, B921600);
#endif

    // Save tty settings, also checking for error
    if(tcsetattr(serial_port, TCSANOW, &tty) != 0)
    {
        printf("Error %i from tcsattr: %s\n", errno, strerror(errno));
        return false;
    }

    // Clear serial IO buffers
    clear_input_output_buffer();

    return true;
}

// dev_name, eg: "/dev/ttyUSB0" or "/dev/ttyS0", etc
bool Serial::open_port(std::string dev_name)
{
    // Open the serial port. Change device path as needed
    // Currently set to a standard FTDI USB-UART cable type device
    std::string device_name = dev_name;
    serial_port             = open(device_name.c_str(), O_RDWR | O_NONBLOCK);

    if(tcgetattr(serial_port, &tty) != 0)
    {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return false;
    }
    return true;
}

void Serial::close_port()
{
    try
    {
        clear_input_output_buffer();
        close(serial_port);
    }
    catch(...)
    {
    }
}

bool Serial::try_recv_for(std::string &s, int milli_secs)
{
    auto start = clk::now();

    s.clear();
    static char buf_[128];

    while(s.empty())
    {
        int num_bytes = read(serial_port, &buf_[0], sizeof(buf_));
        if(num_bytes > 0)
        {
            s = std::string(&buf_[0], num_bytes);
            return true;
        }

        auto end = clk::now();
        auto gap = std::chrono::duration_cast<Ms>(end - start);
        if(gap.count() > milli_secs && milli_secs != 0)
        {
            break;
        }
    }
    return false;
}

#endif  // LINUX_PORT

//----------------------------------------------------------------------------------------------------------

#ifdef WINDOWS_PORT

bool Serial::rm_init()
{
    // COMMTIMEOUTS time_sets;
    DCB rm_sets;
    rm_sets.BaudRate = 115'200;
    rm_sets.Parity   = NOPARITY;
    rm_sets.ByteSize = 8;
    rm_sets.StopBits = ONESTOPBIT;
    SetCommState(hcom, &rm_sets);

    COMMTIMEOUTS TimeOuts;
    TimeOuts.ReadIntervalTimeout         = 20;  // 读间隔超时
    TimeOuts.ReadTotalTimeoutMultiplier  = 10;  // 读时间系数
    TimeOuts.ReadTotalTimeoutConstant    = 20;  // 读时间常量
    TimeOuts.WriteTotalTimeoutMultiplier = 10;  // 写时间系数
    TimeOuts.WriteTotalTimeoutConstant   = 20;  // 写时间常量
    SetCommTimeouts(hcom, &TimeOuts);
}

int Serial::send(const std::string &s)
{
    long unsigned int num = 0;
    WriteFile(hcom, s.c_str(), s.size(), &num, NULL);
    return num;
}

void Serial::recv(std::string &s)
{
    try_recv_for(s, 0);  // block till reading something from port
}

bool Serial::try_recv_for(std::string &s, int milli_secs)
{
    auto start = clk::now();

    s.clear();
    char buf_[256];

    while(s.empty())
    {
        long unsigned int num_bytes;
        ReadFile(hcom, &buf_[0], sizeof(buf_), &num_bytes, NULL);

#ifdef SERIAL_DEBUG
        if(num_bytes > 0)
        {
            printf("Read %i bytes. Received message: %s\n", num_bytes, buf_);
        }
        else
        {
            printf("recv nothing\n");
        }
#endif  // SERIAL_DEBUG

        if(num_bytes > 0)
        {
            s = std::string(&buf_[0], num_bytes);
            return true;
        }

        auto end = clk::now();
        auto gap = std::chrono::duration_cast<Ms>(end - start);
        if(gap.count() > milli_secs && milli_secs != 0)
        {
            break;
        }
    }
    return false;
}

bool Serial::open_port(std::string dev_name)
{
    hcom = WINAPI CreateFile(
        dev_name.c_str(),              // port name
        GENERIC_READ | GENERIC_WRITE,  // Read/Write
        0,                             // No Sharing
        NULL,                          // No Security
        OPEN_EXISTING,                 // Open existing port only
        0,                             // Non Overlapped I/O
        NULL
    );  // Null for Comm Devices
    if(hcom == INVALID_HANDLE_VALUE)
    {
        std::cout << "Error in opening serial port" << std::endl;
        return false;
    }
    else
    {
        std::cout << "opening serial port successful" << std::endl;
        return true;
    }
}

void Serial::close_port()
{
    clear_input_output_buffer();
    CloseHandle(hcom);
}

void Serial::clear_input_buffer()
{
    fflush(stdin);
    PurgeComm(hcom, PURGE_RXCLEAR);
}

void Serial::clear_output_buffer()
{
    fflush(stdout);
    PurgeComm(hcom, PURGE_TXCLEAR);
}

void Serial::clear_input_output_buffer()
{
    fflush(stdout);
    fflush(stdin);
    PurgeComm(hcom, PURGE_TXCLEAR | PURGE_RXCLEAR);
}

Serial::~Serial()
{
    clear_input_output_buffer();
    close_port();
}

#endif  // WINDOWS_PORT

}  // namespace hnurm
