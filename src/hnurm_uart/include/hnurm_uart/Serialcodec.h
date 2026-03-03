/**
 * @file Serialcodec.h
 * @author modify by NeoZng (neozng1@hnu.edu.cn)
 * @brief
 * @version 0.2
 * @date 2021-08-19
 *
 * @copyright Copyright HNUYueLuRM-Vision (c) 2021

 */

/* kiko@idispace.com 2021.01 */

// #define SERIAL_CODEC_DEBUG

/* SerialCodec API
 * 1. SerialCodec(int usb_id);
 * @Brief: open serial_port "/dev/ttyUSB(usb_id)"
 *
 * 2. SerialCode(std::string device_name);
 * @Brief: open serial_port "device_name"
 *
 * 3. bool try_get_recv_data_for(Protocol::VisionRecvData& recv_data, int milli_secs = 10);
 * @Brief: try to retrieve data from serial port within mill_secs milliseconds
 * @return: if data has been fetched and decoded properly, then return true, otherwise return false
 * @NOTICE: this method will block the process for at most mill_secs milliseconds
 *
 * 4. bool send_data(int cam_id, float pitch, float yaw, float distance);
 * @Brief: encode and send data to the serial port
 * @return: true if data sent succeed
 */
#pragma once

#include "hnurm_uart/protocol.h"
#include "hnurm_uart/serial.h"
// #include<opencv2/opencv.hpp>
#include <hnurm_interfaces/msg/vision_recv_data.hpp>
#include <hnurm_interfaces/msg/vision_send_data.hpp>
#include <rclcpp/rclcpp.hpp>

namespace hnurm
{

// Read & Write from or to the serial port
// Ensure integrity of receieved data pack thourgh this
// wrappered class combined with Serial and Protocol
/**
 * @brief class for read & write from or to the serial port
 */
class SerialCodec : public Serial, public Protocol
{
public:
    // explicit SerialCodec(const cv::FileNode &cfg_node);

    explicit SerialCodec(rclcpp::Node::SharedPtr node);

    void init_params(rclcpp::Node::SharedPtr node);

    void init_port();

    /**
     * @breif try to get a frame of data for vision
     * @param recv_data received data
     * @param milli_secs timeout, default 10ms (3ms is enough in test)
     * @return bool flag to indicate whether receiving worked fine
     */
    // bool try_get_recv_data_for(VisionRecvData &recv_data, int milli_secs = 10);

    bool try_get_recv_data_for(hnurm_interfaces::msg::VisionRecvData &recv_data, int milli_secs = 5);

    /**
     * @brief encode and send data to the serial port
     * @param state target state,
     * @param type target type
     * @param pitch
     * @param yaw
     * @return true if send successfully, false otherwise
     */
    // bool send_data(VisionSendData data);

    bool send_data(hnurm_interfaces::msg::VisionSendData data);

    /**
     * @brief set enemy color
     * @return true if succeeded, false otherwise
     */
    bool set_color();

    /**
     * @brief get enemy color, for Detectors' usage
     * @return 0 for red, 1 for blue
     */
    [[nodiscard]] int get_color() const;

    /**
     * @brief get speed of a received data frame
     * @param recvData
     * @return speed, m/s
     */
    //        double get_speed(VisionRecvData &recvData);

public:
    float channel_delay = 0;

private:
    int            _enemy_color = -1;
    rclcpp::Logger logger       = rclcpp::get_logger("Serial");
};

}  // namespace hnurm
