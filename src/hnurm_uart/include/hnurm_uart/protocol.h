/* kiko@idiospace.com */

// #define __packed __attribute__((packed))
#pragma once
#include <hnurm_interfaces/msg/vision_recv_data.hpp>
#include <hnurm_interfaces/msg/vision_send_data.hpp>
#include <stdint.h>
#include <string>

namespace hnurm
{

class Protocol
{

protected:
    static short constexpr PROTOCOL_CMD_ID = 0XA5;
    static short constexpr OFFSET_BYTE     = 8;

public:
    typedef struct
    {
        struct
        {
            uint8_t  sof;
            uint16_t data_length;
            uint8_t  crc_check;
        } header;

        uint16_t cmd_id;
        uint16_t frame_tail;
    } protocol;

    static std::string encode(const hnurm_interfaces::msg::VisionSendData &data);  // 尝试使用rosidl生成的消息类型
    // static std::string encode(const VisionSendData& data);   // Encode data from VisionSendData to std::string
    static bool
    decode(const std::string &s, hnurm_interfaces::msg::VisionRecvData &decoded_data);  // 尝试使用rosidl生成的消息类型
    // static bool decode(const std::string& s,VisionRecvData & decoded_data); // Decode data from std::string to
    // VisionRecvData

private:
    static uint16_t get_protocol_info_vision(uint8_t *rx_buf, uint16_t *flags_register, float *rx_data);

    /**
     * @brief generate data bytes for sending
     * @param send_id id to indicate vision's data, now set to 0x0001, 2 byte
     * @param flags_register a 2-byte register to store some flag, usually shoot/mode etc.
     * @param tx_data data ready to transmit, always float type
     * @param float_length length of tx_data
     * @param tx_buf start position of tx_buf, which is to store all bytes for send
     * @param tx_buf_len bytes, indicate the length of tx_buf
     */
    static void get_protocol_send_data_vision(
        uint16_t     send_id,
        uint16_t     flags_register,
        const float *tx_data,
        uint8_t      float_length,
        uint8_t     *tx_buf,
        uint16_t    *tx_buf_len
    );

    static uint8_t Get_CRC8_Check(uint8_t *pchMessage, uint16_t dwLength);

    static uint8_t CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength);

    static uint16_t Get_CRC16_Check(uint8_t *pchMessage, uint32_t dwLength);

    static uint16_t CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

    static uint8_t protocol_head_check_vision(protocol *pro, uint8_t *rx_buf);

    static float float_protocol(uint8_t *dat_t);
};

}
