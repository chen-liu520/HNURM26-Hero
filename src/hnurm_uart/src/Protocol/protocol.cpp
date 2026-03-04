/*
    @SEASKY---2020/09/05
*/

/*
    kiko@idiospace.com --- 2021.01
*/

#include "hnurm_uart/protocol.h"
#include "hnurm_uart/bsp_crc16.h"
#include "hnurm_uart/bsp_crc8.h"
#include <cstring>

namespace hnurm
{

/**
@param: VisionSendData
    int cam_id;     // formatted in lower 8 bits of flag_register, higher 8 bits for further exploration
    float pitch;    // formatted in float data
    float yaw;      // formatted in float data
    float distance; // formatted in float data

@return: formatted std::string
        which will then be send to STM32 through Serial::send(const std::string& s)
*/
//     std::string Protocol::encode(const VisionSendData &data)
//     {
//         static char ch_arr[255];
//         static float f_arr[2];
//         static uint16_t tx_buf_len;
//         static std::string s = ""; // restoration of modes
//         static uint16_t bits_flags = 0x00;

//         bits_flags = 0x00;
// //        bits_flags |= ((int) data.type << 4); // low-high 4bits for target type
//         bits_flags |= (int) data.state;           // low-low 4bits for target state

//         f_arr[0] = data.pitch, f_arr[1] = data.yaw;

//         get_protocol_send_data_vision(0X0001,                    // vision data identifier(frame head)
//                                       bits_flags,              // bits flags
//                                       f_arr,                   // start pos of float array
//                                       2,                       // pitch, yaw
//                                       (unsigned char *) ch_arr, // start pos of formatted send buffer
//                                       &tx_buf_len);            // get length of valid characters in the sending
//                                       buffer
//         s.resize(tx_buf_len);
//         for (unsigned i = 0; i < s.size(); ++i)
//         {
//             s[i] = ch_arr[i];
//         }
//         return s;
//     }

std::string Protocol::encode(const hnurm_interfaces::msg::VisionSendData &data)
{
    static char        ch_arr[255];
    static float       f_arr[7];
    static uint16_t    tx_buf_len;
    static std::string s          = "";  // restoration of modes
    static uint16_t    bits_flags = 0x00;

    bits_flags = 0x00;
    //        bits_flags |= ((int) data.type << 4); // low-high 4bits for target type
    bits_flags |= (int)data.target_state.data;  // low-low 4bits for target state
    bits_flags |= (int)data.gesture.data << 8;  // high byte for gesture
    f_arr[0] = data.pitch;
    f_arr[1] = data.yaw;
    f_arr[2] = data.vel_x;
    f_arr[3] = data.vel_y;
    f_arr[4] = data.vel_yaw;
    f_arr[5] = data.control_id;
    f_arr[6] = data.spin_ctrl;


    get_protocol_send_data_vision(
        0X0001,                   // vision data identifier(frame head)
        bits_flags,               // bits flags
        f_arr,                    // start pos of float array
        7,                        // pitch, yaw
        (unsigned char *)ch_arr,  // start pos of formatted send buffer
        &tx_buf_len
    );  // get length of valid characters in the sending buffer
    s.resize(tx_buf_len);
    for(unsigned i = 0; i < s.size(); ++i)
    {
        s[i] = ch_arr[i];
    }
    return s;
}

/**
@param: std::string

@return: VisionRecvData
    1-4 bits: self_color Protocol::Self_color        // formatted in flag_register, low  1-4 bits
    5-8 bits: actuator_id int                        // formatted in flag_register, high 5-8 bits
    9-12 bits: mode Protocol::Work_mode              // formatted in flag_register, low  9-12 bits
    13-16 bits: bullet_speed Protocol::Bullet_speed  // formatted in flag_register, low  13-16 bits
*/
// bool Protocol::decode(const std::string &s, VisionRecvData &decoded_data)
// {
//     static char ch_arr[255];
//     static uint16_t msg;
//     static float f_arr[8]; // for further exploration, now receive 3 float data from STM32 (p,y,r)

//     s.copy(ch_arr, s.size());

//     get_protocol_info_vision((uint8_t *) (&ch_arr[0]),        // start pos of received data string
//                              &msg,                      // msg = (self_color: 4bits, work_mode: 4bits) |
//                              (bullet_speed: 4bits | NC: 4bits) f_arr                          // store the float data
//                              sent from STM32, currently no data
//     );

//     /**
//      * @brief if the data length is smaller 8 ,means we don't have to change the protocol
//      * @todo add some other information
//      */
//     // here we get float data .don't worry about different data length!
//     // the protocol will handle changeable data length,as long as the length is smaller than 8

//     // just modify here , use displacement operation to get state code
//     decoded_data = VisionRecvData(static_cast<SelfColor>(msg & 0xf),                    // self_color 0: none 1: red
//     2: blue
//                                   static_cast<WorkMode>((msg >> 4) & 0xf),                 // mode: 0 manual, 1
//                                   auto_shoot, 2 auto_windmill, 5 hook-shoot static_cast<BulletSpeed>((msg >> 8) &
//                                   0xff),             // 0,10,16,15,18,30 f_arr[0], f_arr[1], f_arr[2] // attitude
//     );
//     return true;
// }

bool Protocol::decode(const std::string &s, hnurm_interfaces::msg::VisionRecvData &decoded_data)
{
    static char     ch_arr[1024];
    static uint16_t msg;
    static float    f_arr[15];  // for further exploration, now receive 3 float data from STM32 (p,y,r)

    s.copy(ch_arr, s.size());

    get_protocol_info_vision(
        (uint8_t *)(&ch_arr[0]),  // start pos of received data string
        &msg,                     // msg = (self_color: 4bits, work_mode: 4bits) | (bullet_speed: 4bits | gesture: 4bits)
        f_arr                     // store the float data sent from STM32, currently no data
    );

    /**
     * @brief if the data length is smaller 8 ,means we don't have to change the protocol
     * @todo add some other information
     */
    // here we get float data .don't worry about different data length!
    // the protocol will handle changeable data length,as long as the length is smaller than 8

    /*   ┌──────────────┬─────────────────┬──────────────┬─────────────┐
     *   │    gesture   │   bullet_speed  │  work_mode   │ self_color  │
     *   └──────────────┴─────────────────┴──────────────┴─────────────┘
     *   0            3 4                7 8           11 12           15
     *
     */

    // just modify here , use displacement operation to get state code
    decoded_data.self_color.data   = msg & 0xf;
    decoded_data.work_mode.data    = (msg >> 4) & 0xf;
    decoded_data.bullet_speed.data = (msg >> 8) & 0xf;
    decoded_data.gesture.data      = (msg >> 12) & 0xf;
    decoded_data.yaw              = f_arr[0];
    decoded_data.pitch             = f_arr[1];
    decoded_data.roll              = f_arr[2];
    // decoded_data.vel_x             = f_arr[3];
    // decoded_data.vel_y             = f_arr[4];
    // decoded_data.vel_yaw           = f_arr[5];
    decoded_data.control_id        = f_arr[3];

    decoded_data.game_progress           = f_arr[4];
    decoded_data.current_hp              = f_arr[5];
     decoded_data.current_base_hp 		= f_arr[6];
     decoded_data.allow_fire_amount  	= f_arr[7];
     decoded_data.current_outpost_hp  	= f_arr[8];
     decoded_data.current_enemy_base_hp   = f_arr[9];
     decoded_data.current_enemy_outpost_hp  = f_arr[10];

     decoded_data.hero_x = f_arr[11];
     decoded_data.hero_y = f_arr[12];
     decoded_data.cmd_x = f_arr[13];
     decoded_data.cmd_y = f_arr[14];


    return true;
}

void Protocol::get_protocol_send_data_vision(
    uint16_t     send_id,
    uint16_t     flags_register,
    const float *tx_data,
    uint8_t      float_length,
    uint8_t     *tx_buf,
    uint16_t    *tx_buf_len
)
{
    uint16_t crc16;
    uint16_t data_len;

    data_len = float_length * 4 + 2;

    tx_buf[0] = PROTOCOL_CMD_ID;
    tx_buf[1] = data_len & 0xff;
    tx_buf[2] = (data_len >> 8) & 0xff;
    tx_buf[3] = Get_CRC8_Check(&tx_buf[0], 3);
    tx_buf[4] = send_id & 0xff;
    tx_buf[5] = (send_id >> 8) & 0xff;
    tx_buf[6] = flags_register & 0xff;
    tx_buf[7] = (flags_register >> 8) & 0xff;

    for(int i = 0; i < 4 * float_length; i++)
    {
        // first cast float to char,then read the content in float
        tx_buf[i + 8] = ((uint8_t *)(&tx_data[i / 4]))[i % 4];
        // tx_buf[i + 8] = ((uint8_t*)(&tx_data[i >> 2]))[i % 4];
    }
    // memcpy(tx_buf+8, tx_data, sizeof(tx_data));
    crc16                = Get_CRC16_Check(&tx_buf[0], data_len + 6);
    tx_buf[data_len + 6] = crc16 & 0xff;
    tx_buf[data_len + 7] = (crc16 >> 8) & 0xff;

    *tx_buf_len = data_len + 8;
}

uint16_t Protocol::get_protocol_info_vision(uint8_t *rx_buf, uint16_t *flags_register, float *rx_data)
{
    static protocol pro;
    static uint16_t data_length;
    if(protocol_head_check_vision(&pro, rx_buf))
    {
        data_length = OFFSET_BYTE + pro.header.data_length;
        while(CRC16_Check_Sum(&rx_buf[0], data_length))
        {
            *flags_register = (rx_buf[7] << 8) | rx_buf[6];
            for(int i = 0; i < ((pro.header.data_length - 2) >> 2); i++)
            {
                rx_data[i] = float_protocol(&rx_buf[8 + 4 * i]);
            }
            memset(rx_buf, 0, data_length);
            return pro.cmd_id;
        }
    }
    return false;
}

uint8_t Protocol::Get_CRC8_Check(uint8_t *pchMessage, uint16_t dwLength)
{
    return crc_8(pchMessage, dwLength);
}

uint8_t Protocol::CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength)
{
    uint8_t ucExpected = 0;
    if((pchMessage == 0) || (dwLength <= 2))
        return 0;
    ucExpected = Get_CRC8_Check(pchMessage, dwLength - 1);
    return (ucExpected == pchMessage[dwLength - 1]);
}

uint16_t Protocol::Get_CRC16_Check(uint8_t *pchMessage, uint32_t dwLength)
{
    return crc_16(pchMessage, dwLength);
}

uint16_t Protocol::CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wExpected = 0;
    if((pchMessage == 0) || (dwLength <= 2))
    {
        return 0;
    }
    wExpected = Get_CRC16_Check(pchMessage, dwLength - 2);
    return (
        ((wExpected & 0xff) == pchMessage[dwLength - 2]) && (((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1])
    );
}

uint8_t Protocol::protocol_head_check_vision(protocol *pro, uint8_t *rx_buf)
{
    if(rx_buf[0] == PROTOCOL_CMD_ID)
    {
        pro->header.sof = rx_buf[0];
        if(CRC8_Check_Sum(&rx_buf[0], 4))
        {
            pro->header.data_length = (rx_buf[2] << 8) | rx_buf[1];
            pro->header.crc_check   = rx_buf[3];
            pro->cmd_id             = (rx_buf[5] << 8) | rx_buf[4];
            return 1;
        }
    }
    return 0;
}

float Protocol::float_protocol(uint8_t *dat_t)
{
    uint8_t f_data[4];
    f_data[0] = *(dat_t + 0);
    f_data[1] = *(dat_t + 1);
    f_data[2] = *(dat_t + 2);
    f_data[3] = *(dat_t + 3);
    return *(float *)f_data;
}

}
