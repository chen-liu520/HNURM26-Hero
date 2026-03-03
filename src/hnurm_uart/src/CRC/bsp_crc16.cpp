#include "hnurm_uart/bsp_crc16.h"

static uint8_t  crc_tab16_init = 0;
static uint16_t crc_tab16[256];

/*
 * uint16_t crc_16( const unsigned char *input_str, size_t num_bytes );
 *
 *����crc_16()һ�μ���һ���ֽڵ�16λCRC16
 *�俪ͷ�Ѵ��ݸ��������ַ�����������
 *Ҫ�����ֽ�Ҳ��һ���������ַ����е��ֽ���Ϊ
 *�ܺ㶨��С���ֵ�����ơ�
 */
uint16_t crc_16(const uint8_t *input_str, uint16_t num_bytes)
{
    uint16_t       crc;
    const uint8_t *ptr;
    uint16_t       a;
    if(!crc_tab16_init)
        init_crc16_tab();
    crc = CRC_START_16;
    ptr = input_str;
    if(ptr != 0)
        for(a = 0; a < num_bytes; a++)
        {
            crc = (crc >> 8) ^ crc_tab16[(crc ^ (uint16_t)*ptr++) & 0x00FF];
        }
    return crc;
}

/*
 * uint16_t crc_modbus( const unsigned char *input_str, size_t num_bytes );
 *
 *����crc_modbus()һ�μ���16λmodbusѭ������У��
 *һ���ֽ��ַ������俪ͷ�ѱ����ݸ���������
 *Ҫ�����ֽ���Ҳ��һ��������
 */

uint16_t crc_modbus(const uint8_t *input_str, uint16_t num_bytes)
{
    uint16_t       crc;
    const uint8_t *ptr;
    uint16_t       a;

    if(!crc_tab16_init)
        init_crc16_tab();

    crc = CRC_START_MODBUS;
    ptr = input_str;
    if(ptr != 0)
        for(a = 0; a < num_bytes; a++)
        {

            crc = (crc >> 8) ^ crc_tab16[(crc ^ (uint16_t)*ptr++) & 0x00FF];
        }
    return crc;
}

/*
 * uint16_t update_crc_16( uint16_t crc, unsigned char c );
 *
 *����update_crc_16()����
 *ǰһ��ѭ������У��ֵ����һ��Ҫ���������ֽڡ�
 */
uint16_t update_crc_16(uint16_t crc, uint8_t c)
{
    if(!crc_tab16_init)
        init_crc16_tab();
    return (crc >> 8) ^ crc_tab16[(crc ^ (uint16_t)c) & 0x00FF];
}

/*
 * static void init_crc16_tab( void );
 *
 *Ϊ�˻��������ܣ�ʹ��CRC16���̲��Ҵ���ֵ�ı�
 *����ֱ��������㷨��ʹ�õ��㷨��
 *���ұ��״���init_crc16_tab()���̼���
 *����ѭ������У�麯����
 */
void init_crc16_tab(void)
{
    uint16_t i;
    uint16_t j;
    uint16_t crc;
    uint16_t c;
    for(i = 0; i < 256; i++)
    {
        crc = 0;
        c   = i;
        for(j = 0; j < 8; j++)
        {
            if((crc ^ c) & 0x0001)
                crc = (crc >> 1) ^ CRC_POLY_16;
            else
                crc = crc >> 1;
            c = c >> 1;
        }
        crc_tab16[i] = crc;
    }
    crc_tab16_init = 1;
}
