#ifndef utils_various
#define utils_various
#include <Arduino.h>
#include "errorTypes.h"
#include "rtl8367c_reg.h"

class rtl8367
{
public:
    rtl8367(uint16_t);

    void setTransmissionPins(uint8_t, uint8_t);
    void setTransmissionDelay(uint16_t);

    int32_t probeIc(uint8_t &);

    int32_t getPortStatus(uint8_t, uint8_t &, uint8_t &, uint8_t &);

private:
    void _smi_start();
    void _smi_writeBit(uint16_t, uint32_t);
    void _smi_readBit(uint32_t, uint32_t *);
    void _smi_stop();
    int32_t smi_read(uint32_t, uint32_t *);
    int32_t smi_write(uint32_t, uint32_t);
    int32_t rtl8367c_setAsicReg(uint32_t, uint32_t);
    int32_t rtl8367c_getAsicReg(uint32_t, uint32_t *);
    int32_t rtk_switch_isUtpPort(uint8_t);
    uint32_t rtk_switch_port_L2P_get(uint8_t);
    int32_t rtl8367c_getAsicPHYReg(uint32_t, uint32_t, uint32_t *);
    int32_t rtl8367c_getAsicPHYOCPReg(uint32_t, uint32_t, uint32_t *);
    int32_t rtl8367c_setAsicRegBits(uint32_t, uint32_t, uint32_t);

    uint16_t
        usTransmissionDelay;

    uint8_t
        sdaPin = 0,
        sckPin = 0;

#define RTL8367C_REGBITLENGTH 16
#define RTL8367C_REGDATAMAX 0xFFFF
#define RTL8367C_PHY_REGNOMAX 0x1F
#define RTL8367C_PHY_BASE 0x2000
#define RTL8367C_PHY_OFFSET 5
#define PHY_RESOLVED_REG 26
#define RTK_SWITCH_PORT_NUM (32)
#define UNDEFINE_PHY_PORT (0xFF)

    typedef enum port_type_e
    {
        UTP_PORT = 0,
        EXT_PORT,
        UNKNOWN_PORT = 0xFF,
        PORT_TYPE_END
    } port_type_t;

    typedef struct rtk_switch_halCtrl_s
    {
        uint8_t switch_type;
        uint32_t l2p_port[RTK_SWITCH_PORT_NUM];
        uint32_t p2l_port[RTK_SWITCH_PORT_NUM];
        port_type_t log_port_type[RTK_SWITCH_PORT_NUM];
        uint32_t ptp_port[RTK_SWITCH_PORT_NUM];
        uint32_t valid_portmask;
        uint32_t valid_utp_portmask;
        uint32_t valid_ext_portmask;
        uint32_t valid_cpu_portmask;
        uint32_t min_phy_port;
        uint32_t max_phy_port;
        uint32_t phy_portmask;
        uint32_t combo_logical_port;
        uint32_t hsg_logical_port;
        uint32_t sg_logical_portmask;
        uint32_t max_meter_id;
        uint32_t max_lut_addr_num;
        uint32_t trunk_group_mask;

    } rtk_switch_halCtrl_t;

    typedef enum switch_chip_e
    {
        CHIP_RTL8367C = 0,
        CHIP_RTL8370B,
        CHIP_RTL8364B,
        CHIP_RTL8363SC_VB,
        CHIP_END
    } switch_chip_t;

    typedef enum init_state_e
    {
        INIT_NOT_COMPLETED = 0,
        INIT_COMPLETED,
        INIT_STATE_END
    } init_state_t;

    typedef enum rtk_port_e
    {
        UTP_PORT0 = 0,
        UTP_PORT1,
        UTP_PORT2,
        UTP_PORT3,
        UTP_PORT4,
        UTP_PORT5,
        UTP_PORT6,
        UTP_PORT7,

        EXT_PORT0 = 16,
        EXT_PORT1,
        EXT_PORT2,

        UNDEFINE_PORT = 30,
        RTK_PORT_MAX = 31
    } rtk_port_t;

    typedef struct rtl8367c_rma_s
    {

        uint16_t operation;
        uint16_t discard_storm_filter;
        uint16_t trap_priority;
        uint16_t keep_format;
        uint16_t vlan_leaky;
        uint16_t portiso_leaky;

    } rtl8367c_rma_t;

    rtk_switch_halCtrl_t halCtrl =
        {
            /* Switch Chip */
            CHIP_RTL8367C,

            /* Logical to Physical */
            {0, 1, 2, 3, 4, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
             6, 7, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},

            /* Physical to Logical */
            {UTP_PORT0, UTP_PORT1, UTP_PORT2, UTP_PORT3, UTP_PORT4, UNDEFINE_PORT, EXT_PORT0, EXT_PORT1,
             UNDEFINE_PORT, UNDEFINE_PORT, UNDEFINE_PORT, UNDEFINE_PORT, UNDEFINE_PORT, UNDEFINE_PORT, UNDEFINE_PORT, UNDEFINE_PORT,
             UNDEFINE_PORT, UNDEFINE_PORT, UNDEFINE_PORT, UNDEFINE_PORT, UNDEFINE_PORT, UNDEFINE_PORT, UNDEFINE_PORT, UNDEFINE_PORT,
             UNDEFINE_PORT, UNDEFINE_PORT, UNDEFINE_PORT, UNDEFINE_PORT, UNDEFINE_PORT, UNDEFINE_PORT, UNDEFINE_PORT, UNDEFINE_PORT},

            /* Port Type */
            {UTP_PORT, UTP_PORT, UTP_PORT, UTP_PORT, UTP_PORT, UNKNOWN_PORT, UNKNOWN_PORT, UNKNOWN_PORT,
             UNKNOWN_PORT, UNKNOWN_PORT, UNKNOWN_PORT, UNKNOWN_PORT, UNKNOWN_PORT, UNKNOWN_PORT, UNKNOWN_PORT, UNKNOWN_PORT,
             EXT_PORT, EXT_PORT, UNKNOWN_PORT, UNKNOWN_PORT, UNKNOWN_PORT, UNKNOWN_PORT, UNKNOWN_PORT, UNKNOWN_PORT,
             UNKNOWN_PORT, UNKNOWN_PORT, UNKNOWN_PORT, UNKNOWN_PORT, UNKNOWN_PORT, UNKNOWN_PORT, UNKNOWN_PORT, UNKNOWN_PORT},

            /* PTP port */
            {1, 1, 1, 1, 1, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0},

            /* Valid port mask */
            ((0x1 << UTP_PORT0) | (0x1 << UTP_PORT1) | (0x1 << UTP_PORT2) | (0x1 << UTP_PORT3) | (0x1 << UTP_PORT4) | (0x1 << EXT_PORT0) | (0x1 << EXT_PORT1)),

            /* Valid UTP port mask */
            ((0x1 << UTP_PORT0) | (0x1 << UTP_PORT1) | (0x1 << UTP_PORT2) | (0x1 << UTP_PORT3) | (0x1 << UTP_PORT4)),

            /* Valid EXT port mask */
            ((0x1 << EXT_PORT0) | (0x1 << EXT_PORT1)),

            /* Valid CPU port mask */
            0x00,

            /* Minimum physical port number */
            0,

            /* Maxmum physical port number */
            7,

            /* Physical port mask */
            0xDF,

            /* Combo Logical port ID */
            4,

            /* HSG Logical port ID */
            EXT_PORT0,

            /* SGMII Logical portmask */
            (0x1 << EXT_PORT0),

            /* Max Meter ID */
            31,

            /* MAX LUT Address Number */
            2112,

            /* Trunk Group Mask */
            0x03};
};
#endif
