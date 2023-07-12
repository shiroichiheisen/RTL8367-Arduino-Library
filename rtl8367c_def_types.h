#include "Arduino.h"
#define RTK_TOTAL_NUM_OF_WORD_FOR_1BIT_PORT_LIST 1
#define ETHER_ADDR_LEN 6
#define RTL8367C_LUT_TABLE_SIZE (6)
#define RTL8367C_LUT_AGETIMERMAX (7)
#define RTL8367C_LUT_AGESPEEDMAX (3)
#define RTL8367C_LUT_LEARNLIMITMAX (0x1040)
#define RTL8367C_LUT_ADDRMAX (0x103F)
#define RTL8367C_LUT_IPMCGRP_TABLE_MAX (0x3F)
#define RTL8367C_LUT_ENTRY_SIZE (6)
#define RTL8367C_LUT_BUSY_CHECK_NO (10)
#define RTL8367C_C2SIDXNO 128
#define RTL8367C_C2SIDXMAX (RTL8367C_C2SIDXNO - 1)
#define RTL8367C_MC2SIDXNO 32
#define RTL8367C_MC2SIDXMAX (RTL8367C_MC2SIDXNO - 1)
#define RTL8367C_SP2CIDXNO 128
#define RTL8367C_SP2CMAX (RTL8367C_SP2CIDXNO - 1)
#define RTL8367C_SVLAN_MEMCONF_LEN 4
#define RTL8367C_SVLAN_MC2S_LEN 5
#define RTL8367C_SVLAN_SP2C_LEN 2
#define RTK_MAX_NUM_OF_PROTO_TYPE 0xFFFF
#define RTL8367C_DECISIONPRIMAX 0xFF
#define PHY_CONTROL_REG 0
#define RTK_MAX_NUM_OF_INTERRUPT_TYPE 1
#define ADV_NOT_SUPPORT (0xFFFF)
#define RTL8367C_PORTNO 11
#define RTL8367C_PORTIDMAX (RTL8367C_PORTNO - 1)

#define RTK_PORTMASK_IS_PORT_SET(__portmask__, __port__) (((__portmask__).bits[0] & (0x00000001 << __port__)) ? 1 : 0)
#define RTL8367C_METERNO 64
#define RTL8367C_METERMAX (RTL8367C_METERNO - 1)
#define RTL8367C_TABLE_ACCESS_RDDATA_BASE RTL8367C_REG_TABLE_READ_DATA0

#define RTL8367C_REGBITLENGTH 16
#define RTL8367C_FIDMAX 0xF
#define RTL8367C_EVIDMAX 0x1FFF
#define RTL8367C_PORTMASK 0x7FF
#define RTL8367C_VLAN_MBRCFG_LEN (4)
#define RTL8367C_CVIDXNO 32
#define RTL8367C_CVIDXMAX (RTL8367C_CVIDXNO - 1)
#define RTL8367C_REGDATAMAX 0xFFFF
#define RTL8367C_PHY_REGNOMAX 0x1F
#define RTL8367C_PHY_BASE 0x2000
#define RTL8367C_PHY_OFFSET 5
#define PHY_RESOLVED_REG 26
#define RTK_SWITCH_PORT_NUM (32)
#define UNDEFINE_PHY_PORT (0xFF)
#define RTL8367C_PRIMAX 7
#define RTL8367C_VLAN_MEMBER_CONFIGURATION_BASE RTL8367C_REG_VLAN_MEMBER_CONFIGURATION0_CTRL0
#define RTL8367C_VLAN_4KTABLE_LEN (3)
#define RTL8367C_VIDMAX 0xFFF
#define RTL8367C_TABLE_ACCESS_WRDATA_BASE RTL8367C_REG_TABLE_WRITE_DATA0
#define RTL8367C_TABLE_ACCESS_WRDATA_REG(index) (RTL8367C_TABLE_ACCESS_WRDATA_BASE + index)
#define RTL8367C_TABLE_ACCESS_ADDR_REG RTL8367C_REG_TABLE_ACCESS_ADDR
#define RTL8367C_TABLE_ACCESS_CTRL_REG RTL8367C_REG_TABLE_ACCESS_CTRL
#define RTL8367C_TABLE_ACCESS_REG_DATA(op, target) ((op << 3) | target)
#define RTL8367C_VLAN_PORTBASED_PRIORITY_BASE RTL8367C_REG_VLAN_PORTBASED_PRIORITY_CTRL0
#define RTL8367C_VLAN_PORTBASED_PRIORITY_REG(port) (RTL8367C_VLAN_PORTBASED_PRIORITY_BASE + (port >> 2))
#define RTL8367C_VLAN_PORTBASED_PRIORITY_OFFSET(port) ((port & 0x3) << 2)
#define RTL8367C_VLAN_PORTBASED_PRIORITY_MASK(port) (0x7 << RTL8367C_VLAN_PORTBASED_PRIORITY_OFFSET(port))
#define RTL8367C_PORT_MISC_CFG_BASE RTL8367C_REG_PORT0_MISC_CFG
#define RTL8367C_PORT_MISC_CFG_REG(port) (RTL8367C_PORT_MISC_CFG_BASE + (port << 5))
#define RTK_MAX_NUM_OF_QUEUE 8
#define RTL8367C_QOS_PORT_QUEUE_NUMBER_BASE RTL8367C_REG_QOS_PORT_QUEUE_NUMBER_CTRL0
#define RTL8367C_QOS_PORT_QUEUE_NUMBER_REG(port) (RTL8367C_QOS_PORT_QUEUE_NUMBER_BASE + (port >> 2))
#define RTL8367C_QOS_PORT_QUEUE_NUMBER_OFFSET(port) ((port & 0x3) << 2)
#define RTL8367C_QOS_PORT_QUEUE_NUMBER_MASK(port) (0x7 << RTL8367C_QOS_PORT_QUEUE_NUMBER_OFFSET(port))
#define RTL8367C_QUEUENO 8
#define RTL8367C_QIDMAX (RTL8367C_QUEUENO - 1)
#define RTK_PRIMAX 7
#define RTK_QIDMAX 7
#define RTK_DSCPMAX 63
#define RTL8367C_QOS_1Q_PRIORITY_TO_QID_BASE RTL8367C_REG_QOS_1Q_PRIORITY_TO_QID_CTRL0
#define RTL8367C_QOS_1Q_PRIORITY_TO_QID_REG(index, pri) (RTL8367C_QOS_1Q_PRIORITY_TO_QID_BASE + (index << 1) + (pri >> 2))
#define RTL8367C_QOS_1Q_PRIORITY_TO_QID_OFFSET(pri) ((pri & 0x3) << 2)
#define RTL8367C_QOS_1Q_PRIORITY_TO_QID_MASK(pri) (RTL8367C_QOS_1Q_PRIORITY_TO_QID_CTRL0_PRIORITY0_TO_QID_MASK << RTL8367C_QOS_1Q_PRIORITY_TO_QID_OFFSET(pri))
#define QOS_WEIGHT_MAX 127
#define RTL8367C_QWEIGHTMAX 0x7F
#define RTL8367C_PORT_QUEUE_METER_INDEX_MAX 7

#define RTK_MAC_ADDR_LEN 6
#define RTL8367C_EFIDMAX 0x7

#define RTL8367C_SVIDXNO 64
#define RTL8367C_SVIDXMAX (RTL8367C_SVIDXNO - 1)
#define RTL8367C_SVLAN_MEMCONF_LEN 4
#define RTL8367C_C2SIDXNO 128
#define RTL8367C_C2SIDXMAX (RTL8367C_C2SIDXNO - 1)
#define RTL8367C_MC2SIDXNO 32
#define RTL8367C_MC2SIDXMAX (RTL8367C_MC2SIDXNO - 1)
#define RTL8367C_SVLAN_SP2C_LEN 2
#define RTK_FID_MAX 0xF
#define RTL8367C_DSCPMAX 63
#define RTL8367C_MIB_PORT_OFFSET (0x7C)
#define RTL8367C_MIB_LEARNENTRYDISCARD_OFFSET (0x420)
#define PHY_1000_BASET_CONTROL_REG 9
#define PHY_AN_ADVERTISEMENT_REG 4

#define RTK_PORTMASK_PORT_SET(__portmask__, __port__) ((__portmask__).bits[0] |= (0x00000001 << __port__))

#define RTL8367C_VLAN_BUSY_CHECK_NO (10)
#define RTL8367C_TABLE_ACCESS_STATUS_REG RTL8367C_REG_TABLE_LUT_ADDR
#define RTK_PORTMASK_CLEAR(__portmask__) ((__portmask__).bits[0] = 0)

#define RTL8367C_VLAN_ACCEPT_FRAME_TYPE_BASE RTL8367C_REG_VLAN_ACCEPT_FRAME_TYPE_CTRL0
#define RTL8367C_VLAN_ACCEPT_FRAME_TYPE_REG(port) (RTL8367C_VLAN_ACCEPT_FRAME_TYPE_BASE + (port >> 3))
#define RTL8367C_VLAN_ACCEPT_FRAME_TYPE_MASK(port) (RTL8367C_PORT0_FRAME_TYPE_MASK << ((port & 0x7) << 1))

#define RTL8367C_LEDGROUPNO 3
#define RTL8367C_LEDGROUPMASK 0x7
#define RTL8367C_LED_FORCE_MODE_BASE RTL8367C_REG_CPU_FORCE_LED0_CFG0
#define RTL8367C_LED_FORCE_CTRL RTL8367C_REG_CPU_FORCE_LED_CFG

#define RTL8367C_RMAMAX 0x2F

#define STORM_UNUC_INDEX 28
#define STORM_UNMC_INDEX 29
#define STORM_MC_INDEX 30
#define STORM_BC_INDEX 31

#define RTK_METER_NUM (RTK_MAX_METER_ID + 1)

#define PHY_STATUS_REG 1
#define PHY_AN_LINKPARTNER_REG 5
#define PHY_1000_BASET_STATUS_REG 10

#define RTK_EFID_MAX 0x7

#define RTK_FIBER_FORCE_1000M 3
#define RTK_FIBER_FORCE_100M 5
#define RTK_FIBER_FORCE_100M1000M 7

#define RTK_INDRECT_ACCESS_CRTL 0x1f00
#define RTK_INDRECT_ACCESS_STATUS 0x1f01
#define RTK_INDRECT_ACCESS_ADDRESS 0x1f02
#define RTK_INDRECT_ACCESS_WRITE_DATA 0x1f03
#define RTK_INDRECT_ACCESS_READ_DATA 0x1f04
#define RTK_INDRECT_ACCESS_DELAY 0x1f80
#define RTK_INDRECT_ACCESS_BURST 0x1f81
#define RTK_RW_MASK 0x2
#define RTK_CMD_MASK 0x1
#define RTK_PHY_BUSY_OFFSET 2

#define RTL8367C_MAC7 7
#define RTL8367C_EXTNO 3

#define RTL8367C_RTCT_PAGE (11)
#define RTL8367C_RTCT_RESULT_A_REG (27)
#define RTL8367C_RTCT_RESULT_B_REG (28)
#define RTL8367C_RTCT_RESULT_C_REG (29)
#define RTL8367C_RTCT_RESULT_D_REG (30)
#define RTL8367C_RTCT_STATUS_REG (26)

enum L2_SECURITY_BEHAVE
{
    L2_BEHAVE_FLOODING = 0,
    L2_BEHAVE_DROP,
    L2_BEHAVE_TRAP,
    L2_BEHAVE_END
};

enum L2_UNDA_BEHAVE
{
    L2_UNDA_BEHAVE_FLOODING_PMASK = 0,
    L2_UNDA_BEHAVE_DROP,
    L2_UNDA_BEHAVE_TRAP,
    L2_UNDA_BEHAVE_FLOODING,
    L2_UNDA_BEHAVE_END
};

enum L2_SECURITY_SA_BEHAVE
{
    L2_BEHAVE_SA_FLOODING = 0,
    L2_BEHAVE_SA_DROP,
    L2_BEHAVE_SA_TRAP,
    L2_BEHAVE_SA_COPY28051,
    L2_BEHAVE_SA_END
};

/* enum for port current link speed */
enum SPEEDMODE
{
    SPD_10M = 0,
    SPD_100M,
    SPD_1000M,
    SPD_2500M
};

/* enum for mac link mode */
enum LINKMODE
{
    MAC_NORMAL = 0,
    MAC_FORCE,
};

/* enum for port current link duplex mode */
enum DUPLEXMODE
{
    HALF_DUPLEX = 0,
    FULL_DUPLEX
};

/* enum for port current MST mode */
enum MSTMODE
{
    SLAVE_MODE = 0,
    MASTER_MODE
};

enum EXTMODE
{
    EXT_DISABLE = 0,
    EXT_RGMII,
    EXT_MII_MAC,
    EXT_MII_PHY,
    EXT_TMII_MAC,
    EXT_TMII_PHY,
    EXT_GMII,
    EXT_RMII_MAC,
    EXT_RMII_PHY,
    EXT_SGMII,
    EXT_HSGMII,
    EXT_1000X_100FX,
    EXT_1000X,
    EXT_100FX,
    EXT_RGMII_2,
    EXT_MII_MAC_2,
    EXT_MII_PHY_2,
    EXT_TMII_MAC_2,
    EXT_TMII_PHY_2,
    EXT_RMII_MAC_2,
    EXT_RMII_PHY_2,
    EXT_END
};

enum DOSTYPE
{
    DOS_DAEQSA = 0,
    DOS_LANDATTACKS,
    DOS_BLATATTACKS,
    DOS_SYNFINSCAN,
    DOS_XMASCAN,
    DOS_NULLSCAN,
    DOS_SYN1024,
    DOS_TCPSHORTHDR,
    DOS_TCPFRAGERROR,
    DOS_ICMPFRAGMENT,
    DOS_END,

};

typedef struct rtl8367c_port_status_s
{

    uint16_t lpi1000;
    uint16_t lpi100;
    uint16_t mstfault;
    uint16_t mstmode;
    uint16_t nway;
    uint16_t txpause;
    uint16_t rxpause;
    uint16_t link;
    uint16_t duplex;
    uint16_t speed;

} rtl8367c_port_status_t;

typedef struct rtct_result_s
{
    uint32_t channelAShort;
    uint32_t channelBShort;
    uint32_t channelCShort;
    uint32_t channelDShort;

    uint32_t channelAOpen;
    uint32_t channelBOpen;
    uint32_t channelCOpen;
    uint32_t channelDOpen;

    uint32_t channelAMismatch;
    uint32_t channelBMismatch;
    uint32_t channelCMismatch;
    uint32_t channelDMismatch;

    uint32_t channelALinedriver;
    uint32_t channelBLinedriver;
    uint32_t channelCLinedriver;
    uint32_t channelDLinedriver;

    uint32_t channelALen;
    uint32_t channelBLen;
    uint32_t channelCLen;
    uint32_t channelDLen;
} rtl8367c_port_rtct_result_t;

typedef struct rtl8367c_port_ability_s
{
    uint16_t forcemode;
    uint16_t mstfault;
    uint16_t mstmode;
    uint16_t nway;
    uint16_t txpause;
    uint16_t rxpause;
    uint16_t link;
    uint16_t duplex;
    uint16_t speed;
} rtl8367c_port_ability_t;

typedef enum rtk_mode_ext_e
{
    MODE_EXT_DISABLE = 0,
    MODE_EXT_RGMII,
    MODE_EXT_MII_MAC,
    MODE_EXT_MII_PHY,
    MODE_EXT_TMII_MAC,
    MODE_EXT_TMII_PHY,
    MODE_EXT_GMII,
    MODE_EXT_RMII_MAC,
    MODE_EXT_RMII_PHY,
    MODE_EXT_SGMII,
    MODE_EXT_HSGMII,
    MODE_EXT_1000X_100FX,
    MODE_EXT_1000X,
    MODE_EXT_100FX,
    MODE_EXT_RGMII_2,
    MODE_EXT_MII_MAC_2,
    MODE_EXT_MII_PHY_2,
    MODE_EXT_TMII_MAC_2,
    MODE_EXT_TMII_PHY_2,
    MODE_EXT_RMII_MAC_2,
    MODE_EXT_RMII_PHY_2,
    MODE_EXT_END
} rtk_mode_ext_t;

typedef enum rtk_port_duplex_e
{
    PORT_HALF_DUPLEX = 0,
    PORT_FULL_DUPLEX,
    PORT_DUPLEX_END
} rtk_port_duplex_t;

typedef enum rtk_port_linkStatus_e
{
    PORT_LINKDOWN = 0,
    PORT_LINKUP,
    PORT_LINKSTATUS_END
} rtk_port_linkStatus_t;

typedef struct rtk_port_mac_ability_s
{
    uint32_t forcemode;
    uint32_t speed;
    uint32_t duplex;
    uint32_t link;
    uint32_t nway;
    uint32_t txpause;
    uint32_t rxpause;
} rtk_port_mac_ability_t;

typedef enum rtk_port_phy_mdix_mode_e
{
    PHY_AUTO_CROSSOVER_MODE = 0,
    PHY_FORCE_MDI_MODE,
    PHY_FORCE_MDIX_MODE,
    PHY_FORCE_MODE_END
} rtk_port_phy_mdix_mode_t;

typedef enum rtk_port_phy_mdix_status_e
{
    PHY_STATUS_AUTO_MDI_MODE = 0,
    PHY_STATUS_AUTO_MDIX_MODE,
    PHY_STATUS_FORCE_MDI_MODE,
    PHY_STATUS_FORCE_MDIX_MODE,
    PHY_STATUS_FORCE_MODE_END
} rtk_port_phy_mdix_status_t;

typedef enum rtk_port_phy_test_mode_e
{
    PHY_TEST_MODE_NORMAL = 0,
    PHY_TEST_MODE_1,
    PHY_TEST_MODE_2,
    PHY_TEST_MODE_3,
    PHY_TEST_MODE_4,
    PHY_TEST_MODE_END
} rtk_port_phy_test_mode_t;

typedef enum rtk_port_speed_e
{
    PORT_SPEED_10M = 0,
    PORT_SPEED_100M,
    PORT_SPEED_1000M,
    PORT_SPEED_500M,
    PORT_SPEED_2500M,
    PORT_SPEED_END
} rtk_port_speed_t;

typedef struct rtk_rtctResult_s
{
    rtk_port_speed_t linkType;
    union
    {
        struct fe_result_s
        {
            uint32_t isRxShort;
            uint32_t isTxShort;
            uint32_t isRxOpen;
            uint32_t isTxOpen;
            uint32_t isRxMismatch;
            uint32_t isTxMismatch;
            uint32_t isRxLinedriver;
            uint32_t isTxLinedriver;
            uint32_t rxLen;
            uint32_t txLen;
        } fe_result;

        struct ge_result_s
        {
            uint32_t channelAShort;
            uint32_t channelBShort;
            uint32_t channelCShort;
            uint32_t channelDShort;

            uint32_t channelAOpen;
            uint32_t channelBOpen;
            uint32_t channelCOpen;
            uint32_t channelDOpen;

            uint32_t channelAMismatch;
            uint32_t channelBMismatch;
            uint32_t channelCMismatch;
            uint32_t channelDMismatch;

            uint32_t channelALinedriver;
            uint32_t channelBLinedriver;
            uint32_t channelCLinedriver;
            uint32_t channelDLinedriver;

            uint32_t channelALen;
            uint32_t channelBLen;
            uint32_t channelCLen;
            uint32_t channelDLen;
        } ge_result;
    } result;
} rtk_rtctResult_t;

typedef enum rtk_meter_type_e
{
    METER_TYPE_KBPS = 0, /* Kbps */
    METER_TYPE_PPS,      /* Packet per second */
    METER_TYPE_END
} rtk_meter_type_t;

typedef enum rtk_rate_storm_group_e
{
    STORM_GROUP_UNKNOWN_UNICAST = 0,
    STORM_GROUP_UNKNOWN_MULTICAST,
    STORM_GROUP_MULTICAST,
    STORM_GROUP_BROADCAST,
    STORM_GROUP_END
} rtk_rate_storm_group_t;

typedef enum rtk_storm_bypass_e
{
    BYPASS_BRG_GROUP = 0,
    BYPASS_FD_PAUSE,
    BYPASS_SP_MCAST,
    BYPASS_1X_PAE,
    BYPASS_UNDEF_BRG_04,
    BYPASS_UNDEF_BRG_05,
    BYPASS_UNDEF_BRG_06,
    BYPASS_UNDEF_BRG_07,
    BYPASS_PROVIDER_BRIDGE_GROUP_ADDRESS,
    BYPASS_UNDEF_BRG_09,
    BYPASS_UNDEF_BRG_0A,
    BYPASS_UNDEF_BRG_0B,
    BYPASS_UNDEF_BRG_0C,
    BYPASS_PROVIDER_BRIDGE_GVRP_ADDRESS,
    BYPASS_8021AB,
    BYPASS_UNDEF_BRG_0F,
    BYPASS_BRG_MNGEMENT,
    BYPASS_UNDEFINED_11,
    BYPASS_UNDEFINED_12,
    BYPASS_UNDEFINED_13,
    BYPASS_UNDEFINED_14,
    BYPASS_UNDEFINED_15,
    BYPASS_UNDEFINED_16,
    BYPASS_UNDEFINED_17,
    BYPASS_UNDEFINED_18,
    BYPASS_UNDEFINED_19,
    BYPASS_UNDEFINED_1A,
    BYPASS_UNDEFINED_1B,
    BYPASS_UNDEFINED_1C,
    BYPASS_UNDEFINED_1D,
    BYPASS_UNDEFINED_1E,
    BYPASS_UNDEFINED_1F,
    BYPASS_GMRP,
    BYPASS_GVRP,
    BYPASS_UNDEF_GARP_22,
    BYPASS_UNDEF_GARP_23,
    BYPASS_UNDEF_GARP_24,
    BYPASS_UNDEF_GARP_25,
    BYPASS_UNDEF_GARP_26,
    BYPASS_UNDEF_GARP_27,
    BYPASS_UNDEF_GARP_28,
    BYPASS_UNDEF_GARP_29,
    BYPASS_UNDEF_GARP_2A,
    BYPASS_UNDEF_GARP_2B,
    BYPASS_UNDEF_GARP_2C,
    BYPASS_UNDEF_GARP_2D,
    BYPASS_UNDEF_GARP_2E,
    BYPASS_UNDEF_GARP_2F,
    BYPASS_IGMP,
    BYPASS_CDP,
    BYPASS_CSSTP,
    BYPASS_LLDP,
    BYPASS_END,
} rtk_storm_bypass_t;

enum RTL8367C_RMAOP
{
    RMAOP_FORWARD = 0,
    RMAOP_TRAP_TO_CPU,
    RMAOP_DROP,
    RMAOP_FORWARD_EXCLUDE_CPU,
    RMAOP_END
};

typedef struct rtl8367c_rma_s
{
    uint16_t operation;
    uint16_t discard_storm_filter;
    uint16_t trap_priority;
    uint16_t keep_format;
    uint16_t vlan_leaky;
    uint16_t portiso_leaky;
} rtl8367c_rma_t;

typedef enum rtk_trap_type_e
{
    TRAP_BRG_GROUP = 0,
    TRAP_FD_PAUSE,
    TRAP_SP_MCAST,
    TRAP_1X_PAE,
    TRAP_UNDEF_BRG_04,
    TRAP_UNDEF_BRG_05,
    TRAP_UNDEF_BRG_06,
    TRAP_UNDEF_BRG_07,
    TRAP_PROVIDER_BRIDGE_GROUP_ADDRESS,
    TRAP_UNDEF_BRG_09,
    TRAP_UNDEF_BRG_0A,
    TRAP_UNDEF_BRG_0B,
    TRAP_UNDEF_BRG_0C,
    TRAP_PROVIDER_BRIDGE_GVRP_ADDRESS,
    TRAP_8021AB,
    TRAP_UNDEF_BRG_0F,
    TRAP_BRG_MNGEMENT,
    TRAP_UNDEFINED_11,
    TRAP_UNDEFINED_12,
    TRAP_UNDEFINED_13,
    TRAP_UNDEFINED_14,
    TRAP_UNDEFINED_15,
    TRAP_UNDEFINED_16,
    TRAP_UNDEFINED_17,
    TRAP_UNDEFINED_18,
    TRAP_UNDEFINED_19,
    TRAP_UNDEFINED_1A,
    TRAP_UNDEFINED_1B,
    TRAP_UNDEFINED_1C,
    TRAP_UNDEFINED_1D,
    TRAP_UNDEFINED_1E,
    TRAP_UNDEFINED_1F,
    TRAP_GMRP,
    TRAP_GVRP,
    TRAP_UNDEF_GARP_22,
    TRAP_UNDEF_GARP_23,
    TRAP_UNDEF_GARP_24,
    TRAP_UNDEF_GARP_25,
    TRAP_UNDEF_GARP_26,
    TRAP_UNDEF_GARP_27,
    TRAP_UNDEF_GARP_28,
    TRAP_UNDEF_GARP_29,
    TRAP_UNDEF_GARP_2A,
    TRAP_UNDEF_GARP_2B,
    TRAP_UNDEF_GARP_2C,
    TRAP_UNDEF_GARP_2D,
    TRAP_UNDEF_GARP_2E,
    TRAP_UNDEF_GARP_2F,
    TRAP_CDP,
    TRAP_CSSTP,
    TRAP_LLDP,
    TRAP_END,
} rtk_trap_type_t;

typedef enum rtk_mcast_type_e
{
    MCAST_L2 = 0,
    MCAST_IPV4,
    MCAST_IPV6,
    MCAST_END
} rtk_mcast_type_t;

typedef enum rtk_trap_mcast_action_e
{
    MCAST_ACTION_FORWARD = 0,
    MCAST_ACTION_DROP,
    MCAST_ACTION_TRAP2CPU,
    MCAST_ACTION_ROUTER_PORT,
    MCAST_ACTION_DROP_EX_RMA,
    MCAST_ACTION_END
} rtk_trap_mcast_action_t;

typedef enum rtk_trap_rma_action_e
{
    RMA_ACTION_FORWARD = 0,
    RMA_ACTION_TRAP2CPU,
    RMA_ACTION_DROP,
    RMA_ACTION_FORWARD_EXCLUDE_CPU,
    RMA_ACTION_END
} rtk_trap_rma_action_t;

typedef enum rtk_trap_ucast_action_e
{
    UCAST_ACTION_FORWARD_PMASK = 0,
    UCAST_ACTION_DROP,
    UCAST_ACTION_TRAP2CPU,
    UCAST_ACTION_FLOODING,
    UCAST_ACTION_END
} rtk_trap_ucast_action_t;

typedef enum rtk_trap_ucast_type_e
{
    UCAST_UNKNOWNDA = 0,
    UCAST_UNKNOWNSA,
    UCAST_UNMATCHSA,
    UCAST_END
} rtk_trap_ucast_type_t;

typedef enum rtk_trap_reason_type_e
{
    TRAP_REASON_RMA = 0,
    TRAP_REASON_OAM,
    TRAP_REASON_1XUNAUTH,
    TRAP_REASON_VLANSTACK,
    TRAP_REASON_UNKNOWNMC,
    TRAP_REASON_END,
} rtk_trap_reason_type_t;

typedef enum rtk_led_operation_e
{
    LED_OP_SCAN = 0,
    LED_OP_PARALLEL,
    LED_OP_SERIAL,
    LED_OP_END,
} rtk_led_operation_t;

typedef enum rtk_led_active_e
{
    LED_ACTIVE_HIGH = 0,
    LED_ACTIVE_LOW,
    LED_ACTIVE_END,
} rtk_led_active_t;

typedef enum rtk_led_config_e
{
    LED_CONFIG_LEDOFF = 0,
    LED_CONFIG_DUPCOL,
    LED_CONFIG_LINK_ACT,
    LED_CONFIG_SPD1000,
    LED_CONFIG_SPD100,
    LED_CONFIG_SPD10,
    LED_CONFIG_SPD1000ACT,
    LED_CONFIG_SPD100ACT,
    LED_CONFIG_SPD10ACT,
    LED_CONFIG_SPD10010ACT,
    LED_CONFIG_LOOPDETECT,
    LED_CONFIG_EEE,
    LED_CONFIG_LINKRX,
    LED_CONFIG_LINKTX,
    LED_CONFIG_MASTER,
    LED_CONFIG_ACT,
    LED_CONFIG_END,
} rtk_led_congig_t;

typedef enum rtk_enable_e
{
    DISABLED_RTK = 0,
    ENABLED,
    RTK_ENABLE_END
} rtk_enable_t;

typedef struct rtk_led_ability_s
{
    rtk_enable_t link_10m;
    rtk_enable_t link_100m;
    rtk_enable_t link_500m;
    rtk_enable_t link_1000m;
    rtk_enable_t act_rx;
    rtk_enable_t act_tx;
} rtk_led_ability_t;

typedef enum rtk_led_blink_rate_e
{
    LED_BLINKRATE_32MS = 0,
    LED_BLINKRATE_64MS,
    LED_BLINKRATE_128MS,
    LED_BLINKRATE_256MS,
    LED_BLINKRATE_512MS,
    LED_BLINKRATE_1024MS,
    LED_BLINKRATE_48MS,
    LED_BLINKRATE_96MS,
    LED_BLINKRATE_END,
} rtk_led_blink_rate_t;

typedef enum rtk_led_group_e
{
    LED_GROUP_0 = 0,
    LED_GROUP_1,
    LED_GROUP_2,
    LED_GROUP_END
} rtk_led_group_t;

typedef enum rtk_led_force_mode_e
{
    LED_FORCE_NORMAL = 0,
    LED_FORCE_BLINK,
    LED_FORCE_OFF,
    LED_FORCE_ON,
    LED_FORCE_END
} rtk_led_force_mode_t;

typedef enum rtk_svlan_lookupType_e
{
    SVLAN_LOOKUP_S64MBRCGF = 0,
    SVLAN_LOOKUP_C4KVLAN,
    SVLAN_LOOKUP_END,

} rtk_svlan_lookupType_t;

typedef struct VLANCONFIGUSER
{
    uint16_t evid;
    uint16_t mbr;
    uint16_t fid_msti;
    uint16_t envlanpol;
    uint16_t meteridx;
    uint16_t vbpen;
    uint16_t vbpri;
} rtl8367c_vlanconfiguser;

typedef enum vlan_mbrCfgType_e
{
    MBRCFG_UNUSED = 0,
    MBRCFG_USED_BY_VLAN,
    MBRCFG_END
} vlan_mbrCfgType_t;

typedef struct USER_VLANTABLE
{

    uint16_t vid;
    uint16_t mbr;
    uint16_t untag;
    uint16_t fid_msti;
    uint16_t envlanpol;
    uint16_t meteridx;
    uint16_t vbpen;
    uint16_t vbpri;
    uint16_t ivl_svl;

} rtl8367c_user_vlan4kentry;

enum RTL8367C_TABLE_ACCESS_OP
{
    TB_OP_READ = 0,
    TB_OP_WRITE
};

enum RTL8367C_TABLE_ACCESS_TARGET
{
    TB_TARGET_ACLRULE = 1,
    TB_TARGET_ACLACT,
    TB_TARGET_CVLAN,
    TB_TARGET_L2,
    TB_TARGET_IGMP_GROUP
};
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

enum RTL8367C_LUTREADMETHOD
{

    LUTREADMETHOD_MAC = 0,
    LUTREADMETHOD_ADDRESS,
    LUTREADMETHOD_NEXT_ADDRESS,
    LUTREADMETHOD_NEXT_L2UC,
    LUTREADMETHOD_NEXT_L2MC,
    LUTREADMETHOD_NEXT_L3MC,
    LUTREADMETHOD_NEXT_L2L3MC,
    LUTREADMETHOD_NEXT_L2UCSPA,
};

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

typedef enum rtk_vlan_acceptFrameType_e
{
    ACCEPT_FRAME_TYPE_ALL = 0,    /* untagged, priority-tagged and tagged */
    ACCEPT_FRAME_TYPE_TAG_ONLY,   /* tagged */
    ACCEPT_FRAME_TYPE_UNTAG_ONLY, /* untagged and priority-tagged */
    ACCEPT_FRAME_TYPE_END
} rtk_vlan_acceptFrameType_t;

typedef enum
{
    FRAME_TYPE_BOTH = 0,
    FRAME_TYPE_TAGGED_ONLY,
    FRAME_TYPE_UNTAGGED_ONLY,
    FRAME_TYPE_MAX_BOUND
} rtl8367c_accframetype;

typedef enum
{
    EG_TAG_MODE_ORI = 0,
    EG_TAG_MODE_KEEP,
    EG_TAG_MODE_PRI_TAG,
    EG_TAG_MODE_REAL_KEEP,
    EG_TAG_MODE_END
} rtl8367c_egtagmode;

typedef struct rtk_portmask_s
{
    uint32_t bits[RTK_TOTAL_NUM_OF_WORD_FOR_1BIT_PORT_LIST];
} rtk_portmask_t;

typedef struct rtk_vlan_cfg_s
{
    rtk_portmask_t mbr;
    rtk_portmask_t untag;
    uint16_t ivl_en;
    uint16_t fid_msti;
    uint16_t envlanpol;
    uint16_t meteridx;
    uint16_t vbpen;
    uint16_t vbpri;
} rtk_vlan_cfg_t;

typedef struct rtl8367c_svlan_memconf_s
{

    uint16_t vs_member : 11;
    uint16_t vs_untag : 11;

    uint16_t vs_fid_msti : 4;
    uint16_t vs_priority : 3;
    uint16_t vs_force_fid : 1;
    uint16_t reserved : 8;

    uint16_t vs_svid : 12;
    uint16_t vs_efiden : 1;
    uint16_t vs_efid : 3;

} rtl8367c_svlan_memconf_t;

typedef struct rtl8367c_svlan_mc2s_s
{

    uint16_t valid : 1;
    uint16_t format : 1;
    uint16_t svidx : 6;
    uint32_t sdata;
    uint32_t smask;
} rtl8367c_svlan_mc2s_t;

typedef struct rtl8367c_svlan_s2c_s
{

    uint16_t valid : 1;
    uint16_t svidx : 6;
    uint16_t dstport : 4;
    uint32_t vid : 12;
} rtl8367c_svlan_s2c_t;

enum RTL8367C_SPRISEL
{
    SPRISEL_INTERNALPRI = 0,
    SPRISEL_CTAGPRI,
    SPRISEL_VSPRI,
    SPRISEL_PBPRI,
    SPRISEL_END
};

typedef struct rtk_svlan_memberCfg_s
{
    uint32_t svid;
    rtk_portmask_t memberport;
    rtk_portmask_t untagport;
    uint32_t fiden;
    uint32_t fid;
    uint32_t priority;
    uint32_t efiden;
    uint32_t efid;
} rtk_svlan_memberCfg_t;

typedef enum rtk_svlan_untag_action_e
{
    UNTAG_DROP = 0,
    UNTAG_TRAP,
    UNTAG_ASSIGN,
    UNTAG_END
} rtk_svlan_untag_action_t;

typedef enum rtk_svlan_unmatch_action_e
{
    UNMATCH_DROP = 0,
    UNMATCH_TRAP,
    UNMATCH_ASSIGN,
    UNMATCH_END
} rtk_svlan_unmatch_action_t;

typedef struct rtk_mac_s
{
    uint8_t octet[ETHER_ADDR_LEN];
} rtk_mac_t;

/* l2 address table - unicast data structure */
typedef struct rtk_l2_ucastAddr_s
{
    rtk_mac_t mac;
    uint32_t ivl;
    uint32_t cvid;
    uint32_t fid;
    uint32_t efid;
    uint32_t port;
    uint32_t sa_block;
    uint32_t da_block;
    uint32_t auth;
    uint32_t is_static;
    uint32_t priority;
    uint32_t sa_pri_en;
    uint32_t fwd_pri_en;
    uint32_t address;
} rtk_l2_ucastAddr_t;

typedef struct ether_addr_s
{
    uint8_t octet[ETHER_ADDR_LEN];
} ether_addr_t;

typedef struct LUTTABLE
{

    uint32_t sip;
    uint32_t dip;
    ether_addr_t mac;
    uint16_t ivl_svl : 1;
    uint16_t cvid_fid : 12;
    uint16_t fid : 4;
    uint16_t efid : 3;

    uint16_t nosalearn : 1;
    uint16_t da_block : 1;
    uint16_t sa_block : 1;
    uint16_t auth : 1;
    uint16_t lut_pri : 3;
    uint16_t sa_en : 1;
    uint16_t fwd_en : 1;
    uint16_t mbr : 11;
    uint16_t spa : 4;
    uint16_t age : 3;
    uint16_t l3lookup : 1;
    uint16_t igmp_asic : 1;
    uint16_t igmpidx : 8;

    uint16_t lookup_hit : 1;
    uint16_t lookup_busy : 1;
    uint16_t address : 13;

    uint16_t l3vidlookup : 1;
    uint16_t l3_vid : 12;

    uint16_t wait_time;

} rtl8367c_luttb;

typedef enum rtk_l2_read_method_e
{

    READMETHOD_MAC = 0,
    READMETHOD_ADDRESS,
    READMETHOD_NEXT_ADDRESS,
    READMETHOD_NEXT_L2UC,
    READMETHOD_NEXT_L2MC,
    READMETHOD_NEXT_L3MC,
    READMETHOD_NEXT_L2L3MC,
    READMETHOD_NEXT_L2UCSPA,
    READMETHOD_END
} rtk_l2_read_method_t;

/* l2 address table - multicast data structure */
typedef struct rtk_l2_mcastAddr_s
{
    uint32_t vid;
    rtk_mac_t mac;
    uint32_t fid;
    rtk_portmask_t portmask;
    uint32_t ivl;
    uint32_t priority;
    uint32_t fwd_pri_en;
    uint32_t igmp_asic;
    uint32_t igmp_index;
    uint32_t address;
} rtk_l2_mcastAddr_t;

/* l2 address table - ip multicast data structure */
typedef struct rtk_l2_ipMcastAddr_s
{
    uint32_t dip;
    uint32_t sip;
    rtk_portmask_t portmask;
    uint32_t priority;
    uint32_t fwd_pri_en;
    uint32_t igmp_asic;
    uint32_t igmp_index;
    uint32_t address;
} rtk_l2_ipMcastAddr_t;

/* l2 address table - ip VID multicast data structure */
typedef struct rtk_l2_ipVidMcastAddr_s
{
    uint32_t dip;
    uint32_t sip;
    uint32_t vid;
    rtk_portmask_t portmask;
    uint32_t address;
} rtk_l2_ipVidMcastAddr_t;

enum FLOW_CONTROL_TYPE
{
    FC_EGRESS = 0,
    FC_INGRESS,
};

/* enum Priority Selection Index */
typedef enum rtk_qos_priDecTbl_e
{
    PRIDECTBL_IDX0 = 0,
    PRIDECTBL_IDX1,
    PRIDECTBL_END,
} rtk_qos_priDecTbl_t;

/* enum Priority Selection Types */
enum PRIDECISION
{
    PRIDEC_PORT = 0,
    PRIDEC_ACL,
    PRIDEC_DSCP,
    PRIDEC_1Q,
    PRIDEC_1AD,
    PRIDEC_CVLAN,
    PRIDEC_DA,
    PRIDEC_SA,
    PRIDEC_END,
};

/* enum Priority Selection Index */
enum RTL8367C_PRIDEC_TABLE
{
    PRIDEC_IDX0 = 0,
    PRIDEC_IDX1,
    PRIDEC_IDX_END,
};

typedef struct rtk_priority_select_s
{
    uint32_t port_pri;
    uint32_t dot1q_pri;
    uint32_t acl_pri;
    uint32_t dscp_pri;
    uint32_t cvlan_pri;
    uint32_t svlan_pri;
    uint32_t dmac_pri;
    uint32_t smac_pri;
} rtk_priority_select_t;

typedef struct rtk_qos_pri2queue_s
{
#define RTK_MAX_NUM_OF_PRIORITY 8
#define RTK_MAX_NUM_OF_QUEUE 8

    uint32_t pri2queue[RTK_MAX_NUM_OF_PRIORITY];
} rtk_qos_pri2queue_t;

typedef struct rtk_qos_queue_weights_s
{
    uint32_t weights[RTK_MAX_NUM_OF_QUEUE];
} rtk_qos_queue_weights_t;

/* enum for queue type */
enum QUEUETYPE
{
    QTYPE_STRICT = 0,
    QTYPE_WFQ,
};

typedef enum rtk_cpu_insert_e
{
    CPU_INSERT_TO_ALL = 0,
    CPU_INSERT_TO_TRAPPING,
    CPU_INSERT_TO_NONE,
    CPU_INSERT_END
} rtk_cpu_insert_t;

enum CPUTAG_INSERT_MODE
{
    CPUTAG_INSERT_TO_ALL = 0,
    CPUTAG_INSERT_TO_TRAPPING,
    CPUTAG_INSERT_TO_NO,
    CPUTAG_INSERT_END
};

typedef enum rtk_int_polarity_e
{
    INT_POLAR_HIGH = 0,
    INT_POLAR_LOW,
    INT_POLAR_END
} rtk_int_polarity_t;

typedef enum rtk_int_type_e
{
    INT_TYPE_LINK_STATUS = 0,
    INT_TYPE_METER_EXCEED,
    INT_TYPE_LEARN_LIMIT,
    INT_TYPE_LINK_SPEED,
    INT_TYPE_CONGEST,
    INT_TYPE_GREEN_FEATURE,
    INT_TYPE_LOOP_DETECT,
    INT_TYPE_8051,
    INT_TYPE_CABLE_DIAG,
    INT_TYPE_ACL,
    INT_TYPE_RESERVED, /* Unused */
    INT_TYPE_SLIENT,
    INT_TYPE_END
} rtk_int_type_t;

typedef struct rtk_int_status_s
{
    uint16_t value[RTK_MAX_NUM_OF_INTERRUPT_TYPE];
} rtk_int_status_t;

typedef enum rtk_int_advType_e
{
    ADV_L2_LEARN_PORT_MASK = 0,
    ADV_SPEED_CHANGE_PORT_MASK,
    ADV_SPECIAL_CONGESTION_PORT_MASK,
    ADV_PORT_LINKDOWN_PORT_MASK,
    ADV_PORT_LINKUP_PORT_MASK,
    ADV_METER_EXCEED_MASK,
    ADV_RLDP_LOOPED,
    ADV_RLDP_RELEASED,
    ADV_END,
} rtk_int_advType_t;

typedef struct rtk_int_info_s
{
    rtk_portmask_t portMask;
    uint32_t meterMask;
    uint32_t systemLearnOver;
} rtk_int_info_t;

typedef enum RTL8367C_INTR_INDICATOR_E
{
    INTRST_L2_LEARN = 0,
    INTRST_SPEED_CHANGE,
    INTRST_SPECIAL_CONGESTION,
    INTRST_PORT_LINKDOWN,
    INTRST_PORT_LINKUP,
    INTRST_METER0_15,
    INTRST_METER16_31,
    INTRST_RLDP_LOOPED,
    INTRST_RLDP_RELEASED,
    INTRST_SYS_LEARN,
    INTRST_END,
} RTL8367C_INTR_INDICATOR;

/* port statistic counter index */
typedef enum rtk_stat_port_type_e
{
    STAT_IfInOctets = 0,
    STAT_Dot3StatsFCSErrors,
    STAT_Dot3StatsSymbolErrors,
    STAT_Dot3InPauseFrames,
    STAT_Dot3ControlInUnknownOpcodes,
    STAT_EtherStatsFragments,
    STAT_EtherStatsJabbers,
    STAT_IfInUcastPkts,
    STAT_EtherStatsDropEvents,
    STAT_EtherStatsOctets,
    STAT_EtherStatsUnderSizePkts,
    STAT_EtherOversizeStats,
    STAT_EtherStatsPkts64Octets,
    STAT_EtherStatsPkts65to127Octets,
    STAT_EtherStatsPkts128to255Octets,
    STAT_EtherStatsPkts256to511Octets,
    STAT_EtherStatsPkts512to1023Octets,
    STAT_EtherStatsPkts1024to1518Octets,
    STAT_EtherStatsMulticastPkts,
    STAT_EtherStatsBroadcastPkts,
    STAT_IfOutOctets,
    STAT_Dot3StatsSingleCollisionFrames,
    STAT_Dot3StatsMultipleCollisionFrames,
    STAT_Dot3StatsDeferredTransmissions,
    STAT_Dot3StatsLateCollisions,
    STAT_EtherStatsCollisions,
    STAT_Dot3StatsExcessiveCollisions,
    STAT_Dot3OutPauseFrames,
    STAT_Dot1dBasePortDelayExceededDiscards,
    STAT_Dot1dTpPortInDiscards,
    STAT_IfOutUcastPkts,
    STAT_IfOutMulticastPkts,
    STAT_IfOutBroadcastPkts,
    STAT_OutOampduPkts,
    STAT_InOampduPkts,
    STAT_PktgenPkts,
    STAT_InMldChecksumError,
    STAT_InIgmpChecksumError,
    STAT_InMldSpecificQuery,
    STAT_InMldGeneralQuery,
    STAT_InIgmpSpecificQuery,
    STAT_InIgmpGeneralQuery,
    STAT_InMldLeaves,
    STAT_InIgmpInterfaceLeaves,
    STAT_InIgmpJoinsSuccess,
    STAT_InIgmpJoinsFail,
    STAT_InMldJoinsSuccess,
    STAT_InMldJoinsFail,
    STAT_InReportSuppressionDrop,
    STAT_InLeaveSuppressionDrop,
    STAT_OutIgmpReports,
    STAT_OutIgmpLeaves,
    STAT_OutIgmpGeneralQuery,
    STAT_OutIgmpSpecificQuery,
    STAT_OutMldReports,
    STAT_OutMldLeaves,
    STAT_OutMldGeneralQuery,
    STAT_OutMldSpecificQuery,
    STAT_InKnownMulticastPkts,
    STAT_IfInMulticastPkts,
    STAT_IfInBroadcastPkts,
    STAT_IfOutDiscards,
    STAT_PORT_CNTR_END
} rtk_stat_port_type_t;

typedef enum RTL8367C_MIBCOUNTER_E
{

    /* RX */
    ifInOctets = 0,

    dot3StatsFCSErrors,
    dot3StatsSymbolErrors,
    dot3InPauseFrames,
    dot3ControlInUnknownOpcodes,

    etherStatsFragments,
    etherStatsJabbers,
    ifInUcastPkts,
    etherStatsDropEvents,

    ifInMulticastPkts,
    ifInBroadcastPkts,
    inMldChecksumError,
    inIgmpChecksumError,
    inMldSpecificQuery,
    inMldGeneralQuery,
    inIgmpSpecificQuery,
    inIgmpGeneralQuery,
    inMldLeaves,
    inIgmpLeaves,

    /* TX/RX */
    etherStatsOctets,

    etherStatsUnderSizePkts,
    etherOversizeStats,
    etherStatsPkts64Octets,
    etherStatsPkts65to127Octets,
    etherStatsPkts128to255Octets,
    etherStatsPkts256to511Octets,
    etherStatsPkts512to1023Octets,
    etherStatsPkts1024to1518Octets,

    /* TX */
    ifOutOctets,

    dot3StatsSingleCollisionFrames,
    dot3StatMultipleCollisionFrames,
    dot3sDeferredTransmissions,
    dot3StatsLateCollisions,
    etherStatsCollisions,
    dot3StatsExcessiveCollisions,
    dot3OutPauseFrames,
    ifOutDiscards,

    /* ALE */
    dot1dTpPortInDiscards,
    ifOutUcastPkts,
    ifOutMulticastPkts,
    ifOutBroadcastPkts,
    outOampduPkts,
    inOampduPkts,

    inIgmpJoinsSuccess,
    inIgmpJoinsFail,
    inMldJoinsSuccess,
    inMldJoinsFail,
    inReportSuppressionDrop,
    inLeaveSuppressionDrop,
    outIgmpReports,
    outIgmpLeaves,
    outIgmpGeneralQuery,
    outIgmpSpecificQuery,
    outMldReports,
    outMldLeaves,
    outMldGeneralQuery,
    outMldSpecificQuery,
    inKnownMulticastPkts,

    /*Device only */
    dot1dTpLearnedEntryDiscards,
    RTL8367C_MIBS_NUMBER,

} RTL8367C_MIBCOUNTER;

typedef struct rtk_port_phy_ability_s
{
    uint32_t AutoNegotiation; /*PHY register 0.12 setting for auto-negotiation process*/
    uint32_t Half_10;         /*PHY register 4.5 setting for 10BASE-TX half duplex capable*/
    uint32_t Full_10;         /*PHY register 4.6 setting for 10BASE-TX full duplex capable*/
    uint32_t Half_100;        /*PHY register 4.7 setting for 100BASE-TX half duplex capable*/
    uint32_t Full_100;        /*PHY register 4.8 setting for 100BASE-TX full duplex capable*/
    uint32_t Full_1000;       /*PHY register 9.9 setting for 1000BASE-T full duplex capable*/
    uint32_t FC;              /*PHY register 4.10 setting for flow control capability*/
    uint32_t AsyFC;           /*PHY register 4.11 setting for  asymmetric flow control capability*/
} rtk_port_phy_ability_t;
typedef enum rtk_port_phy_reg_e
{
    PHY_REG_CONTROL = 0,
    PHY_REG_STATUS,
    PHY_REG_IDENTIFIER_1,
    PHY_REG_IDENTIFIER_2,
    PHY_REG_AN_ADVERTISEMENT,
    PHY_REG_AN_LINKPARTNER,
    PHY_REG_1000_BASET_CONTROL = 9,
    PHY_REG_1000_BASET_STATUS,
    PHY_REG_END = 32
} rtk_port_phy_reg_t;

typedef enum rtk_port_media_e
{
    PORT_MEDIA_COPPER = 0,
    PORT_MEDIA_FIBER,
    PORT_MEDIA_END
} rtk_port_media_t;

enum RTL8367C_LEDOP
{

    LEDOP_SCAN0 = 0,
    LEDOP_SCAN1,
    LEDOP_PARALLEL,
    LEDOP_SERIAL,
    LEDOP_END,
};

enum RTL8367C_LEDSERACT
{

    LEDSERACT_HIGH = 0,
    LEDSERACT_LOW,
    LEDSERACT_MAX,
};

enum RTL8367C_LEDSER
{

    LEDSER_16G = 0,
    LEDSER_8G,
    LEDSER_MAX,
};

enum RTL8367C_LEDCONF
{

    LEDCONF_LEDOFF = 0,
    LEDCONF_DUPCOL,
    LEDCONF_LINK_ACT,
    LEDCONF_SPD1000,
    LEDCONF_SPD100,
    LEDCONF_SPD10,
    LEDCONF_SPD1000ACT,
    LEDCONF_SPD100ACT,
    LEDCONF_SPD10ACT,
    LEDCONF_SPD10010ACT,
    LEDCONF_LOOPDETECT,
    LEDCONF_EEE,
    LEDCONF_LINKRX,
    LEDCONF_LINKTX,
    LEDCONF_MASTER,
    LEDCONF_ACT,
    LEDCONF_END
};

enum RTL8367C_LEDBLINKRATE
{

    LEDBLINKRATE_32MS = 0,
    LEDBLINKRATE_64MS,
    LEDBLINKRATE_128MS,
    LEDBLINKRATE_256MS,
    LEDBLINKRATE_512MS,
    LEDBLINKRATE_1024MS,
    LEDBLINKRATE_48MS,
    LEDBLINKRATE_96MS,
    LEDBLINKRATE_END,
};

enum RTL8367C_LEDFORCEMODE
{

    LEDFORCEMODE_NORMAL = 0,
    LEDFORCEMODE_BLINK,
    LEDFORCEMODE_OFF,
    LEDFORCEMODE_ON,
    LEDFORCEMODE_END,
};

enum RTL8367C_LEDFORCERATE
{

    LEDFORCERATE_512MS = 0,
    LEDFORCERATE_1024MS,
    LEDFORCERATE_2048MS,
    LEDFORCERATE_NORMAL,
    LEDFORCERATE_END,

};

enum RTL8367C_LEDMODE
{
    RTL8367C_LED_MODE_0 = 0,
    RTL8367C_LED_MODE_1,
    RTL8367C_LED_MODE_2,
    RTL8367C_LED_MODE_3,
    RTL8367C_LED_MODE_END
};