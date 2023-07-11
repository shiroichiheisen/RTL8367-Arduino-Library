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

    int32_t rtk_switch_probe(uint8_t &);

    int32_t getPortStatus(uint8_t, uint8_t &, uint8_t &, uint8_t &);

    int32_t rtk_svlan_init();

#define RTK_TOTAL_NUM_OF_WORD_FOR_1BIT_PORT_LIST 1
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

    /* Function Name:
     *      rtk_switch_maxMeterId_get
     * Description:
     *      Get Max Meter ID
     * Input:
     *      None
     * Output:
     *      None
     * Return:
     *      0x00                - Not Initialize
     *      Other value         - Max Meter ID
     * Note:
     *
     */
    uint32_t rtk_switch_maxMeterId_get()
    {
        return (halCtrl.max_meter_id);
    }

#define RTK_MAX_METER_ID (rtk_switch_maxMeterId_get())

    int32_t rtk_vlan_set(uint32_t vid, rtk_vlan_cfg_t *pVlanCfg);
    int32_t _rtk_vlan_get(uint32_t vid, rtk_vlan_cfg_t *pVlanCfg);

private:
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

    /* Function Name:
     *      rtk_switch_phyPortMask_get
     * Description:
     *      Get physical portmask
     * Input:
     *      None
     * Output:
     *      None
     * Return:
     *      0x00                - Not Initialize
     *      Other value         - Physical port mask
     * Note:
     *
     */
    uint32_t rtk_switch_phyPortMask_get()
    {
        return (halCtrl.phy_portmask);
    }

#define RTK_SCAN_ALL_PHY_PORTMASK(__port__)                        \
    for (__port__ = 0; __port__ < RTK_SWITCH_PORT_NUM; __port__++) \
        if ((rtk_switch_phyPortMask_get() & (0x00000001 << __port__)))

    /* Function Name:
     *      rtk_switch_isPortMaskValid
     * Description:
     *      Check portmask is valid or not
     * Input:
     *      pPmask       - logical port mask
     * Output:
     *      None
     * Return:
     *      RT_ERR_OK           - port mask is valid
     *      RT_ERR_FAILED       - port mask is not valid
     *      RT_ERR_NOT_INIT     - Not Initialize
     *      RT_ERR_NULL_POINTER - Null pointer
     * Note:
     *
     */
    int32_t rtk_switch_isPortMaskValid(rtk_portmask_t *pPmask)
    {
        if ((pPmask->bits[0] | halCtrl.valid_portmask) != halCtrl.valid_portmask)
            return RT_ERR_FAILED;
        else
            return RT_ERR_OK;
    }

#define RTK_CHK_PORTMASK_VALID(__portmask__)                       \
    do                                                             \
    {                                                              \
        if (rtk_switch_isPortMaskValid(__portmask__) != RT_ERR_OK) \
        {                                                          \
            return RT_ERR_PORT_MASK;                               \
        }                                                          \
    } while (0)

#define RTL8367C_PORTNO 11
#define RTL8367C_PORTIDMAX (RTL8367C_PORTNO - 1)

#define RTL8367C_VLAN_PVID_CTRL_BASE RTL8367C_REG_VLAN_PVID_CTRL0
#define RTL8367C_VLAN_PVID_CTRL_REG(port) (RTL8367C_VLAN_PVID_CTRL_BASE + (port >> 1))
#define RTL8367C_PORT_VIDX_OFFSET(port) ((port & 1) << 3)
#define RTL8367C_PORT_VIDX_MASK(port) (RTL8367C_PORT0_VIDX_MASK << RTL8367C_PORT_VIDX_OFFSET(port))
#define RTL8367C_VLAN_EGRESS_MDOE_MASK RTL8367C_PORT0_MISC_CFG_VLAN_EGRESS_MODE_MASK
#define RTL8367C_VLAN_INGRESS_REG RTL8367C_REG_VLAN_INGRESS

/* Port mask defination */
#define RTK_PHY_PORTMASK_ALL (rtk_switch_phyPortMask_get())

#define RTL8367C_METERNO 64
#define RTL8367C_METERMAX (RTL8367C_METERNO - 1)
#define RTL8367C_TABLE_ACCESS_RDDATA_BASE RTL8367C_REG_TABLE_READ_DATA0

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

    uint16_t
        usTransmissionDelay;

    uint8_t
        sdaPin = 0,
        sckPin = 0;

    typedef enum vlan_mbrCfgType_e
    {
        MBRCFG_UNUSED = 0,
        MBRCFG_USED_BY_VLAN,
        MBRCFG_END
    } vlan_mbrCfgType_t;

    uint32_t vlan_mbrCfgVid[RTL8367C_CVIDXNO];
    vlan_mbrCfgType_t vlan_mbrCfgUsage[RTL8367C_CVIDXNO];

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

    typedef enum
    {
        EG_TAG_MODE_ORI = 0,
        EG_TAG_MODE_KEEP,
        EG_TAG_MODE_PRI_TAG,
        EG_TAG_MODE_REAL_KEEP,
        EG_TAG_MODE_END
    } rtl8367c_egtagmode;

    typedef enum rtk_enable_e
    {
        DISABLED_RTK = 0,
        ENABLED,
        RTK_ENABLE_END
    } rtk_enable_t;

#define RTK_PORTMASK_IS_PORT_SET(__portmask__, __port__) (((__portmask__).bits[0] & (0x00000001 << __port__)) ? 1 : 0)

#define RTK_PORTMASK_SCAN(__portmask__, __port__)                  \
    for (__port__ = 0; __port__ < RTK_SWITCH_PORT_NUM; __port__++) \
        if (RTK_PORTMASK_IS_PORT_SET(__portmask__, __port__))

#define RTL8367C_VLAN_BUSY_CHECK_NO (10)
#define RTL8367C_TABLE_ACCESS_STATUS_REG RTL8367C_REG_TABLE_LUT_ADDR
#define RTK_PORTMASK_CLEAR(__portmask__) ((__portmask__).bits[0] = 0)

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
    int32_t rtl8367c_setAsicRegBit(uint32_t, uint32_t, uint32_t);
    int32_t rtl8367c_getAsicRegBit(uint32_t, uint32_t, uint32_t *);

    /* Function Name:
     *      rtk_switch_port_P2L_get
     * Description:
     *      Get logical port ID
     * Input:
     *      physicalPort       - physical port ID
     * Output:
     *      None
     * Return:
     *      logical port ID
     * Note:
     *
     */
    uint32_t rtk_switch_port_P2L_get(uint32_t physicalPort)
    {
        if (physicalPort >= RTK_SWITCH_PORT_NUM)
            return UNDEFINE_PORT;

        return (halCtrl.p2l_port[physicalPort]);
    }

#define RTK_PORTMASK_PORT_SET(__portmask__, __port__) ((__portmask__).bits[0] |= (0x00000001 << __port__))

    int32_t rtl8367c_setAsicVlanMemberConfig(uint32_t, rtl8367c_vlanconfiguser *);
    void _rtl8367c_VlanMCStUser2Smi(rtl8367c_vlanconfiguser *, uint16_t *);
    int32_t rtl8367c_setAsicVlan4kEntry(rtl8367c_user_vlan4kentry *);
    void _rtl8367c_Vlan4kStUser2Smi(rtl8367c_user_vlan4kentry *, uint16_t *);
    int32_t rtl8367c_setAsicVlanPortBasedVID(uint32_t, uint32_t, uint32_t);
    int32_t rtl8367c_setAsicVlanEgressTagMode(uint32_t, rtl8367c_egtagmode);
    int32_t rtl8367c_setAsicVlanIngressFilter(uint32_t, uint32_t);
    int32_t rtl8367c_setAsicVlanFilter(uint32_t);
    int32_t rtk_switch_portmask_L2P_get(rtk_portmask_t *, uint32_t *);
    int32_t rtl8367c_getAsicVlan4kEntry(rtl8367c_user_vlan4kentry *);
    void _rtl8367c_Vlan4kStSmi2User(uint16_t *, rtl8367c_user_vlan4kentry *);
    int32_t rtk_switch_portmask_P2L_get(uint32_t, rtk_portmask_t *);
    int32_t rtl8367c_getAsicVlanMemberConfig(uint32_t, rtl8367c_vlanconfiguser *);
    void _rtl8367c_VlanMCStSmi2User(uint16_t *, rtl8367c_vlanconfiguser *);
};
#endif
