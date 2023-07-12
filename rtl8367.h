#ifndef utils_various
#define utils_various
#include <Arduino.h>
#include "errorTypes.h"
#include "rtl8367c_reg.h"
#include "rtl8367c_base.h"

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

typedef enum rtk_enable_e
{
    DISABLED_RTK = 0,
    ENABLED,
    RTK_ENABLE_END
} rtk_enable_t;

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

#define RTL8367C_DECISIONPRIMAX 0xFF

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

#define RTK_MAX_NUM_OF_INTERRUPT_TYPE 1
#define ADV_NOT_SUPPORT (0xFFFF)
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
#define PHY_CONTROL_REG 0

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

class rtl8367
{
public:
    rtl8367(uint16_t);

    void setTransmissionPins(uint8_t, uint8_t);
    void setTransmissionDelay(uint16_t);

    int32_t rtk_switch_probe(uint8_t &);

    int32_t rtk_port_phyStatus_get(uint8_t, uint8_t &, uint8_t &, uint8_t &);

    int32_t rtk_vlan_init();

    int32_t rtk_vlan_portPvid_set(rtk_port_t port, uint32_t pvid, uint32_t priority);
    int32_t rtk_vlan_portPvid_get(rtk_port_t port, uint32_t *pPvid, uint32_t *pPriority);

    int32_t rtk_vlan_portIgrFilterEnable_set(rtk_port_t port, rtk_enable_t igr_filter);

    int32_t rtk_vlan_portAcceptFrameType_set(rtk_port_t port, rtk_vlan_acceptFrameType_t accept_frame_type);

    int32_t rtk_vlan_tagMode_set(rtk_port_t port, rtl8367c_egtagmode tag_mode);

    int32_t rtk_vlan_transparent_set(rtk_port_t egr_port, rtk_portmask_t *pIgr_pmask);

    int32_t rtk_svlan_init();

    int32_t rtk_svlan_servicePort_add(rtk_port_t port);

    int32_t rtk_svlan_memberPortEntry_set(uint32_t svid, rtk_svlan_memberCfg_t *pSvlan_cfg);

    int32_t rtk_svlan_defaultSvlan_set(rtk_port_t port, uint32_t svid);

    int32_t rtk_svlan_c2s_add(uint32_t vid, rtk_port_t src_port, uint32_t svid);

    int32_t rtk_svlan_sp2c_add(uint32_t svid, rtk_port_t dst_port, uint32_t cvid);

    int32_t rtk_svlan_untag_action_set(rtk_svlan_untag_action_t action, uint32_t svid);

    int32_t rtk_svlan_unmatch_action_set(rtk_svlan_unmatch_action_t action, uint32_t svid);

    int32_t rtk_svlan_dmac_vidsel_set(rtk_port_t port, rtk_enable_t enable);

    int32_t rtk_l2_addr_add(rtk_mac_t *pMac, rtk_l2_ucastAddr_t *pL2_data);

    int32_t rtk_l2_addr_del(rtk_mac_t *pMac, rtk_l2_ucastAddr_t *pL2_data);

    int32_t rtk_l2_addr_get(rtk_mac_t *pMac, rtk_l2_ucastAddr_t *pL2_data);

    int32_t rtk_l2_addr_next_get(rtk_l2_read_method_t read_method, rtk_port_t port, uint32_t *pAddress, rtk_l2_ucastAddr_t *pL2_data);

    int32_t rtk_l2_mcastAddr_add(rtk_l2_mcastAddr_t *pMcastAddr);

    int32_t rtk_l2_mcastAddr_del(rtk_l2_mcastAddr_t *pMcastAddr);

    int32_t rtk_l2_mcastAddr_get(rtk_l2_mcastAddr_t *pMcastAddr);

    int32_t rtk_l2_mcastAddr_next_get(uint32_t *pAddress, rtk_l2_mcastAddr_t *pMcastAddr);

    int32_t rtk_l2_ipMcastAddr_add(rtk_l2_ipMcastAddr_t *pIpMcastAddr);

    int32_t rtk_l2_ipMcastAddr_del(rtk_l2_ipMcastAddr_t *pIpMcastAddr);

    int32_t rtk_l2_ipMcastAddr_get(rtk_l2_ipMcastAddr_t *pIpMcastAddr);

    int32_t rtk_l2_ipMcastAddr_next_get(uint32_t *pAddress, rtk_l2_ipMcastAddr_t *pIpMcastAddr);

    int32_t rtk_l2_ipVidMcastAddr_add(rtk_l2_ipVidMcastAddr_t *pIpVidMcastAddr);

    int32_t rtk_l2_ipVidMcastAddr_del(rtk_l2_ipVidMcastAddr_t *pIpVidMcastAddr);

    int32_t rtk_l2_ipVidMcastAddr_get(rtk_l2_ipVidMcastAddr_t *pIpVidMcastAddr);

    int32_t rtk_l2_ipVidMcastAddr_next_get(uint32_t *pAddress, rtk_l2_ipVidMcastAddr_t *pIpVidMcastAddr);

    int32_t rtk_qos_init(uint32_t queueNum);

    int32_t rtk_qos_portPri_set(rtk_port_t port, uint32_t int_pri);

    int32_t rtl8367c_setAsicFlowControlSelect(uint32_t select);

    int32_t rtk_qos_1pPriRemap_set(uint32_t dot1p_pri, uint32_t int_pri);

    int32_t rtk_qos_priSel_set(rtk_qos_priDecTbl_t index, rtk_priority_select_t *pPriDec);

    int32_t rtk_qos_portPriSelIndex_set(rtk_port_t port, rtk_qos_priDecTbl_t index);

    int32_t rtk_qos_priMap_set(uint32_t queue_num, rtk_qos_pri2queue_t *pPri2qid);

    int32_t rtk_qos_schedulingQueue_set(rtk_port_t port, rtk_qos_queue_weights_t *pQweights);

    int32_t rtk_cpu_enable_set(rtk_enable_t enable);

    int32_t rtk_cpu_tagPort_set(rtk_port_t port, rtk_cpu_insert_t mode);

    int32_t rtk_cpu_tagPort_get(rtk_port_t *pPort, rtk_cpu_insert_t *pMode);

    int32_t rtk_int_polarity_set(rtk_int_polarity_t type);

    int32_t rtk_int_polarity_get(rtk_int_polarity_t *pType);

    int32_t rtk_int_control_set(rtk_int_type_t type, rtk_enable_t enable);

    int32_t rtk_int_control_get(rtk_int_type_t type, rtk_enable_t *pEnable);

    int32_t rtk_int_status_get(rtk_int_status_t *pStatusMask);

    int32_t rtk_int_status_set(rtk_int_status_t *pStatusMask);

    int32_t rtk_int_advanceInfo_get(rtk_int_advType_t adv_type, rtk_int_info_t *pInfo);

    int32_t rtk_stat_port_get(rtk_port_t port, rtk_stat_port_type_t cntr_idx, uint64_t *pCntr);

    int32_t rtk_stat_port_reset(rtk_port_t port);

    int32_t rtk_port_phyEnableAll_set(rtk_enable_t enable);

    int32_t rtk_port_phyAutoNegoAbility_set(rtk_port_t port, rtk_port_phy_ability_t *pAbility);

    int32_t rtk_port_phyAutoNegoAbility_get(rtk_port_t port, rtk_port_phy_ability_t *pAbility);
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
    int32_t rtk_vlan_get(uint32_t vid, rtk_vlan_cfg_t *pVlanCfg);

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

    /* Function Name:
     *      rtk_switch_maxLutAddrNumber_get
     * Description:
     *      Get Max LUT Address number
     * Input:
     *      None
     * Output:
     *      None
     * Return:
     *      0x00                - Not Initialize
     *      Other value         - Max LUT Address number
     * Note:
     *
     */
    uint32_t rtk_switch_maxLutAddrNumber_get()
    {
        return (halCtrl.max_lut_addr_num);
    }

#define RTK_MAX_NUM_OF_LEARN_LIMIT (rtk_switch_maxLutAddrNumber_get())

#define RTK_MAC_ADDR_LEN 6
#define RTK_MAX_LUT_ADDRESS (RTK_MAX_NUM_OF_LEARN_LIMIT)
#define RTK_MAX_LUT_ADDR_ID (RTK_MAX_LUT_ADDRESS - 1)
#define RTL8367C_EFIDMAX 0x7

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

    /* Function Name:
     *      rtk_switch_logicalPortCheck
     * Description:
     *      Check logical port ID.
     * Input:
     *      logicalPort     - logical port ID
     * Output:
     *      None
     * Return:
     *      RT_ERR_OK       - Port ID is correct
     *      RT_ERR_FAILED   - Port ID is not correct
     *      RT_ERR_NOT_INIT - Not Initialize
     * Note:
     *
     */
    int32_t rtk_switch_logicalPortCheck(uint32_t logicalPort)
    {

        if (logicalPort >= RTK_SWITCH_PORT_NUM)
            return RT_ERR_FAILED;

        if (halCtrl.l2p_port[logicalPort] == 0xFF)
            return RT_ERR_FAILED;

        return RT_ERR_OK;
    }

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

#define RTL8367C_VLAN_ACCEPT_FRAME_TYPE_BASE RTL8367C_REG_VLAN_ACCEPT_FRAME_TYPE_CTRL0
#define RTL8367C_VLAN_ACCEPT_FRAME_TYPE_REG(port) (RTL8367C_VLAN_ACCEPT_FRAME_TYPE_BASE + (port >> 3))
#define RTL8367C_VLAN_ACCEPT_FRAME_TYPE_MASK(port) (RTL8367C_PORT0_FRAME_TYPE_MASK << ((port & 0x7) << 1))

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

    typedef enum rtk_svlan_lookupType_e
    {
        SVLAN_LOOKUP_S64MBRCGF = 0,
        SVLAN_LOOKUP_C4KVLAN,
        SVLAN_LOOKUP_END,

    } rtk_svlan_lookupType_t;

    rtk_svlan_lookupType_t svlan_lookupType;
    uint8_t svlan_mbrCfgUsage[RTL8367C_SVIDXNO];
    uint16_t svlan_mbrCfgVid[RTL8367C_SVIDXNO];

#define RTK_SCAN_ALL_LOG_PORT(__port__)                            \
    for (__port__ = 0; __port__ < RTK_SWITCH_PORT_NUM; __port__++) \
        if (rtk_switch_logicalPortCheck(__port__) == RT_ERR_OK)

#define RTK_CHK_PORT_IS_COMBO(__port__)                    \
    do                                                     \
    {                                                      \
        if (rtk_switch_isComboPort(__port__) != RT_ERR_OK) \
        {                                                  \
            return RT_ERR_PORT_ID;                         \
        }                                                  \
    } while (0)

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
    int32_t rtl8367c_getAsicRegBits(uint32_t, uint32_t, uint32_t *);
    int32_t rtl8367c_setAsicRegBit(uint32_t, uint32_t, uint32_t);
    int32_t rtl8367c_getAsicRegBit(uint32_t, uint32_t, uint32_t *);
    int32_t rtk_vlan_checkAndCreateMbr(uint32_t, uint32_t *);
    int32_t rtl8367c_getAsicVlanPortBasedVID(uint32_t, uint32_t *, uint32_t *);
    int32_t rtl8367c_setAsicVlanAccpetFrameType(uint32_t port, rtl8367c_accframetype frameType);
    int32_t rtl8367c_setAsicVlanTransparent(uint32_t port, uint32_t portmask);
    int32_t rtl8367c_setAsicSvlanPrioritySel(uint32_t priSel);
    int32_t rtl8367c_setAsicSvlanIngressUntag(uint32_t mode);
    int32_t rtl8367c_setAsicSvlanIngressUnmatch(uint32_t mode);
    int32_t rtl8367c_setAsicSvlanTpid(uint32_t protocolType);
    int32_t rtl8367c_setAsicSvlanUplinkPortMask(uint32_t portMask);
    int32_t rtl8367c_setAsicSvlanMemberConfiguration(uint32_t index, rtl8367c_svlan_memconf_t *pSvlanMemCfg);
    void _rtl8367c_svlanConfStUser2Smi(rtl8367c_svlan_memconf_t *pUserSt, uint16_t *pSmiSt);
    int32_t rtl8367c_setAsicSvlanC2SConf(uint32_t index, uint32_t evid, uint32_t portmask, uint32_t svidx);
    int32_t rtl8367c_setAsicSvlanSP2CConf(uint32_t index, rtl8367c_svlan_s2c_t *pSvlanSp2cCfg);
    void _rtl8367c_svlanSp2cStUser2Smi(rtl8367c_svlan_s2c_t *pUserSt, uint16_t *pSmiSt);
    int32_t rtl8367c_setAsicSvlanMC2SConf(uint32_t index, rtl8367c_svlan_mc2s_t *pSvlanMc2sCfg);
    void _rtl8367c_svlanMc2sStUser2Smi(rtl8367c_svlan_mc2s_t *pUserSt, uint16_t *pSmiSt);
    int32_t _rtk_svlan_lookupType_set(rtk_svlan_lookupType_t type);
    int32_t rtl8367c_setAsicSvlanLookupType(uint32_t type);
    int32_t rtl8367c_getAsicSvlanUplinkPortMask(uint32_t *pPortmask);
    int32_t rtk_svlan_tpidEntry_set(uint32_t svlan_tag_id);
    int32_t rtl8367c_getAsicSvlanMemberConfiguration(uint32_t index, rtl8367c_svlan_memconf_t *pSvlanMemCfg);
    void _rtl8367c_svlanConfStSmi2User(rtl8367c_svlan_memconf_t *pUserSt, uint16_t *pSmiSt);
    int32_t rtl8367c_setAsicSvlanDefaultVlan(uint32_t port, uint32_t index);
    int32_t rtl8367c_getAsicSvlanC2SConf(uint32_t index, uint32_t *pEvid, uint32_t *pPortmask, uint32_t *pSvidx);
    int32_t rtl8367c_getAsicSvlanSP2CConf(uint32_t index, rtl8367c_svlan_s2c_t *pSvlanSp2cCfg);
    void _rtl8367c_svlanSp2cStSmi2User(rtl8367c_svlan_s2c_t *pUserSt, uint16_t *pSmiSt);
    int32_t rtl8367c_setAsicSvlanUntagVlan(uint32_t index);
    int32_t rtl8367c_setAsicSvlanUnmatchVlan(uint32_t index);
    int32_t rtl8367c_setAsicSvlanDmacCvidSel(uint32_t port, uint32_t enabled);
    int32_t rtl8367c_getAsicL2LookupTb(uint32_t method, rtl8367c_luttb *pL2Table);
    void _rtl8367c_fdbStUser2Smi(rtl8367c_luttb *pLutSt, uint16_t *pFdbSmi);
    void _rtl8367c_fdbStSmi2User(rtl8367c_luttb *pLutSt, uint16_t *pFdbSmi);
    int32_t rtl8367c_setAsicL2LookupTb(rtl8367c_luttb *pL2Table);
    int32_t rtl8367c_setAsicOutputQueueMappingIndex(uint32_t port, uint32_t index);
    int32_t rtl8367c_setAsicPriorityToQIDMappingTable(uint32_t index, uint32_t priority, uint32_t qid);
    int32_t rtl8367c_setAsicPriorityPortBased(uint32_t port, uint32_t priority);
    int32_t rtl8367c_setAsicPriorityDecision(uint32_t index, uint32_t prisrc, uint32_t decisionPri);
    int32_t rtl8367c_setAsicRemarkingDot1pAbility(uint32_t port, uint32_t enabled);
    int32_t rtl8367c_setAsicRemarkingDscpAbility(uint32_t enabled);
    int32_t rtl8367c_setAsicPriorityDot1qRemapping(uint32_t srcpriority, uint32_t priority);
    int32_t rtl8367c_setAsicRemarkingDot1pParameter(uint32_t priority, uint32_t newPriority);
    int32_t rtl8367c_setAsicRemarkingDscpParameter(uint32_t priority, uint32_t newDscp);
    int32_t rtl8367c_setAsicPriorityDscpBased(uint32_t dscp, uint32_t priority);
    int32_t rtl8367c_setAsicPortPriorityDecisionIndex(uint32_t port, uint32_t index);
    int32_t rtl8367c_setAsicQueueType(uint32_t port, uint32_t qid, uint32_t queueType);
    int32_t rtl8367c_setAsicWFQWeight(uint32_t port, uint32_t qid, uint32_t qWeight);
    int32_t rtl8367c_setAsicCputagEnable(uint32_t enabled);
    int32_t rtl8367c_setAsicCputagPortmask(uint32_t portmask);
    int32_t rtl8367c_setAsicCputagTrapPort(uint32_t port);
    int32_t rtl8367c_setAsicCputagInsertMode(uint32_t mode);
    int32_t rtl8367c_getAsicCputagPortmask(uint32_t *pPortmask);
    int32_t rtl8367c_getAsicCputagTrapPort(uint32_t *pPort);
    int32_t rtl8367c_getAsicCputagInsertMode(uint32_t *pMode);
    int32_t rtl8367c_setAsicInterruptPolarity(uint32_t polarity);
    int32_t rtl8367c_getAsicInterruptPolarity(uint32_t *pPolarity);
    int32_t rtl8367c_getAsicInterruptMask(uint32_t *pImr);
    int32_t rtl8367c_setAsicInterruptMask(uint32_t imr);
    int32_t rtl8367c_getAsicInterruptStatus(uint32_t *pIms);
    int32_t rtl8367c_setAsicInterruptStatus(uint32_t ims);
    int32_t _rtk_int_Advidx_get(rtk_int_advType_t adv_type, uint32_t *pAsic_idx);
    int32_t rtl8367c_getAsicInterruptRelatedStatus(uint32_t type, uint32_t *pStatus);
    int32_t rtl8367c_setAsicInterruptRelatedStatus(uint32_t type, uint32_t status);
    int32_t _get_asic_mib_idx(rtk_stat_port_type_t cnt_idx, RTL8367C_MIBCOUNTER *pMib_idx);
    int32_t rtl8367c_getAsicMIBsCounter(uint32_t port, RTL8367C_MIBCOUNTER mibIdx, uint64_t *pCounter);
    int32_t rtl8367c_setAsicMIBsCounterReset(uint32_t greset, uint32_t qmreset, uint32_t portmask);
    int32_t rtl8367c_setAsicPortEnableAll(uint32_t enable);
    int32_t _rtk_port_phyReg_get(rtk_port_t port, rtk_port_phy_reg_t reg, uint32_t *pData);
    int32_t _rtk_port_phyReg_set(rtk_port_t port, rtk_port_phy_reg_t reg, uint32_t regData);
    int32_t rtl8367c_setAsicPHYReg(uint32_t phyNo, uint32_t phyAddr, uint32_t phyData);
    int32_t rtl8367c_setAsicPHYOCPReg(uint32_t phyNo, uint32_t ocpAddr, uint32_t ocpData);
    int32_t rtk_switch_isComboPort(rtk_port_t logicalPort);
    int32_t _rtk_port_phyComboPortMedia_get(rtk_port_t port, rtk_port_media_t *pMedia);
    int32_t _rtk_port_FiberModeAbility_set(rtk_port_t port, rtk_port_phy_ability_t *pAbility);
    int32_t _rtk_port_FiberModeAbility_get(rtk_port_t port, rtk_port_phy_ability_t *pAbility);
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
