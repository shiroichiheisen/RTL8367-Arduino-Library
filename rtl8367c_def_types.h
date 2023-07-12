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

#define RTL8367C_ACLRULENO 96

#define RTL8367C_ACLRULEMAX (RTL8367C_ACLRULENO - 1)
#define RTL8367C_ACLRULEFIELDNO 8
#define RTL8367C_ACLTEMPLATENO 5
#define RTL8367C_ACLTYPEMAX (RTL8367C_ACLTEMPLATENO - 1)

#define RTL8367C_ACLRULETBLEN 9
#define RTL8367C_ACLACTTBLEN 4
#define RTL8367C_ACLRULETBADDR(type, rule) ((type << 6) | rule)
#define RTL8367C_ACLRULETBADDR2(type, rule) ((type << 5) | (rule + 64))

#define ACL_ACT_CVLAN_ENABLE_MASK 0x1
#define ACL_ACT_SVLAN_ENABLE_MASK 0x2
#define ACL_ACT_PRIORITY_ENABLE_MASK 0x4
#define ACL_ACT_POLICING_ENABLE_MASK 0x8
#define ACL_ACT_FWD_ENABLE_MASK 0x10
#define ACL_ACT_INTGPIO_ENABLE_MASK 0x20

#define RTL8367C_ACLRULETAGBITS 5

#define RTL8367C_ACLRANGENO 16

#define RTL8367C_ACLRANGEMAX (RTL8367C_ACLRANGENO - 1)

#define RTL8367C_ACL_PORTRANGEMAX (0xFFFF)
#define RTL8367C_ACL_ACT_TABLE_LEN (4)

#define RTK_FILTER_RAW_FIELD_NUMBER 8

#define ACL_DEFAULT_ABILITY 0
#define ACL_DEFAULT_UNMATCH_PERMIT 1

#define ACL_RULE_FREE 0
#define ACL_RULE_INAVAILABLE 1
#define ACL_RULE_CARETAG_MASK 0x1F
#define FILTER_POLICING_MAX 4
#define FILTER_LOGGING_MAX 8
#define FILTER_PATTERN_MAX 4

#define FILTER_ENACT_CVLAN_MASK 0x01
#define FILTER_ENACT_SVLAN_MASK 0x02
#define FILTER_ENACT_PRIORITY_MASK 0x04
#define FILTER_ENACT_POLICING_MASK 0x08
#define FILTER_ENACT_FWD_MASK 0x10
#define FILTER_ENACT_INTGPIO_MASK 0x20
#define FILTER_ENACT_INIT_MASK 0x3F

#define RTK_MAX_NUM_OF_FILTER_TYPE 5
#define RTK_MAX_NUM_OF_FILTER_FIELD 8

#define RTK_DOT_1AS_TIMESTAMP_UNIT_IN_WORD_LENGTH 3UL
#define RTK_IPV6_ADDR_WORD_LENGTH 4UL

#define FILTER_ENACT_CVLAN_TYPE(type) (type - FILTER_ENACT_CVLAN_INGRESS)
#define FILTER_ENACT_SVLAN_TYPE(type) (type - FILTER_ENACT_SVLAN_INGRESS)
#define FILTER_ENACT_FWD_TYPE(type) (type - FILTER_ENACT_ADD_DSTPORT)
#define FILTER_ENACT_PRI_TYPE(type) (type - FILTER_ENACT_PRIORITY)

#define RTK_FILTER_FIELD_USED_MAX 8
#define RTK_FILTER_FIELD_INDEX(template, index) ((template << 4) + index)

#define RTL8367C_FIELDSEL_FORMAT_NUMBER (16)
#define RTL8367C_FIELDSEL_MAX_OFFSET (255)

#define RTL8367C_MIB_LEARNENTRYDISCARD_OFFSET (0x420)

#define RTL8367C_MAX_LOG_CNT_NUM (32)
#define RTL8367C_MIB_MAX_LOG_CNT_IDX (RTL8367C_MAX_LOG_CNT_NUM - 1)
#define RTL8367C_MIB_LOG_CNT_OFFSET (0x3E0)
#define RTL8367C_MIB_MAX_LOG_MODE_IDX (16 - 1)

typedef enum rtk_filter_act_cactext_e
{
    FILTER_ENACT_CACTEXT_VLANONLY = 0,
    FILTER_ENACT_CACTEXT_BOTHVLANTAG,
    FILTER_ENACT_CACTEXT_TAGONLY,
    FILTER_ENACT_CACTEXT_END,

} rtk_filter_act_cactext_t;

typedef enum rtk_filter_act_ctagfmt_e
{
    FILTER_CTAGFMT_UNTAG = 0,
    FILTER_CTAGFMT_TAG,
    FILTER_CTAGFMT_KEEP,
    FILTER_CTAGFMT_KEEP1PRMK,

} rtk_filter_act_ctag_t;

enum FIELDSEL_FORMAT_FORMAT
{
    FIELDSEL_FORMAT_DEFAULT = 0,
    FIELDSEL_FORMAT_RAW,
    FIELDSEL_FORMAT_LLC,
    FIELDSEL_FORMAT_IPV4,
    FIELDSEL_FORMAT_ARP,
    FIELDSEL_FORMAT_IPV6,
    FIELDSEL_FORMAT_IPPAYLOAD,
    FIELDSEL_FORMAT_L4PAYLOAD,
    FIELDSEL_FORMAT_END
};
typedef enum rtk_filter_act_enable_e
{
    /* CVLAN */
    FILTER_ENACT_CVLAN_INGRESS = 0,
    FILTER_ENACT_CVLAN_EGRESS,
    FILTER_ENACT_CVLAN_SVID,
    FILTER_ENACT_POLICING_1,

    /* SVLAN */
    FILTER_ENACT_SVLAN_INGRESS,
    FILTER_ENACT_SVLAN_EGRESS,
    FILTER_ENACT_SVLAN_CVID,
    FILTER_ENACT_POLICING_2,

    /* Policing and Logging */
    FILTER_ENACT_POLICING_0,

    /* Forward */
    FILTER_ENACT_COPY_CPU,
    FILTER_ENACT_DROP,
    FILTER_ENACT_ADD_DSTPORT,
    FILTER_ENACT_REDIRECT,
    FILTER_ENACT_MIRROR,
    FILTER_ENACT_TRAP_CPU,
    FILTER_ENACT_ISOLATION,

    /* QoS */
    FILTER_ENACT_PRIORITY,
    FILTER_ENACT_DSCP_REMARK,
    FILTER_ENACT_1P_REMARK,
    FILTER_ENACT_POLICING_3,

    /* Interrutp and GPO */
    FILTER_ENACT_INTERRUPT,
    FILTER_ENACT_GPO,

    /*VLAN tag*/
    FILTER_ENACT_EGRESSCTAG_UNTAG,
    FILTER_ENACT_EGRESSCTAG_TAG,
    FILTER_ENACT_EGRESSCTAG_KEEP,
    FILTER_ENACT_EGRESSCTAG_KEEPAND1PRMK,

    FILTER_ENACT_END,
} rtk_filter_act_enable_t;

typedef struct rtk_portmask_s
{
    uint32_t bits[RTK_TOTAL_NUM_OF_WORD_FOR_1BIT_PORT_LIST];
} rtk_portmask_t;

typedef struct
{
    rtk_filter_act_enable_t actEnable[FILTER_ENACT_END];

    /* CVLAN acton */
    uint32_t filterCvlanVid;
    uint32_t filterCvlanIdx;
    /* SVLAN action */
    uint32_t filterSvlanVid;
    uint32_t filterSvlanIdx;

    /* Policing action */
    uint32_t filterPolicingIdx[FILTER_POLICING_MAX];

    /* Forwarding action */
    rtk_portmask_t filterPortmask;

    /* QOS action */
    uint32_t filterPriority;

    /*GPO*/
    uint32_t filterPin;

} rtk_filter_action_t;

typedef struct rtk_filter_flag_s
{
    uint32_t value;
    uint32_t mask;
} rtk_filter_flag_t;

typedef enum rtk_filter_care_tag_index_e
{
    CARE_TAG_CTAG = 0,
    CARE_TAG_STAG,
    CARE_TAG_PPPOE,
    CARE_TAG_IPV4,
    CARE_TAG_IPV6,
    CARE_TAG_TCP,
    CARE_TAG_UDP,
    CARE_TAG_ARP,
    CARE_TAG_RSV1,
    CARE_TAG_RSV2,
    CARE_TAG_ICMP,
    CARE_TAG_IGMP,
    CARE_TAG_LLC,
    CARE_TAG_RSV3,
    CARE_TAG_HTTP,
    CARE_TAG_RSV4,
    CARE_TAG_RSV5,
    CARE_TAG_DHCP,
    CARE_TAG_DHCPV6,
    CARE_TAG_SNMP,
    CARE_TAG_OAM,
    CARE_TAG_END,
} rtk_filter_care_tag_index_t;

typedef struct rtk_filter_care_tag_s
{
    rtk_filter_flag_t tagType[CARE_TAG_END];
} rtk_filter_care_tag_t;

typedef struct rtk_filter_field rtk_filter_field_t;

typedef struct
{
    uint32_t value[RTK_DOT_1AS_TIMESTAMP_UNIT_IN_WORD_LENGTH];
} rtk_filter_dot1as_timestamp_t;

typedef enum rtk_filter_field_data_type_e
{
    FILTER_FIELD_DATA_MASK = 0,
    FILTER_FIELD_DATA_RANGE,
    FILTER_FIELD_DATA_END,
} rtk_filter_field_data_type_t;

typedef struct rtk_filter_ip_s
{
    uint32_t dataType;
    uint32_t rangeStart;
    uint32_t rangeEnd;
    uint32_t value;
    uint32_t mask;
} rtk_filter_ip_t;

typedef struct rtk_mac_s
{
    uint8_t octet[ETHER_ADDR_LEN];
} rtk_mac_t;

typedef struct rtk_filter_mac_s
{
    uint32_t dataType;
    rtk_mac_t value;
    rtk_mac_t mask;
    rtk_mac_t rangeStart;
    rtk_mac_t rangeEnd;
} rtk_filter_mac_t;

typedef struct rtk_filter_value_s
{
    uint32_t dataType;
    uint32_t value;
    uint32_t mask;
    uint32_t rangeStart;
    uint32_t rangeEnd;

} rtk_filter_value_t;

typedef struct rtk_filter_activeport_s
{
    rtk_portmask_t value;
    rtk_portmask_t mask;

} rtk_filter_activeport_t;

typedef struct rtk_filter_tag_s
{
    rtk_filter_value_t pri;
    rtk_filter_flag_t cfi;
    rtk_filter_value_t vid;
} rtk_filter_tag_t;

typedef struct rtk_filter_ipFlag_s
{
    rtk_filter_flag_t xf;
    rtk_filter_flag_t mf;
    rtk_filter_flag_t df;
} rtk_filter_ipFlag_t;

typedef struct
{
    uint32_t addr[RTK_IPV6_ADDR_WORD_LENGTH];
} rtk_filter_ip6_addr_t;

typedef struct
{
    uint32_t dataType;
    rtk_filter_ip6_addr_t value;
    rtk_filter_ip6_addr_t mask;
    rtk_filter_ip6_addr_t rangeStart;
    rtk_filter_ip6_addr_t rangeEnd;
} rtk_filter_ip6_t;

typedef struct rtk_filter_pattern_s
{
    uint32_t value[FILTER_PATTERN_MAX];
    uint32_t mask[FILTER_PATTERN_MAX];
} rtk_filter_pattern_t;

typedef struct rtk_filter_tcpFlag_s
{
    rtk_filter_flag_t urg;
    rtk_filter_flag_t ack;
    rtk_filter_flag_t psh;
    rtk_filter_flag_t rst;
    rtk_filter_flag_t syn;
    rtk_filter_flag_t fin;
    rtk_filter_flag_t ns;
    rtk_filter_flag_t cwr;
    rtk_filter_flag_t ece;
} rtk_filter_tcpFlag_t;

typedef enum rtk_filter_field_temple_input_e
{
    FILTER_FIELD_TEMPLE_INPUT_TYPE = 0,
    FILTER_FIELD_TEMPLE_INPUT_INDEX,
    FILTER_FIELD_TEMPLE_INPUT_MAX,
} rtk_filter_field_temple_input_t;

struct rtk_filter_field
{
    uint32_t fieldType;

    union
    {
        /* L2 struct */
        rtk_filter_mac_t dmac;
        rtk_filter_mac_t smac;
        rtk_filter_value_t etherType;
        rtk_filter_tag_t ctag;
        rtk_filter_tag_t relayCtag;
        rtk_filter_tag_t stag;
        rtk_filter_tag_t l2tag;
        rtk_filter_dot1as_timestamp_t dot1asTimeStamp;
        rtk_filter_mac_t mac;

        /* L3 struct */
        rtk_filter_ip_t sip;
        rtk_filter_ip_t dip;
        rtk_filter_ip_t ip;
        rtk_filter_value_t protocol;
        rtk_filter_value_t ipTos;
        rtk_filter_ipFlag_t ipFlag;
        rtk_filter_value_t ipOffset;
        rtk_filter_ip6_t sipv6;
        rtk_filter_ip6_t dipv6;
        rtk_filter_ip6_t ipv6;
        rtk_filter_value_t ipv6TrafficClass;
        rtk_filter_value_t ipv6NextHeader;
        rtk_filter_value_t flowLabel;

        /* L4 struct */
        rtk_filter_value_t tcpSrcPort;
        rtk_filter_value_t tcpDstPort;
        rtk_filter_tcpFlag_t tcpFlag;
        rtk_filter_value_t tcpSeqNumber;
        rtk_filter_value_t tcpAckNumber;
        rtk_filter_value_t udpSrcPort;
        rtk_filter_value_t udpDstPort;
        rtk_filter_value_t icmpCode;
        rtk_filter_value_t icmpType;
        rtk_filter_value_t igmpType;

        /* pattern match */
        rtk_filter_pattern_t pattern;

        rtk_filter_value_t inData;

    } filter_pattern_union;

    uint32_t fieldTemplateNo;
    uint32_t fieldTemplateIdx[RTK_FILTER_FIELD_USED_MAX];

    struct rtk_filter_field *next;
};

typedef enum rtk_filter_field_type_e
{
    FILTER_FIELD_DMAC = 0,
    FILTER_FIELD_SMAC,
    FILTER_FIELD_ETHERTYPE,
    FILTER_FIELD_CTAG,
    FILTER_FIELD_STAG,

    FILTER_FIELD_IPV4_SIP,
    FILTER_FIELD_IPV4_DIP,
    FILTER_FIELD_IPV4_TOS,
    FILTER_FIELD_IPV4_PROTOCOL,
    FILTER_FIELD_IPV4_FLAG,
    FILTER_FIELD_IPV4_OFFSET,
    FILTER_FIELD_IPV6_SIPV6,
    FILTER_FIELD_IPV6_DIPV6,
    FILTER_FIELD_IPV6_TRAFFIC_CLASS,
    FILTER_FIELD_IPV6_NEXT_HEADER,

    FILTER_FIELD_TCP_SPORT,
    FILTER_FIELD_TCP_DPORT,
    FILTER_FIELD_TCP_FLAG,
    FILTER_FIELD_UDP_SPORT,
    FILTER_FIELD_UDP_DPORT,
    FILTER_FIELD_ICMP_CODE,
    FILTER_FIELD_ICMP_TYPE,
    FILTER_FIELD_IGMP_TYPE,

    FILTER_FIELD_VID_RANGE,
    FILTER_FIELD_IP_RANGE,
    FILTER_FIELD_PORT_RANGE,

    FILTER_FIELD_USER_DEFINED00,
    FILTER_FIELD_USER_DEFINED01,
    FILTER_FIELD_USER_DEFINED02,
    FILTER_FIELD_USER_DEFINED03,
    FILTER_FIELD_USER_DEFINED04,
    FILTER_FIELD_USER_DEFINED05,
    FILTER_FIELD_USER_DEFINED06,
    FILTER_FIELD_USER_DEFINED07,
    FILTER_FIELD_USER_DEFINED08,
    FILTER_FIELD_USER_DEFINED09,
    FILTER_FIELD_USER_DEFINED10,
    FILTER_FIELD_USER_DEFINED11,
    FILTER_FIELD_USER_DEFINED12,
    FILTER_FIELD_USER_DEFINED13,
    FILTER_FIELD_USER_DEFINED14,
    FILTER_FIELD_USER_DEFINED15,

    FILTER_FIELD_PATTERN_MATCH,

    FILTER_FIELD_END,
} rtk_filter_field_type_t;

typedef enum rtk_filter_field_type_raw_e
{
    FILTER_FIELD_RAW_UNUSED = 0,
    FILTER_FIELD_RAW_DMAC_15_0,
    FILTER_FIELD_RAW_DMAC_31_16,
    FILTER_FIELD_RAW_DMAC_47_32,
    FILTER_FIELD_RAW_SMAC_15_0,
    FILTER_FIELD_RAW_SMAC_31_16,
    FILTER_FIELD_RAW_SMAC_47_32,
    FILTER_FIELD_RAW_ETHERTYPE,
    FILTER_FIELD_RAW_STAG,
    FILTER_FIELD_RAW_CTAG,

    FILTER_FIELD_RAW_IPV4_SIP_15_0 = 0x10,
    FILTER_FIELD_RAW_IPV4_SIP_31_16,
    FILTER_FIELD_RAW_IPV4_DIP_15_0,
    FILTER_FIELD_RAW_IPV4_DIP_31_16,

    FILTER_FIELD_RAW_IPV6_SIP_15_0 = 0x20,
    FILTER_FIELD_RAW_IPV6_SIP_31_16,
    FILTER_FIELD_RAW_IPV6_DIP_15_0 = 0x28,
    FILTER_FIELD_RAW_IPV6_DIP_31_16,

    FILTER_FIELD_RAW_VIDRANGE = 0x30,
    FILTER_FIELD_RAW_IPRANGE,
    FILTER_FIELD_RAW_PORTRANGE,
    FILTER_FIELD_RAW_FIELD_VALID,

    FILTER_FIELD_RAW_FIELD_SELECT00 = 0x40,
    FILTER_FIELD_RAW_FIELD_SELECT01,
    FILTER_FIELD_RAW_FIELD_SELECT02,
    FILTER_FIELD_RAW_FIELD_SELECT03,
    FILTER_FIELD_RAW_FIELD_SELECT04,
    FILTER_FIELD_RAW_FIELD_SELECT05,
    FILTER_FIELD_RAW_FIELD_SELECT06,
    FILTER_FIELD_RAW_FIELD_SELECT07,
    FILTER_FIELD_RAW_FIELD_SELECT08,
    FILTER_FIELD_RAW_FIELD_SELECT09,
    FILTER_FIELD_RAW_FIELD_SELECT10,
    FILTER_FIELD_RAW_FIELD_SELECT11,
    FILTER_FIELD_RAW_FIELD_SELECT12,
    FILTER_FIELD_RAW_FIELD_SELECT13,
    FILTER_FIELD_RAW_FIELD_SELECT14,
    FILTER_FIELD_RAW_FIELD_SELECT15,

    FILTER_FIELD_RAW_END,
} rtk_filter_field_type_raw_t;

typedef enum rtk_filter_flag_care_type_e
{
    FILTER_FLAG_CARE_DONT_CARE = 0,
    FILTER_FLAG_CARE_1,
    FILTER_FLAG_CARE_0,
    FILTER_FLAG_END
} rtk_filter_flag_care_type_t;

typedef uint32_t rtk_filter_id_t; /* filter id type */

typedef enum rtk_filter_invert_e
{
    FILTER_INVERT_DISABLE = 0,
    FILTER_INVERT_ENABLE,
    FILTER_INVERT_END,
} rtk_filter_invert_t;

typedef uint32_t rtk_filter_state_t;

typedef uint32_t rtk_filter_unmatch_action_t;

typedef enum rtk_filter_unmatch_action_e
{
    FILTER_UNMATCH_DROP = 0,
    FILTER_UNMATCH_PERMIT,
    FILTER_UNMATCH_END,
} rtk_filter_unmatch_action_type_t;

typedef struct
{
    rtk_filter_field_t *fieldHead;
    rtk_filter_care_tag_t careTag;
    rtk_filter_activeport_t activeport;

    rtk_filter_invert_t invert;
} rtk_filter_cfg_t;

typedef enum rtk_enable_e
{
    DISABLED_RTK = 0,
    ENABLED,
    RTK_ENABLE_END
} rtk_enable_t;

typedef struct
{
    uint32_t dataFieldRaw[RTK_FILTER_RAW_FIELD_NUMBER];
    uint32_t careFieldRaw[RTK_FILTER_RAW_FIELD_NUMBER];
    rtk_filter_field_type_raw_t fieldRawType[RTK_FILTER_RAW_FIELD_NUMBER];
    rtk_filter_care_tag_t careTag;
    rtk_filter_activeport_t activeport;

    rtk_filter_invert_t invert;
    rtk_enable_t valid;
} rtk_filter_cfg_raw_t;

typedef struct
{
    uint32_t index;
    rtk_filter_field_type_raw_t fieldType[RTK_FILTER_RAW_FIELD_NUMBER];
} rtk_filter_template_t;

typedef enum rtk_field_sel_e
{
    FORMAT_DEFAULT = 0,
    FORMAT_RAW,
    FORMAT_LLC,
    FORMAT_IPV4,
    FORMAT_ARP,
    FORMAT_IPV6,
    FORMAT_IPPAYLOAD,
    FORMAT_L4PAYLOAD,
    FORMAT_END
} rtk_field_sel_t;

typedef enum rtk_filter_iprange_e
{
    IPRANGE_UNUSED = 0,
    IPRANGE_IPV4_SIP,
    IPRANGE_IPV4_DIP,
    IPRANGE_IPV6_SIP,
    IPRANGE_IPV6_DIP,
    IPRANGE_END
} rtk_filter_iprange_t;

typedef enum rtk_filter_vidrange_e
{
    VIDRANGE_UNUSED = 0,
    VIDRANGE_CVID,
    VIDRANGE_SVID,
    VIDRANGE_END
} rtk_filter_vidrange_t;

typedef enum rtk_filter_portrange_e
{
    PORTRANGE_UNUSED = 0,
    PORTRANGE_SPORT,
    PORTRANGE_DPORT,
    PORTRANGE_END
} rtk_filter_portrange_t;

enum ACLTCAMTYPES
{
    CAREBITS = 0,
    DATABITS
};

typedef enum aclFwdAct
{
    RTL8367C_ACL_FWD_MIRROR = 0,
    RTL8367C_ACL_FWD_REDIRECT,
    RTL8367C_ACL_FWD_MIRRORFUNTION,
    RTL8367C_ACL_FWD_TRAP,
} rtl8367c_aclFwd_t;

enum ACLFIELDTYPES
{
    ACL_UNUSED,
    ACL_DMAC0,
    ACL_DMAC1,
    ACL_DMAC2,
    ACL_SMAC0,
    ACL_SMAC1,
    ACL_SMAC2,
    ACL_ETHERTYPE,
    ACL_STAG,
    ACL_CTAG,
    ACL_IP4SIP0 = 0x10,
    ACL_IP4SIP1,
    ACL_IP4DIP0,
    ACL_IP4DIP1,
    ACL_IP6SIP0WITHIPV4 = 0x20,
    ACL_IP6SIP1WITHIPV4,
    ACL_IP6DIP0WITHIPV4 = 0x28,
    ACL_IP6DIP1WITHIPV4,
    ACL_VIDRANGE = 0x30,
    ACL_IPRANGE,
    ACL_PORTRANGE,
    ACL_FIELD_VALID,
    ACL_FIELD_SELECT00 = 0x40,
    ACL_FIELD_SELECT01,
    ACL_FIELD_SELECT02,
    ACL_FIELD_SELECT03,
    ACL_FIELD_SELECT04,
    ACL_FIELD_SELECT05,
    ACL_FIELD_SELECT06,
    ACL_FIELD_SELECT07,
    ACL_FIELD_SELECT08,
    ACL_FIELD_SELECT09,
    ACL_FIELD_SELECT10,
    ACL_FIELD_SELECT11,
    ACL_FIELD_SELECT12,
    ACL_FIELD_SELECT13,
    ACL_FIELD_SELECT14,
    ACL_FIELD_SELECT15,
    ACL_TCPSPORT = 0x80,
    ACL_TCPDPORT,
    ACL_TCPFLAG,
    ACL_UDPSPORT,
    ACL_UDPDPORT,
    ACL_ICMPCODETYPE,
    ACL_IGMPTYPE,
    ACL_SPORT,
    ACL_DPORT,
    ACL_IP4TOSPROTO,
    ACL_IP4FLAGOFF,
    ACL_TCNH,
    ACL_CPUTAG,
    ACL_L2PAYLOAD,
    ACL_IP6SIP0,
    ACL_IP6SIP1,
    ACL_IP6SIP2,
    ACL_IP6SIP3,
    ACL_IP6SIP4,
    ACL_IP6SIP5,
    ACL_IP6SIP6,
    ACL_IP6SIP7,
    ACL_IP6DIP0,
    ACL_IP6DIP1,
    ACL_IP6DIP2,
    ACL_IP6DIP3,
    ACL_IP6DIP4,
    ACL_IP6DIP5,
    ACL_IP6DIP6,
    ACL_IP6DIP7,
    ACL_TYPE_END
};

struct acl_rule_smi_st
{
    uint16_t rule_info;
    uint16_t field[RTL8367C_ACLRULEFIELDNO];
};

struct acl_rule_smi_ext_st
{
    uint16_t rule_info;
};

typedef struct ACLRULESMI
{
    struct acl_rule_smi_st care_bits;
    uint16_t valid : 1;
    struct acl_rule_smi_st data_bits;

    struct acl_rule_smi_ext_st care_bits_ext;
    struct acl_rule_smi_ext_st data_bits_ext;
} rtl8367c_aclrulesmi;

struct acl_rule_st
{
    uint16_t active_portmsk : 11;
    uint16_t type : 3;
    uint16_t tag_exist : 5;
    uint16_t field[RTL8367C_ACLRULEFIELDNO];
};

typedef struct ACLRULE
{
    struct acl_rule_st data_bits;
    uint16_t valid : 1;
    struct acl_rule_st care_bits;
} rtl8367c_aclrule;

typedef struct rtl8367c_acltemplate_s
{
    uint8_t field[8];
} rtl8367c_acltemplate_t;

typedef struct acl_act_s
{
    uint16_t cvidx_cact : 7;
    uint16_t cact : 2;
    uint16_t svidx_sact : 7;
    uint16_t sact : 2;

    uint16_t aclmeteridx : 7;
    uint16_t fwdpmask : 11;
    uint16_t fwdact : 2;

    uint16_t pridx : 7;
    uint16_t priact : 2;
    uint16_t gpio_pin : 4;
    uint16_t gpio_en : 1;
    uint16_t aclint : 1;

    uint16_t cact_ext : 2;
    uint16_t fwdact_ext : 1;
    uint16_t tag_fmt : 2;
} rtl8367c_acl_act_t;

typedef struct acl_rule_union_s
{
    rtl8367c_aclrule aclRule;
    rtl8367c_acl_act_t aclAct;
    uint32_t aclActCtrl;
    uint32_t aclNot;
} rtl8367c_acl_rule_union_t;

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