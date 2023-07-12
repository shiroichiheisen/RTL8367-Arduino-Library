#ifndef rtl8367_h
#define rtl8367_h
#include <Arduino.h>
#include "rtl8367c_errorCodes.h"
#include "rtl8367c_reg.h"
#include "rtl8367c_base.h"
#include "rtl8367c_def_types.h"

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

    int32_t rtk_vlan_set(uint32_t vid, rtk_vlan_cfg_t *pVlanCfg);
    int32_t rtk_vlan_get(uint32_t vid, rtk_vlan_cfg_t *pVlanCfg);

private:
    uint32_t vlan_mbrCfgVid[RTL8367C_CVIDXNO];
    vlan_mbrCfgType_t vlan_mbrCfgUsage[RTL8367C_CVIDXNO];
    uint16_t
        usTransmissionDelay;

    uint8_t
        sdaPin = 0,
        sckPin = 0;

    rtk_svlan_lookupType_t svlan_lookupType;
    uint8_t svlan_mbrCfgUsage[RTL8367C_SVIDXNO];
    uint16_t svlan_mbrCfgVid[RTL8367C_SVIDXNO];

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

    int32_t rtk_switch_logicalPortCheck(uint32_t logicalPort);
    int32_t rtk_switch_isPortMaskValid(rtk_portmask_t *pPmask);

#define RTK_SCAN_ALL_PHY_PORTMASK(__port__)                        \
    for (__port__ = 0; __port__ < RTK_SWITCH_PORT_NUM; __port__++) \
        if ((halCtrl.phy_portmask & (0x00000001 << __port__)))

#define RTK_CHK_PORT_VALID(__port__)                            \
    do                                                          \
    {                                                           \
        if (rtk_switch_logicalPortCheck(__port__) != RT_ERR_OK) \
        {                                                       \
            return RT_ERR_PORT_ID;                              \
        }                                                       \
    } while (0)

#define RTK_CHK_PORTMASK_VALID(__portmask__)                       \
    do                                                             \
    {                                                              \
        if (rtk_switch_isPortMaskValid(__portmask__) != RT_ERR_OK) \
        {                                                          \
            return RT_ERR_PORT_MASK;                               \
        }                                                          \
    } while (0)

#define RTK_PORTMASK_SCAN(__portmask__, __port__)                  \
    for (__port__ = 0; __port__ < RTK_SWITCH_PORT_NUM; __port__++) \
        if (RTK_PORTMASK_IS_PORT_SET(__portmask__, __port__))

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
    uint32_t rtk_switch_port_P2L_get(uint32_t physicalPort);
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
