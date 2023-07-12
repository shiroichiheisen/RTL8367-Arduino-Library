#include "rtl8367.h"
#include "rtl8367c_i2cPart.h"

rtl8367::rtl8367(uint16_t usTransmissionDelay)
{
    this->usTransmissionDelay = usTransmissionDelay;
}

void rtl8367::setTransmissionPins(uint8_t sckPin, uint8_t sdaPin)
{
    this->sdaPin = sdaPin;
    this->sckPin = sckPin;

    pinMode(sdaPin, OUTPUT);
    pinMode(sckPin, OUTPUT);
}

void rtl8367::setTransmissionDelay(uint16_t usTransmissionDelay)
{
    this->usTransmissionDelay = usTransmissionDelay;
}

int32_t rtl8367::rtk_switch_logicalPortCheck(uint32_t logicalPort)
{

    if (logicalPort >= RTK_SWITCH_PORT_NUM)
        return RT_ERR_FAILED;

    if (halCtrl.l2p_port[logicalPort] == 0xFF)
        return RT_ERR_FAILED;

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_switch_isPortMaskValid(rtk_portmask_t *pPmask)
{
    if ((pPmask->bits[0] | halCtrl.valid_portmask) != halCtrl.valid_portmask)
        return RT_ERR_FAILED;
    else
        return RT_ERR_OK;
}

/* Function Name:
 *      rtk_switch_probe
 * Description:
 *      Probe switch
 * Input:
 *      None
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Switch probed
 *      RT_ERR_FAILED   - Switch Unprobed.
 * Note:
 *
 */
int32_t rtl8367::rtk_switch_probe(uint8_t &pSwitchChip)
{
    uint32_t retVal;
    uint32_t data, regValue;

    if ((retVal = rtl8367c_setAsicReg(0x13C2, 0x0249)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_getAsicReg(0x1300, &data)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_getAsicReg(0x1301, &regValue)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicReg(0x13C2, 0x0000)) != RT_ERR_OK)
        return retVal;

    switch (data)
    {
    case 0x0276:
    case 0x0597:
    case 0x6367:
        Serial.println("RTL8367C");
        pSwitchChip = 0;
        break;
    default:
        return RT_ERR_FAILED;
    }

    // return RT_ERR_OK;
    return retVal;
}

/* Function Name:
 *      rtk_switch_isUtpPort
 * Description:
 *      Check is logical port a UTP port
 * Input:
 *      logicalPort     - logical port ID
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Port ID is a UTP port
 *      RT_ERR_FAILED   - Port ID is not a UTP port
 *      RT_ERR_NOT_INIT - Not Initialize
 * Note:
 *
 */
int32_t rtl8367::rtk_switch_isUtpPort(uint8_t logicalPort)
{
    if (logicalPort >= RTK_SWITCH_PORT_NUM)
        return RT_ERR_FAILED;

    if (halCtrl.log_port_type[logicalPort] == UTP_PORT)
        return RT_ERR_OK;
    else
        return RT_ERR_FAILED;
}

#define RTK_CHK_PORT_IS_UTP(__port__)                    \
    do                                                   \
    {                                                    \
        if (rtk_switch_isUtpPort(__port__) != RT_ERR_OK) \
        {                                                \
            return RT_ERR_PORT_ID;                       \
        }                                                \
    } while (0)

/* Function Name:
 *      rtk_switch_port_L2P_get
 * Description:
 *      Get physical port ID
 * Input:
 *      logicalPort       - logical port ID
 * Output:
 *      None
 * Return:
 *      Physical port ID
 * Note:
 *
 */
uint32_t rtl8367::rtk_switch_port_L2P_get(uint8_t logicalPort)
{
    if (logicalPort >= RTK_SWITCH_PORT_NUM)
        return UNDEFINE_PHY_PORT;

    return (halCtrl.l2p_port[logicalPort]);
}

/* Function Name:
 *      rtl8367c_getAsicPHYOCPReg
 * Description:
 *      Get PHY OCP registers
 * Input:
 *      phyNo   - Physical port number (0~7)
 *      ocpAddr - PHY address
 *      pRegData - read data
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK               - Success
 *      RT_ERR_SMI              - SMI access error
 *      RT_ERR_PHY_REG_ID       - invalid PHY address
 *      RT_ERR_PHY_ID           - invalid PHY no
 *      RT_ERR_BUSYWAIT_TIMEOUT - PHY access busy
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicPHYOCPReg(uint32_t phyNo, uint32_t ocpAddr, uint32_t *pRegData)
{
    int32_t retVal;
    uint32_t regData;
    uint32_t busyFlag, checkCounter;
    uint32_t ocpAddrPrefix, ocpAddr9_6, ocpAddr5_1;
    /*Check internal phy access busy or not*/
    /*retVal = rtl8367c_getAsicRegBit(RTL8367C_REG_INDRECT_ACCESS_STATUS, RTL8367C_INDRECT_ACCESS_STATUS_OFFSET,&busyFlag);*/
    retVal = rtl8367c_getAsicReg(RTL8367C_REG_INDRECT_ACCESS_STATUS, &busyFlag);
    if (retVal != RT_ERR_OK)
        return retVal;

    if (busyFlag)
        return RT_ERR_BUSYWAIT_TIMEOUT;

    /* OCP prefix */
    ocpAddrPrefix = ((ocpAddr & 0xFC00) >> 10);
    if ((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_GPHY_OCP_MSB_0, RTL8367C_CFG_CPU_OCPADR_MSB_MASK, ocpAddrPrefix)) != RT_ERR_OK)
        return retVal;

    /*prepare access address*/
    ocpAddr9_6 = ((ocpAddr >> 6) & 0x000F);
    ocpAddr5_1 = ((ocpAddr >> 1) & 0x001F);
    regData = RTL8367C_PHY_BASE | (ocpAddr9_6 << 8) | (phyNo << RTL8367C_PHY_OFFSET) | ocpAddr5_1;
    retVal = rtl8367c_setAsicReg(RTL8367C_REG_INDRECT_ACCESS_ADDRESS, regData);
    if (retVal != RT_ERR_OK)
        return retVal;

    /*Set READ Command*/
    retVal = rtl8367c_setAsicReg(RTL8367C_REG_INDRECT_ACCESS_CTRL, RTL8367C_CMD_MASK);
    if (retVal != RT_ERR_OK)
        return retVal;

    checkCounter = 100;
    while (checkCounter)
    {
        retVal = rtl8367c_getAsicReg(RTL8367C_REG_INDRECT_ACCESS_STATUS, &busyFlag);
        if ((retVal != RT_ERR_OK) || busyFlag)
        {
            checkCounter--;
            if (0 == checkCounter)
                return RT_ERR_FAILED;
        }
        else
        {
            checkCounter = 0;
        }
    }

    /*get PHY register*/
    retVal = rtl8367c_getAsicReg(RTL8367C_REG_INDRECT_ACCESS_READ_DATA, &regData);
    if (retVal != RT_ERR_OK)
        return retVal;

    *pRegData = regData;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_getAsicPHYReg
 * Description:
 *      Get PHY registers
 * Input:
 *      phyNo   - Physical port number (0~7)
 *      phyAddr - PHY address (0~31)
 *      pRegData - Writing data
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK               - Success
 *      RT_ERR_SMI              - SMI access error
 *      RT_ERR_PHY_REG_ID       - invalid PHY address
 *      RT_ERR_PHY_ID           - invalid PHY no
 *      RT_ERR_BUSYWAIT_TIMEOUT - PHY access busy
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicPHYReg(uint32_t phyNo, uint32_t phyAddr, uint32_t *pRegData)
{
    uint32_t ocp_addr;

    if (phyAddr > RTL8367C_PHY_REGNOMAX)
        return RT_ERR_PHY_REG_ID;

    ocp_addr = 0xa400 + phyAddr * 2;

    return rtl8367c_getAsicPHYOCPReg(phyNo, ocp_addr, pRegData);
}

/* Function Name:
 *      rtk_port_phyStatus_get
 * Description:
 *      Get ethernet PHY linking status
 * Input:
 *      port - Port id.
 * Output:
 *      linkStatus  - PHY link status
 *      speed       - PHY link speed
 *      duplex      - PHY duplex mode
 * Return:
 *      RT_ERR_OK               - OK
 *      RT_ERR_FAILED           - Failed
 *      RT_ERR_SMI              - SMI access error
 *      RT_ERR_PORT_ID          - Invalid port number.
 *      RT_ERR_PHY_REG_ID       - Invalid PHY address
 *      RT_ERR_INPUT            - Invalid input parameters.
 *      RT_ERR_BUSYWAIT_TIMEOUT - PHY access busy
 * Note:
 *      API will return auto negotiation status of phy.
 */
int32_t rtl8367::rtk_port_phyStatus_get(uint8_t port, uint8_t &pLinkStatus, uint8_t &pSpeed, uint8_t &pDuplex)
{
    int32_t retVal;
    uint32_t phyData;

    /* Check Port Valid */
    RTK_CHK_PORT_IS_UTP(port);

    /*Get PHY resolved register*/
    if ((retVal = rtl8367c_getAsicPHYReg(rtk_switch_port_L2P_get(port), PHY_RESOLVED_REG, &phyData)) != RT_ERR_OK)
        return retVal;

    /*check link status*/
    if (phyData & (1 << 2))
    {
        pLinkStatus = 1;

        /*check link speed*/
        pSpeed = (phyData & 0x0030) >> 4;

        /*check link duplex*/
        pDuplex = (phyData & 0x0008) >> 3;
    }
    else
    {
        pLinkStatus = 0;
        pSpeed = 0;
        pDuplex = 0;
    }

    return RT_ERR_OK;
}

////////////////// Vlan Part

#include "Arduino.h"
#include "rtl8367.h"

void rtl8367::_rtl8367c_VlanMCStUser2Smi(rtl8367c_vlanconfiguser *pVlanCg, uint16_t *pSmiVlanCfg)
{
    pSmiVlanCfg[0] |= pVlanCg->mbr & 0x07FF;

    pSmiVlanCfg[1] |= pVlanCg->fid_msti & 0x000F;

    pSmiVlanCfg[2] |= pVlanCg->vbpen & 0x0001;
    pSmiVlanCfg[2] |= (pVlanCg->vbpri & 0x0007) << 1;
    pSmiVlanCfg[2] |= (pVlanCg->envlanpol & 0x0001) << 4;
    pSmiVlanCfg[2] |= (pVlanCg->meteridx & 0x003F) << 5;

    pSmiVlanCfg[3] |= pVlanCg->evid & 0x1FFF;
}

/* Function Name:
 *      rtl8367c_setAsicVlanMemberConfig
 * Description:
 *      Set 32 VLAN member configurations
 * Input:
 *      index       - VLAN member configuration index (0~31)
 *      pVlanCg - VLAN member configuration
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK                   - Success
 *      RT_ERR_SMI                  - SMI access error
 *      RT_ERR_INPUT                - Invalid input parameter
 *      RT_ERR_L2_FID               - Invalid FID
 *      RT_ERR_PORT_MASK            - Invalid portmask
 *      RT_ERR_FILTER_METER_ID      - Invalid meter
 *      RT_ERR_QOS_INT_PRIORITY     - Invalid priority
 *      RT_ERR_VLAN_ENTRY_NOT_FOUND - Invalid VLAN member configuration index
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicVlanMemberConfig(uint32_t index, rtl8367c_vlanconfiguser *pVlanCg)
{
    int32_t retVal;
    uint32_t regAddr;
    uint32_t regData;
    uint16_t *tableAddr;
    uint32_t page_idx;
    uint16_t smi_vlancfg[RTL8367C_VLAN_MBRCFG_LEN];

    /* Error Checking  */
    if (index > RTL8367C_CVIDXMAX)
        return RT_ERR_VLAN_ENTRY_NOT_FOUND;

    if (pVlanCg->evid > RTL8367C_EVIDMAX)
        return RT_ERR_INPUT;

    if (pVlanCg->mbr > RTL8367C_PORTMASK)
        return RT_ERR_PORT_MASK;

    if (pVlanCg->fid_msti > RTL8367C_FIDMAX)
        return RT_ERR_L2_FID;

    if (pVlanCg->meteridx > RTL8367C_METERMAX)
        return RT_ERR_FILTER_METER_ID;

    if (pVlanCg->vbpri > RTL8367C_PRIMAX)
        return RT_ERR_QOS_INT_PRIORITY;

    memset(smi_vlancfg, 0x00, sizeof(uint16_t) * RTL8367C_VLAN_MBRCFG_LEN);
    _rtl8367c_VlanMCStUser2Smi(pVlanCg, smi_vlancfg);
    tableAddr = smi_vlancfg;

    for (page_idx = 0; page_idx < 4; page_idx++) /* 4 pages per VLAN Member Config */
    {
        regAddr = RTL8367C_VLAN_MEMBER_CONFIGURATION_BASE + (index * 4) + page_idx;
        regData = *tableAddr;

        retVal = rtl8367c_setAsicReg(regAddr, regData);
        if (retVal != RT_ERR_OK)
            return retVal;

        tableAddr++;
    }

    return RT_ERR_OK;
}

void rtl8367::_rtl8367c_Vlan4kStUser2Smi(rtl8367c_user_vlan4kentry *pUserVlan4kEntry, uint16_t *pSmiVlan4kEntry)
{
    pSmiVlan4kEntry[0] |= (pUserVlan4kEntry->mbr & 0x00FF);
    pSmiVlan4kEntry[0] |= (pUserVlan4kEntry->untag & 0x00FF) << 8;

    pSmiVlan4kEntry[1] |= (pUserVlan4kEntry->fid_msti & 0x000F);
    pSmiVlan4kEntry[1] |= (pUserVlan4kEntry->vbpen & 0x0001) << 4;
    pSmiVlan4kEntry[1] |= (pUserVlan4kEntry->vbpri & 0x0007) << 5;
    pSmiVlan4kEntry[1] |= (pUserVlan4kEntry->envlanpol & 0x0001) << 8;
    pSmiVlan4kEntry[1] |= (pUserVlan4kEntry->meteridx & 0x001F) << 9;
    pSmiVlan4kEntry[1] |= (pUserVlan4kEntry->ivl_svl & 0x0001) << 14;

    pSmiVlan4kEntry[2] |= ((pUserVlan4kEntry->mbr & 0x0700) >> 8);
    pSmiVlan4kEntry[2] |= ((pUserVlan4kEntry->untag & 0x0700) >> 8) << 3;
    pSmiVlan4kEntry[2] |= ((pUserVlan4kEntry->meteridx & 0x0020) >> 5) << 6;
}

/* Function Name:
 *      rtl8367c_setAsicVlan4kEntry
 * Description:
 *      Set VID mapped entry to 4K VLAN table
 * Input:
 *      pVlan4kEntry - 4K VLAN configuration
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK                   - Success
 *      RT_ERR_SMI                  - SMI access error
 *      RT_ERR_INPUT                - Invalid input parameter
 *      RT_ERR_L2_FID               - Invalid FID
 *      RT_ERR_VLAN_VID             - Invalid VID parameter (0~4095)
 *      RT_ERR_PORT_MASK            - Invalid portmask
 *      RT_ERR_FILTER_METER_ID      - Invalid meter
 *      RT_ERR_QOS_INT_PRIORITY     - Invalid priority
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicVlan4kEntry(rtl8367c_user_vlan4kentry *pVlan4kEntry)
{
    uint16_t vlan_4k_entry[RTL8367C_VLAN_4KTABLE_LEN];
    uint32_t page_idx;
    uint16_t *tableAddr;
    int32_t retVal;
    uint32_t regData;

    if (pVlan4kEntry->vid > RTL8367C_VIDMAX)
        return RT_ERR_VLAN_VID;

    if (pVlan4kEntry->mbr > RTL8367C_PORTMASK)
        return RT_ERR_PORT_MASK;

    if (pVlan4kEntry->untag > RTL8367C_PORTMASK)
        return RT_ERR_PORT_MASK;

    if (pVlan4kEntry->fid_msti > RTL8367C_FIDMAX)
        return RT_ERR_L2_FID;

    if (pVlan4kEntry->meteridx > RTL8367C_METERMAX)
        return RT_ERR_FILTER_METER_ID;

    if (pVlan4kEntry->vbpri > RTL8367C_PRIMAX)
        return RT_ERR_QOS_INT_PRIORITY;

    memset(vlan_4k_entry, 0x00, sizeof(uint16_t) * RTL8367C_VLAN_4KTABLE_LEN);
    _rtl8367c_Vlan4kStUser2Smi(pVlan4kEntry, vlan_4k_entry);

    /* Prepare Data */
    tableAddr = vlan_4k_entry;
    for (page_idx = 0; page_idx < RTL8367C_VLAN_4KTABLE_LEN; page_idx++)
    {
        regData = *tableAddr;
        retVal = rtl8367c_setAsicReg(RTL8367C_TABLE_ACCESS_WRDATA_BASE + page_idx, regData);
        if (retVal != RT_ERR_OK)
            return retVal;

        tableAddr++;
    }

    /* Write Address (VLAN_ID) */
    regData = pVlan4kEntry->vid;
    retVal = rtl8367c_setAsicReg(RTL8367C_TABLE_ACCESS_ADDR_REG, regData);
    if (retVal != RT_ERR_OK)
        return retVal;

    /* Write Command */
    retVal = rtl8367c_setAsicRegBits(RTL8367C_TABLE_ACCESS_CTRL_REG, RTL8367C_TABLE_TYPE_MASK | RTL8367C_COMMAND_TYPE_MASK, RTL8367C_TABLE_ACCESS_REG_DATA(TB_OP_WRITE, TB_TARGET_CVLAN));
    if (retVal != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsicVlanPortBasedVID
 * Description:
 *      Set port based VID which is indexed to 32 VLAN member configurations
 * Input:
 *      port    - Physical port number (0~10)
 *      index   - Index to VLAN member configuration
 *      pri     - 1Q Port based VLAN priority
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK                   - Success
 *      RT_ERR_SMI                  - SMI access error
 *      RT_ERR_PORT_ID              - Invalid port number
 *      RT_ERR_QOS_INT_PRIORITY     - Invalid priority
 *      RT_ERR_VLAN_ENTRY_NOT_FOUND - Invalid VLAN member configuration index
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicVlanPortBasedVID(uint32_t port, uint32_t index, uint32_t pri)
{
    uint32_t regAddr, bit_mask;
    int32_t retVal;

    if (port > RTL8367C_PORTIDMAX)
        return RT_ERR_PORT_ID;

    if (index > RTL8367C_CVIDXMAX)
        return RT_ERR_VLAN_ENTRY_NOT_FOUND;

    if (pri > RTL8367C_PRIMAX)
        return RT_ERR_QOS_INT_PRIORITY;

    regAddr = RTL8367C_VLAN_PVID_CTRL_REG(port);
    bit_mask = RTL8367C_PORT_VIDX_MASK(port);
    retVal = rtl8367c_setAsicRegBits(regAddr, bit_mask, index);
    if (retVal != RT_ERR_OK)
        return retVal;

    regAddr = RTL8367C_VLAN_PORTBASED_PRIORITY_REG(port);
    bit_mask = RTL8367C_VLAN_PORTBASED_PRIORITY_MASK(port);
    retVal = rtl8367c_setAsicRegBits(regAddr, bit_mask, pri);
    if (retVal != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsicVlanEgressTagMode
 * Description:
 *      Set CVLAN egress tag mode
 * Input:
 *      port        - Physical port number (0~10)
 *      tagMode     - The egress tag mode. Including Original mode, Keep tag mode and Priority tag mode
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_SMI      - SMI access error
 *      RT_ERR_INPUT    - Invalid input parameter
 *      RT_ERR_PORT_ID  - Invalid port number
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicVlanEgressTagMode(uint32_t port, rtl8367c_egtagmode tagMode)
{
    if (port > RTL8367C_PORTIDMAX)
        return RT_ERR_PORT_ID;

    if (tagMode >= EG_TAG_MODE_END)
        return RT_ERR_INPUT;

    return rtl8367c_setAsicRegBits(RTL8367C_PORT_MISC_CFG_REG(port), RTL8367C_VLAN_EGRESS_MDOE_MASK, tagMode);
}

/* Function Name:
 *      rtl8367c_setAsicVlanIngressFilter
 * Description:
 *      Set VLAN Ingress Filter
 * Input:
 *      port        - Physical port number (0~10)
 *      enabled     - Enable or disable Ingress filter
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_SMI      - SMI access error
 *      RT_ERR_PORT_ID  - Invalid port number
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicVlanIngressFilter(uint32_t port, uint32_t enabled)
{
    if (port > RTL8367C_PORTIDMAX)
        return RT_ERR_PORT_ID;

    return rtl8367c_setAsicRegBit(RTL8367C_VLAN_INGRESS_REG, port, enabled);
}

/* Function Name:
 *      rtl8367c_setAsicVlanFilter
 * Description:
 *      Set enable CVLAN filtering function
 * Input:
 *      enabled - 1: enabled, 0:  DISABLED_RTK
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_SMI      - SMI access error
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicVlanFilter(uint32_t enabled)
{
    return rtl8367c_setAsicRegBit(RTL8367C_REG_VLAN_CTRL, RTL8367C_VLAN_CTRL_OFFSET, enabled);
}

int32_t rtl8367::rtk_vlan_init()
{
    int32_t retVal;
    uint32_t i;
    rtl8367c_user_vlan4kentry vlan4K;
    rtl8367c_vlanconfiguser vlanMC;

    /* Clean Database */
    memset(vlan_mbrCfgVid, 0x00, sizeof(uint32_t) * RTL8367C_CVIDXNO);
    memset(vlan_mbrCfgUsage, 0x00, sizeof(vlan_mbrCfgType_t) * RTL8367C_CVIDXNO);

    /* clean 32 VLAN member configuration */
    for (i = 0; i <= RTL8367C_CVIDXMAX; i++)
    {
        vlanMC.evid = 0;
        vlanMC.mbr = 0;
        vlanMC.fid_msti = 0;
        vlanMC.envlanpol = 0;
        vlanMC.meteridx = 0;
        vlanMC.vbpen = 0;
        vlanMC.vbpri = 0;
        if ((retVal = rtl8367c_setAsicVlanMemberConfig(i, &vlanMC)) != RT_ERR_OK)
            return retVal;
    }

    /* Set a default VLAN with vid 1 to 4K table for all ports */
    memset(&vlan4K, 0, sizeof(rtl8367c_user_vlan4kentry));
    vlan4K.vid = 1;
    vlan4K.mbr = halCtrl.phy_portmask;
    vlan4K.untag = halCtrl.phy_portmask;
    vlan4K.fid_msti = 0;
    if ((retVal = rtl8367c_setAsicVlan4kEntry(&vlan4K)) != RT_ERR_OK)
        return retVal;

    /* Also set the default VLAN to 32 member configuration index 0 */
    memset(&vlanMC, 0, sizeof(rtl8367c_vlanconfiguser));
    vlanMC.evid = 1;
    vlanMC.mbr = halCtrl.phy_portmask;
    vlanMC.fid_msti = 0;
    if ((retVal = rtl8367c_setAsicVlanMemberConfig(0, &vlanMC)) != RT_ERR_OK)
        return retVal;

    /* Set all ports PVID to default VLAN and tag-mode to original */
    RTK_SCAN_ALL_PHY_PORTMASK(i)
    {
        if ((retVal = rtl8367c_setAsicVlanPortBasedVID(i, 0, 0)) != RT_ERR_OK)
            return retVal;
        if ((retVal = rtl8367c_setAsicVlanEgressTagMode(i, EG_TAG_MODE_ORI)) != RT_ERR_OK)
            return retVal;
    }

    /* Updata Databse */
    vlan_mbrCfgUsage[0] = MBRCFG_USED_BY_VLAN;
    vlan_mbrCfgVid[0] = 1;

    /* Enable Ingress filter */
    RTK_SCAN_ALL_PHY_PORTMASK(i)
    {
        if ((retVal = rtl8367c_setAsicVlanIngressFilter(i, ENABLED)) != RT_ERR_OK)
            return retVal;
    }

    /* enable VLAN */
    if ((retVal = rtl8367c_setAsicVlanFilter(ENABLED)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtk_switch_portmask_L2P_get
 * Description:
 *      Get physicl portmask from logical portmask
 * Input:
 *      pLogicalPmask       - logical port mask
 * Output:
 *      pPhysicalPortmask   - physical port mask
 * Return:
 *      RT_ERR_OK           - OK
 *      RT_ERR_NOT_INIT     - Not Initialize
 *      RT_ERR_NULL_POINTER - Null pointer
 *      RT_ERR_PORT_MASK    - Error port mask
 * Note:
 *
 */
int32_t rtl8367::rtk_switch_portmask_L2P_get(rtk_portmask_t *pLogicalPmask, uint32_t *pPhysicalPortmask)
{
    uint32_t log_port, phy_port;

    if (rtk_switch_isPortMaskValid(pLogicalPmask) != RT_ERR_OK)
        return RT_ERR_PORT_MASK;

    /* reset physical port mask */
    *pPhysicalPortmask = 0;

    RTK_PORTMASK_SCAN((*pLogicalPmask), log_port)
    {
        phy_port = rtk_switch_port_L2P_get((rtk_port_t)log_port);
        *pPhysicalPortmask |= (0x0001 << phy_port);
    }

    return RT_ERR_OK;
}

/* Function Name:
 *      rtk_vlan_set
 * Description:
 *      Set a VLAN entry.
 * Input:
 *      vid - VLAN ID to configure.
 *      pVlanCfg - VLAN Configuration
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK                   - OK
 *      RT_ERR_FAILED               - Failed
 *      RT_ERR_SMI                  - SMI access error
 *      RT_ERR_INPUT                - Invalid input parameters.
 *      RT_ERR_L2_FID               - Invalid FID.
 *      RT_ERR_VLAN_PORT_MBR_EXIST  - Invalid member port mask.
 *      RT_ERR_VLAN_VID             - Invalid VID parameter.
 * Note:
 *
 */
int32_t rtl8367::rtk_vlan_set(uint32_t vid, rtk_vlan_cfg_t *pVlanCfg)
{
    int32_t retVal;
    uint32_t phyMbrPmask;
    uint32_t phyUntagPmask;
    rtl8367c_user_vlan4kentry vlan4K;
    rtl8367c_vlanconfiguser vlanMC;
    uint32_t idx;
    uint32_t empty_index = 0xffff;
    uint32_t update_evid = 0;

    /* vid must be 0~8191 */
    if (vid > RTL8367C_EVIDMAX)
        return RT_ERR_VLAN_VID;

    /* Null pointer check */
    if (NULL == pVlanCfg)
        return RT_ERR_NULL_POINTER;

    /* Check port mask valid */
    RTK_CHK_PORTMASK_VALID(&(pVlanCfg->mbr));

    if (vid <= RTL8367C_VIDMAX)
    {
        /* Check untag port mask valid */
        RTK_CHK_PORTMASK_VALID(&(pVlanCfg->untag));
    }

    /* IVL_EN */
    if (pVlanCfg->ivl_en >= RTK_ENABLE_END)
        return RT_ERR_ENABLE;

    /* fid must be 0~15 */
    if (pVlanCfg->fid_msti > RTL8367C_FIDMAX)
        return RT_ERR_L2_FID;

    /* Policing */
    if (pVlanCfg->envlanpol >= RTK_ENABLE_END)
        return RT_ERR_ENABLE;

    /* Meter ID */
    if (pVlanCfg->meteridx > halCtrl.max_meter_id)
        return RT_ERR_INPUT;

    /* VLAN based priority */
    if (pVlanCfg->vbpen >= RTK_ENABLE_END)
        return RT_ERR_ENABLE;

    /* Priority */
    if (pVlanCfg->vbpri > RTL8367C_PRIMAX)
        return RT_ERR_INPUT;

    /* Get physical port mask */
    if (rtk_switch_portmask_L2P_get(&(pVlanCfg->mbr), &phyMbrPmask) != RT_ERR_OK)
        return RT_ERR_FAILED;

    if (rtk_switch_portmask_L2P_get(&(pVlanCfg->untag), &phyUntagPmask) != RT_ERR_OK)
        return RT_ERR_FAILED;

    if (vid <= RTL8367C_VIDMAX)
    {
        /* update 4K table */
        memset(&vlan4K, 0, sizeof(rtl8367c_user_vlan4kentry));
        vlan4K.vid = vid;

        vlan4K.mbr = (phyMbrPmask & 0xFFFF);
        vlan4K.untag = (phyUntagPmask & 0xFFFF);

        vlan4K.ivl_svl = pVlanCfg->ivl_en;
        vlan4K.fid_msti = pVlanCfg->fid_msti;
        vlan4K.envlanpol = pVlanCfg->envlanpol;
        vlan4K.meteridx = pVlanCfg->meteridx;
        vlan4K.vbpen = pVlanCfg->vbpen;
        vlan4K.vbpri = pVlanCfg->vbpri;

        if ((retVal = rtl8367c_setAsicVlan4kEntry(&vlan4K)) != RT_ERR_OK)
            return retVal;

        /* Update Member configuration if exist */
        for (idx = 0; idx <= RTL8367C_CVIDXMAX; idx++)
        {
            if (vlan_mbrCfgUsage[idx] == MBRCFG_USED_BY_VLAN)
            {
                if (vlan_mbrCfgVid[idx] == vid)
                {
                    /* Found! Update */
                    if (phyMbrPmask == 0x00)
                    {
                        /* Member port = 0x00, delete this VLAN from Member Configuration */
                        memset(&vlanMC, 0x00, sizeof(rtl8367c_vlanconfiguser));
                        if ((retVal = rtl8367c_setAsicVlanMemberConfig(idx, &vlanMC)) != RT_ERR_OK)
                            return retVal;

                        /* Clear Database */
                        vlan_mbrCfgUsage[idx] = MBRCFG_UNUSED;
                        vlan_mbrCfgVid[idx] = 0;
                    }
                    else
                    {
                        /* Normal VLAN config, update to member configuration */
                        vlanMC.evid = vid;
                        vlanMC.mbr = vlan4K.mbr;
                        vlanMC.fid_msti = vlan4K.fid_msti;
                        vlanMC.meteridx = vlan4K.meteridx;
                        vlanMC.envlanpol = vlan4K.envlanpol;
                        vlanMC.vbpen = vlan4K.vbpen;
                        vlanMC.vbpri = vlan4K.vbpri;
                        if ((retVal = rtl8367c_setAsicVlanMemberConfig(idx, &vlanMC)) != RT_ERR_OK)
                            return retVal;
                    }

                    break;
                }
            }
        }
    }
    else
    {
        /* vid > 4095 */
        for (idx = 0; idx <= RTL8367C_CVIDXMAX; idx++)
        {
            if (vlan_mbrCfgUsage[idx] == MBRCFG_USED_BY_VLAN)
            {
                if (vlan_mbrCfgVid[idx] == vid)
                {
                    /* Found! Update */
                    if (phyMbrPmask == 0x00)
                    {
                        /* Member port = 0x00, delete this VLAN from Member Configuration */
                        memset(&vlanMC, 0x00, sizeof(rtl8367c_vlanconfiguser));
                        if ((retVal = rtl8367c_setAsicVlanMemberConfig(idx, &vlanMC)) != RT_ERR_OK)
                            return retVal;

                        /* Clear Database */
                        vlan_mbrCfgUsage[idx] = MBRCFG_UNUSED;
                        vlan_mbrCfgVid[idx] = 0;
                    }
                    else
                    {
                        /* Normal VLAN config, update to member configuration */
                        vlanMC.evid = vid;
                        vlanMC.mbr = phyMbrPmask;
                        vlanMC.fid_msti = pVlanCfg->fid_msti;
                        vlanMC.meteridx = pVlanCfg->meteridx;
                        vlanMC.envlanpol = pVlanCfg->envlanpol;
                        vlanMC.vbpen = pVlanCfg->vbpen;
                        vlanMC.vbpri = pVlanCfg->vbpri;
                        if ((retVal = rtl8367c_setAsicVlanMemberConfig(idx, &vlanMC)) != RT_ERR_OK)
                            return retVal;

                        break;
                    }

                    update_evid = 1;
                }
            }

            if (vlan_mbrCfgUsage[idx] == MBRCFG_UNUSED)
            {
                if (0xffff == empty_index)
                    empty_index = idx;
            }
        }

        /* doesn't find out same EVID entry and there is empty index in member configuration */
        if ((phyMbrPmask != 0x00) && (update_evid == 0) && (empty_index != 0xFFFF))
        {
            vlanMC.evid = vid;
            vlanMC.mbr = phyMbrPmask;
            vlanMC.fid_msti = pVlanCfg->fid_msti;
            vlanMC.meteridx = pVlanCfg->meteridx;
            vlanMC.envlanpol = pVlanCfg->envlanpol;
            vlanMC.vbpen = pVlanCfg->vbpen;
            vlanMC.vbpri = pVlanCfg->vbpri;
            if ((retVal = rtl8367c_setAsicVlanMemberConfig(empty_index, &vlanMC)) != RT_ERR_OK)
                return retVal;

            vlan_mbrCfgUsage[empty_index] = MBRCFG_USED_BY_VLAN;
            vlan_mbrCfgVid[empty_index] = vid;
        }
    }

    return RT_ERR_OK;
}

void rtl8367::_rtl8367c_Vlan4kStSmi2User(uint16_t *pSmiVlan4kEntry, rtl8367c_user_vlan4kentry *pUserVlan4kEntry)
{
    pUserVlan4kEntry->mbr = (pSmiVlan4kEntry[0] & 0x00FF) | ((pSmiVlan4kEntry[2] & 0x0007) << 8);
    pUserVlan4kEntry->untag = ((pSmiVlan4kEntry[0] & 0xFF00) >> 8) | (((pSmiVlan4kEntry[2] & 0x0038) >> 3) << 8);
    pUserVlan4kEntry->fid_msti = pSmiVlan4kEntry[1] & 0x000F;
    pUserVlan4kEntry->vbpen = (pSmiVlan4kEntry[1] & 0x0010) >> 4;
    pUserVlan4kEntry->vbpri = (pSmiVlan4kEntry[1] & 0x00E0) >> 5;
    pUserVlan4kEntry->envlanpol = (pSmiVlan4kEntry[1] & 0x0100) >> 8;
    pUserVlan4kEntry->meteridx = ((pSmiVlan4kEntry[1] & 0x3E00) >> 9) | (((pSmiVlan4kEntry[2] & 0x0040) >> 6) << 5);
    pUserVlan4kEntry->ivl_svl = (pSmiVlan4kEntry[1] & 0x4000) >> 14;
}

/* Function Name:
 *      rtl8367c_getAsicVlan4kEntry
 * Description:
 *      Get VID mapped entry to 4K VLAN table
 * Input:
 *      pVlan4kEntry - 4K VLAN configuration
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK               - Success
 *      RT_ERR_SMI              - SMI access error
 *      RT_ERR_VLAN_VID         - Invalid VID parameter (0~4095)
 *      RT_ERR_BUSYWAIT_TIMEOUT - LUT is busy at retrieving
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicVlan4kEntry(rtl8367c_user_vlan4kentry *pVlan4kEntry)
{
    uint16_t vlan_4k_entry[RTL8367C_VLAN_4KTABLE_LEN];
    uint32_t page_idx;
    uint16_t *tableAddr;
    int32_t retVal;
    uint32_t regData;
    uint32_t busyCounter;

    if (pVlan4kEntry->vid > RTL8367C_VIDMAX)
        return RT_ERR_VLAN_VID;

    /* Polling status */
    busyCounter = RTL8367C_VLAN_BUSY_CHECK_NO;
    while (busyCounter)
    {
        retVal = rtl8367c_getAsicRegBit(RTL8367C_TABLE_ACCESS_STATUS_REG, RTL8367C_TABLE_LUT_ADDR_BUSY_FLAG_OFFSET, &regData);
        if (retVal != RT_ERR_OK)
            return retVal;

        if (regData == 0)
            break;

        busyCounter--;
        if (busyCounter == 0)
            return RT_ERR_BUSYWAIT_TIMEOUT;
    }

    /* Write Address (VLAN_ID) */
    regData = pVlan4kEntry->vid;
    retVal = rtl8367c_setAsicReg(RTL8367C_TABLE_ACCESS_ADDR_REG, regData);
    if (retVal != RT_ERR_OK)
        return retVal;

    /* Read Command */
    retVal = rtl8367c_setAsicRegBits(RTL8367C_TABLE_ACCESS_CTRL_REG, RTL8367C_TABLE_TYPE_MASK | RTL8367C_COMMAND_TYPE_MASK, RTL8367C_TABLE_ACCESS_REG_DATA(TB_OP_READ, TB_TARGET_CVLAN));
    if (retVal != RT_ERR_OK)
        return retVal;

    /* Polling status */
    busyCounter = RTL8367C_VLAN_BUSY_CHECK_NO;
    while (busyCounter)
    {
        retVal = rtl8367c_getAsicRegBit(RTL8367C_TABLE_ACCESS_STATUS_REG, RTL8367C_TABLE_LUT_ADDR_BUSY_FLAG_OFFSET, &regData);
        if (retVal != RT_ERR_OK)
            return retVal;

        if (regData == 0)
            break;

        busyCounter--;
        if (busyCounter == 0)
            return RT_ERR_BUSYWAIT_TIMEOUT;
    }

    /* Read VLAN data from register */
    tableAddr = vlan_4k_entry;
    for (page_idx = 0; page_idx < RTL8367C_VLAN_4KTABLE_LEN; page_idx++)
    {
        retVal = rtl8367c_getAsicReg(RTL8367C_TABLE_ACCESS_RDDATA_BASE + page_idx, &regData);
        if (retVal != RT_ERR_OK)
            return retVal;

        *tableAddr = regData;
        tableAddr++;
    }

    _rtl8367c_Vlan4kStSmi2User(vlan_4k_entry, pVlan4kEntry);

    return RT_ERR_OK;
}

uint32_t rtl8367::rtk_switch_port_P2L_get(uint32_t physicalPort)
{
    if (physicalPort >= RTK_SWITCH_PORT_NUM)
        return UNDEFINE_PORT;

    return (halCtrl.p2l_port[physicalPort]);
}

/* Function Name:
 *      rtk_switch_portmask_P2L_get
 * Description:
 *      Get logical portmask from physical portmask
 * Input:
 *      physicalPortmask    - physical port mask
 * Output:
 *      pLogicalPmask       - logical port mask
 * Return:
 *      RT_ERR_OK           - OK
 *      RT_ERR_NOT_INIT     - Not Initialize
 *      RT_ERR_NULL_POINTER - Null pointer
 *      RT_ERR_PORT_MASK    - Error port mask
 * Note:
 *
 */
int32_t rtl8367::rtk_switch_portmask_P2L_get(uint32_t physicalPortmask, rtk_portmask_t *pLogicalPmask)
{
    uint32_t log_port, phy_port;

    if (NULL == pLogicalPmask)
        return RT_ERR_NULL_POINTER;

    RTK_PORTMASK_CLEAR(*pLogicalPmask);

    for (phy_port = halCtrl.min_phy_port; phy_port <= halCtrl.max_phy_port; phy_port++)
    {
        if (physicalPortmask & (0x0001 << phy_port))
        {
            log_port = rtk_switch_port_P2L_get(phy_port);
            if (log_port != UNDEFINE_PORT)
            {
                RTK_PORTMASK_PORT_SET(*pLogicalPmask, log_port);
            }
        }
    }

    return RT_ERR_OK;
}

void rtl8367::_rtl8367c_VlanMCStSmi2User(uint16_t *pSmiVlanCfg, rtl8367c_vlanconfiguser *pVlanCg)
{
    pVlanCg->mbr = pSmiVlanCfg[0] & 0x07FF;
    pVlanCg->fid_msti = pSmiVlanCfg[1] & 0x000F;
    pVlanCg->meteridx = (pSmiVlanCfg[2] >> 5) & 0x003F;
    pVlanCg->envlanpol = (pSmiVlanCfg[2] >> 4) & 0x0001;
    pVlanCg->vbpri = (pSmiVlanCfg[2] >> 1) & 0x0007;
    pVlanCg->vbpen = pSmiVlanCfg[2] & 0x0001;
    pVlanCg->evid = pSmiVlanCfg[3] & 0x1FFF;
}

/* Function Name:
 *      rtl8367c_getAsicVlanMemberConfig
 * Description:
 *      Get 32 VLAN member configurations
 * Input:
 *      index       - VLAN member configuration index (0~31)
 *      pVlanCg - VLAN member configuration
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK                   - Success
 *      RT_ERR_SMI                  - SMI access error
 *      RT_ERR_INPUT                - Invalid input parameter
 *      RT_ERR_VLAN_ENTRY_NOT_FOUND - Invalid VLAN member configuration index
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicVlanMemberConfig(uint32_t index, rtl8367c_vlanconfiguser *pVlanCg)
{
    int32_t retVal;
    uint32_t page_idx;
    uint32_t regAddr;
    uint32_t regData;
    uint16_t *tableAddr;
    uint16_t smi_vlancfg[RTL8367C_VLAN_MBRCFG_LEN];

    if (index > RTL8367C_CVIDXMAX)
        return RT_ERR_VLAN_ENTRY_NOT_FOUND;

    memset(smi_vlancfg, 0x00, sizeof(uint16_t) * RTL8367C_VLAN_MBRCFG_LEN);
    tableAddr = smi_vlancfg;

    for (page_idx = 0; page_idx < 4; page_idx++) /* 4 pages per VLAN Member Config */
    {
        regAddr = RTL8367C_VLAN_MEMBER_CONFIGURATION_BASE + (index * 4) + page_idx;

        retVal = rtl8367c_getAsicReg(regAddr, &regData);
        if (retVal != RT_ERR_OK)
            return retVal;

        *tableAddr = (uint16_t)regData;
        tableAddr++;
    }

    _rtl8367c_VlanMCStSmi2User(smi_vlancfg, pVlanCg);
    return RT_ERR_OK;
}

int32_t rtl8367::rtk_vlan_get(uint32_t vid, rtk_vlan_cfg_t *pVlanCfg)
{
    int32_t retVal;
    uint32_t phyMbrPmask;
    uint32_t phyUntagPmask;
    rtl8367c_user_vlan4kentry vlan4K;
    rtl8367c_vlanconfiguser vlanMC;
    uint32_t idx;

    /* vid must be 0~8191 */
    if (vid > RTL8367C_EVIDMAX)
        return RT_ERR_VLAN_VID;

    /* Null pointer check */
    if (NULL == pVlanCfg)
        return RT_ERR_NULL_POINTER;

    if (vid <= RTL8367C_VIDMAX)
    {
        vlan4K.vid = vid;

        if ((retVal = rtl8367c_getAsicVlan4kEntry(&vlan4K)) != RT_ERR_OK)
            return retVal;

        phyMbrPmask = vlan4K.mbr;
        phyUntagPmask = vlan4K.untag;
        if (rtk_switch_portmask_P2L_get(phyMbrPmask, &(pVlanCfg->mbr)) != RT_ERR_OK)
            return RT_ERR_FAILED;

        if (rtk_switch_portmask_P2L_get(phyUntagPmask, &(pVlanCfg->untag)) != RT_ERR_OK)
            return RT_ERR_FAILED;

        pVlanCfg->ivl_en = vlan4K.ivl_svl;
        pVlanCfg->fid_msti = vlan4K.fid_msti;
        pVlanCfg->envlanpol = vlan4K.envlanpol;
        pVlanCfg->meteridx = vlan4K.meteridx;
        pVlanCfg->vbpen = vlan4K.vbpen;
        pVlanCfg->vbpri = vlan4K.vbpri;
    }
    else
    {
        for (idx = 0; idx <= RTL8367C_CVIDXMAX; idx++)
        {
            if (vlan_mbrCfgUsage[idx] == MBRCFG_USED_BY_VLAN)
            {
                if (vlan_mbrCfgVid[idx] == vid)
                {
                    if ((retVal = rtl8367c_getAsicVlanMemberConfig(idx, &vlanMC)) != RT_ERR_OK)
                        return retVal;

                    phyMbrPmask = vlanMC.mbr;
                    if (rtk_switch_portmask_P2L_get(phyMbrPmask, &(pVlanCfg->mbr)) != RT_ERR_OK)
                        return RT_ERR_FAILED;

                    pVlanCfg->untag.bits[0] = 0;
                    pVlanCfg->ivl_en = 0;
                    pVlanCfg->fid_msti = vlanMC.fid_msti;
                    pVlanCfg->envlanpol = vlanMC.envlanpol;
                    pVlanCfg->meteridx = vlanMC.meteridx;
                    pVlanCfg->vbpen = vlanMC.vbpen;
                    pVlanCfg->vbpri = vlanMC.vbpri;
                }
            }
        }
    }

    return RT_ERR_OK;
}

/* Function Name:
 *      rtk_vlan_checkAndCreateMbr
 * Description:
 *      Check and create Member configuration and return index
 * Input:
 *      vid  - VLAN id.
 * Output:
 *      pIndex  - Member configuration index
 * Return:
 *      RT_ERR_OK           - OK
 *      RT_ERR_FAILED       - Failed
 *      RT_ERR_SMI          - SMI access error
 *      RT_ERR_VLAN_VID     - Invalid VLAN ID.
 *      RT_ERR_VLAN_ENTRY_NOT_FOUND - VLAN not found
 *      RT_ERR_TBL_FULL     - Member Configuration table full
 * Note:
 *
 */
int32_t rtl8367::rtk_vlan_checkAndCreateMbr(uint32_t vid, uint32_t *pIndex)
{
    int32_t retVal;
    rtl8367c_user_vlan4kentry vlan4K;
    rtl8367c_vlanconfiguser vlanMC;
    uint32_t idx;
    uint32_t empty_idx = 0xFFFF;

    /* vid must be 0~8191 */
    if (vid > RTL8367C_EVIDMAX)
        return RT_ERR_VLAN_VID;

    /* Null pointer check */
    if (NULL == pIndex)
        return RT_ERR_NULL_POINTER;

    /* Get 4K VLAN */
    if (vid <= RTL8367C_VIDMAX)
    {
        memset(&vlan4K, 0x00, sizeof(rtl8367c_user_vlan4kentry));
        vlan4K.vid = vid;
        if ((retVal = rtl8367c_getAsicVlan4kEntry(&vlan4K)) != RT_ERR_OK)
            return retVal;
    }

    /* Search exist entry */
    for (idx = 0; idx <= RTL8367C_CVIDXMAX; idx++)
    {
        if (vlan_mbrCfgUsage[idx] == MBRCFG_USED_BY_VLAN)
        {
            if (vlan_mbrCfgVid[idx] == vid)
            {
                /* Found! return index */
                *pIndex = idx;
                return RT_ERR_OK;
            }
        }
    }

    /* Not found, Read H/W Member Configuration table to update database */
    for (idx = 0; idx <= RTL8367C_CVIDXMAX; idx++)
    {
        if ((retVal = rtl8367c_getAsicVlanMemberConfig(idx, &vlanMC)) != RT_ERR_OK)
            return retVal;

        if ((vlanMC.evid == 0) && (vlanMC.mbr == 0x00))
        {
            vlan_mbrCfgUsage[idx] = MBRCFG_UNUSED;
            vlan_mbrCfgVid[idx] = 0;
        }
        else
        {
            vlan_mbrCfgUsage[idx] = MBRCFG_USED_BY_VLAN;
            vlan_mbrCfgVid[idx] = vlanMC.evid;
        }
    }

    /* Search exist entry again */
    for (idx = 0; idx <= RTL8367C_CVIDXMAX; idx++)
    {
        if (vlan_mbrCfgUsage[idx] == MBRCFG_USED_BY_VLAN)
        {
            if (vlan_mbrCfgVid[idx] == vid)
            {
                /* Found! return index */
                *pIndex = idx;
                return RT_ERR_OK;
            }
        }
    }

    /* try to look up an empty index */
    for (idx = 0; idx <= RTL8367C_CVIDXMAX; idx++)
    {
        if (vlan_mbrCfgUsage[idx] == MBRCFG_UNUSED)
        {
            empty_idx = idx;
            break;
        }
    }

    if (empty_idx == 0xFFFF)
    {
        /* No empty index */
        return RT_ERR_TBL_FULL;
    }

    if (vid > RTL8367C_VIDMAX)
    {
        /* > 4K, there is no 4K entry, create on member configuration directly */
        memset(&vlanMC, 0x00, sizeof(rtl8367c_vlanconfiguser));
        vlanMC.evid = vid;
        if ((retVal = rtl8367c_setAsicVlanMemberConfig(empty_idx, &vlanMC)) != RT_ERR_OK)
            return retVal;
    }
    else
    {
        /* Copy from 4K table */
        vlanMC.evid = vid;
        vlanMC.mbr = vlan4K.mbr;
        vlanMC.fid_msti = vlan4K.fid_msti;
        vlanMC.meteridx = vlan4K.meteridx;
        vlanMC.envlanpol = vlan4K.envlanpol;
        vlanMC.vbpen = vlan4K.vbpen;
        vlanMC.vbpri = vlan4K.vbpri;
        if ((retVal = rtl8367c_setAsicVlanMemberConfig(empty_idx, &vlanMC)) != RT_ERR_OK)
            return retVal;
    }

    /* Update Database */
    vlan_mbrCfgUsage[empty_idx] = MBRCFG_USED_BY_VLAN;
    vlan_mbrCfgVid[empty_idx] = vid;

    *pIndex = empty_idx;
    return RT_ERR_OK;
}

int32_t rtl8367::rtk_vlan_portPvid_set(rtk_port_t port, uint32_t pvid, uint32_t priority)
{
    int32_t retVal;
    uint32_t index;

    /* Check Port Valid */
    RTK_CHK_PORT_VALID(port);

    /* vid must be 0~8191 */
    if (pvid > RTL8367C_EVIDMAX)
        return RT_ERR_VLAN_VID;

    /* priority must be 0~7 */
    if (priority > RTL8367C_PRIMAX)
        return RT_ERR_VLAN_PRIORITY;

    if ((retVal = rtk_vlan_checkAndCreateMbr(pvid, &index)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicVlanPortBasedVID(rtk_switch_port_L2P_get(port), index, priority)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_getAsicVlanPortBasedVID
 * Description:
 *      Get port based VID which is indexed to 32 VLAN member configurations
 * Input:
 *      port    - Physical port number (0~10)
 *      pIndex  - Index to VLAN member configuration
 *      pPri    - 1Q Port based VLAN priority
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_SMI      - SMI access error
 *      RT_ERR_PORT_ID  - Invalid port number
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicVlanPortBasedVID(uint32_t port, uint32_t *pIndex, uint32_t *pPri)
{
    uint32_t regAddr, bit_mask;
    int32_t retVal;

    if (port > RTL8367C_PORTIDMAX)
        return RT_ERR_PORT_ID;

    regAddr = RTL8367C_VLAN_PVID_CTRL_REG(port);
    bit_mask = RTL8367C_PORT_VIDX_MASK(port);
    retVal = rtl8367c_getAsicRegBits(regAddr, bit_mask, pIndex);
    if (retVal != RT_ERR_OK)
        return retVal;

    regAddr = RTL8367C_VLAN_PORTBASED_PRIORITY_REG(port);
    bit_mask = RTL8367C_VLAN_PORTBASED_PRIORITY_MASK(port);
    retVal = rtl8367c_getAsicRegBits(regAddr, bit_mask, pPri);
    if (retVal != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_vlan_portPvid_get(rtk_port_t port, uint32_t *pPvid, uint32_t *pPriority)
{
    int32_t retVal;
    uint32_t index, pri;
    rtl8367c_vlanconfiguser mbrCfg;

    /* Check Port Valid */
    RTK_CHK_PORT_VALID(port);

    if (NULL == pPvid)
        return RT_ERR_NULL_POINTER;

    if (NULL == pPriority)
        return RT_ERR_NULL_POINTER;

    if ((retVal = rtl8367c_getAsicVlanPortBasedVID(rtk_switch_port_L2P_get(port), &index, &pri)) != RT_ERR_OK)
        return retVal;

    memset(&mbrCfg, 0x00, sizeof(rtl8367c_vlanconfiguser));
    if ((retVal = rtl8367c_getAsicVlanMemberConfig(index, &mbrCfg)) != RT_ERR_OK)
        return retVal;

    *pPvid = mbrCfg.evid;
    *pPriority = pri;

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_vlan_portIgrFilterEnable_set(rtk_port_t port, rtk_enable_t igr_filter)
{
    int32_t retVal;

    /* Check Port Valid */
    RTK_CHK_PORT_VALID(port);

    if (igr_filter >= RTK_ENABLE_END)
        return RT_ERR_ENABLE;

    if ((retVal = rtl8367c_setAsicVlanIngressFilter(rtk_switch_port_L2P_get(port), igr_filter)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsicVlanAccpetFrameType
 * Description:
 *      Set per-port acceptable frame type
 * Input:
 *      port        - Physical port number (0~10)
 *      frameType   - The acceptable frame type
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK                       - Success
 *      RT_ERR_SMI                      - SMI access error
 *      RT_ERR_PORT_ID                  - Invalid port number
 *      RT_ERR_VLAN_ACCEPT_FRAME_TYPE   - Invalid frame type
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicVlanAccpetFrameType(uint32_t port, rtl8367c_accframetype frameType)
{
    if (port > RTL8367C_PORTIDMAX)
        return RT_ERR_PORT_ID;

    if (frameType >= FRAME_TYPE_MAX_BOUND)
        return RT_ERR_VLAN_ACCEPT_FRAME_TYPE;

    return rtl8367c_setAsicRegBits(RTL8367C_VLAN_ACCEPT_FRAME_TYPE_REG(port), RTL8367C_VLAN_ACCEPT_FRAME_TYPE_MASK(port), frameType);
}

int32_t rtl8367::rtk_vlan_portAcceptFrameType_set(rtk_port_t port, rtk_vlan_acceptFrameType_t accept_frame_type)
{
    int32_t retVal;

    /* Check Port Valid */
    RTK_CHK_PORT_VALID(port);

    if (accept_frame_type >= ACCEPT_FRAME_TYPE_END)
        return RT_ERR_VLAN_ACCEPT_FRAME_TYPE;

    if ((retVal = rtl8367c_setAsicVlanAccpetFrameType(rtk_switch_port_L2P_get(port), (rtl8367c_accframetype)accept_frame_type)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_vlan_tagMode_set(rtk_port_t port, rtl8367c_egtagmode tag_mode)
{
    int32_t retVal;

    /* Check Port Valid */
    RTK_CHK_PORT_VALID(port);

    if (tag_mode >= EG_TAG_MODE_END)
        return RT_ERR_PORT_ID;

    if ((retVal = rtl8367c_setAsicVlanEgressTagMode(rtk_switch_port_L2P_get(port), tag_mode)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsicVlanTransparent
 * Description:
 *      Set VLAN transparent
 * Input:
 *      port        - Physical port number (0~10)
 *      portmask    - portmask(0~0xFF)
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK           - Success
 *      RT_ERR_SMI          - SMI access error
 *      RT_ERR_PORT_MASK    - Invalid portmask
 *      RT_ERR_PORT_ID      - Invalid port number
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicVlanTransparent(uint32_t port, uint32_t portmask)
{
    if (port > RTL8367C_PORTIDMAX)
        return RT_ERR_PORT_ID;

    if (portmask > RTL8367C_PORTMASK)
        return RT_ERR_PORT_MASK;

    return rtl8367c_setAsicRegBits(RTL8367C_REG_VLAN_EGRESS_TRANS_CTRL0 + port, RTL8367C_VLAN_EGRESS_TRANS_CTRL0_MASK, portmask);
}

int32_t rtl8367::rtk_vlan_transparent_set(rtk_port_t egr_port, rtk_portmask_t *pIgr_pmask)
{
    int32_t retVal;
    uint32_t pmask;

    /* Check Port Valid */
    RTK_CHK_PORT_VALID(egr_port);

    if (NULL == pIgr_pmask)
        return RT_ERR_NULL_POINTER;

    RTK_CHK_PORTMASK_VALID(pIgr_pmask);

    if (rtk_switch_portmask_L2P_get(pIgr_pmask, &pmask) != RT_ERR_OK)
        return RT_ERR_FAILED;

    if ((retVal = rtl8367c_setAsicVlanTransparent(rtk_switch_port_L2P_get(egr_port), pmask)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsicSvlanPrioritySel
 * Description:
 *      Set SVLAN priority field setting
 * Input:
 *      priSel  - S-priority assignment method, 0:internal priority 1:C-tag priority 2:using Svlan member configuration
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_SMI      - SMI access error
 *      RT_ERR_INPUT    - Invalid input parameter
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicSvlanPrioritySel(uint32_t priSel)
{
    if (priSel >= SPRISEL_END)
        return RT_ERR_QOS_INT_PRIORITY;

    return rtl8367c_setAsicRegBits(RTL8367C_REG_SVLAN_CFG, RTL8367C_VS_SPRISEL_MASK, priSel);
}

/* Function Name:
 *      rtl8367c_setAsicSvlanIngressUntag
 * Description:
 *      Set action received un-Stag frame from unplink port
 * Input:
 *      mode        - 0:Drop 1:Trap 2:Assign SVLAN
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK   - Success
 *      RT_ERR_SMI  - SMI access error
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicSvlanIngressUntag(uint32_t mode)
{
    return rtl8367c_setAsicRegBits(RTL8367C_REG_SVLAN_CFG, RTL8367C_VS_UNTAG_MASK, mode);
}

/* Function Name:
 *      rtl8367c_setAsicSvlanIngressUnmatch
 * Description:
 *      Set action received unmatched Stag frame from unplink port
 * Input:
 *      mode        - 0:Drop 1:Trap 2:Assign SVLAN
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK   - Success
 *      RT_ERR_SMI  - SMI access error
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicSvlanIngressUnmatch(uint32_t mode)
{
    return rtl8367c_setAsicRegBits(RTL8367C_REG_SVLAN_CFG, RTL8367C_VS_UNMAT_MASK, mode);
}

/* Function Name:
 *      rtl8367c_setAsicSvlanTpid
 * Description:
 *      Set accepted S-VLAN ether type. The default ether type of S-VLAN is 0x88a8
 * Input:
 *      protocolType    - Ether type of S-tag frame parsing in uplink ports
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK   - Success
 *      RT_ERR_SMI  - SMI access error
 * Note:
 *      Ether type of S-tag in 802.1ad is 0x88a8 and there are existed ether type 0x9100 and 0x9200
 *      for Q-in-Q SLAN design. User can set mathced ether type as service provider supported protocol
 */
int32_t rtl8367::rtl8367c_setAsicSvlanTpid(uint32_t protocolType)
{
    return rtl8367c_setAsicReg(RTL8367C_REG_VS_TPID, protocolType);
}

/* Function Name:
 *      rtl8367c_setAsicSvlanUplinkPortMask
 * Description:
 *      Set uplink ports mask
 * Input:
 *      portMask    - Uplink port mask setting
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK   - Success
 *      RT_ERR_SMI  - SMI access error
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicSvlanUplinkPortMask(uint32_t portMask)
{
    return rtl8367c_setAsicReg(RTL8367C_REG_SVLAN_UPLINK_PORTMASK, portMask);
}

void rtl8367::_rtl8367c_svlanConfStUser2Smi(rtl8367c_svlan_memconf_t *pUserSt, uint16_t *pSmiSt)
{
    pSmiSt[0] |= (pUserSt->vs_member & 0x00FF);
    pSmiSt[0] |= (pUserSt->vs_untag & 0x00FF) << 8;

    pSmiSt[1] |= (pUserSt->vs_fid_msti & 0x000F);
    pSmiSt[1] |= (pUserSt->vs_priority & 0x0007) << 4;
    pSmiSt[1] |= (pUserSt->vs_force_fid & 0x0001) << 7;

    pSmiSt[2] |= (pUserSt->vs_svid & 0x0FFF);
    pSmiSt[2] |= (pUserSt->vs_efiden & 0x0001) << 12;
    pSmiSt[2] |= (pUserSt->vs_efid & 0x0007) << 13;

    pSmiSt[3] |= ((pUserSt->vs_member & 0x0700) >> 8);
    pSmiSt[3] |= ((pUserSt->vs_untag & 0x0700) >> 8) << 3;
}

/* Function Name:
 *      rtl8367c_setAsicSvlanMemberConfiguration
 * Description:
 *      Set system 64 S-tag content
 * Input:
 *      index           - index of 64 s-tag configuration
 *      pSvlanMemCfg    - SVLAN member configuration
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK                   - Success
 *      RT_ERR_SMI                  - SMI access error
 *      RT_ERR_SVLAN_ENTRY_INDEX    - Invalid SVLAN index parameter
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicSvlanMemberConfiguration(uint32_t index, rtl8367c_svlan_memconf_t *pSvlanMemCfg)
{
    int32_t retVal;
    uint32_t regAddr, regData;
    uint16_t *accessPtr;
    uint32_t i;
    uint16_t smiSvlanMemConf[RTL8367C_SVLAN_MEMCONF_LEN];

    if (index > RTL8367C_SVIDXMAX)
        return RT_ERR_SVLAN_ENTRY_INDEX;

    memset(smiSvlanMemConf, 0x00, sizeof(uint16_t) * RTL8367C_SVLAN_MEMCONF_LEN);
    _rtl8367c_svlanConfStUser2Smi(pSvlanMemCfg, smiSvlanMemConf);

    accessPtr = smiSvlanMemConf;

    regData = *accessPtr;
    for (i = 0; i < 3; i++)
    {
        retVal = rtl8367c_setAsicReg(RTL8367C_SVLAN_MEMBERCFG_BASE_REG(index) + i, regData);
        if (retVal != RT_ERR_OK)
            return retVal;

        accessPtr++;
        regData = *accessPtr;
    }

    if (index < 63)
        regAddr = RTL8367C_REG_SVLAN_MEMBERCFG0_CTRL4 + index;
    else if (index == 63)
        regAddr = RTL8367C_REG_SVLAN_MEMBERCFG63_CTRL4;

    retVal = rtl8367c_setAsicReg(regAddr, regData);
    if (retVal != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsicSvlanC2SConf
 * Description:
 *      Set SVLAN C2S table
 * Input:
 *      index   - index of 128 Svlan C2S configuration
 *      evid    - Enhanced VID
 *      portmask    - available c2s port mask
 *      svidx   - index of 64 Svlan member configuration
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK           - Success
 *      RT_ERR_SMI          - SMI access error
 *      RT_ERR_ENTRY_INDEX  - Invalid entry index
 * Note:
 *      ASIC will check upstream's VID and assign related SVID to mathed packet
 */
int32_t rtl8367::rtl8367c_setAsicSvlanC2SConf(uint32_t index, uint32_t evid, uint32_t portmask, uint32_t svidx)
{
    int32_t retVal;

    if (index > RTL8367C_C2SIDXMAX)
        return RT_ERR_ENTRY_INDEX;

    retVal = rtl8367c_setAsicReg(RTL8367C_SVLAN_C2SCFG_BASE_REG(index), svidx);
    if (retVal != RT_ERR_OK)
        return retVal;

    retVal = rtl8367c_setAsicReg(RTL8367C_SVLAN_C2SCFG_BASE_REG(index) + 1, portmask);
    if (retVal != RT_ERR_OK)
        return retVal;

    retVal = rtl8367c_setAsicReg(RTL8367C_SVLAN_C2SCFG_BASE_REG(index) + 2, evid);
    if (retVal != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

void rtl8367::_rtl8367c_svlanSp2cStUser2Smi(rtl8367c_svlan_s2c_t *pUserSt, uint16_t *pSmiSt)
{
    pSmiSt[0] |= (pUserSt->dstport & 0x0007);
    pSmiSt[0] |= (pUserSt->svidx & 0x003F) << 3;
    pSmiSt[0] |= ((pUserSt->dstport & 0x0008) >> 3) << 9;

    pSmiSt[1] |= (pUserSt->vid & 0x0FFF);
    pSmiSt[1] |= (pUserSt->valid & 0x0001) << 12;
}

/* Function Name:
 *      rtl8367c_setAsicSvlanSP2CConf
 * Description:
 *      Set system 128 SP2C content
 * Input:
 *      index           - index of 128 SVLAN & Port to CVLAN configuration
 *      pSvlanSp2cCfg   - SVLAN & Port to CVLAN configuration
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK           - Success
 *      RT_ERR_SMI          - SMI access error
 *      RT_ERR_ENTRY_INDEX  - Invalid entry index
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicSvlanSP2CConf(uint32_t index, rtl8367c_svlan_s2c_t *pSvlanSp2cCfg)
{
    int32_t retVal;
    uint32_t regData;
    uint16_t *accessPtr;
    uint32_t i;
    uint16_t smiSvlanSP2C[RTL8367C_SVLAN_SP2C_LEN];

    if (index > RTL8367C_SP2CMAX)
        return RT_ERR_ENTRY_INDEX;

    memset(smiSvlanSP2C, 0x00, sizeof(uint16_t) * RTL8367C_SVLAN_SP2C_LEN);
    _rtl8367c_svlanSp2cStUser2Smi(pSvlanSp2cCfg, smiSvlanSP2C);

    accessPtr = smiSvlanSP2C;

    for (i = 0; i < 2; i++)
    {
        regData = *(accessPtr + i);
        retVal = rtl8367c_setAsicReg(RTL8367C_SVLAN_S2C_ENTRY_BASE_REG(index) + i, regData);
        if (retVal != RT_ERR_OK)
            return retVal;
    }

    return retVal;
}

void rtl8367::_rtl8367c_svlanMc2sStUser2Smi(rtl8367c_svlan_mc2s_t *pUserSt, uint16_t *pSmiSt)
{
    pSmiSt[0] |= (pUserSt->svidx & 0x003F);
    pSmiSt[0] |= (pUserSt->format & 0x0001) << 6;
    pSmiSt[0] |= (pUserSt->valid & 0x0001) << 7;

    pSmiSt[1] = (uint16_t)(pUserSt->smask & 0x0000FFFF);
    pSmiSt[2] = (uint16_t)((pUserSt->smask & 0xFFFF0000) >> 16);

    pSmiSt[3] = (uint16_t)(pUserSt->sdata & 0x0000FFFF);
    pSmiSt[4] = (uint16_t)((pUserSt->sdata & 0xFFFF0000) >> 16);
}

/* Function Name:
 *      rtl8367c_setAsicSvlanMC2SConf
 * Description:
 *      Set system MC2S content
 * Input:
 *      index           - index of 32 SVLAN 32 MC2S configuration
 *      pSvlanMc2sCfg   - SVLAN Multicast to SVLAN member configuration
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK           - Success
 *      RT_ERR_SMI          - SMI access error
 *      RT_ERR_ENTRY_INDEX  - Invalid entry index
 * Note:
 *      If upstream packet is L2 multicast or IPv4 multicast packet and DMAC/DIP is matched MC2S
 *      configuration, ASIC will assign egress SVID to the packet
 */
int32_t rtl8367::rtl8367c_setAsicSvlanMC2SConf(uint32_t index, rtl8367c_svlan_mc2s_t *pSvlanMc2sCfg)
{
    int32_t retVal;
    uint32_t regData;
    uint16_t *accessPtr;
    uint32_t i;
    uint16_t smiSvlanMC2S[RTL8367C_SVLAN_MC2S_LEN];

    if (index > RTL8367C_MC2SIDXMAX)
        return RT_ERR_ENTRY_INDEX;

    memset(smiSvlanMC2S, 0x00, sizeof(uint16_t) * RTL8367C_SVLAN_MC2S_LEN);
    _rtl8367c_svlanMc2sStUser2Smi(pSvlanMc2sCfg, smiSvlanMC2S);

    accessPtr = smiSvlanMC2S;

    for (i = 0; i < 5; i++)
    {
        regData = *(accessPtr + i);
        retVal = rtl8367c_setAsicReg(RTL8367C_SVLAN_MCAST2S_ENTRY_BASE_REG(index) + i, regData);
        if (retVal != RT_ERR_OK)
            return retVal;
    }

    return retVal;
}

/* Function Name:
 *      rtl8367c_setAsicSvlanLookupType
 * Description:
 *      Set svlan lookup table selection
 * Input:
 *      type    - lookup type
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK                   - Success
 *      RT_ERR_SMI                  - SMI access error
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicSvlanLookupType(uint32_t type)
{
    return rtl8367c_setAsicRegBit(RTL8367C_REG_SVLAN_LOOKUP_TYPE, RTL8367C_SVLAN_LOOKUP_TYPE_OFFSET, type);
}

int32_t rtl8367::_rtk_svlan_lookupType_set(rtk_svlan_lookupType_t type)
{
    int32_t retVal;

    if (type >= SVLAN_LOOKUP_END)
        return RT_ERR_CHIP_NOT_SUPPORTED;

    svlan_lookupType = type;

    retVal = rtl8367c_setAsicSvlanLookupType((uint32_t)type);

    return retVal;
}

int32_t rtl8367::rtk_svlan_init()
{
    uint32_t i;
    int32_t retVal;
    rtl8367c_svlan_memconf_t svlanMemConf;
    rtl8367c_svlan_s2c_t svlanSP2CConf;
    rtl8367c_svlan_mc2s_t svlanMC2SConf;
    uint32_t svidx;

    /*default use C-priority*/
    if ((retVal = rtl8367c_setAsicSvlanPrioritySel(SPRISEL_CTAGPRI)) != RT_ERR_OK)
        return retVal;

    /*Drop SVLAN untag frame*/
    if ((retVal = rtl8367c_setAsicSvlanIngressUntag(UNTAG_DROP)) != RT_ERR_OK)
        return retVal;

    /*Drop SVLAN unmatch frame*/
    if ((retVal = rtl8367c_setAsicSvlanIngressUnmatch(UNMATCH_DROP)) != RT_ERR_OK)
        return retVal;

    /*Set TPID to 0x88a8*/
    if ((retVal = rtl8367c_setAsicSvlanTpid(0x88a8)) != RT_ERR_OK)
        return retVal;

    /*Clean Uplink Port Mask to none*/
    if ((retVal = rtl8367c_setAsicSvlanUplinkPortMask(0)) != RT_ERR_OK)
        return retVal;

    /*Clean SVLAN Member Configuration*/
    for (i = 0; i <= RTL8367C_SVIDXMAX; i++)
    {
        memset(&svlanMemConf, 0, sizeof(rtl8367c_svlan_memconf_t));
        if ((retVal = rtl8367c_setAsicSvlanMemberConfiguration(i, &svlanMemConf)) != RT_ERR_OK)
            return retVal;
    }

    /*Clean C2S Configuration*/
    for (i = 0; i <= RTL8367C_C2SIDXMAX; i++)
    {
        if ((retVal = rtl8367c_setAsicSvlanC2SConf(i, 0, 0, 0)) != RT_ERR_OK)
            return retVal;
    }

    /*Clean SP2C Configuration*/
    for (i = 0; i <= RTL8367C_SP2CMAX; i++)
    {
        memset(&svlanSP2CConf, 0, sizeof(rtl8367c_svlan_s2c_t));
        if ((retVal = rtl8367c_setAsicSvlanSP2CConf(i, &svlanSP2CConf)) != RT_ERR_OK)
            return retVal;
    }

    /*Clean MC2S Configuration*/
    for (i = 0; i <= RTL8367C_MC2SIDXMAX; i++)
    {
        memset(&svlanMC2SConf, 0, sizeof(rtl8367c_svlan_mc2s_t));
        if ((retVal = rtl8367c_setAsicSvlanMC2SConf(i, &svlanMC2SConf)) != RT_ERR_OK)
            return retVal;
    }

    if ((retVal = _rtk_svlan_lookupType_set(SVLAN_LOOKUP_S64MBRCGF)) != RT_ERR_OK)
        return retVal;

    for (svidx = 0; svidx <= RTL8367C_SVIDXMAX; svidx++)
    {
        svlan_mbrCfgUsage[svidx] = 0;
    }

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_getAsicSvlanUplinkPortMask
 * Description:
 *      Get uplink ports mask
 * Input:
 *      pPortmask   - Uplink port mask setting
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK   - Success
 *      RT_ERR_SMI  - SMI access error
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicSvlanUplinkPortMask(uint32_t *pPortmask)
{
    return rtl8367c_getAsicReg(RTL8367C_REG_SVLAN_UPLINK_PORTMASK, pPortmask);
}

int32_t rtl8367::rtk_svlan_servicePort_add(rtk_port_t port)
{
    int32_t retVal;
    uint32_t pmsk;

    /* check port valid */
    RTK_CHK_PORT_VALID(port);

    if ((retVal = rtl8367c_getAsicSvlanUplinkPortMask(&pmsk)) != RT_ERR_OK)
        return retVal;

    pmsk = pmsk | (1 << rtk_switch_port_L2P_get(port));

    if ((retVal = rtl8367c_setAsicSvlanUplinkPortMask(pmsk)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_svlan_tpidEntry_set(uint32_t svlan_tag_id)
{
    int32_t retVal;

    if (svlan_tag_id > RTK_MAX_NUM_OF_PROTO_TYPE)
        return RT_ERR_INPUT;

    if ((retVal = rtl8367c_setAsicSvlanTpid(svlan_tag_id)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_svlan_memberPortEntry_set(uint32_t svid, rtk_svlan_memberCfg_t *pSvlan_cfg)
{
    int32_t retVal;
    int32_t i;
    uint32_t empty_idx;
    rtl8367c_svlan_memconf_t svlanMemConf;
    uint32_t phyMbrPmask;
    rtk_vlan_cfg_t vlanCfg;

    if (NULL == pSvlan_cfg)
        return RT_ERR_NULL_POINTER;

    if (svid > RTL8367C_VIDMAX)
        return RT_ERR_SVLAN_VID;

    RTK_CHK_PORTMASK_VALID(&(pSvlan_cfg->memberport));

    RTK_CHK_PORTMASK_VALID(&(pSvlan_cfg->untagport));

    if (pSvlan_cfg->fiden > ENABLED)
        return RT_ERR_ENABLE;

    if (pSvlan_cfg->fid > RTL8367C_FIDMAX)
        return RT_ERR_L2_FID;

    if (pSvlan_cfg->priority > RTL8367C_PRIMAX)
        return RT_ERR_VLAN_PRIORITY;

    if (pSvlan_cfg->efiden > ENABLED)
        return RT_ERR_ENABLE;

    if (pSvlan_cfg->efid > RTL8367C_EFIDMAX)
        return RT_ERR_L2_FID;

    if (SVLAN_LOOKUP_C4KVLAN == svlan_lookupType)
    {
        if ((retVal = rtk_vlan_get(svid, &vlanCfg)) != RT_ERR_OK)
            return retVal;

        vlanCfg.mbr = pSvlan_cfg->memberport;
        vlanCfg.untag = pSvlan_cfg->untagport;

        if ((retVal = rtk_vlan_set(svid, &vlanCfg)) != RT_ERR_OK)
            return retVal;

        empty_idx = 0xFF;

        for (i = 0; i <= RTL8367C_SVIDXMAX; i++)
        {
            if (svid == svlan_mbrCfgVid[i] && 1 == svlan_mbrCfgUsage[i])
            {
                memset(&svlanMemConf, 0, sizeof(rtl8367c_svlan_memconf_t));
                svlanMemConf.vs_svid = svid;
                svlanMemConf.vs_efiden = pSvlan_cfg->efiden;
                svlanMemConf.vs_efid = pSvlan_cfg->efid;
                svlanMemConf.vs_priority = pSvlan_cfg->priority;

                /*for create check*/
                if (0 == svlanMemConf.vs_efiden && 0 == svlanMemConf.vs_efid)
                    svlanMemConf.vs_efid = 1;

                if ((retVal = rtl8367c_setAsicSvlanMemberConfiguration(i, &svlanMemConf)) != RT_ERR_OK)
                    return retVal;

                return RT_ERR_OK;
            }
            else if (0 == svlan_mbrCfgUsage[i] && 0xFF == empty_idx)
            {
                empty_idx = i;
            }
        }

        if (empty_idx != 0xFF)
        {
            svlan_mbrCfgUsage[empty_idx] = 1;
            svlan_mbrCfgVid[empty_idx] = svid;

            memset(&svlanMemConf, 0, sizeof(rtl8367c_svlan_memconf_t));
            svlanMemConf.vs_svid = svid;
            svlanMemConf.vs_efiden = pSvlan_cfg->efiden;
            svlanMemConf.vs_efid = pSvlan_cfg->efid;
            svlanMemConf.vs_priority = pSvlan_cfg->priority;

            /*for create check*/
            if (0 == svlanMemConf.vs_efiden && 0 == svlanMemConf.vs_efid)
                svlanMemConf.vs_efid = 1;

            if ((retVal = rtl8367c_setAsicSvlanMemberConfiguration(empty_idx, &svlanMemConf)) != RT_ERR_OK)
                return retVal;
        }

        return RT_ERR_OK;
    }

    empty_idx = 0xFF;

    for (i = 0; i <= RTL8367C_SVIDXMAX; i++)
    {
        /*
        if ((retVal = rtl8367c_getAsicSvlanMemberConfiguration(i, &svlanMemConf)) != RT_ERR_OK)
            return retVal;
        */
        if (svid == svlan_mbrCfgVid[i] && 1 == svlan_mbrCfgUsage[i])
        {
            svlanMemConf.vs_svid = svid;

            if (rtk_switch_portmask_L2P_get(&(pSvlan_cfg->memberport), &phyMbrPmask) != RT_ERR_OK)
                return RT_ERR_FAILED;

            svlanMemConf.vs_member = phyMbrPmask;

            if (rtk_switch_portmask_L2P_get(&(pSvlan_cfg->untagport), &phyMbrPmask) != RT_ERR_OK)
                return RT_ERR_FAILED;

            svlanMemConf.vs_untag = phyMbrPmask;

            svlanMemConf.vs_force_fid = pSvlan_cfg->fiden;
            svlanMemConf.vs_fid_msti = pSvlan_cfg->fid;
            svlanMemConf.vs_priority = pSvlan_cfg->priority;
            svlanMemConf.vs_efiden = pSvlan_cfg->efiden;
            svlanMemConf.vs_efid = pSvlan_cfg->efid;

            /*all items are reset means deleting*/
            if (0 == svlanMemConf.vs_member &&
                0 == svlanMemConf.vs_untag &&
                0 == svlanMemConf.vs_force_fid &&
                0 == svlanMemConf.vs_fid_msti &&
                0 == svlanMemConf.vs_priority &&
                0 == svlanMemConf.vs_efiden &&
                0 == svlanMemConf.vs_efid)
            {
                svlan_mbrCfgUsage[i] = 0;
                svlan_mbrCfgVid[i] = 0;

                /* Clear SVID also */
                svlanMemConf.vs_svid = 0;
            }
            else
            {
                svlan_mbrCfgUsage[i] = 1;
                svlan_mbrCfgVid[i] = svlanMemConf.vs_svid;

                if (0 == svlanMemConf.vs_svid)
                {
                    /*for create check*/
                    if (0 == svlanMemConf.vs_efiden && 0 == svlanMemConf.vs_efid)
                    {
                        svlanMemConf.vs_efid = 1;
                    }
                }
            }

            if ((retVal = rtl8367c_setAsicSvlanMemberConfiguration(i, &svlanMemConf)) != RT_ERR_OK)
                return retVal;

            return RT_ERR_OK;
        }
        else if (0 == svlan_mbrCfgUsage[i] && 0xFF == empty_idx)
        {
            empty_idx = i;
        }
    }

    if (empty_idx != 0xFF)
    {
        memset(&svlanMemConf, 0, sizeof(rtl8367c_svlan_memconf_t));
        svlanMemConf.vs_svid = svid;

        if (rtk_switch_portmask_L2P_get(&(pSvlan_cfg->memberport), &phyMbrPmask) != RT_ERR_OK)
            return RT_ERR_FAILED;

        svlanMemConf.vs_member = phyMbrPmask;

        if (rtk_switch_portmask_L2P_get(&(pSvlan_cfg->untagport), &phyMbrPmask) != RT_ERR_OK)
            return RT_ERR_FAILED;

        svlanMemConf.vs_untag = phyMbrPmask;

        svlanMemConf.vs_force_fid = pSvlan_cfg->fiden;
        svlanMemConf.vs_fid_msti = pSvlan_cfg->fid;
        svlanMemConf.vs_priority = pSvlan_cfg->priority;

        svlanMemConf.vs_efiden = pSvlan_cfg->efiden;
        svlanMemConf.vs_efid = pSvlan_cfg->efid;

        /*change efid for empty svid 0*/
        if (0 == svlanMemConf.vs_svid)
        { /*for create check*/
            if (0 == svlanMemConf.vs_efiden && 0 == svlanMemConf.vs_efid)
            {
                svlanMemConf.vs_efid = 1;
            }
        }

        svlan_mbrCfgUsage[empty_idx] = 1;
        svlan_mbrCfgVid[empty_idx] = svlanMemConf.vs_svid;

        if ((retVal = rtl8367c_setAsicSvlanMemberConfiguration(empty_idx, &svlanMemConf)) != RT_ERR_OK)
        {
            return retVal;
        }

        return RT_ERR_OK;
    }

    return RT_ERR_SVLAN_TABLE_FULL;
}

void rtl8367::_rtl8367c_svlanConfStSmi2User(rtl8367c_svlan_memconf_t *pUserSt, uint16_t *pSmiSt)
{
    pUserSt->vs_member = (pSmiSt[0] & 0x00FF) | ((pSmiSt[3] & 0x0007) << 8);
    pUserSt->vs_untag = ((pSmiSt[0] & 0xFF00) >> 8) | (((pSmiSt[3] & 0x0038) >> 3) << 8);

    pUserSt->vs_fid_msti = (pSmiSt[1] & 0x000F);
    pUserSt->vs_priority = (pSmiSt[1] & 0x0070) >> 4;
    pUserSt->vs_force_fid = (pSmiSt[1] & 0x0080) >> 7;

    pUserSt->vs_svid = (pSmiSt[2] & 0x0FFF);
    pUserSt->vs_efiden = (pSmiSt[2] & 0x1000) >> 12;
    pUserSt->vs_efid = (pSmiSt[2] & 0xE000) >> 13;
}

/* Function Name:
 *      rtl8367c_getAsicSvlanMemberConfiguration
 * Description:
 *      Get system 64 S-tag content
 * Input:
 *      index           - index of 64 s-tag configuration
 *      pSvlanMemCfg    - SVLAN member configuration
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK                   - Success
 *      RT_ERR_SMI                  - SMI access error
 *      RT_ERR_SVLAN_ENTRY_INDEX    - Invalid SVLAN index parameter
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicSvlanMemberConfiguration(uint32_t index, rtl8367c_svlan_memconf_t *pSvlanMemCfg)
{
    int32_t retVal;
    uint32_t regAddr, regData;
    uint16_t *accessPtr;
    uint32_t i;
    uint16_t smiSvlanMemConf[RTL8367C_SVLAN_MEMCONF_LEN];

    if (index > RTL8367C_SVIDXMAX)
        return RT_ERR_SVLAN_ENTRY_INDEX;

    memset(smiSvlanMemConf, 0x00, sizeof(uint16_t) * RTL8367C_SVLAN_MEMCONF_LEN);

    accessPtr = smiSvlanMemConf;

    for (i = 0; i < 3; i++)
    {
        retVal = rtl8367c_getAsicReg(RTL8367C_SVLAN_MEMBERCFG_BASE_REG(index) + i, &regData);
        if (retVal != RT_ERR_OK)
            return retVal;

        *accessPtr = regData;

        accessPtr++;
    }

    if (index < 63)
        regAddr = RTL8367C_REG_SVLAN_MEMBERCFG0_CTRL4 + index;
    else if (index == 63)
        regAddr = RTL8367C_REG_SVLAN_MEMBERCFG63_CTRL4;

    retVal = rtl8367c_getAsicReg(regAddr, &regData);
    if (retVal != RT_ERR_OK)
        return retVal;

    *accessPtr = regData;

    _rtl8367c_svlanConfStSmi2User(pSvlanMemCfg, smiSvlanMemConf);

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsicSvlanDefaultVlan
 * Description:
 *      Set default egress SVLAN
 * Input:
 *      port    - Physical port number (0~10)
 *      index   - index SVLAN member configuration
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK                   - Success
 *      RT_ERR_SMI                  - SMI access error
 *      RT_ERR_PORT_ID              - Invalid port number
 *      RT_ERR_SVLAN_ENTRY_INDEX    - Invalid SVLAN index parameter
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicSvlanDefaultVlan(uint32_t port, uint32_t index)
{
    int32_t retVal;

    if (port > RTL8367C_PORTIDMAX)
        return RT_ERR_PORT_ID;

    if (index > RTL8367C_SVIDXMAX)
        return RT_ERR_SVLAN_ENTRY_INDEX;

    if (port < 8)
    {
        if (port & 1)
            retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_SVLAN_PORTBASED_SVIDX_CTRL0 + (port >> 1), RTL8367C_VS_PORT1_SVIDX_MASK, index);
        else
            retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_SVLAN_PORTBASED_SVIDX_CTRL0 + (port >> 1), RTL8367C_VS_PORT0_SVIDX_MASK, index);
    }
    else
    {
        switch (port)
        {
        case 8:
            retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_SVLAN_PORTBASED_SVIDX_CTRL4, RTL8367C_VS_PORT8_SVIDX_MASK, index);
            break;

        case 9:
            retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_SVLAN_PORTBASED_SVIDX_CTRL4, RTL8367C_VS_PORT9_SVIDX_MASK, index);
            break;

        case 10:
            retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_SVLAN_PORTBASED_SVIDX_CTRL5, RTL8367C_SVLAN_PORTBASED_SVIDX_CTRL5_MASK, index);
            break;
        }
    }

    return retVal;
}

int32_t rtl8367::rtk_svlan_defaultSvlan_set(rtk_port_t port, uint32_t svid)
{
    int32_t retVal;
    uint32_t i;
    rtl8367c_svlan_memconf_t svlanMemConf;

    /* Check port Valid */
    RTK_CHK_PORT_VALID(port);

    /* svid must be 0~4095 */
    if (svid > RTL8367C_VIDMAX)
        return RT_ERR_SVLAN_VID;

    for (i = 0; i < RTL8367C_SVIDXNO; i++)
    {
        if ((retVal = rtl8367c_getAsicSvlanMemberConfiguration(i, &svlanMemConf)) != RT_ERR_OK)
            return retVal;

        if (svid == svlanMemConf.vs_svid)
        {
            if ((retVal = rtl8367c_setAsicSvlanDefaultVlan(rtk_switch_port_L2P_get(port), i)) != RT_ERR_OK)
                return retVal;

            return RT_ERR_OK;
        }
    }

    return RT_ERR_SVLAN_ENTRY_NOT_FOUND;
}

/* Function Name:
 *      rtl8367c_getAsicSvlanC2SConf
 * Description:
 *      Get SVLAN C2S table
 * Input:
 *      index   - index of 128 Svlan C2S configuration
 *      pEvid   - Enhanced VID
 *      pPortmask   - available c2s port mask
 *      pSvidx  - index of 64 Svlan member configuration
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK           - Success
 *      RT_ERR_SMI          - SMI access error
 *      RT_ERR_ENTRY_INDEX  - Invalid entry index
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicSvlanC2SConf(uint32_t index, uint32_t *pEvid, uint32_t *pPortmask, uint32_t *pSvidx)
{
    int32_t retVal;

    if (index > RTL8367C_C2SIDXMAX)
        return RT_ERR_ENTRY_INDEX;

    retVal = rtl8367c_getAsicReg(RTL8367C_SVLAN_C2SCFG_BASE_REG(index), pSvidx);
    if (retVal != RT_ERR_OK)
        return retVal;

    retVal = rtl8367c_getAsicReg(RTL8367C_SVLAN_C2SCFG_BASE_REG(index) + 1, pPortmask);
    if (retVal != RT_ERR_OK)
        return retVal;

    retVal = rtl8367c_getAsicReg(RTL8367C_SVLAN_C2SCFG_BASE_REG(index) + 2, pEvid);
    if (retVal != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_svlan_c2s_add(uint32_t vid, rtk_port_t src_port, uint32_t svid)
{
    int32_t retVal, i;
    uint32_t empty_idx;
    uint32_t evid, pmsk, svidx, c2s_svidx;
    rtl8367c_svlan_memconf_t svlanMemConf;
    uint32_t phyPort;
    uint16_t doneFlag;

    if (vid > RTL8367C_VIDMAX)
        return RT_ERR_VLAN_VID;

    if (svid > RTL8367C_VIDMAX)
        return RT_ERR_SVLAN_VID;

    /* Check port Valid */
    RTK_CHK_PORT_VALID(src_port);

    phyPort = rtk_switch_port_L2P_get(src_port);

    empty_idx = 0xFFFF;
    svidx = 0xFFFF;
    doneFlag = 0;

    for (i = 0; i <= RTL8367C_SVIDXMAX; i++)
    {
        if ((retVal = rtl8367c_getAsicSvlanMemberConfiguration(i, &svlanMemConf)) != RT_ERR_OK)
            return retVal;

        if (svid == svlanMemConf.vs_svid)
        {
            svidx = i;
            break;
        }
    }

    if (0xFFFF == svidx)
        return RT_ERR_SVLAN_VID;

    for (i = RTL8367C_C2SIDXMAX; i >= 0; i--)
    {
        if ((retVal = rtl8367c_getAsicSvlanC2SConf(i, &evid, &pmsk, &c2s_svidx)) != RT_ERR_OK)
            return retVal;

        if (evid == vid)
        {
            /* Check Src_port */
            if (pmsk & (1 << phyPort))
            {
                /* Check SVIDX */
                if (c2s_svidx == svidx)
                {
                    /* All the same, do nothing */
                }
                else
                {
                    /* New svidx, remove src_port and find a new slot to add a new enrty */
                    pmsk = pmsk & ~(1 << phyPort);
                    if (pmsk == 0)
                        c2s_svidx = 0;

                    if ((retVal = rtl8367c_setAsicSvlanC2SConf(i, vid, pmsk, c2s_svidx)) != RT_ERR_OK)
                        return retVal;
                }
            }
            else
            {
                if (c2s_svidx == svidx && doneFlag == 0)
                {
                    pmsk = pmsk | (1 << phyPort);
                    if ((retVal = rtl8367c_setAsicSvlanC2SConf(i, vid, pmsk, svidx)) != RT_ERR_OK)
                        return retVal;

                    doneFlag = 1;
                }
            }
        }
        else if (evid == 0 && pmsk == 0)
        {
            empty_idx = i;
        }
    }

    if (0xFFFF != empty_idx && doneFlag == 0)
    {
        if ((retVal = rtl8367c_setAsicSvlanC2SConf(empty_idx, vid, (1 << phyPort), svidx)) != RT_ERR_OK)
            return retVal;

        return RT_ERR_OK;
    }
    else if (doneFlag == 1)
    {
        return RT_ERR_OK;
    }

    return RT_ERR_OUT_OF_RANGE;
}

void rtl8367::_rtl8367c_svlanSp2cStSmi2User(rtl8367c_svlan_s2c_t *pUserSt, uint16_t *pSmiSt)
{
    pUserSt->dstport = (((pSmiSt[0] & 0x0200) >> 9) << 3) | (pSmiSt[0] & 0x0007);
    pUserSt->svidx = (pSmiSt[0] & 0x01F8) >> 3;
    pUserSt->vid = (pSmiSt[1] & 0x0FFF);
    pUserSt->valid = (pSmiSt[1] & 0x1000) >> 12;
}

/* Function Name:
 *      rtl8367c_getAsicSvlanSP2CConf
 * Description:
 *      Get system 128 SP2C content
 * Input:
 *      index           - index of 128 SVLAN & Port to CVLAN configuration
 *      pSvlanSp2cCfg   - SVLAN & Port to CVLAN configuration
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK           - Success
 *      RT_ERR_SMI          - SMI access error
 *      RT_ERR_ENTRY_INDEX  - Invalid entry index
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicSvlanSP2CConf(uint32_t index, rtl8367c_svlan_s2c_t *pSvlanSp2cCfg)
{
    int32_t retVal;
    uint32_t regData;
    uint16_t *accessPtr;
    uint32_t i;
    uint16_t smiSvlanSP2C[RTL8367C_SVLAN_SP2C_LEN];

    if (index > RTL8367C_SP2CMAX)
        return RT_ERR_ENTRY_INDEX;

    memset(smiSvlanSP2C, 0x00, sizeof(uint16_t) * RTL8367C_SVLAN_SP2C_LEN);

    accessPtr = smiSvlanSP2C;

    for (i = 0; i < 2; i++)
    {
        retVal = rtl8367c_getAsicReg(RTL8367C_SVLAN_S2C_ENTRY_BASE_REG(index) + i, &regData);
        if (retVal != RT_ERR_OK)
            return retVal;

        *accessPtr = regData;

        accessPtr++;
    }

    _rtl8367c_svlanSp2cStSmi2User(pSvlanSp2cCfg, smiSvlanSP2C);

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_svlan_sp2c_add(uint32_t svid, rtk_port_t dst_port, uint32_t cvid)
{
    int32_t retVal, i;
    uint32_t empty_idx, svidx;
    rtl8367c_svlan_memconf_t svlanMemConf;
    rtl8367c_svlan_s2c_t svlanSP2CConf;
    uint32_t port;

    if (svid > RTL8367C_VIDMAX)
        return RT_ERR_SVLAN_VID;

    if (cvid > RTL8367C_VIDMAX)
        return RT_ERR_VLAN_VID;

    /* Check port Valid */
    RTK_CHK_PORT_VALID(dst_port);
    port = rtk_switch_port_L2P_get(dst_port);

    svidx = 0xFFFF;

    for (i = 0; i < RTL8367C_SVIDXNO; i++)
    {
        if ((retVal = rtl8367c_getAsicSvlanMemberConfiguration(i, &svlanMemConf)) != RT_ERR_OK)
            return retVal;

        if (svid == svlanMemConf.vs_svid)
        {
            svidx = i;
            break;
        }
    }

    if (0xFFFF == svidx)
        return RT_ERR_SVLAN_ENTRY_NOT_FOUND;

    empty_idx = 0xFFFF;

    for (i = RTL8367C_SP2CMAX; i >= 0; i--)
    {
        if ((retVal = rtl8367c_getAsicSvlanSP2CConf(i, &svlanSP2CConf)) != RT_ERR_OK)
            return retVal;

        if ((svlanSP2CConf.svidx == svidx) && (svlanSP2CConf.dstport == port) && (svlanSP2CConf.valid == 1))
        {
            empty_idx = i;
            break;
        }
        else if (svlanSP2CConf.valid == 0)
        {
            empty_idx = i;
        }
    }

    if (empty_idx != 0xFFFF)
    {
        svlanSP2CConf.valid = 1;
        svlanSP2CConf.vid = cvid;
        svlanSP2CConf.svidx = svidx;
        svlanSP2CConf.dstport = port;

        if ((retVal = rtl8367c_setAsicSvlanSP2CConf(empty_idx, &svlanSP2CConf)) != RT_ERR_OK)
            return retVal;
        return RT_ERR_OK;
    }

    return RT_ERR_OUT_OF_RANGE;
}

/* Function Name:
 *      rtl8367c_setAsicSvlanUntagVlan
 * Description:
 *      Set default ingress untag SVLAN
 * Input:
 *      index   - index SVLAN member configuration
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK                   - Success
 *      RT_ERR_SMI                  - SMI access error
 *      RT_ERR_SVLAN_ENTRY_INDEX    - Invalid SVLAN index parameter
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicSvlanUntagVlan(uint32_t index)
{
    if (index > RTL8367C_SVIDXMAX)
        return RT_ERR_SVLAN_ENTRY_INDEX;

    return rtl8367c_setAsicRegBits(RTL8367C_REG_SVLAN_UNTAG_UNMAT_CFG, RTL8367C_VS_UNTAG_SVIDX_MASK, index);
}

int32_t rtl8367::rtk_svlan_untag_action_set(rtk_svlan_untag_action_t action, uint32_t svid)
{
    int32_t retVal;
    uint32_t i;
    rtl8367c_svlan_memconf_t svlanMemConf;

    if (action >= UNTAG_END)
        return RT_ERR_OUT_OF_RANGE;

    if (action == UNTAG_ASSIGN)
    {
        if (svid > RTL8367C_VIDMAX)
            return RT_ERR_SVLAN_VID;
    }

    if ((retVal = rtl8367c_setAsicSvlanIngressUntag((uint32_t)action)) != RT_ERR_OK)
        return retVal;

    if (action == UNTAG_ASSIGN)
    {
        for (i = 0; i < RTL8367C_SVIDXNO; i++)
        {
            if ((retVal = rtl8367c_getAsicSvlanMemberConfiguration(i, &svlanMemConf)) != RT_ERR_OK)
                return retVal;

            if (svid == svlanMemConf.vs_svid)
            {
                if ((retVal = rtl8367c_setAsicSvlanUntagVlan(i)) != RT_ERR_OK)
                    return retVal;

                return RT_ERR_OK;
            }
        }

        return RT_ERR_SVLAN_ENTRY_NOT_FOUND;
    }

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsicSvlanUnmatchVlan
 * Description:
 *      Set default ingress unmatch SVLAN
 * Input:
 *      index   - index SVLAN member configuration
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK                   - Success
 *      RT_ERR_SMI                  - SMI access error
 *      RT_ERR_SVLAN_ENTRY_INDEX    - Invalid SVLAN index parameter
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicSvlanUnmatchVlan(uint32_t index)
{
    if (index > RTL8367C_SVIDXMAX)
        return RT_ERR_SVLAN_ENTRY_INDEX;

    return rtl8367c_setAsicRegBits(RTL8367C_REG_SVLAN_UNTAG_UNMAT_CFG, RTL8367C_VS_UNMAT_SVIDX_MASK, index);
}

int32_t rtl8367::rtk_svlan_unmatch_action_set(rtk_svlan_unmatch_action_t action, uint32_t svid)
{
    int32_t retVal;
    uint32_t i;
    rtl8367c_svlan_memconf_t svlanMemConf;

    if (action >= UNMATCH_END)
        return RT_ERR_OUT_OF_RANGE;

    if (action == UNMATCH_ASSIGN)
    {
        if (svid > RTL8367C_VIDMAX)
            return RT_ERR_SVLAN_VID;
    }

    if ((retVal = rtl8367c_setAsicSvlanIngressUnmatch((uint32_t)action)) != RT_ERR_OK)
        return retVal;

    if (action == UNMATCH_ASSIGN)
    {
        for (i = 0; i < RTL8367C_SVIDXNO; i++)
        {
            if ((retVal = rtl8367c_getAsicSvlanMemberConfiguration(i, &svlanMemConf)) != RT_ERR_OK)
                return retVal;

            if (svid == svlanMemConf.vs_svid)
            {
                if ((retVal = rtl8367c_setAsicSvlanUnmatchVlan(i)) != RT_ERR_OK)
                    return retVal;

                return RT_ERR_OK;
            }
        }

        return RT_ERR_SVLAN_ENTRY_NOT_FOUND;
    }

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsicSvlanDmacCvidSel
 * Description:
 *      Set downstream CVID decision by DMAC
 * Input:
 *      port        - Physical port number (0~7)
 *      enabled     - 0: DISABLED_RTK, 1:enabled
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_SMI      - SMI access error
 *      RT_ERR_PORT_ID  - Invalid port number
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicSvlanDmacCvidSel(uint32_t port, uint32_t enabled)
{
    if (port > RTL8367C_PORTIDMAX)
        return RT_ERR_PORT_ID;

    if (port < 8)
        return rtl8367c_setAsicRegBit(RTL8367C_REG_SVLAN_CFG, RTL8367C_VS_PORT0_DMACVIDSEL_OFFSET + port, enabled);
    else
        return rtl8367c_setAsicRegBit(RTL8367C_REG_SVLAN_CFG_EXT, RTL8367C_VS_PORT8_DMACVIDSEL_OFFSET + (port - 8), enabled);
}

int32_t rtl8367::rtk_svlan_dmac_vidsel_set(rtk_port_t port, rtk_enable_t enable)
{
    int32_t retVal;

    /* Check port Valid */
    RTK_CHK_PORT_VALID(port);

    if (enable >= RTK_ENABLE_END)
        return RT_ERR_ENABLE;

    if ((retVal = rtl8367c_setAsicSvlanDmacCvidSel(rtk_switch_port_L2P_get(port), enable)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

//----------------- LOOK UP TABLE -----------------//

void rtl8367::_rtl8367c_fdbStUser2Smi(rtl8367c_luttb *pLutSt, uint16_t *pFdbSmi)
{
    /* L3 lookup */
    if (pLutSt->l3lookup)
    {
        if (pLutSt->l3vidlookup)
        {
            pFdbSmi[0] = (pLutSt->sip & 0x0000FFFF);
            pFdbSmi[1] = (pLutSt->sip & 0xFFFF0000) >> 16;

            pFdbSmi[2] = (pLutSt->dip & 0x0000FFFF);
            pFdbSmi[3] = (pLutSt->dip & 0x0FFF0000) >> 16;

            pFdbSmi[3] |= (pLutSt->l3lookup & 0x0001) << 12;
            pFdbSmi[3] |= (pLutSt->l3vidlookup & 0x0001) << 13;
            pFdbSmi[3] |= ((pLutSt->mbr & 0x0300) >> 8) << 14;

            pFdbSmi[4] |= (pLutSt->mbr & 0x00FF);
            pFdbSmi[4] |= (pLutSt->l3_vid & 0x00FF) << 8;

            pFdbSmi[5] |= ((pLutSt->l3_vid & 0x0F00) >> 8);
            pFdbSmi[5] |= (pLutSt->nosalearn & 0x0001) << 5;
            pFdbSmi[5] |= ((pLutSt->mbr & 0x0400) >> 10) << 7;
        }
        else
        {
            pFdbSmi[0] = (pLutSt->sip & 0x0000FFFF);
            pFdbSmi[1] = (pLutSt->sip & 0xFFFF0000) >> 16;

            pFdbSmi[2] = (pLutSt->dip & 0x0000FFFF);
            pFdbSmi[3] = (pLutSt->dip & 0x0FFF0000) >> 16;

            pFdbSmi[3] |= (pLutSt->l3lookup & 0x0001) << 12;
            pFdbSmi[3] |= (pLutSt->l3vidlookup & 0x0001) << 13;
            pFdbSmi[3] |= ((pLutSt->mbr & 0x0300) >> 8) << 14;

            pFdbSmi[4] |= (pLutSt->mbr & 0x00FF);
            pFdbSmi[4] |= (pLutSt->igmpidx & 0x00FF) << 8;

            pFdbSmi[5] |= (pLutSt->igmp_asic & 0x0001);
            pFdbSmi[5] |= (pLutSt->lut_pri & 0x0007) << 1;
            pFdbSmi[5] |= (pLutSt->fwd_en & 0x0001) << 4;
            pFdbSmi[5] |= (pLutSt->nosalearn & 0x0001) << 5;
            pFdbSmi[5] |= ((pLutSt->mbr & 0x0400) >> 10) << 7;
        }
    }
    else if (pLutSt->mac.octet[0] & 0x01) /*Multicast L2 Lookup*/
    {
        pFdbSmi[0] |= pLutSt->mac.octet[5];
        pFdbSmi[0] |= pLutSt->mac.octet[4] << 8;

        pFdbSmi[1] |= pLutSt->mac.octet[3];
        pFdbSmi[1] |= pLutSt->mac.octet[2] << 8;

        pFdbSmi[2] |= pLutSt->mac.octet[1];
        pFdbSmi[2] |= pLutSt->mac.octet[0] << 8;

        pFdbSmi[3] |= pLutSt->cvid_fid;
        pFdbSmi[3] |= (pLutSt->l3lookup & 0x0001) << 12;
        pFdbSmi[3] |= (pLutSt->ivl_svl & 0x0001) << 13;
        pFdbSmi[3] |= ((pLutSt->mbr & 0x0300) >> 8) << 14;

        pFdbSmi[4] |= (pLutSt->mbr & 0x00FF);
        pFdbSmi[4] |= (pLutSt->igmpidx & 0x00FF) << 8;

        pFdbSmi[5] |= pLutSt->igmp_asic;
        pFdbSmi[5] |= (pLutSt->lut_pri & 0x0007) << 1;
        pFdbSmi[5] |= (pLutSt->fwd_en & 0x0001) << 4;
        pFdbSmi[5] |= (pLutSt->nosalearn & 0x0001) << 5;
        pFdbSmi[5] |= ((pLutSt->mbr & 0x0400) >> 10) << 7;
    }
    else /*Asic auto-learning*/
    {
        pFdbSmi[0] |= pLutSt->mac.octet[5];
        pFdbSmi[0] |= pLutSt->mac.octet[4] << 8;

        pFdbSmi[1] |= pLutSt->mac.octet[3];
        pFdbSmi[1] |= pLutSt->mac.octet[2] << 8;

        pFdbSmi[2] |= pLutSt->mac.octet[1];
        pFdbSmi[2] |= pLutSt->mac.octet[0] << 8;

        pFdbSmi[3] |= pLutSt->cvid_fid;
        pFdbSmi[3] |= (pLutSt->l3lookup & 0x0001) << 12;
        pFdbSmi[3] |= (pLutSt->ivl_svl & 0x0001) << 13;
        pFdbSmi[3] |= ((pLutSt->spa & 0x0008) >> 3) << 15;

        pFdbSmi[4] |= pLutSt->efid;
        pFdbSmi[4] |= (pLutSt->fid & 0x000F) << 3;
        pFdbSmi[4] |= (pLutSt->sa_en & 0x0001) << 7;
        pFdbSmi[4] |= (pLutSt->spa & 0x0007) << 8;
        pFdbSmi[4] |= (pLutSt->age & 0x0007) << 11;
        pFdbSmi[4] |= (pLutSt->auth & 0x0001) << 14;
        pFdbSmi[4] |= (pLutSt->sa_block & 0x0001) << 15;

        pFdbSmi[5] |= pLutSt->da_block;
        pFdbSmi[5] |= (pLutSt->lut_pri & 0x0007) << 1;
        pFdbSmi[5] |= (pLutSt->fwd_en & 0x0001) << 4;
        pFdbSmi[5] |= (pLutSt->nosalearn & 0x0001) << 5;
    }
}

void rtl8367::_rtl8367c_fdbStSmi2User(rtl8367c_luttb *pLutSt, uint16_t *pFdbSmi)
{
    /*L3 lookup*/
    if (pFdbSmi[3] & 0x1000)
    {
        if (pFdbSmi[3] & 0x2000)
        {
            pLutSt->sip = pFdbSmi[0] | (pFdbSmi[1] << 16);
            pLutSt->dip = 0xE0000000 | pFdbSmi[2] | ((pFdbSmi[3] & 0x0FFF) << 16);

            pLutSt->mbr = (pFdbSmi[4] & 0x00FF) | (((pFdbSmi[3] & 0xC000) >> 14) << 8) | (((pFdbSmi[5] & 0x0080) >> 7) << 10);
            pLutSt->l3_vid = ((pFdbSmi[4] & 0xFF00) >> 8) | (pFdbSmi[5] & 0x000F);

            pLutSt->l3lookup = (pFdbSmi[3] & 0x1000) >> 12;
            pLutSt->l3vidlookup = (pFdbSmi[3] & 0x2000) >> 13;
            pLutSt->nosalearn = (pFdbSmi[5] & 0x0020) >> 5;
        }
        else
        {
            pLutSt->sip = pFdbSmi[0] | (pFdbSmi[1] << 16);
            pLutSt->dip = 0xE0000000 | pFdbSmi[2] | ((pFdbSmi[3] & 0x0FFF) << 16);

            pLutSt->lut_pri = (pFdbSmi[5] & 0x000E) >> 1;
            pLutSt->fwd_en = (pFdbSmi[5] & 0x0010) >> 4;

            pLutSt->mbr = (pFdbSmi[4] & 0x00FF) | (((pFdbSmi[3] & 0xC000) >> 14) << 8) | (((pFdbSmi[5] & 0x0080) >> 7) << 10);
            pLutSt->igmpidx = (pFdbSmi[4] & 0xFF00) >> 8;

            pLutSt->igmp_asic = (pFdbSmi[5] & 0x0001);
            pLutSt->l3lookup = (pFdbSmi[3] & 0x1000) >> 12;
            pLutSt->nosalearn = (pFdbSmi[5] & 0x0020) >> 5;
        }
    }
    else if (pFdbSmi[2] & 0x0100) /*Multicast L2 Lookup*/
    {
        pLutSt->mac.octet[0] = (pFdbSmi[2] & 0xFF00) >> 8;
        pLutSt->mac.octet[1] = (pFdbSmi[2] & 0x00FF);
        pLutSt->mac.octet[2] = (pFdbSmi[1] & 0xFF00) >> 8;
        pLutSt->mac.octet[3] = (pFdbSmi[1] & 0x00FF);
        pLutSt->mac.octet[4] = (pFdbSmi[0] & 0xFF00) >> 8;
        pLutSt->mac.octet[5] = (pFdbSmi[0] & 0x00FF);

        pLutSt->cvid_fid = pFdbSmi[3] & 0x0FFF;
        pLutSt->lut_pri = (pFdbSmi[5] & 0x000E) >> 1;
        pLutSt->fwd_en = (pFdbSmi[5] & 0x0010) >> 4;

        pLutSt->mbr = (pFdbSmi[4] & 0x00FF) | (((pFdbSmi[3] & 0xC000) >> 14) << 8) | (((pFdbSmi[5] & 0x0080) >> 7) << 10);
        pLutSt->igmpidx = (pFdbSmi[4] & 0xFF00) >> 8;

        pLutSt->igmp_asic = (pFdbSmi[5] & 0x0001);
        pLutSt->l3lookup = (pFdbSmi[3] & 0x1000) >> 12;
        pLutSt->ivl_svl = (pFdbSmi[3] & 0x2000) >> 13;
        pLutSt->nosalearn = (pFdbSmi[5] & 0x0020) >> 5;
    }
    else /*Asic auto-learning*/
    {
        pLutSt->mac.octet[0] = (pFdbSmi[2] & 0xFF00) >> 8;
        pLutSt->mac.octet[1] = (pFdbSmi[2] & 0x00FF);
        pLutSt->mac.octet[2] = (pFdbSmi[1] & 0xFF00) >> 8;
        pLutSt->mac.octet[3] = (pFdbSmi[1] & 0x00FF);
        pLutSt->mac.octet[4] = (pFdbSmi[0] & 0xFF00) >> 8;
        pLutSt->mac.octet[5] = (pFdbSmi[0] & 0x00FF);

        pLutSt->cvid_fid = pFdbSmi[3] & 0x0FFF;
        pLutSt->lut_pri = (pFdbSmi[5] & 0x000E) >> 1;
        pLutSt->fwd_en = (pFdbSmi[5] & 0x0010) >> 4;

        pLutSt->sa_en = (pFdbSmi[4] & 0x0080) >> 7;
        pLutSt->auth = (pFdbSmi[4] & 0x4000) >> 14;
        pLutSt->spa = ((pFdbSmi[4] & 0x0700) >> 8) | (((pFdbSmi[3] & 0x8000) >> 15) << 3);
        pLutSt->age = (pFdbSmi[4] & 0x3800) >> 11;
        pLutSt->fid = (pFdbSmi[4] & 0x0078) >> 3;
        pLutSt->efid = (pFdbSmi[4] & 0x0007);
        pLutSt->sa_block = (pFdbSmi[4] & 0x8000) >> 15;

        pLutSt->da_block = (pFdbSmi[5] & 0x0001);
        pLutSt->l3lookup = (pFdbSmi[3] & 0x1000) >> 12;
        pLutSt->ivl_svl = (pFdbSmi[3] & 0x2000) >> 13;
        pLutSt->nosalearn = (pFdbSmi[3] & 0x0020) >> 5;
    }
}
/* Function Name:
 *      rtl8367c_getAsicL2LookupTb
 * Description:
 *      Get filtering database entry
 * Input:
 *      pL2Table    - L2 table entry writing to 2K+64 filtering database
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK               - Success
 *      RT_ERR_SMI              - SMI access error
 *      RT_ERR_INPUT            - Invalid input parameter
 *      RT_ERR_BUSYWAIT_TIMEOUT - LUT is busy at retrieving
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicL2LookupTb(uint32_t method, rtl8367c_luttb *pL2Table)
{
    int32_t retVal;
    uint32_t regData;
    uint16_t *accessPtr;
    uint32_t i;
    uint16_t smil2Table[RTL8367C_LUT_TABLE_SIZE];
    uint32_t busyCounter;
    uint32_t tblCmd;

    if (pL2Table->wait_time == 0)
        busyCounter = RTL8367C_LUT_BUSY_CHECK_NO;
    else
        busyCounter = pL2Table->wait_time;

    while (busyCounter)
    {
        retVal = rtl8367c_getAsicRegBit(RTL8367C_TABLE_ACCESS_STATUS_REG, RTL8367C_TABLE_LUT_ADDR_BUSY_FLAG_OFFSET, &regData);
        if (retVal != RT_ERR_OK)
            return retVal;

        pL2Table->lookup_busy = regData;
        if (!pL2Table->lookup_busy)
            break;

        busyCounter--;
        if (busyCounter == 0)
            return RT_ERR_BUSYWAIT_TIMEOUT;
    }

    tblCmd = (method << RTL8367C_ACCESS_METHOD_OFFSET) & RTL8367C_ACCESS_METHOD_MASK;

    switch (method)
    {
    case LUTREADMETHOD_ADDRESS:
    case LUTREADMETHOD_NEXT_ADDRESS:
    case LUTREADMETHOD_NEXT_L2UC:
    case LUTREADMETHOD_NEXT_L2MC:
    case LUTREADMETHOD_NEXT_L3MC:
    case LUTREADMETHOD_NEXT_L2L3MC:
        retVal = rtl8367c_setAsicReg(RTL8367C_TABLE_ACCESS_ADDR_REG, pL2Table->address);
        if (retVal != RT_ERR_OK)
            return retVal;
        break;
    case LUTREADMETHOD_MAC:
        memset(smil2Table, 0x00, sizeof(uint16_t) * RTL8367C_LUT_TABLE_SIZE);
        _rtl8367c_fdbStUser2Smi(pL2Table, smil2Table);

        accessPtr = smil2Table;
        regData = *accessPtr;
        for (i = 0; i < RTL8367C_LUT_ENTRY_SIZE; i++)
        {
            retVal = rtl8367c_setAsicReg(RTL8367C_TABLE_ACCESS_WRDATA_BASE + i, regData);
            if (retVal != RT_ERR_OK)
                return retVal;

            accessPtr++;
            regData = *accessPtr;
        }
        break;
    case LUTREADMETHOD_NEXT_L2UCSPA:
        retVal = rtl8367c_setAsicReg(RTL8367C_TABLE_ACCESS_ADDR_REG, pL2Table->address);
        if (retVal != RT_ERR_OK)
            return retVal;

        tblCmd = tblCmd | ((pL2Table->spa << RTL8367C_TABLE_ACCESS_CTRL_SPA_OFFSET) & RTL8367C_TABLE_ACCESS_CTRL_SPA_MASK);

        break;
    default:
        return RT_ERR_INPUT;
    }

    tblCmd = tblCmd | ((RTL8367C_TABLE_ACCESS_REG_DATA(TB_OP_READ, TB_TARGET_L2)) & (RTL8367C_TABLE_TYPE_MASK | RTL8367C_COMMAND_TYPE_MASK));
    /* Read Command */
    retVal = rtl8367c_setAsicReg(RTL8367C_TABLE_ACCESS_CTRL_REG, tblCmd);
    if (retVal != RT_ERR_OK)
        return retVal;

    if (pL2Table->wait_time == 0)
        busyCounter = RTL8367C_LUT_BUSY_CHECK_NO;
    else
        busyCounter = pL2Table->wait_time;

    while (busyCounter)
    {
        retVal = rtl8367c_getAsicRegBit(RTL8367C_TABLE_ACCESS_STATUS_REG, RTL8367C_TABLE_LUT_ADDR_BUSY_FLAG_OFFSET, &regData);
        if (retVal != RT_ERR_OK)
            return retVal;

        pL2Table->lookup_busy = regData;
        if (!pL2Table->lookup_busy)
            break;

        busyCounter--;
        if (busyCounter == 0)
            return RT_ERR_BUSYWAIT_TIMEOUT;
    }

    retVal = rtl8367c_getAsicRegBit(RTL8367C_TABLE_ACCESS_STATUS_REG, RTL8367C_HIT_STATUS_OFFSET, &regData);
    if (retVal != RT_ERR_OK)
        return retVal;
    pL2Table->lookup_hit = regData;
    if (!pL2Table->lookup_hit)
        return RT_ERR_L2_ENTRY_NOTFOUND;

    /*Read access address*/
    // retVal = rtl8367c_getAsicRegBits(RTL8367C_TABLE_ACCESS_STATUS_REG, RTL8367C_TABLE_LUT_ADDR_TYPE_MASK | RTL8367C_TABLE_LUT_ADDR_ADDRESS_MASK,&regData);
    retVal = rtl8367c_getAsicReg(RTL8367C_TABLE_ACCESS_STATUS_REG, &regData);
    if (retVal != RT_ERR_OK)
        return retVal;

    pL2Table->address = (regData & 0x7ff) | ((regData & 0x4000) >> 3) | ((regData & 0x800) << 1);

    /*read L2 entry */
    memset(smil2Table, 0x00, sizeof(uint16_t) * RTL8367C_LUT_TABLE_SIZE);

    accessPtr = smil2Table;

    for (i = 0; i < RTL8367C_LUT_ENTRY_SIZE; i++)
    {
        retVal = rtl8367c_getAsicReg(RTL8367C_TABLE_ACCESS_RDDATA_BASE + i, &regData);
        if (retVal != RT_ERR_OK)
            return retVal;

        *accessPtr = regData;

        accessPtr++;
    }

    _rtl8367c_fdbStSmi2User(pL2Table, smil2Table);

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsicL2LookupTb
 * Description:
 *      Set filtering database entry
 * Input:
 *      pL2Table    - L2 table entry writing to 8K+64 filtering database
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK   - Success
 *      RT_ERR_SMI  - SMI access error
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicL2LookupTb(rtl8367c_luttb *pL2Table)
{
    int32_t retVal;
    uint32_t regData;
    uint16_t *accessPtr;
    uint32_t i;
    uint16_t smil2Table[RTL8367C_LUT_TABLE_SIZE];
    uint32_t tblCmd;
    uint32_t busyCounter;

    memset(smil2Table, 0x00, sizeof(uint16_t) * RTL8367C_LUT_TABLE_SIZE);
    _rtl8367c_fdbStUser2Smi(pL2Table, smil2Table);

    if (pL2Table->wait_time == 0)
        busyCounter = RTL8367C_LUT_BUSY_CHECK_NO;
    else
        busyCounter = pL2Table->wait_time;

    while (busyCounter)
    {
        retVal = rtl8367c_getAsicRegBit(RTL8367C_TABLE_ACCESS_STATUS_REG, RTL8367C_TABLE_LUT_ADDR_BUSY_FLAG_OFFSET, &regData);
        if (retVal != RT_ERR_OK)
            return retVal;

        pL2Table->lookup_busy = regData;
        if (!regData)
            break;

        busyCounter--;
        if (busyCounter == 0)
            return RT_ERR_BUSYWAIT_TIMEOUT;
    }

    accessPtr = smil2Table;

    for (i = 0; i < RTL8367C_LUT_ENTRY_SIZE; i++)
    {
        regData = *(accessPtr + i);
        retVal = rtl8367c_setAsicReg(RTL8367C_TABLE_ACCESS_WRDATA_BASE + i, regData);
        if (retVal != RT_ERR_OK)
            return retVal;
    }

    tblCmd = (RTL8367C_TABLE_ACCESS_REG_DATA(TB_OP_WRITE, TB_TARGET_L2)) & (RTL8367C_TABLE_TYPE_MASK | RTL8367C_COMMAND_TYPE_MASK);
    /* Write Command */
    retVal = rtl8367c_setAsicReg(RTL8367C_TABLE_ACCESS_CTRL_REG, tblCmd);
    if (retVal != RT_ERR_OK)
        return retVal;

    if (pL2Table->wait_time == 0)
        busyCounter = RTL8367C_LUT_BUSY_CHECK_NO;
    else
        busyCounter = pL2Table->wait_time;

    while (busyCounter)
    {
        retVal = rtl8367c_getAsicRegBit(RTL8367C_TABLE_ACCESS_STATUS_REG, RTL8367C_TABLE_LUT_ADDR_BUSY_FLAG_OFFSET, &regData);
        if (retVal != RT_ERR_OK)
            return retVal;

        pL2Table->lookup_busy = regData;
        if (!regData)
            break;

        busyCounter--;
        if (busyCounter == 0)
            return RT_ERR_BUSYWAIT_TIMEOUT;
    }

    /*Read access status*/
    retVal = rtl8367c_getAsicRegBit(RTL8367C_TABLE_ACCESS_STATUS_REG, RTL8367C_HIT_STATUS_OFFSET, &regData);
    if (retVal != RT_ERR_OK)
        return retVal;

    pL2Table->lookup_hit = regData;
    if (!pL2Table->lookup_hit)
        return RT_ERR_FAILED;

    retVal = rtl8367c_getAsicReg(RTL8367C_TABLE_ACCESS_STATUS_REG, &regData);
    if (retVal != RT_ERR_OK)
        return retVal;

    pL2Table->address = (regData & 0x7ff) | ((regData & 0x4000) >> 3) | ((regData & 0x800) << 1);
    pL2Table->lookup_busy = 0;

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_l2_addr_add(rtk_mac_t *pMac, rtk_l2_ucastAddr_t *pL2_data)
{
    int32_t retVal;
    uint32_t method;
    rtl8367c_luttb l2Table;

    /* must be unicast address */
    if ((pMac == NULL) || (pMac->octet[0] & 0x1))
        return RT_ERR_MAC;

    if (pL2_data == NULL)
        return RT_ERR_MAC;

    RTK_CHK_PORT_VALID(pL2_data->port);

    if (pL2_data->ivl >= RTK_ENABLE_END)
        return RT_ERR_INPUT;

    if (pL2_data->cvid > RTL8367C_VIDMAX)
        return RT_ERR_L2_VID;

    if (pL2_data->fid > RTL8367C_FIDMAX)
        return RT_ERR_L2_FID;

    if (pL2_data->is_static >= RTK_ENABLE_END)
        return RT_ERR_INPUT;

    if (pL2_data->sa_block >= RTK_ENABLE_END)
        return RT_ERR_INPUT;

    if (pL2_data->da_block >= RTK_ENABLE_END)
        return RT_ERR_INPUT;

    if (pL2_data->auth >= RTK_ENABLE_END)
        return RT_ERR_INPUT;

    if (pL2_data->efid > RTL8367C_EFIDMAX)
        return RT_ERR_INPUT;

    if (pL2_data->priority > RTL8367C_PRIMAX)
        return RT_ERR_INPUT;

    if (pL2_data->sa_pri_en >= RTK_ENABLE_END)
        return RT_ERR_INPUT;

    if (pL2_data->fwd_pri_en >= RTK_ENABLE_END)
        return RT_ERR_INPUT;

    memset(&l2Table, 0, sizeof(rtl8367c_luttb));

    /* fill key (MAC,FID) to get L2 entry */
    memcpy(l2Table.mac.octet, pMac->octet, ETHER_ADDR_LEN);
    l2Table.ivl_svl = pL2_data->ivl;
    l2Table.fid = pL2_data->fid;
    l2Table.cvid_fid = pL2_data->cvid;
    l2Table.efid = pL2_data->efid;
    method = LUTREADMETHOD_MAC;
    retVal = rtl8367c_getAsicL2LookupTb(method, &l2Table);
    if (RT_ERR_OK == retVal)
    {
        memcpy(l2Table.mac.octet, pMac->octet, ETHER_ADDR_LEN);
        l2Table.ivl_svl = pL2_data->ivl;
        l2Table.cvid_fid = pL2_data->cvid;
        l2Table.fid = pL2_data->fid;
        l2Table.efid = pL2_data->efid;
        l2Table.spa = rtk_switch_port_L2P_get(pL2_data->port);
        l2Table.nosalearn = pL2_data->is_static;
        l2Table.sa_block = pL2_data->sa_block;
        l2Table.da_block = pL2_data->da_block;
        l2Table.l3lookup = 0;
        l2Table.auth = pL2_data->auth;
        l2Table.age = 6;
        l2Table.lut_pri = pL2_data->priority;
        l2Table.sa_en = pL2_data->sa_pri_en;
        l2Table.fwd_en = pL2_data->fwd_pri_en;
        if ((retVal = rtl8367c_setAsicL2LookupTb(&l2Table)) != RT_ERR_OK)
            return retVal;

        pL2_data->address = l2Table.address;
        return RT_ERR_OK;
    }
    else if (RT_ERR_L2_ENTRY_NOTFOUND == retVal)
    {
        memset(&l2Table, 0, sizeof(rtl8367c_luttb));
        memcpy(l2Table.mac.octet, pMac->octet, ETHER_ADDR_LEN);
        l2Table.ivl_svl = pL2_data->ivl;
        l2Table.cvid_fid = pL2_data->cvid;
        l2Table.fid = pL2_data->fid;
        l2Table.efid = pL2_data->efid;
        l2Table.spa = rtk_switch_port_L2P_get(pL2_data->port);
        l2Table.nosalearn = pL2_data->is_static;
        l2Table.sa_block = pL2_data->sa_block;
        l2Table.da_block = pL2_data->da_block;
        l2Table.l3lookup = 0;
        l2Table.auth = pL2_data->auth;
        l2Table.age = 6;
        l2Table.lut_pri = pL2_data->priority;
        l2Table.sa_en = pL2_data->sa_pri_en;
        l2Table.fwd_en = pL2_data->fwd_pri_en;

        if ((retVal = rtl8367c_setAsicL2LookupTb(&l2Table)) != RT_ERR_OK)
            return retVal;

        pL2_data->address = l2Table.address;

        method = LUTREADMETHOD_MAC;
        retVal = rtl8367c_getAsicL2LookupTb(method, &l2Table);
        if (RT_ERR_L2_ENTRY_NOTFOUND == retVal)
            return RT_ERR_L2_INDEXTBL_FULL;
        else
            return retVal;
    }
    else
        return retVal;
}

int32_t rtl8367::rtk_l2_addr_del(rtk_mac_t *pMac, rtk_l2_ucastAddr_t *pL2_data)
{
    int32_t retVal;
    uint32_t method;
    rtl8367c_luttb l2Table;

    /* must be unicast address */
    if ((pMac == NULL) || (pMac->octet[0] & 0x1))
        return RT_ERR_MAC;

    if (pL2_data->fid > RTL8367C_FIDMAX || pL2_data->efid > RTL8367C_EFIDMAX)
        return RT_ERR_L2_FID;

    memset(&l2Table, 0, sizeof(rtl8367c_luttb));

    /* fill key (MAC,FID) to get L2 entry */
    memcpy(l2Table.mac.octet, pMac->octet, ETHER_ADDR_LEN);
    l2Table.ivl_svl = pL2_data->ivl;
    l2Table.cvid_fid = pL2_data->cvid;
    l2Table.fid = pL2_data->fid;
    l2Table.efid = pL2_data->efid;
    method = LUTREADMETHOD_MAC;
    retVal = rtl8367c_getAsicL2LookupTb(method, &l2Table);
    if (RT_ERR_OK == retVal)
    {
        memcpy(l2Table.mac.octet, pMac->octet, ETHER_ADDR_LEN);
        l2Table.ivl_svl = pL2_data->ivl;
        l2Table.cvid_fid = pL2_data->cvid;
        l2Table.fid = pL2_data->fid;
        l2Table.efid = pL2_data->efid;
        l2Table.spa = 0;
        l2Table.nosalearn = 0;
        l2Table.sa_block = 0;
        l2Table.da_block = 0;
        l2Table.auth = 0;
        l2Table.age = 0;
        l2Table.lut_pri = 0;
        l2Table.sa_en = 0;
        l2Table.fwd_en = 0;
        if ((retVal = rtl8367c_setAsicL2LookupTb(&l2Table)) != RT_ERR_OK)
            return retVal;

        pL2_data->address = l2Table.address;
        return RT_ERR_OK;
    }
    else
        return retVal;
}

int32_t rtl8367::rtk_l2_addr_get(rtk_mac_t *pMac, rtk_l2_ucastAddr_t *pL2_data)
{
    int32_t retVal;
    uint32_t method;
    rtl8367c_luttb l2Table;

    /* must be unicast address */
    if ((pMac == NULL) || (pMac->octet[0] & 0x1))
        return RT_ERR_MAC;

    if (pL2_data->fid > RTL8367C_FIDMAX || pL2_data->efid > RTL8367C_EFIDMAX)
        return RT_ERR_L2_FID;

    memset(&l2Table, 0, sizeof(rtl8367c_luttb));

    memcpy(l2Table.mac.octet, pMac->octet, ETHER_ADDR_LEN);
    l2Table.ivl_svl = pL2_data->ivl;
    l2Table.cvid_fid = pL2_data->cvid;
    l2Table.fid = pL2_data->fid;
    l2Table.efid = pL2_data->efid;
    method = LUTREADMETHOD_MAC;

    if ((retVal = rtl8367c_getAsicL2LookupTb(method, &l2Table)) != RT_ERR_OK)
        return retVal;

    memcpy(pL2_data->mac.octet, pMac->octet, ETHER_ADDR_LEN);
    pL2_data->port = rtk_switch_port_P2L_get(l2Table.spa);
    pL2_data->fid = l2Table.fid;
    pL2_data->efid = l2Table.efid;
    pL2_data->ivl = l2Table.ivl_svl;
    pL2_data->cvid = l2Table.cvid_fid;
    pL2_data->is_static = l2Table.nosalearn;
    pL2_data->auth = l2Table.auth;
    pL2_data->sa_block = l2Table.sa_block;
    pL2_data->da_block = l2Table.da_block;
    pL2_data->priority = l2Table.lut_pri;
    pL2_data->sa_pri_en = l2Table.sa_en;
    pL2_data->fwd_pri_en = l2Table.fwd_en;
    pL2_data->address = l2Table.address;

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_l2_addr_next_get(rtk_l2_read_method_t read_method, rtk_port_t port, uint32_t *pAddress, rtk_l2_ucastAddr_t *pL2_data)
{
    int32_t retVal;
    uint32_t method;
    rtl8367c_luttb l2Table;

    /* Error Checking */
    if ((pL2_data == NULL) || (pAddress == NULL))
        return RT_ERR_MAC;

    if (read_method == READMETHOD_NEXT_L2UC)
        method = LUTREADMETHOD_NEXT_L2UC;
    else if (read_method == READMETHOD_NEXT_L2UCSPA)
        method = LUTREADMETHOD_NEXT_L2UCSPA;
    else
        return RT_ERR_INPUT;

    /* Check Port Valid */
    RTK_CHK_PORT_VALID(port);

    if (*pAddress > halCtrl.max_lut_addr_num - 1)
        return RT_ERR_L2_L2UNI_PARAM;

    memset(&l2Table, 0, sizeof(rtl8367c_luttb));
    l2Table.address = *pAddress;

    if (read_method == READMETHOD_NEXT_L2UCSPA)
        l2Table.spa = rtk_switch_port_L2P_get(port);

    if ((retVal = rtl8367c_getAsicL2LookupTb(method, &l2Table)) != RT_ERR_OK)
        return retVal;

    if (l2Table.address < *pAddress)
        return RT_ERR_L2_ENTRY_NOTFOUND;

    memcpy(pL2_data->mac.octet, l2Table.mac.octet, ETHER_ADDR_LEN);
    pL2_data->port = rtk_switch_port_P2L_get(l2Table.spa);
    pL2_data->fid = l2Table.fid;
    pL2_data->efid = l2Table.efid;
    pL2_data->ivl = l2Table.ivl_svl;
    pL2_data->cvid = l2Table.cvid_fid;
    pL2_data->is_static = l2Table.nosalearn;
    pL2_data->auth = l2Table.auth;
    pL2_data->sa_block = l2Table.sa_block;
    pL2_data->da_block = l2Table.da_block;
    pL2_data->priority = l2Table.lut_pri;
    pL2_data->sa_pri_en = l2Table.sa_en;
    pL2_data->fwd_pri_en = l2Table.fwd_en;
    pL2_data->address = l2Table.address;

    *pAddress = l2Table.address;

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_l2_mcastAddr_add(rtk_l2_mcastAddr_t *pMcastAddr)
{
    int32_t retVal;
    uint32_t method;
    rtl8367c_luttb l2Table;
    uint32_t pmask;

    if (NULL == pMcastAddr)
        return RT_ERR_NULL_POINTER;

    /* must be L2 multicast address */
    if ((pMcastAddr->mac.octet[0] & 0x01) != 0x01)
        return RT_ERR_MAC;

    RTK_CHK_PORTMASK_VALID(&pMcastAddr->portmask);

    if (pMcastAddr->ivl == 1)
    {
        if (pMcastAddr->vid > RTL8367C_VIDMAX)
            return RT_ERR_L2_VID;
    }
    else if (pMcastAddr->ivl == 0)
    {
        if (pMcastAddr->fid > RTL8367C_FIDMAX)
            return RT_ERR_L2_FID;
    }
    else
        return RT_ERR_INPUT;

    if (pMcastAddr->fwd_pri_en >= RTK_ENABLE_END)
        return RT_ERR_INPUT;

    if (pMcastAddr->priority > RTL8367C_PRIMAX)
        return RT_ERR_INPUT;

    /* Get physical port mask */
    if ((retVal = rtk_switch_portmask_L2P_get(&pMcastAddr->portmask, &pmask)) != RT_ERR_OK)
        return retVal;

    memset(&l2Table, 0, sizeof(rtl8367c_luttb));

    /* fill key (MAC,FID) to get L2 entry */
    memcpy(l2Table.mac.octet, pMcastAddr->mac.octet, ETHER_ADDR_LEN);
    l2Table.ivl_svl = pMcastAddr->ivl;

    if (pMcastAddr->ivl)
        l2Table.cvid_fid = pMcastAddr->vid;
    else
        l2Table.cvid_fid = pMcastAddr->fid;

    method = LUTREADMETHOD_MAC;
    retVal = rtl8367c_getAsicL2LookupTb(method, &l2Table);
    if (RT_ERR_OK == retVal)
    {
        memcpy(l2Table.mac.octet, pMcastAddr->mac.octet, ETHER_ADDR_LEN);
        l2Table.ivl_svl = pMcastAddr->ivl;

        if (pMcastAddr->ivl)
            l2Table.cvid_fid = pMcastAddr->vid;
        else
            l2Table.cvid_fid = pMcastAddr->fid;

        l2Table.mbr = pmask;
        l2Table.nosalearn = 1;
        l2Table.l3lookup = 0;
        l2Table.lut_pri = pMcastAddr->priority;
        l2Table.fwd_en = pMcastAddr->fwd_pri_en;
        if ((retVal = rtl8367c_setAsicL2LookupTb(&l2Table)) != RT_ERR_OK)
            return retVal;

        pMcastAddr->address = l2Table.address;
        return RT_ERR_OK;
    }
    else if (RT_ERR_L2_ENTRY_NOTFOUND == retVal)
    {
        memset(&l2Table, 0, sizeof(rtl8367c_luttb));
        memcpy(l2Table.mac.octet, pMcastAddr->mac.octet, ETHER_ADDR_LEN);
        l2Table.ivl_svl = pMcastAddr->ivl;
        if (pMcastAddr->ivl)
            l2Table.cvid_fid = pMcastAddr->vid;
        else
            l2Table.cvid_fid = pMcastAddr->fid;

        l2Table.mbr = pmask;
        l2Table.nosalearn = 1;
        l2Table.l3lookup = 0;
        l2Table.lut_pri = pMcastAddr->priority;
        l2Table.fwd_en = pMcastAddr->fwd_pri_en;
        if ((retVal = rtl8367c_setAsicL2LookupTb(&l2Table)) != RT_ERR_OK)
            return retVal;

        pMcastAddr->address = l2Table.address;

        method = LUTREADMETHOD_MAC;
        retVal = rtl8367c_getAsicL2LookupTb(method, &l2Table);
        if (RT_ERR_L2_ENTRY_NOTFOUND == retVal)
            return RT_ERR_L2_INDEXTBL_FULL;
        else
            return retVal;
    }
    else
        return retVal;
}

int32_t rtl8367::rtk_l2_mcastAddr_del(rtk_l2_mcastAddr_t *pMcastAddr)
{
    int32_t retVal;
    uint32_t method;
    rtl8367c_luttb l2Table;

    if (NULL == pMcastAddr)
        return RT_ERR_NULL_POINTER;

    /* must be L2 multicast address */
    if ((pMcastAddr->mac.octet[0] & 0x01) != 0x01)
        return RT_ERR_MAC;

    if (pMcastAddr->ivl == 1)
    {
        if (pMcastAddr->vid > RTL8367C_VIDMAX)
            return RT_ERR_L2_VID;
    }
    else if (pMcastAddr->ivl == 0)
    {
        if (pMcastAddr->fid > RTL8367C_FIDMAX)
            return RT_ERR_L2_FID;
    }
    else
        return RT_ERR_INPUT;

    memset(&l2Table, 0, sizeof(rtl8367c_luttb));

    /* fill key (MAC,FID) to get L2 entry */
    memcpy(l2Table.mac.octet, pMcastAddr->mac.octet, ETHER_ADDR_LEN);
    l2Table.ivl_svl = pMcastAddr->ivl;

    if (pMcastAddr->ivl)
        l2Table.cvid_fid = pMcastAddr->vid;
    else
        l2Table.cvid_fid = pMcastAddr->fid;

    method = LUTREADMETHOD_MAC;
    retVal = rtl8367c_getAsicL2LookupTb(method, &l2Table);
    if (RT_ERR_OK == retVal)
    {
        memcpy(l2Table.mac.octet, pMcastAddr->mac.octet, ETHER_ADDR_LEN);
        l2Table.ivl_svl = pMcastAddr->ivl;

        if (pMcastAddr->ivl)
            l2Table.cvid_fid = pMcastAddr->vid;
        else
            l2Table.cvid_fid = pMcastAddr->fid;

        l2Table.mbr = 0;
        l2Table.nosalearn = 0;
        l2Table.sa_block = 0;
        l2Table.l3lookup = 0;
        l2Table.lut_pri = 0;
        l2Table.fwd_en = 0;
        if ((retVal = rtl8367c_setAsicL2LookupTb(&l2Table)) != RT_ERR_OK)
            return retVal;

        pMcastAddr->address = l2Table.address;
        return RT_ERR_OK;
    }
    else
        return retVal;
}

int32_t rtl8367::rtk_l2_mcastAddr_get(rtk_l2_mcastAddr_t *pMcastAddr)
{
    int32_t retVal;
    uint32_t method;
    rtl8367c_luttb l2Table;

    if (NULL == pMcastAddr)
        return RT_ERR_NULL_POINTER;

    /* must be L2 multicast address */
    if ((pMcastAddr->mac.octet[0] & 0x01) != 0x01)
        return RT_ERR_MAC;

    if (pMcastAddr->ivl == 1)
    {
        if (pMcastAddr->vid > RTL8367C_VIDMAX)
            return RT_ERR_L2_VID;
    }
    else if (pMcastAddr->ivl == 0)
    {
        if (pMcastAddr->fid > RTL8367C_FIDMAX)
            return RT_ERR_L2_FID;
    }
    else
        return RT_ERR_INPUT;

    memset(&l2Table, 0, sizeof(rtl8367c_luttb));
    memcpy(l2Table.mac.octet, pMcastAddr->mac.octet, ETHER_ADDR_LEN);
    l2Table.ivl_svl = pMcastAddr->ivl;

    if (pMcastAddr->ivl)
        l2Table.cvid_fid = pMcastAddr->vid;
    else
        l2Table.cvid_fid = pMcastAddr->fid;

    method = LUTREADMETHOD_MAC;

    if ((retVal = rtl8367c_getAsicL2LookupTb(method, &l2Table)) != RT_ERR_OK)
        return retVal;

    pMcastAddr->priority = l2Table.lut_pri;
    pMcastAddr->fwd_pri_en = l2Table.fwd_en;
    pMcastAddr->igmp_asic = l2Table.igmp_asic;
    pMcastAddr->igmp_index = l2Table.igmpidx;
    pMcastAddr->address = l2Table.address;

    /* Get Logical port mask */
    if ((retVal = rtk_switch_portmask_P2L_get(l2Table.mbr, &pMcastAddr->portmask)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_l2_mcastAddr_next_get(uint32_t *pAddress, rtk_l2_mcastAddr_t *pMcastAddr)
{
    int32_t retVal;
    rtl8367c_luttb l2Table;

    /* Error Checking */
    if ((pAddress == NULL) || (pMcastAddr == NULL))
        return RT_ERR_INPUT;

    if (*pAddress > halCtrl.max_lut_addr_num - 1)
        return RT_ERR_L2_L2UNI_PARAM;

    memset(&l2Table, 0, sizeof(rtl8367c_luttb));
    l2Table.address = *pAddress;

    if ((retVal = rtl8367c_getAsicL2LookupTb(LUTREADMETHOD_NEXT_L2MC, &l2Table)) != RT_ERR_OK)
        return retVal;

    if (l2Table.address < *pAddress)
        return RT_ERR_L2_ENTRY_NOTFOUND;

    memcpy(pMcastAddr->mac.octet, l2Table.mac.octet, ETHER_ADDR_LEN);
    pMcastAddr->ivl = l2Table.ivl_svl;

    if (pMcastAddr->ivl)
        pMcastAddr->vid = l2Table.cvid_fid;
    else
        pMcastAddr->fid = l2Table.cvid_fid;

    pMcastAddr->priority = l2Table.lut_pri;
    pMcastAddr->fwd_pri_en = l2Table.fwd_en;
    pMcastAddr->igmp_asic = l2Table.igmp_asic;
    pMcastAddr->igmp_index = l2Table.igmpidx;
    pMcastAddr->address = l2Table.address;

    /* Get Logical port mask */
    if ((retVal = rtk_switch_portmask_P2L_get(l2Table.mbr, &pMcastAddr->portmask)) != RT_ERR_OK)
        return retVal;

    *pAddress = l2Table.address;

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_l2_ipMcastAddr_add(rtk_l2_ipMcastAddr_t *pIpMcastAddr)
{
    int32_t retVal;
    uint32_t method;
    rtl8367c_luttb l2Table;
    uint32_t pmask;

    if (NULL == pIpMcastAddr)
        return RT_ERR_NULL_POINTER;

    /* check port mask */
    RTK_CHK_PORTMASK_VALID(&pIpMcastAddr->portmask);

    if ((pIpMcastAddr->dip & 0xF0000000) != 0xE0000000)
        return RT_ERR_INPUT;

    if (pIpMcastAddr->fwd_pri_en >= RTK_ENABLE_END)
        return RT_ERR_ENABLE;

    if (pIpMcastAddr->priority > RTL8367C_PRIMAX)
        return RT_ERR_INPUT;

    /* Get Physical port mask */
    if ((retVal = rtk_switch_portmask_L2P_get(&pIpMcastAddr->portmask, &pmask)) != RT_ERR_OK)
        return retVal;

    memset(&l2Table, 0x00, sizeof(rtl8367c_luttb));
    l2Table.sip = pIpMcastAddr->sip;
    l2Table.dip = pIpMcastAddr->dip;
    l2Table.l3lookup = 1;
    l2Table.l3vidlookup = 0;
    method = LUTREADMETHOD_MAC;
    retVal = rtl8367c_getAsicL2LookupTb(method, &l2Table);
    if (RT_ERR_OK == retVal)
    {
        l2Table.sip = pIpMcastAddr->sip;
        l2Table.dip = pIpMcastAddr->dip;
        l2Table.mbr = pmask;
        l2Table.nosalearn = 1;
        l2Table.l3lookup = 1;
        l2Table.l3vidlookup = 0;
        l2Table.lut_pri = pIpMcastAddr->priority;
        l2Table.fwd_en = pIpMcastAddr->fwd_pri_en;
        if ((retVal = rtl8367c_setAsicL2LookupTb(&l2Table)) != RT_ERR_OK)
            return retVal;

        pIpMcastAddr->address = l2Table.address;
        return RT_ERR_OK;
    }
    else if (RT_ERR_L2_ENTRY_NOTFOUND == retVal)
    {
        memset(&l2Table, 0, sizeof(rtl8367c_luttb));
        l2Table.sip = pIpMcastAddr->sip;
        l2Table.dip = pIpMcastAddr->dip;
        l2Table.mbr = pmask;
        l2Table.nosalearn = 1;
        l2Table.l3lookup = 1;
        l2Table.l3vidlookup = 0;
        l2Table.lut_pri = pIpMcastAddr->priority;
        l2Table.fwd_en = pIpMcastAddr->fwd_pri_en;
        if ((retVal = rtl8367c_setAsicL2LookupTb(&l2Table)) != RT_ERR_OK)
            return retVal;

        pIpMcastAddr->address = l2Table.address;

        method = LUTREADMETHOD_MAC;
        retVal = rtl8367c_getAsicL2LookupTb(method, &l2Table);
        if (RT_ERR_L2_ENTRY_NOTFOUND == retVal)
            return RT_ERR_L2_INDEXTBL_FULL;
        else
            return retVal;
    }
    else
        return retVal;
}

int32_t rtl8367::rtk_l2_ipMcastAddr_del(rtk_l2_ipMcastAddr_t *pIpMcastAddr)
{
    int32_t retVal;
    uint32_t method;
    rtl8367c_luttb l2Table;

    /* Error Checking */
    if (pIpMcastAddr == NULL)
        return RT_ERR_INPUT;

    if ((pIpMcastAddr->dip & 0xF0000000) != 0xE0000000)
        return RT_ERR_INPUT;

    memset(&l2Table, 0x00, sizeof(rtl8367c_luttb));
    l2Table.sip = pIpMcastAddr->sip;
    l2Table.dip = pIpMcastAddr->dip;
    l2Table.l3lookup = 1;
    l2Table.l3vidlookup = 0;
    method = LUTREADMETHOD_MAC;
    retVal = rtl8367c_getAsicL2LookupTb(method, &l2Table);
    if (RT_ERR_OK == retVal)
    {
        l2Table.sip = pIpMcastAddr->sip;
        l2Table.dip = pIpMcastAddr->dip;
        l2Table.mbr = 0;
        l2Table.nosalearn = 0;
        l2Table.l3lookup = 1;
        l2Table.l3vidlookup = 0;
        l2Table.lut_pri = 0;
        l2Table.fwd_en = 0;
        if ((retVal = rtl8367c_setAsicL2LookupTb(&l2Table)) != RT_ERR_OK)
            return retVal;

        pIpMcastAddr->address = l2Table.address;
        return RT_ERR_OK;
    }
    else
        return retVal;
}

int32_t rtl8367::rtk_l2_ipMcastAddr_get(rtk_l2_ipMcastAddr_t *pIpMcastAddr)
{
    int32_t retVal;
    uint32_t method;
    rtl8367c_luttb l2Table;

    if (NULL == pIpMcastAddr)
        return RT_ERR_NULL_POINTER;

    if ((pIpMcastAddr->dip & 0xF0000000) != 0xE0000000)
        return RT_ERR_INPUT;

    memset(&l2Table, 0x00, sizeof(rtl8367c_luttb));
    l2Table.sip = pIpMcastAddr->sip;
    l2Table.dip = pIpMcastAddr->dip;
    l2Table.l3lookup = 1;
    l2Table.l3vidlookup = 0;
    method = LUTREADMETHOD_MAC;
    if ((retVal = rtl8367c_getAsicL2LookupTb(method, &l2Table)) != RT_ERR_OK)
        return retVal;

    /* Get Logical port mask */
    if ((retVal = rtk_switch_portmask_P2L_get(l2Table.mbr, &pIpMcastAddr->portmask)) != RT_ERR_OK)
        return retVal;

    pIpMcastAddr->priority = l2Table.lut_pri;
    pIpMcastAddr->fwd_pri_en = l2Table.fwd_en;
    pIpMcastAddr->igmp_asic = l2Table.igmp_asic;
    pIpMcastAddr->igmp_index = l2Table.igmpidx;
    pIpMcastAddr->address = l2Table.address;

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_l2_ipMcastAddr_next_get(uint32_t *pAddress, rtk_l2_ipMcastAddr_t *pIpMcastAddr)
{
    int32_t retVal;
    rtl8367c_luttb l2Table;

    /* Error Checking */
    if ((pAddress == NULL) || (pIpMcastAddr == NULL))
        return RT_ERR_INPUT;

    if (*pAddress > halCtrl.max_lut_addr_num - 1)
        return RT_ERR_L2_L2UNI_PARAM;

    memset(&l2Table, 0, sizeof(rtl8367c_luttb));
    l2Table.address = *pAddress;

    do
    {
        if ((retVal = rtl8367c_getAsicL2LookupTb(LUTREADMETHOD_NEXT_L3MC, &l2Table)) != RT_ERR_OK)
            return retVal;

        if (l2Table.address < *pAddress)
            return RT_ERR_L2_ENTRY_NOTFOUND;

    } while (l2Table.l3vidlookup == 1);

    pIpMcastAddr->sip = l2Table.sip;
    pIpMcastAddr->dip = l2Table.dip;

    /* Get Logical port mask */
    if ((retVal = rtk_switch_portmask_P2L_get(l2Table.mbr, &pIpMcastAddr->portmask)) != RT_ERR_OK)
        return retVal;

    pIpMcastAddr->priority = l2Table.lut_pri;
    pIpMcastAddr->fwd_pri_en = l2Table.fwd_en;
    pIpMcastAddr->igmp_asic = l2Table.igmp_asic;
    pIpMcastAddr->igmp_index = l2Table.igmpidx;
    pIpMcastAddr->address = l2Table.address;
    *pAddress = l2Table.address;

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_l2_ipVidMcastAddr_add(rtk_l2_ipVidMcastAddr_t *pIpVidMcastAddr)
{
    int32_t retVal;
    uint32_t method;
    rtl8367c_luttb l2Table;
    uint32_t pmask;

    if (NULL == pIpVidMcastAddr)
        return RT_ERR_NULL_POINTER;

    /* check port mask */
    RTK_CHK_PORTMASK_VALID(&pIpVidMcastAddr->portmask);

    if (pIpVidMcastAddr->vid > RTL8367C_VIDMAX)
        return RT_ERR_L2_VID;

    if ((pIpVidMcastAddr->dip & 0xF0000000) != 0xE0000000)
        return RT_ERR_INPUT;

    /* Get Physical port mask */
    if ((retVal = rtk_switch_portmask_L2P_get(&pIpVidMcastAddr->portmask, &pmask)) != RT_ERR_OK)
        return retVal;

    memset(&l2Table, 0x00, sizeof(rtl8367c_luttb));
    l2Table.sip = pIpVidMcastAddr->sip;
    l2Table.dip = pIpVidMcastAddr->dip;
    l2Table.l3lookup = 1;
    l2Table.l3vidlookup = 1;
    l2Table.l3_vid = pIpVidMcastAddr->vid;
    method = LUTREADMETHOD_MAC;
    retVal = rtl8367c_getAsicL2LookupTb(method, &l2Table);
    if (RT_ERR_OK == retVal)
    {
        l2Table.sip = pIpVidMcastAddr->sip;
        l2Table.dip = pIpVidMcastAddr->dip;
        l2Table.mbr = pmask;
        l2Table.nosalearn = 1;
        l2Table.l3lookup = 1;
        l2Table.l3vidlookup = 1;
        l2Table.l3_vid = pIpVidMcastAddr->vid;
        if ((retVal = rtl8367c_setAsicL2LookupTb(&l2Table)) != RT_ERR_OK)
            return retVal;

        pIpVidMcastAddr->address = l2Table.address;
        return RT_ERR_OK;
    }
    else if (RT_ERR_L2_ENTRY_NOTFOUND == retVal)
    {
        memset(&l2Table, 0, sizeof(rtl8367c_luttb));
        l2Table.sip = pIpVidMcastAddr->sip;
        l2Table.dip = pIpVidMcastAddr->dip;
        l2Table.mbr = pmask;
        l2Table.nosalearn = 1;
        l2Table.l3lookup = 1;
        l2Table.l3vidlookup = 1;
        l2Table.l3_vid = pIpVidMcastAddr->vid;
        if ((retVal = rtl8367c_setAsicL2LookupTb(&l2Table)) != RT_ERR_OK)
            return retVal;

        pIpVidMcastAddr->address = l2Table.address;

        method = LUTREADMETHOD_MAC;
        retVal = rtl8367c_getAsicL2LookupTb(method, &l2Table);
        if (RT_ERR_L2_ENTRY_NOTFOUND == retVal)
            return RT_ERR_L2_INDEXTBL_FULL;
        else
            return retVal;
    }
    else
        return retVal;
}

int32_t rtl8367::rtk_l2_ipVidMcastAddr_del(rtk_l2_ipVidMcastAddr_t *pIpVidMcastAddr)
{
    int32_t retVal;
    uint32_t method;
    rtl8367c_luttb l2Table;

    if (NULL == pIpVidMcastAddr)
        return RT_ERR_NULL_POINTER;

    if (pIpVidMcastAddr->vid > RTL8367C_VIDMAX)
        return RT_ERR_L2_VID;

    if ((pIpVidMcastAddr->dip & 0xF0000000) != 0xE0000000)
        return RT_ERR_INPUT;

    memset(&l2Table, 0x00, sizeof(rtl8367c_luttb));
    l2Table.sip = pIpVidMcastAddr->sip;
    l2Table.dip = pIpVidMcastAddr->dip;
    l2Table.l3lookup = 1;
    l2Table.l3vidlookup = 1;
    l2Table.l3_vid = pIpVidMcastAddr->vid;
    method = LUTREADMETHOD_MAC;
    retVal = rtl8367c_getAsicL2LookupTb(method, &l2Table);
    if (RT_ERR_OK == retVal)
    {
        l2Table.sip = pIpVidMcastAddr->sip;
        l2Table.dip = pIpVidMcastAddr->dip;
        l2Table.mbr = 0;
        l2Table.nosalearn = 0;
        l2Table.l3lookup = 1;
        l2Table.l3vidlookup = 1;
        l2Table.l3_vid = pIpVidMcastAddr->vid;
        if ((retVal = rtl8367c_setAsicL2LookupTb(&l2Table)) != RT_ERR_OK)
            return retVal;

        pIpVidMcastAddr->address = l2Table.address;
        return RT_ERR_OK;
    }
    else
        return retVal;
}

int32_t rtl8367::rtk_l2_ipVidMcastAddr_get(rtk_l2_ipVidMcastAddr_t *pIpVidMcastAddr)
{
    int32_t retVal;
    uint32_t method;
    rtl8367c_luttb l2Table;

    if (NULL == pIpVidMcastAddr)
        return RT_ERR_NULL_POINTER;

    if (pIpVidMcastAddr->vid > RTL8367C_VIDMAX)
        return RT_ERR_L2_VID;

    if ((pIpVidMcastAddr->dip & 0xF0000000) != 0xE0000000)
        return RT_ERR_INPUT;

    memset(&l2Table, 0x00, sizeof(rtl8367c_luttb));
    l2Table.sip = pIpVidMcastAddr->sip;
    l2Table.dip = pIpVidMcastAddr->dip;
    l2Table.l3lookup = 1;
    l2Table.l3vidlookup = 1;
    l2Table.l3_vid = pIpVidMcastAddr->vid;
    method = LUTREADMETHOD_MAC;
    if ((retVal = rtl8367c_getAsicL2LookupTb(method, &l2Table)) != RT_ERR_OK)
        return retVal;

    pIpVidMcastAddr->address = l2Table.address;

    /* Get Logical port mask */
    if ((retVal = rtk_switch_portmask_P2L_get(l2Table.mbr, &pIpVidMcastAddr->portmask)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_l2_ipVidMcastAddr_next_get(uint32_t *pAddress, rtk_l2_ipVidMcastAddr_t *pIpVidMcastAddr)
{
    int32_t retVal;
    rtl8367c_luttb l2Table;

    /* Error Checking */
    if ((pAddress == NULL) || (pIpVidMcastAddr == NULL))
        return RT_ERR_INPUT;

    if (*pAddress > halCtrl.max_lut_addr_num - 1)
        return RT_ERR_L2_L2UNI_PARAM;

    memset(&l2Table, 0, sizeof(rtl8367c_luttb));
    l2Table.address = *pAddress;

    do
    {
        if ((retVal = rtl8367c_getAsicL2LookupTb(LUTREADMETHOD_NEXT_L3MC, &l2Table)) != RT_ERR_OK)
            return retVal;

        if (l2Table.address < *pAddress)
            return RT_ERR_L2_ENTRY_NOTFOUND;

    } while (l2Table.l3vidlookup == 0);

    pIpVidMcastAddr->sip = l2Table.sip;
    pIpVidMcastAddr->dip = l2Table.dip;
    pIpVidMcastAddr->vid = l2Table.l3_vid;
    pIpVidMcastAddr->address = l2Table.address;

    /* Get Logical port mask */
    if ((retVal = rtk_switch_portmask_P2L_get(l2Table.mbr, &pIpVidMcastAddr->portmask)) != RT_ERR_OK)
        return retVal;

    *pAddress = l2Table.address;

    return RT_ERR_OK;
}

// ----------------------- QoS -----------------------

/* Function Name:
 *      rtl8367c_setAsicOutputQueueMappingIndex
 * Description:
 *      Set output queue number for each port
 * Input:
 *      port     - Physical port number (0~7)
 *      index     - Mapping table index
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK             - Success
 *      RT_ERR_SMI          - SMI access error
 *      RT_ERR_PORT_ID      - Invalid port number
 *      RT_ERR_QUEUE_NUM      - Invalid queue number
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicOutputQueueMappingIndex(uint32_t port, uint32_t index)
{
    if (port > RTL8367C_PORTIDMAX)
        return RT_ERR_PORT_ID;

    if (index >= RTL8367C_QUEUENO)
        return RT_ERR_QUEUE_NUM;

    return rtl8367c_setAsicRegBits(RTL8367C_QOS_PORT_QUEUE_NUMBER_REG(port), RTL8367C_QOS_PORT_QUEUE_NUMBER_MASK(port), index);
}

/* Function Name:
 *      rtl8367c_setAsicPriorityToQIDMappingTable
 * Description:
 *      Set priority to QID mapping table parameters
 * Input:
 *      index         - Mapping table index
 *      priority     - The priority value
 *      qid         - Queue id
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK                 - Success
 *      RT_ERR_SMI              - SMI access error
 *      RT_ERR_QUEUE_ID          - Invalid queue id
 *      RT_ERR_QUEUE_NUM          - Invalid queue number
 *      RT_ERR_QOS_INT_PRIORITY - Invalid priority
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicPriorityToQIDMappingTable(uint32_t index, uint32_t priority, uint32_t qid)
{
    if (index >= RTL8367C_QUEUENO)
        return RT_ERR_QUEUE_NUM;

    if (priority > RTL8367C_PRIMAX)
        return RT_ERR_QOS_INT_PRIORITY;

    if (qid > RTL8367C_QIDMAX)
        return RT_ERR_QUEUE_ID;

    return rtl8367c_setAsicRegBits(RTL8367C_QOS_1Q_PRIORITY_TO_QID_REG(index, priority), RTL8367C_QOS_1Q_PRIORITY_TO_QID_MASK(priority), qid);
}

/* Function Name:
 *      rtl8367c_setAsicFlowControlSelect
 * Description:
 *      Set system flow control type
 * Input:
 *      select      - System flow control type 1: Ingress flow control 0:Egress flow control
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK   - Success
 *      RT_ERR_SMI  - SMI access error
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicFlowControlSelect(uint32_t select)
{
    return rtl8367c_setAsicRegBit(RTL8367C_REG_FLOWCTRL_CTRL0, RTL8367C_FLOWCTRL_TYPE_OFFSET, select);
}

/* Function Name:
 *      rtl8367c_setAsicPriorityDecision
 * Description:
 *      Set priority decision table
 * Input:
 *      prisrc         - Priority decision source
 *      decisionPri - Decision priority assignment
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK                     - Success
 *      RT_ERR_SMI                  - SMI access error
 *      RT_ERR_QOS_INT_PRIORITY        - Invalid priority
 *      RT_ERR_QOS_SEL_PRI_SOURCE    - Invalid priority decision source parameter
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicPriorityDecision(uint32_t index, uint32_t prisrc, uint32_t decisionPri)
{
    int32_t retVal;

    if (index >= PRIDEC_IDX_END)
        return RT_ERR_ENTRY_INDEX;

    if (prisrc >= PRIDEC_END)
        return RT_ERR_QOS_SEL_PRI_SOURCE;

    if (decisionPri > RTL8367C_DECISIONPRIMAX)
        return RT_ERR_QOS_INT_PRIORITY;

    switch (index)
    {
    case PRIDEC_IDX0:
        if ((retVal = rtl8367c_setAsicRegBits(RTL8367C_QOS_INTERNAL_PRIORITY_DECISION_REG(prisrc), RTL8367C_QOS_INTERNAL_PRIORITY_DECISION_MASK(prisrc), decisionPri)) != RT_ERR_OK)
            return retVal;
        break;
    case PRIDEC_IDX1:
        if ((retVal = rtl8367c_setAsicRegBits(RTL8367C_QOS_INTERNAL_PRIORITY_DECISION2_REG(prisrc), RTL8367C_QOS_INTERNAL_PRIORITY_DECISION2_MASK(prisrc), decisionPri)) != RT_ERR_OK)
            return retVal;
        break;
    default:
        break;
    };

    return RT_ERR_OK;
}
/* Function Name:
 *      rtl8367c_setAsicRemarkingDot1pAbility
 * Description:
 *      Set 802.1p remarking ability
 * Input:
 *      port     - Physical port number (0~7)
 *      enabled - 1: enabled, 0: disabled
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK             - Success
 *      RT_ERR_SMI          - SMI access error
 *      RT_ERR_PORT_ID      - Invalid port number
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicRemarkingDot1pAbility(uint32_t port, uint32_t enabled)
{
    return rtl8367c_setAsicRegBit(RTL8367C_PORT_MISC_CFG_REG(port), RTL8367C_1QREMARK_ENABLE_OFFSET, enabled);
}
/* Function Name:
 *      rtl8367c_setAsicRemarkingDscpAbility
 * Description:
 *      Set DSCP remarking ability
 * Input:
 *      enabled     - 1: enabled, 0: disabled
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK     - Success
 *      RT_ERR_SMI  - SMI access error
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicRemarkingDscpAbility(uint32_t enabled)
{
    return rtl8367c_setAsicRegBit(RTL8367C_REMARKING_CTRL_REG, RTL8367C_REMARKING_DSCP_ENABLE_OFFSET, enabled);
}
/* Function Name:
 *      rtl8367c_setAsicPriorityDot1qRemapping
 * Description:
 *      Set 802.1Q absolutely priority
 * Input:
 *      srcpriority - Priority value
 *      priority     - Absolute priority value
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK                 - Success
 *      RT_ERR_SMI              - SMI access error
 *      RT_ERR_QOS_INT_PRIORITY    - Invalid priority
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicPriorityDot1qRemapping(uint32_t srcpriority, uint32_t priority)
{
    if ((srcpriority > RTL8367C_PRIMAX) || (priority > RTL8367C_PRIMAX))
        return RT_ERR_QOS_INT_PRIORITY;

    return rtl8367c_setAsicRegBits(RTL8367C_QOS_1Q_PRIORITY_REMAPPING_REG(srcpriority), RTL8367C_QOS_1Q_PRIORITY_REMAPPING_MASK(srcpriority), priority);
}
/* Function Name:
 *      rtl8367c_setAsicRemarkingDot1pParameter
 * Description:
 *      Set 802.1p remarking parameter
 * Input:
 *      priority     - Priority value
 *      newPriority - New priority value
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK                 - Success
 *      RT_ERR_SMI              - SMI access error
 *      RT_ERR_QOS_INT_PRIORITY - Invalid priority
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicRemarkingDot1pParameter(uint32_t priority, uint32_t newPriority)
{
    if (priority > RTL8367C_PRIMAX || newPriority > RTL8367C_PRIMAX)
        return RT_ERR_QOS_INT_PRIORITY;

    return rtl8367c_setAsicRegBits(RTL8367C_QOS_1Q_REMARK_REG(priority), RTL8367C_QOS_1Q_REMARK_MASK(priority), newPriority);
}
/* Function Name:
 *      rtl8367c_setAsicRemarkingDscpParameter
 * Description:
 *      Set DSCP remarking parameter
 * Input:
 *      priority     - Priority value
 *      newDscp     - New DSCP value
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK                 - Success
 *      RT_ERR_SMI              - SMI access error
 *      RT_ERR_QOS_DSCP_VALUE    - Invalid DSCP value
 *      RT_ERR_QOS_INT_PRIORITY    - Invalid priority
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicRemarkingDscpParameter(uint32_t priority, uint32_t newDscp)
{
    if (priority > RTL8367C_PRIMAX)
        return RT_ERR_QOS_INT_PRIORITY;

    if (newDscp > RTL8367C_DSCPMAX)
        return RT_ERR_QOS_DSCP_VALUE;

    return rtl8367c_setAsicRegBits(RTL8367C_QOS_DSCP_REMARK_REG(priority), RTL8367C_QOS_DSCP_REMARK_MASK(priority), newDscp);
}
/* Function Name:
 *      rtl8367c_setAsicPriorityDscpBased
 * Description:
 *      Set DSCP-based priority
 * Input:
 *      dscp         - DSCP value
 *      priority     - Priority value
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK                 - Success
 *      RT_ERR_SMI              - SMI access error
 *      RT_ERR_QOS_DSCP_VALUE    - Invalid DSCP value
 *      RT_ERR_QOS_INT_PRIORITY    - Invalid priority
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicPriorityDscpBased(uint32_t dscp, uint32_t priority)
{
    if (priority > RTL8367C_PRIMAX)
        return RT_ERR_QOS_INT_PRIORITY;

    if (dscp > RTL8367C_DSCPMAX)
        return RT_ERR_QOS_DSCP_VALUE;

    return rtl8367c_setAsicRegBits(RTL8367C_QOS_DSCP_TO_PRIORITY_REG(dscp), RTL8367C_QOS_DSCP_TO_PRIORITY_MASK(dscp), priority);
}
int32_t rtl8367::rtk_qos_init(uint32_t queueNum)
{
    const uint16_t g_prioritytToQid[8][8] = {
        {0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 7, 7, 7, 7},
        {0, 0, 0, 0, 1, 1, 7, 7},
        {0, 0, 1, 1, 2, 2, 7, 7},
        {0, 0, 1, 1, 2, 3, 7, 7},
        {0, 0, 1, 2, 3, 4, 7, 7},
        {0, 0, 1, 2, 3, 4, 5, 7},
        {0, 1, 2, 3, 4, 5, 6, 7}};

    const uint32_t g_priorityDecision[8] = {0x01, 0x80, 0x04, 0x02, 0x20, 0x40, 0x10, 0x08};
    const uint32_t g_prioritytRemap[8] = {0, 1, 2, 3, 4, 5, 6, 7};

    int32_t retVal;
    uint32_t qmapidx;
    uint32_t priority;
    uint32_t priDec;
    uint32_t port;
    uint32_t dscp;

    if (queueNum <= 0 || queueNum > RTK_MAX_NUM_OF_QUEUE)
        return RT_ERR_QUEUE_NUM;

    /*Set Output Queue Number*/
    if (RTK_MAX_NUM_OF_QUEUE == queueNum)
        qmapidx = 0;
    else
        qmapidx = queueNum;

    RTK_SCAN_ALL_PHY_PORTMASK(port)
    {
        if ((retVal = rtl8367c_setAsicOutputQueueMappingIndex(port, qmapidx)) != RT_ERR_OK)
            return retVal;
    }

    /*Set Priority to Qid*/
    for (priority = 0; priority <= RTK_PRIMAX; priority++)
    {
        if ((retVal = rtl8367c_setAsicPriorityToQIDMappingTable(queueNum - 1, priority, g_prioritytToQid[queueNum - 1][priority])) != RT_ERR_OK)
            return retVal;
    }

    /*Set Flow Control Type to Ingress Flow Control*/
    if ((retVal = rtl8367c_setAsicFlowControlSelect(FC_INGRESS)) != RT_ERR_OK)
        return retVal;

    /*Priority Decision Order*/
    for (priDec = 0; priDec < PRIDEC_END; priDec++)
    {
        if ((retVal = rtl8367c_setAsicPriorityDecision(PRIDECTBL_IDX0, priDec, g_priorityDecision[priDec])) != RT_ERR_OK)
            return retVal;
        if ((retVal = rtl8367c_setAsicPriorityDecision(PRIDECTBL_IDX1, priDec, g_priorityDecision[priDec])) != RT_ERR_OK)
            return retVal;
    }

    /*Set Port-based Priority to 0*/
    RTK_SCAN_ALL_PHY_PORTMASK(port)
    {
        if ((retVal = rtl8367c_setAsicPriorityPortBased(port, 0)) != RT_ERR_OK)
            return retVal;
    }

    /*Disable 1p Remarking*/
    RTK_SCAN_ALL_PHY_PORTMASK(port)
    {
        if ((retVal = rtl8367c_setAsicRemarkingDot1pAbility(port, DISABLED)) != RT_ERR_OK)
            return retVal;
    }

    /*Disable DSCP Remarking*/
    if ((retVal = rtl8367c_setAsicRemarkingDscpAbility(DISABLED)) != RT_ERR_OK)
        return retVal;

    /*Set 1p & DSCP  Priority Remapping & Remarking*/
    for (priority = 0; priority <= RTL8367C_PRIMAX; priority++)
    {
        if ((retVal = rtl8367c_setAsicPriorityDot1qRemapping(priority, g_prioritytRemap[priority])) != RT_ERR_OK)
            return retVal;

        if ((retVal = rtl8367c_setAsicRemarkingDot1pParameter(priority, 0)) != RT_ERR_OK)
            return retVal;

        if ((retVal = rtl8367c_setAsicRemarkingDscpParameter(priority, 0)) != RT_ERR_OK)
            return retVal;
    }

    /*Set DSCP Priority*/
    for (dscp = 0; dscp <= 63; dscp++)
    {
        if ((retVal = rtl8367c_setAsicPriorityDscpBased(dscp, 0)) != RT_ERR_OK)
            return retVal;
    }

    /* Finetune B/T value */
    if ((retVal = rtl8367c_setAsicReg(0x1722, 0x1158)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsicPriorityPortBased
 * Description:
 *      Set port based priority
 * Input:
 *      port         - Physical port number (0~7)
 *      priority     - Priority value
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK                 - Success
 *      RT_ERR_SMI              - SMI access error
 *      RT_ERR_PORT_ID          - Invalid port number
 *      RT_ERR_QOS_INT_PRIORITY    - Invalid priority
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicPriorityPortBased(uint32_t port, uint32_t priority)
{
    int32_t retVal;

    if (port > RTL8367C_PORTIDMAX)
        return RT_ERR_PORT_ID;

    if (priority > RTL8367C_PRIMAX)
        return RT_ERR_QOS_INT_PRIORITY;

    if (port < 8)
    {
        retVal = rtl8367c_setAsicRegBits(RTL8367C_QOS_PORTBASED_PRIORITY_REG(port), RTL8367C_QOS_PORTBASED_PRIORITY_MASK(port), priority);
        if (retVal != RT_ERR_OK)
            return retVal;
    }
    else
    {
        retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_QOS_PORTBASED_PRIORITY_CTRL2, 0x7 << ((port - 8) << 2), priority);
        if (retVal != RT_ERR_OK)
            return retVal;
    }

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_qos_portPri_set(rtk_port_t port, uint32_t int_pri)
{
    int32_t retVal;

    /* Check Port Valid */
    RTK_CHK_PORT_VALID(port);

    if (int_pri > RTL8367C_PRIMAX)
        return RT_ERR_QOS_INT_PRIORITY;

    if ((retVal = rtl8367c_setAsicPriorityPortBased(rtk_switch_port_L2P_get(port), int_pri)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_qos_1pPriRemap_set(uint32_t dot1p_pri, uint32_t int_pri)
{
    int32_t retVal;

    if (dot1p_pri > RTL8367C_PRIMAX || int_pri > RTL8367C_PRIMAX)
        return RT_ERR_VLAN_PRIORITY;

    if ((retVal = rtl8367c_setAsicPriorityDot1qRemapping(dot1p_pri, int_pri)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_qos_priSel_set(rtk_qos_priDecTbl_t index, rtk_priority_select_t *pPriDec)
{
    int32_t retVal;
    uint32_t port_pow;
    uint32_t dot1q_pow;
    uint32_t dscp_pow;
    uint32_t acl_pow;
    uint32_t svlan_pow;
    uint32_t cvlan_pow;
    uint32_t smac_pow;
    uint32_t dmac_pow;
    uint32_t i;

    if (index < 0 || index >= PRIDECTBL_END)
        return RT_ERR_ENTRY_INDEX;

    if (pPriDec->port_pri >= 8 || pPriDec->dot1q_pri >= 8 || pPriDec->acl_pri >= 8 || pPriDec->dscp_pri >= 8 ||
        pPriDec->cvlan_pri >= 8 || pPriDec->svlan_pri >= 8 || pPriDec->dmac_pri >= 8 || pPriDec->smac_pri >= 8)
        return RT_ERR_QOS_SEL_PRI_SOURCE;

    port_pow = 1;
    for (i = pPriDec->port_pri; i > 0; i--)
        port_pow = (port_pow)*2;

    dot1q_pow = 1;
    for (i = pPriDec->dot1q_pri; i > 0; i--)
        dot1q_pow = (dot1q_pow)*2;

    acl_pow = 1;
    for (i = pPriDec->acl_pri; i > 0; i--)
        acl_pow = (acl_pow)*2;

    dscp_pow = 1;
    for (i = pPriDec->dscp_pri; i > 0; i--)
        dscp_pow = (dscp_pow)*2;

    svlan_pow = 1;
    for (i = pPriDec->svlan_pri; i > 0; i--)
        svlan_pow = (svlan_pow)*2;

    cvlan_pow = 1;
    for (i = pPriDec->cvlan_pri; i > 0; i--)
        cvlan_pow = (cvlan_pow)*2;

    dmac_pow = 1;
    for (i = pPriDec->dmac_pri; i > 0; i--)
        dmac_pow = (dmac_pow)*2;

    smac_pow = 1;
    for (i = pPriDec->smac_pri; i > 0; i--)
        smac_pow = (smac_pow)*2;

    if ((retVal = rtl8367c_setAsicPriorityDecision(index, PRIDEC_PORT, port_pow)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicPriorityDecision(index, PRIDEC_ACL, acl_pow)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicPriorityDecision(index, PRIDEC_DSCP, dscp_pow)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicPriorityDecision(index, PRIDEC_1Q, dot1q_pow)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicPriorityDecision(index, PRIDEC_1AD, svlan_pow)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicPriorityDecision(index, PRIDEC_CVLAN, cvlan_pow)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicPriorityDecision(index, PRIDEC_DA, dmac_pow)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicPriorityDecision(index, PRIDEC_SA, smac_pow)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsicPortPriorityDecisionIndex
 * Description:
 *      Set priority decision index for each port
 * Input:
 *      port     - Physical port number (0~7)
 *      index     - Table index
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK             - Success
 *      RT_ERR_SMI          - SMI access error
 *      RT_ERR_PORT_ID      - Invalid port number
 *      RT_ERR_QUEUE_NUM      - Invalid queue number
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicPortPriorityDecisionIndex(uint32_t port, uint32_t index)
{
    if (port > RTL8367C_PORTIDMAX)
        return RT_ERR_PORT_ID;

    if (index >= PRIDEC_IDX_END)
        return RT_ERR_ENTRY_INDEX;

    return rtl8367c_setAsicRegBit(RTL8367C_QOS_INTERNAL_PRIORITY_DECISION_IDX_CTRL, port, index);
}

int32_t rtl8367::rtk_qos_portPriSelIndex_set(rtk_port_t port, rtk_qos_priDecTbl_t index)
{
    int32_t retVal;

    /* Check Port Valid */
    RTK_CHK_PORT_VALID(port);

    if (index >= PRIDECTBL_END)
        return RT_ERR_ENTRY_INDEX;

    if ((retVal = rtl8367c_setAsicPortPriorityDecisionIndex(rtk_switch_port_L2P_get(port), index)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_qos_priMap_set(uint32_t queue_num, rtk_qos_pri2queue_t *pPri2qid)
{
    int32_t retVal;
    uint32_t pri;

    if ((0 == queue_num) || (queue_num > RTK_MAX_NUM_OF_QUEUE))
        return RT_ERR_QUEUE_NUM;

    for (pri = 0; pri <= RTK_PRIMAX; pri++)
    {
        if (pPri2qid->pri2queue[pri] > RTK_QIDMAX)
            return RT_ERR_QUEUE_ID;

        if ((retVal = rtl8367c_setAsicPriorityToQIDMappingTable(queue_num - 1, pri, pPri2qid->pri2queue[pri])) != RT_ERR_OK)
            return retVal;
    }

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsicQueueType
 * Description:
 *      Set type of a queue
 * Input:
 *      port        - Physical port number (0~10)
 *      qid         - The queue ID wanted to set
 *      queueType   - The specified queue type. 0b0: Strict priority, 0b1: WFQ
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_SMI      - SMI access error
 *      RT_ERR_PORT_ID  - Invalid port number
 *      RT_ERR_QUEUE_ID - Invalid queue id
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicQueueType(uint32_t port, uint32_t qid, uint32_t queueType)
{
    int32_t retVal;

    /* Invalid input parameter */
    if (port > RTL8367C_PORTIDMAX)
        return RT_ERR_PORT_ID;

    if (qid > RTL8367C_QIDMAX)
        return RT_ERR_QUEUE_ID;

    /* Set Related Registers */
    retVal = rtl8367c_setAsicRegBit(RTL8367C_SCHEDULE_QUEUE_TYPE_REG(port), RTL8367C_SCHEDULE_QUEUE_TYPE_OFFSET(port, qid), queueType);

    return retVal;
}

/* Function Name:
 *      rtl8367c_setAsicWFQWeight
 * Description:
 *      Set weight  of a queue
 * Input:
 *      port    - Physical port number (0~10)
 *      qid     - The queue ID wanted to set
 *      qWeight - The weight value wanted to set (valid:0~127)
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK               - Success
 *      RT_ERR_SMI              - SMI access error
 *      RT_ERR_PORT_ID          - Invalid port number
 *      RT_ERR_QUEUE_ID         - Invalid queue id
 *      RT_ERR_QOS_QUEUE_WEIGHT - Invalid queue weight
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicWFQWeight(uint32_t port, uint32_t qid, uint32_t qWeight)
{
    int32_t retVal;

    /* Invalid input parameter */
    if (port > RTL8367C_PORTIDMAX)
        return RT_ERR_PORT_ID;

    if (qid > RTL8367C_QIDMAX)
        return RT_ERR_QUEUE_ID;

    if (qWeight > RTL8367C_QWEIGHTMAX && qid > 0)
        return RT_ERR_QOS_QUEUE_WEIGHT;

    retVal = rtl8367c_setAsicReg(RTL8367C_SCHEDULE_PORT_QUEUE_WFQ_WEIGHT_REG(port, qid), qWeight);

    return retVal;
}
int32_t rtl8367::rtk_qos_schedulingQueue_set(rtk_port_t port, rtk_qos_queue_weights_t *pQweights)
{
    int32_t retVal;
    uint32_t qid;

    /* Check Port Valid */
    RTK_CHK_PORT_VALID(port);

    for (qid = 0; qid < RTL8367C_QUEUENO; qid++)
    {

        if (pQweights->weights[qid] > QOS_WEIGHT_MAX)
            return RT_ERR_QOS_QUEUE_WEIGHT;

        if (0 == pQweights->weights[qid])
        {
            if ((retVal = rtl8367c_setAsicQueueType(rtk_switch_port_L2P_get(port), qid, QTYPE_STRICT)) != RT_ERR_OK)
                return retVal;
        }
        else
        {
            if ((retVal = rtl8367c_setAsicQueueType(rtk_switch_port_L2P_get(port), qid, QTYPE_WFQ)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicWFQWeight(rtk_switch_port_L2P_get(port), qid, pQweights->weights[qid])) != RT_ERR_OK)
                return retVal;
        }
    }

    return RT_ERR_OK;
}

// ----------------- CPU -----------------

/* Function Name:
 *      rtl8367c_setAsicCputagEnable
 * Description:
 *      Set cpu tag function enable/disable
 * Input:
 *      enabled - 1: enabled, 0: disabled
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_SMI      - SMI access error
 *      RT_ERR_ENABLE   - Invalid enable/disable input
 * Note:
 *      If CPU tag function is disabled, CPU tag will not be added to frame
 *      forwarded to CPU port, and all ports cannot parse CPU tag.
 */
int32_t rtl8367::rtl8367c_setAsicCputagEnable(uint32_t enabled)
{
    if (enabled > 1)
        return RT_ERR_ENABLE;

    return rtl8367c_setAsicRegBit(RTL8367C_REG_CPU_CTRL, RTL8367C_CPU_EN_OFFSET, enabled);
}

/* Function Name:
 *      rtl8367c_setAsicCputagPortmask
 * Description:
 *      Set ports that can parse CPU tag
 * Input:
 *      portmask - port mask
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK           - Success
 *      RT_ERR_SMI          - SMI access error
 *      RT_ERR_PORT_MASK    - Invalid portmask
 * Note:
 *     None
 */
int32_t rtl8367::rtl8367c_setAsicCputagPortmask(uint32_t portmask)
{
    if (portmask > RTL8367C_PORTMASK)
        return RT_ERR_PORT_MASK;

    return rtl8367c_setAsicReg(RTL8367C_CPU_PORT_MASK_REG, portmask);
}

int32_t rtl8367::rtk_cpu_enable_set(rtk_enable_t enable)
{
    int32_t retVal;

    if (enable >= RTK_ENABLE_END)
        return RT_ERR_ENABLE;

    if ((retVal = rtl8367c_setAsicCputagEnable(enable)) != RT_ERR_OK)
        return retVal;

    if (DISABLED == enable)
    {
        if ((retVal = rtl8367c_setAsicCputagPortmask(0)) != RT_ERR_OK)
            return retVal;
    }

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsicCputagTrapPort
 * Description:
 *      Set cpu tag trap port
 * Input:
 *      port - port number
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_SMI      - SMI access error
 *      RT_ERR_PORT_ID  - Invalid port number
 * Note:
 *     API can set destination port of trapping frame
 */
int32_t rtl8367::rtl8367c_setAsicCputagTrapPort(uint32_t port)
{
    int32_t retVal;

    if (port >= RTL8367C_PORTNO)
        return RT_ERR_PORT_ID;

    retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_CPU_CTRL, RTL8367C_CPU_TRAP_PORT_MASK, port & 7);
    if (retVal != RT_ERR_OK)
        return retVal;

    retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_CPU_CTRL, RTL8367C_CPU_TRAP_PORT_EXT_MASK, (port >> 3) & 1);
    if (retVal != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsicCputagInsertMode
 * Description:
 *      Set CPU-tag insert mode
 * Input:
 *      mode - 0: insert to all packets; 1: insert to trapped packets; 2: don't insert
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK           - Success
 *      RT_ERR_SMI          - SMI access error
 *      RT_ERR_NOT_ALLOWED  - Actions not allowed by the function
 * Note:
 *     None
 */
int32_t rtl8367::rtl8367c_setAsicCputagInsertMode(uint32_t mode)
{
    if (mode >= CPUTAG_INSERT_END)
        return RT_ERR_NOT_ALLOWED;

    return rtl8367c_setAsicRegBits(RTL8367C_REG_CPU_CTRL, RTL8367C_CPU_INSERTMODE_MASK, mode);
}
int32_t rtl8367::rtk_cpu_tagPort_set(rtk_port_t port, rtk_cpu_insert_t mode)
{
    int32_t retVal;

    /* Check port Valid */
    RTK_CHK_PORT_VALID(port);

    if (mode >= CPU_INSERT_END)
        return RT_ERR_INPUT;

    if ((retVal = rtl8367c_setAsicCputagPortmask(1 << rtk_switch_port_L2P_get(port))) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicCputagTrapPort(rtk_switch_port_L2P_get(port))) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicCputagInsertMode(mode)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_getAsicCputagPortmask
 * Description:
 *      Get ports that can parse CPU tag
 * Input:
 *      pPortmask - port mask
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK           - Success
 *      RT_ERR_SMI          - SMI access error
 * Note:
 *     None
 */
int32_t rtl8367::rtl8367c_getAsicCputagPortmask(uint32_t *pPortmask)
{
    return rtl8367c_getAsicReg(RTL8367C_CPU_PORT_MASK_REG, pPortmask);
}
/* Function Name:
 *      rtl8367c_getAsicCputagTrapPort
 * Description:
 *      Get cpu tag trap port
 * Input:
 *      pPort - port number
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_SMI      - SMI access error
 * Note:
 *     None
 */
int32_t rtl8367::rtl8367c_getAsicCputagTrapPort(uint32_t *pPort)
{
    int32_t retVal;
    uint32_t tmpPort;

    retVal = rtl8367c_getAsicRegBits(RTL8367C_REG_CPU_CTRL, RTL8367C_CPU_TRAP_PORT_MASK, &tmpPort);
    if (retVal != RT_ERR_OK)
        return retVal;
    *pPort = tmpPort;

    retVal = rtl8367c_getAsicRegBits(RTL8367C_REG_CPU_CTRL, RTL8367C_CPU_TRAP_PORT_EXT_MASK, &tmpPort);
    if (retVal != RT_ERR_OK)
        return retVal;
    *pPort |= (tmpPort & 1) << 3;

    return RT_ERR_OK;
}
/* Function Name:
 *      rtl8367c_getAsicCputagInsertMode
 * Description:
 *      Get CPU-tag insert mode
 * Input:
 *      pMode - 0: insert to all packets; 1: insert to trapped packets; 2: don't insert
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK           - Success
 *      RT_ERR_SMI          - SMI access error
 * Note:
 *     None
 */
int32_t rtl8367::rtl8367c_getAsicCputagInsertMode(uint32_t *pMode)
{
    return rtl8367c_getAsicRegBits(RTL8367C_REG_CPU_CTRL, RTL8367C_CPU_INSERTMODE_MASK, pMode);
}
int32_t rtl8367::rtk_cpu_tagPort_get(rtk_port_t *pPort, rtk_cpu_insert_t *pMode)
{
    int32_t retVal;
    uint32_t pmsk, port;

    if (NULL == pPort)
        return RT_ERR_NULL_POINTER;

    if (NULL == pMode)
        return RT_ERR_NULL_POINTER;

    if ((retVal = rtl8367c_getAsicCputagPortmask(&pmsk)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_getAsicCputagTrapPort(&port)) != RT_ERR_OK)
        return retVal;

    *pPort = (rtk_port_t)rtk_switch_port_P2L_get(port);

    if ((retVal = rtl8367c_getAsicCputagInsertMode((uint32_t *)pMode)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

// -------------- INTERRUPT --------------

/* Function Name:
 *      rtl8367c_setAsicInterruptPolarity
 * Description:
 *      Set interrupt trigger polarity
 * Input:
 *      polarity    - 0:pull high 1: pull low
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK   - Success
 *      RT_ERR_SMI  - SMI access error
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicInterruptPolarity(uint32_t polarity)
{
    return rtl8367c_setAsicRegBit(RTL8367C_REG_INTR_CTRL, RTL8367C_INTR_CTRL_OFFSET, polarity);
}
int32_t rtl8367::rtk_int_polarity_set(rtk_int_polarity_t type)
{
    int32_t retVal;

    if (type >= INT_POLAR_END)
        return RT_ERR_INPUT;

    if ((retVal = rtl8367c_setAsicInterruptPolarity(type)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_getAsicInterruptPolarity
 * Description:
 *      Get interrupt trigger polarity
 * Input:
 *      pPolarity   - 0:pull high 1: pull low
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK   - Success
 *      RT_ERR_SMI  - SMI access error
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicInterruptPolarity(uint32_t *pPolarity)
{
    return rtl8367c_getAsicRegBit(RTL8367C_REG_INTR_CTRL, RTL8367C_INTR_CTRL_OFFSET, pPolarity);
}

int32_t rtl8367::rtk_int_polarity_get(rtk_int_polarity_t *pType)
{
    int32_t retVal;

    if (NULL == pType)
        return RT_ERR_NULL_POINTER;

    if ((retVal = rtl8367c_getAsicInterruptPolarity((uint32_t *)pType)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_getAsicInterruptMask
 * Description:
 *      Get interrupt enable mask
 * Input:
 *      pImr    - Interrupt mask
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK   - Success
 *      RT_ERR_SMI  - SMI access error
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicInterruptMask(uint32_t *pImr)
{
    return rtl8367c_getAsicReg(RTL8367C_REG_INTR_IMR, pImr);
}
/* Function Name:
 *      rtl8367c_setAsicInterruptMask
 * Description:
 *      Set interrupt enable mask
 * Input:
 *      imr     - Interrupt mask
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK   - Success
 *      RT_ERR_SMI  - SMI access error
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicInterruptMask(uint32_t imr)
{
    return rtl8367c_setAsicReg(RTL8367C_REG_INTR_IMR, imr);
}
int32_t rtl8367::rtk_int_control_set(rtk_int_type_t type, rtk_enable_t enable)
{
    int32_t retVal;
    uint32_t mask;

    if (type >= INT_TYPE_END)
        return RT_ERR_INPUT;

    if (type == INT_TYPE_RESERVED)
        return RT_ERR_INPUT;

    if ((retVal = rtl8367c_getAsicInterruptMask(&mask)) != RT_ERR_OK)
        return retVal;

    if (ENABLED == enable)
        mask = mask | (1 << type);
    else if (DISABLED == enable)
        mask = mask & ~(1 << type);
    else
        return RT_ERR_INPUT;

    if ((retVal = rtl8367c_setAsicInterruptMask(mask)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_int_control_get(rtk_int_type_t type, rtk_enable_t *pEnable)
{
    int32_t retVal;
    uint32_t mask;

    if (NULL == pEnable)
        return RT_ERR_NULL_POINTER;

    if ((retVal = rtl8367c_getAsicInterruptMask(&mask)) != RT_ERR_OK)
        return retVal;

    if (0 == (mask & (1 << type)))
        *pEnable = (rtk_enable_t)0;
    else
        *pEnable = (rtk_enable_t)1;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_getAsicInterruptStatus
 * Description:
 *      Get interrupt enable mask
 * Input:
 *      pIms    - Interrupt status mask
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK   - Success
 *      RT_ERR_SMI  - SMI access error
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicInterruptStatus(uint32_t *pIms)
{
    return rtl8367c_getAsicReg(RTL8367C_REG_INTR_IMS, pIms);
}

int32_t rtl8367::rtk_int_status_get(rtk_int_status_t *pStatusMask)
{
    int32_t retVal;
    uint32_t ims_mask;

    if (NULL == pStatusMask)
        return RT_ERR_NULL_POINTER;

    if ((retVal = rtl8367c_getAsicInterruptStatus(&ims_mask)) != RT_ERR_OK)
        return retVal;

    pStatusMask->value[0] = (ims_mask & 0x00000FFF);
    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsicInterruptMask
 * Description:
 *      Clear interrupt enable mask
 * Input:
 *      ims     - Interrupt status mask
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK   - Success
 *      RT_ERR_SMI  - SMI access error
 * Note:
 *      This API can be used to clear ASIC interrupt status and register will be cleared by writting 1.
 *      [0]:Link change,
 *      [1]:Share meter exceed,
 *      [2]:Learn number overed,
 *      [3]:Speed Change,
 *      [4]:Tx special congestion
 *      [5]:1 second green feature
 *      [6]:loop detection
 *      [7]:interrupt from 8051
 *      [8]:Cable diagnostic finish
 *      [9]:ACL action interrupt trigger
 *      [11]: Silent Start
 */
int32_t rtl8367::rtl8367c_setAsicInterruptStatus(uint32_t ims)
{
    return rtl8367c_setAsicReg(RTL8367C_REG_INTR_IMS, ims);
}
int32_t rtl8367::rtk_int_status_set(rtk_int_status_t *pStatusMask)
{
    int32_t retVal;

    if (NULL == pStatusMask)
        return RT_ERR_NULL_POINTER;

    if (pStatusMask->value[0] & (0x0001 << INT_TYPE_RESERVED))
        return RT_ERR_INPUT;

    if (pStatusMask->value[0] >= (0x0001 << INT_TYPE_END))
        return RT_ERR_INPUT;

    if ((retVal = rtl8367c_setAsicInterruptStatus((uint32_t)pStatusMask->value[0])) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

int32_t rtl8367::_rtk_int_Advidx_get(rtk_int_advType_t adv_type, uint32_t *pAsic_idx)
{
    uint32_t asic_idx[ADV_END] =
        {
            INTRST_L2_LEARN,
            INTRST_SPEED_CHANGE,
            INTRST_SPECIAL_CONGESTION,
            INTRST_PORT_LINKDOWN,
            INTRST_PORT_LINKUP,
            ADV_NOT_SUPPORT,
            INTRST_RLDP_LOOPED,
            INTRST_RLDP_RELEASED,
        };

    if (adv_type >= ADV_END)
        return RT_ERR_INPUT;

    if (asic_idx[adv_type] == ADV_NOT_SUPPORT)
        return RT_ERR_CHIP_NOT_SUPPORTED;

    *pAsic_idx = asic_idx[adv_type];
    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_getAsicInterruptRelatedStatus
 * Description:
 *      Get interrupt status
 * Input:
 *      type    - per port Learn over, per-port speed change, per-port special congest, share meter exceed status
 *      pStatus     - exceed status, write 1 to clear
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK           - Success
 *      RT_ERR_SMI          - SMI access error
 *      RT_ERR_OUT_OF_RANGE - input parameter out of range
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicInterruptRelatedStatus(uint32_t type, uint32_t *pStatus)
{
    const uint32_t indicatorAddress[INTRST_END] = {RTL8367C_REG_LEARN_OVER_INDICATOR,
                                                   RTL8367C_REG_SPEED_CHANGE_INDICATOR,
                                                   RTL8367C_REG_SPECIAL_CONGEST_INDICATOR,
                                                   RTL8367C_REG_PORT_LINKDOWN_INDICATOR,
                                                   RTL8367C_REG_PORT_LINKUP_INDICATOR,
                                                   RTL8367C_REG_METER_OVERRATE_INDICATOR0,
                                                   RTL8367C_REG_METER_OVERRATE_INDICATOR1,
                                                   RTL8367C_REG_RLDP_LOOPED_INDICATOR,
                                                   RTL8367C_REG_RLDP_RELEASED_INDICATOR,
                                                   RTL8367C_REG_SYSTEM_LEARN_OVER_INDICATOR};

    if (type >= INTRST_END)
        return RT_ERR_OUT_OF_RANGE;

    return rtl8367c_getAsicReg(indicatorAddress[type], pStatus);
}
/* Function Name:
 *      rtl8367c_setAsicInterruptRelatedStatus
 * Description:
 *      Clear interrupt status
 * Input:
 *      type    - per port Learn over, per-port speed change, per-port special congest, share meter exceed status
 *      status  - exceed status, write 1 to clear
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK           - Success
 *      RT_ERR_SMI          - SMI access error
 *      RT_ERR_OUT_OF_RANGE - input parameter out of range
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicInterruptRelatedStatus(uint32_t type, uint32_t status)
{
    const uint32_t indicatorAddress[INTRST_END] = {RTL8367C_REG_LEARN_OVER_INDICATOR,
                                                   RTL8367C_REG_SPEED_CHANGE_INDICATOR,
                                                   RTL8367C_REG_SPECIAL_CONGEST_INDICATOR,
                                                   RTL8367C_REG_PORT_LINKDOWN_INDICATOR,
                                                   RTL8367C_REG_PORT_LINKUP_INDICATOR,
                                                   RTL8367C_REG_METER_OVERRATE_INDICATOR0,
                                                   RTL8367C_REG_METER_OVERRATE_INDICATOR1,
                                                   RTL8367C_REG_RLDP_LOOPED_INDICATOR,
                                                   RTL8367C_REG_RLDP_RELEASED_INDICATOR,
                                                   RTL8367C_REG_SYSTEM_LEARN_OVER_INDICATOR};

    if (type >= INTRST_END)
        return RT_ERR_OUT_OF_RANGE;

    return rtl8367c_setAsicReg(indicatorAddress[type], status);
}
int32_t rtl8367::rtk_int_advanceInfo_get(rtk_int_advType_t adv_type, rtk_int_info_t *pInfo)
{
    int32_t retVal;
    uint32_t data;
    uint32_t intAdvType;

    if (adv_type >= ADV_END)
        return RT_ERR_INPUT;

    if (NULL == pInfo)
        return RT_ERR_NULL_POINTER;

    if (adv_type != ADV_METER_EXCEED_MASK)
    {
        if ((retVal = _rtk_int_Advidx_get(adv_type, &intAdvType)) != RT_ERR_OK)
            return retVal;
    }

    switch (adv_type)
    {
    case ADV_L2_LEARN_PORT_MASK:
        /* Get physical portmask */
        if ((retVal = rtl8367c_getAsicInterruptRelatedStatus(intAdvType, &data)) != RT_ERR_OK)
            return retVal;

        /* Clear Advanced Info */
        if ((retVal = rtl8367c_setAsicInterruptRelatedStatus(intAdvType, 0xFFFF)) != RT_ERR_OK)
            return retVal;

        /* Translate to logical portmask */
        if ((retVal = rtk_switch_portmask_P2L_get(data, &(pInfo->portMask))) != RT_ERR_OK)
            return retVal;

        /* Get system learn */
        if ((retVal = rtl8367c_getAsicInterruptRelatedStatus(INTRST_SYS_LEARN, &data)) != RT_ERR_OK)
            return retVal;

        /* Clear system learn */
        if ((retVal = rtl8367c_setAsicInterruptRelatedStatus(INTRST_SYS_LEARN, 0x0001)) != RT_ERR_OK)
            return retVal;

        pInfo->systemLearnOver = data;
        break;
    case ADV_SPEED_CHANGE_PORT_MASK:
    case ADV_SPECIAL_CONGESTION_PORT_MASK:
    case ADV_PORT_LINKDOWN_PORT_MASK:
    case ADV_PORT_LINKUP_PORT_MASK:
    case ADV_RLDP_LOOPED:
    case ADV_RLDP_RELEASED:
        /* Get physical portmask */
        if ((retVal = rtl8367c_getAsicInterruptRelatedStatus(intAdvType, &data)) != RT_ERR_OK)
            return retVal;

        /* Clear Advanced Info */
        if ((retVal = rtl8367c_setAsicInterruptRelatedStatus(intAdvType, 0xFFFF)) != RT_ERR_OK)
            return retVal;

        /* Translate to logical portmask */
        if ((retVal = rtk_switch_portmask_P2L_get(data, &(pInfo->portMask))) != RT_ERR_OK)
            return retVal;

        break;
    case ADV_METER_EXCEED_MASK:
        /* Get Meter Mask */
        if ((retVal = rtl8367c_getAsicInterruptRelatedStatus(INTRST_METER0_15, &data)) != RT_ERR_OK)
            return retVal;

        /* Clear Advanced Info */
        if ((retVal = rtl8367c_setAsicInterruptRelatedStatus(INTRST_METER0_15, 0xFFFF)) != RT_ERR_OK)
            return retVal;

        pInfo->meterMask = data & 0xFFFF;

        /* Get Meter Mask */
        if ((retVal = rtl8367c_getAsicInterruptRelatedStatus(INTRST_METER16_31, &data)) != RT_ERR_OK)
            return retVal;

        /* Clear Advanced Info */
        if ((retVal = rtl8367c_setAsicInterruptRelatedStatus(INTRST_METER16_31, 0xFFFF)) != RT_ERR_OK)
            return retVal;

        pInfo->meterMask = pInfo->meterMask | ((data << 16) & 0xFFFF0000);

        break;
    default:
        return RT_ERR_INPUT;
    }

    return RT_ERR_OK;
}

// ----------------------- MIB -----------------------

#define MIB_NOT_SUPPORT (0xFFFF)
int32_t rtl8367::_get_asic_mib_idx(rtk_stat_port_type_t cnt_idx, RTL8367C_MIBCOUNTER *pMib_idx)
{
    RTL8367C_MIBCOUNTER mib_asic_idx[STAT_PORT_CNTR_END] =
        {
            ifInOctets,                           /* STAT_IfInOctets */
            dot3StatsFCSErrors,                   /* STAT_Dot3StatsFCSErrors */
            dot3StatsSymbolErrors,                /* STAT_Dot3StatsSymbolErrors */
            dot3InPauseFrames,                    /* STAT_Dot3InPauseFrames */
            dot3ControlInUnknownOpcodes,          /* STAT_Dot3ControlInUnknownOpcodes */
            etherStatsFragments,                  /* STAT_EtherStatsFragments */
            etherStatsJabbers,                    /* STAT_EtherStatsJabbers */
            ifInUcastPkts,                        /* STAT_IfInUcastPkts */
            etherStatsDropEvents,                 /* STAT_EtherStatsDropEvents */
            etherStatsOctets,                     /* STAT_EtherStatsOctets */
            etherStatsUnderSizePkts,              /* STAT_EtherStatsUnderSizePkts */
            etherOversizeStats,                   /* STAT_EtherOversizeStats */
            etherStatsPkts64Octets,               /* STAT_EtherStatsPkts64Octets */
            etherStatsPkts65to127Octets,          /* STAT_EtherStatsPkts65to127Octets */
            etherStatsPkts128to255Octets,         /* STAT_EtherStatsPkts128to255Octets */
            etherStatsPkts256to511Octets,         /* STAT_EtherStatsPkts256to511Octets */
            etherStatsPkts512to1023Octets,        /* STAT_EtherStatsPkts512to1023Octets */
            etherStatsPkts1024to1518Octets,       /* STAT_EtherStatsPkts1024to1518Octets */
            ifInMulticastPkts,                    /* STAT_EtherStatsMulticastPkts */
            ifInBroadcastPkts,                    /* STAT_EtherStatsBroadcastPkts */
            ifOutOctets,                          /* STAT_IfOutOctets */
            dot3StatsSingleCollisionFrames,       /* STAT_Dot3StatsSingleCollisionFrames */
            dot3StatMultipleCollisionFrames,      /* STAT_Dot3StatsMultipleCollisionFrames */
            dot3sDeferredTransmissions,           /* STAT_Dot3StatsDeferredTransmissions */
            dot3StatsLateCollisions,              /* STAT_Dot3StatsLateCollisions */
            etherStatsCollisions,                 /* STAT_EtherStatsCollisions */
            dot3StatsExcessiveCollisions,         /* STAT_Dot3StatsExcessiveCollisions */
            dot3OutPauseFrames,                   /* STAT_Dot3OutPauseFrames */
            (RTL8367C_MIBCOUNTER)MIB_NOT_SUPPORT, /* STAT_Dot1dBasePortDelayExceededDiscards */
            dot1dTpPortInDiscards,                /* STAT_Dot1dTpPortInDiscards */
            ifOutUcastPkts,                       /* STAT_IfOutUcastPkts */
            ifOutMulticastPkts,                   /* STAT_IfOutMulticastPkts */
            ifOutBroadcastPkts,                   /* STAT_IfOutBroadcastPkts */
            outOampduPkts,                        /* STAT_OutOampduPkts */
            inOampduPkts,                         /* STAT_InOampduPkts */
            (RTL8367C_MIBCOUNTER)MIB_NOT_SUPPORT, /* STAT_PktgenPkts */
            inMldChecksumError,                   /* STAT_InMldChecksumError */
            inIgmpChecksumError,                  /* STAT_InIgmpChecksumError */
            inMldSpecificQuery,                   /* STAT_InMldSpecificQuery */
            inMldGeneralQuery,                    /* STAT_InMldGeneralQuery */
            inIgmpSpecificQuery,                  /* STAT_InIgmpSpecificQuery */
            inIgmpGeneralQuery,                   /* STAT_InIgmpGeneralQuery */
            inMldLeaves,                          /* STAT_InMldLeaves */
            inIgmpLeaves,                         /* STAT_InIgmpInterfaceLeaves */
            inIgmpJoinsSuccess,                   /* STAT_InIgmpJoinsSuccess */
            inIgmpJoinsFail,                      /* STAT_InIgmpJoinsFail */
            inMldJoinsSuccess,                    /* STAT_InMldJoinsSuccess */
            inMldJoinsFail,                       /* STAT_InMldJoinsFail */
            inReportSuppressionDrop,              /* STAT_InReportSuppressionDrop */
            inLeaveSuppressionDrop,               /* STAT_InLeaveSuppressionDrop */
            outIgmpReports,                       /* STAT_OutIgmpReports */
            outIgmpLeaves,                        /* STAT_OutIgmpLeaves */
            outIgmpGeneralQuery,                  /* STAT_OutIgmpGeneralQuery */
            outIgmpSpecificQuery,                 /* STAT_OutIgmpSpecificQuery */
            outMldReports,                        /* STAT_OutMldReports */
            outMldLeaves,                         /* STAT_OutMldLeaves */
            outMldGeneralQuery,                   /* STAT_OutMldGeneralQuery */
            outMldSpecificQuery,                  /* STAT_OutMldSpecificQuery */
            inKnownMulticastPkts,                 /* STAT_InKnownMulticastPkts */
            ifInMulticastPkts,                    /* STAT_IfInMulticastPkts */
            ifInBroadcastPkts,                    /* STAT_IfInBroadcastPkts */
            ifOutDiscards                         /* STAT_IfOutDiscards */
        };

    if (cnt_idx >= STAT_PORT_CNTR_END)
        return RT_ERR_STAT_INVALID_PORT_CNTR;

    if (mib_asic_idx[cnt_idx] == MIB_NOT_SUPPORT)
        return RT_ERR_CHIP_NOT_SUPPORTED;

    *pMib_idx = mib_asic_idx[cnt_idx];
    return RT_ERR_OK;
}
/* Function Name:
 *      rtl8367c_getAsicMIBsCounter
 * Description:
 *      Get MIBs counter
 * Input:
 *      port        - Physical port number (0~7)
 *      mibIdx      - MIB counter index
 *      pCounter    - MIB retrived counter
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK               - Success
 *      RT_ERR_SMI              - SMI access error
 *      RT_ERR_PORT_ID          - Invalid port number
 *      RT_ERR_BUSYWAIT_TIMEOUT - MIB is busy at retrieving
 *      RT_ERR_STAT_CNTR_FAIL   - MIB is resetting
 * Note:
 *      Before MIBs counter retrieving, writting accessing address to ASIC at first and check the MIB
 *      control register status. If busy bit of MIB control is set, that means MIB counter have been
 *      waiting for preparing, then software must wait atfer this busy flag reset by ASIC. This driver
 *      did not recycle reading user desired counter. Software must use driver again to get MIB counter
 *      if return value is not RT_ERR_OK.
 */
int32_t rtl8367::rtl8367c_getAsicMIBsCounter(uint32_t port, RTL8367C_MIBCOUNTER mibIdx, uint64_t *pCounter)
{
    int32_t retVal;
    uint32_t regAddr;
    uint32_t regData;
    uint32_t mibAddr;
    uint32_t mibOff = 0;

    /* address offset to MIBs counter */
    const uint16_t mibLength[RTL8367C_MIBS_NUMBER] = {
        4, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
        4, 2, 2, 2, 2, 2, 2, 2, 2,
        4, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
        2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2};

    uint16_t i;
    uint64_t mibCounter;

    if (port > RTL8367C_PORTIDMAX)
        return RT_ERR_PORT_ID;

    if (mibIdx >= RTL8367C_MIBS_NUMBER)
        return RT_ERR_STAT_INVALID_CNTR;

    if (dot1dTpLearnedEntryDiscards == mibIdx)
    {
        mibAddr = RTL8367C_MIB_LEARNENTRYDISCARD_OFFSET;
    }
    else
    {
        i = 0;
        mibOff = RTL8367C_MIB_PORT_OFFSET * port;

        if (port > 7)
            mibOff = mibOff + 68;

        while (i < mibIdx)
        {
            mibOff += mibLength[i];
            i++;
        }

        mibAddr = mibOff;
    }

    /* Read MIB addr before writing */
    retVal = rtl8367c_getAsicReg(RTL8367C_REG_MIB_ADDRESS, &regData);
    if (retVal != RT_ERR_OK)
        return retVal;

    if (regData == (mibAddr >> 2))
    {
        /* Write MIB addr to an alternate value */
        retVal = rtl8367c_setAsicReg(RTL8367C_REG_MIB_ADDRESS, (mibAddr >> 2) + 1);
        if (retVal != RT_ERR_OK)
            return retVal;

        while (1)
        {
            retVal = rtl8367c_getAsicReg(RTL8367C_REG_MIB_ADDRESS, &regData);
            if (retVal != RT_ERR_OK)
                return retVal;

            if (regData == ((mibAddr >> 2) + 1))
            {
                break;
            }

            retVal = rtl8367c_setAsicReg(RTL8367C_REG_MIB_ADDRESS, (mibAddr >> 2) + 1);
            if (retVal != RT_ERR_OK)
                return retVal;
        }

        /* polling busy flag */
        i = 100;
        while (i > 0)
        {
            /*read MIB control register*/
            retVal = rtl8367c_getAsicReg(RTL8367C_MIB_CTRL_REG, &regData);
            if (retVal != RT_ERR_OK)
                return retVal;

            if ((regData & RTL8367C_MIB_CTRL0_BUSY_FLAG_MASK) == 0)
            {
                break;
            }

            i--;
        }

        if (regData & RTL8367C_MIB_CTRL0_BUSY_FLAG_MASK)
            return RT_ERR_BUSYWAIT_TIMEOUT;

        if (regData & RTL8367C_RESET_FLAG_MASK)
            return RT_ERR_STAT_CNTR_FAIL;
    }

    /*writing access counter address first*/
    /*This address is SRAM address, and SRAM address = MIB register address >> 2*/
    /*then ASIC will prepare 64bits counter wait for being retrived*/
    /*Write Mib related address to access control register*/
    retVal = rtl8367c_setAsicReg(RTL8367C_REG_MIB_ADDRESS, (mibAddr >> 2));
    if (retVal != RT_ERR_OK)
        return retVal;

    /* polling MIB Addr register */
    while (1)
    {
        retVal = rtl8367c_getAsicReg(RTL8367C_REG_MIB_ADDRESS, &regData);
        if (retVal != RT_ERR_OK)
            return retVal;

        if (regData == (mibAddr >> 2))
        {
            break;
        }

        retVal = rtl8367c_setAsicReg(RTL8367C_REG_MIB_ADDRESS, (mibAddr >> 2));
        if (retVal != RT_ERR_OK)
            return retVal;
    }

    /* polling busy flag */
    i = 100;
    while (i > 0)
    {
        /*read MIB control register*/
        retVal = rtl8367c_getAsicReg(RTL8367C_MIB_CTRL_REG, &regData);
        if (retVal != RT_ERR_OK)
            return retVal;

        if ((regData & RTL8367C_MIB_CTRL0_BUSY_FLAG_MASK) == 0)
        {
            break;
        }

        i--;
    }

    if (regData & RTL8367C_MIB_CTRL0_BUSY_FLAG_MASK)
        return RT_ERR_BUSYWAIT_TIMEOUT;

    if (regData & RTL8367C_RESET_FLAG_MASK)
        return RT_ERR_STAT_CNTR_FAIL;

    mibCounter = 0;
    i = mibLength[mibIdx];
    if (4 == i)
        regAddr = RTL8367C_MIB_COUNTER_BASE_REG + 3;
    else
        regAddr = RTL8367C_MIB_COUNTER_BASE_REG + ((mibOff + 1) % 4);

    while (i)
    {
        retVal = rtl8367c_getAsicReg(regAddr, &regData);
        if (retVal != RT_ERR_OK)
            return retVal;

        mibCounter = (mibCounter << 16) | (regData & 0xFFFF);

        regAddr--;
        i--;
    }

    *pCounter = mibCounter;

    return RT_ERR_OK;
}
int32_t rtl8367::rtk_stat_port_get(rtk_port_t port, rtk_stat_port_type_t cntr_idx, uint64_t *pCntr)
{
    int32_t retVal;
    RTL8367C_MIBCOUNTER mib_idx;
    uint64_t second_cnt;

    if (NULL == pCntr)
        return RT_ERR_NULL_POINTER;

    /* Check port valid */
    RTK_CHK_PORT_VALID(port);

    if (cntr_idx >= STAT_PORT_CNTR_END)
        return RT_ERR_STAT_INVALID_PORT_CNTR;

    if ((retVal = _get_asic_mib_idx(cntr_idx, &mib_idx)) != RT_ERR_OK)
        return retVal;

    if (mib_idx == MIB_NOT_SUPPORT)
        return RT_ERR_CHIP_NOT_SUPPORTED;

    if ((retVal = rtl8367c_getAsicMIBsCounter(rtk_switch_port_L2P_get(port), mib_idx, pCntr)) != RT_ERR_OK)
        return retVal;

    if (cntr_idx == STAT_EtherStatsMulticastPkts)
    {
        if ((retVal = _get_asic_mib_idx(STAT_IfOutMulticastPkts, &mib_idx)) != RT_ERR_OK)
            return retVal;

        if ((retVal = rtl8367c_getAsicMIBsCounter(rtk_switch_port_L2P_get(port), mib_idx, &second_cnt)) != RT_ERR_OK)
            return retVal;

        *pCntr += second_cnt;
    }

    if (cntr_idx == STAT_EtherStatsBroadcastPkts)
    {
        if ((retVal = _get_asic_mib_idx(STAT_IfOutBroadcastPkts, &mib_idx)) != RT_ERR_OK)
            return retVal;

        if ((retVal = rtl8367c_getAsicMIBsCounter(rtk_switch_port_L2P_get(port), mib_idx, &second_cnt)) != RT_ERR_OK)
            return retVal;

        *pCntr += second_cnt;
    }

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsicMIBsCounterReset
 * Description:
 *      Reset global/queue manage or per-port MIB counter
 * Input:
 *      greset  - Global reset
 *      qmreset - Queue maganement reset
 *      portmask    - Port reset mask
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK               - Success
 *      RT_ERR_SMI              - SMI access error
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicMIBsCounterReset(uint32_t greset, uint32_t qmreset, uint32_t portmask)
{
    int32_t retVal;
    uint32_t regData;
    uint32_t regBits;

    regBits = RTL8367C_GLOBAL_RESET_MASK |
              RTL8367C_QM_RESET_MASK |
              RTL8367C_MIB_PORT07_MASK |
              ((uint32_t)0x7 << 13);
    regData = ((greset << RTL8367C_GLOBAL_RESET_OFFSET) & RTL8367C_GLOBAL_RESET_MASK) |
              ((qmreset << RTL8367C_QM_RESET_OFFSET) & RTL8367C_QM_RESET_MASK) |
              (((portmask & 0xFF) << RTL8367C_PORT0_RESET_OFFSET) & RTL8367C_MIB_PORT07_MASK) |
              (((portmask >> 8) & 0x7) << 13);

    retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_MIB_CTRL0, regBits, (regData >> RTL8367C_PORT0_RESET_OFFSET));

    return retVal;
}
int32_t rtl8367::rtk_stat_port_reset(rtk_port_t port)
{
    int32_t retVal;

    /* Check port valid */
    RTK_CHK_PORT_VALID(port);

    if ((retVal = rtl8367c_setAsicMIBsCounterReset(0, 0, 1 << rtk_switch_port_L2P_get(port))) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

// ---------------------- PHY -------------------------

/* Function Name:
 *      rtl8370_setAsicPortEnableAll
 * Description:
 *      Set ALL ports enable.
 * Input:s
 *      enable - enable all ports.
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK           - Success
 *      RT_ERR_SMI          - SMI access error
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicPortEnableAll(uint32_t enable)
{
    if (enable >= 2)
        return RT_ERR_INPUT;

    return rtl8367c_setAsicRegBit(RTL8367C_REG_PHY_AD, RTL8367C_PDNPHY_OFFSET, !enable);
}

int32_t rtl8367::_rtk_port_phyReg_get(rtk_port_t port, rtk_port_phy_reg_t reg, uint32_t *pData)
{
    int32_t retVal;

    /* Check Port Valid */
    RTK_CHK_PORT_IS_UTP(port);

    if ((retVal = rtl8367c_getAsicPHYReg(rtk_switch_port_L2P_get(port), reg, pData)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsicPHYOCPReg
 * Description:
 *      Set PHY OCP registers
 * Input:
 *      phyNo   - Physical port number (0~7)
 *      ocpAddr - OCP address
 *      ocpData - Writing data
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK               - Success
 *      RT_ERR_SMI              - SMI access error
 *      RT_ERR_PHY_REG_ID       - invalid PHY address
 *      RT_ERR_PHY_ID           - invalid PHY no
 *      RT_ERR_BUSYWAIT_TIMEOUT - PHY access busy
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicPHYOCPReg(uint32_t phyNo, uint32_t ocpAddr, uint32_t ocpData)
{
    int32_t retVal;
    uint32_t regData;
    uint32_t busyFlag, checkCounter;
    uint32_t ocpAddrPrefix, ocpAddr9_6, ocpAddr5_1;

    /*Check internal phy access busy or not*/
    /*retVal = rtl8367c_getAsicRegBit(RTL8367C_REG_INDRECT_ACCESS_STATUS, RTL8367C_INDRECT_ACCESS_STATUS_OFFSET,&busyFlag);*/
    retVal = rtl8367c_getAsicReg(RTL8367C_REG_INDRECT_ACCESS_STATUS, &busyFlag);
    if (retVal != RT_ERR_OK)
        return retVal;

    if (busyFlag)
        return RT_ERR_BUSYWAIT_TIMEOUT;

    /* OCP prefix */
    ocpAddrPrefix = ((ocpAddr & 0xFC00) >> 10);
    if ((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_GPHY_OCP_MSB_0, RTL8367C_CFG_CPU_OCPADR_MSB_MASK, ocpAddrPrefix)) != RT_ERR_OK)
        return retVal;

    /*prepare access data*/
    retVal = rtl8367c_setAsicReg(RTL8367C_REG_INDRECT_ACCESS_WRITE_DATA, ocpData);
    if (retVal != RT_ERR_OK)
        return retVal;

    /*prepare access address*/
    ocpAddr9_6 = ((ocpAddr >> 6) & 0x000F);
    ocpAddr5_1 = ((ocpAddr >> 1) & 0x001F);
    regData = RTL8367C_PHY_BASE | (ocpAddr9_6 << 8) | (phyNo << RTL8367C_PHY_OFFSET) | ocpAddr5_1;
    retVal = rtl8367c_setAsicReg(RTL8367C_REG_INDRECT_ACCESS_ADDRESS, regData);
    if (retVal != RT_ERR_OK)
        return retVal;

    /*Set WRITE Command*/
    retVal = rtl8367c_setAsicReg(RTL8367C_REG_INDRECT_ACCESS_CTRL, RTL8367C_CMD_MASK | RTL8367C_RW_MASK);

    checkCounter = 100;
    while (checkCounter)
    {
        retVal = rtl8367c_getAsicReg(RTL8367C_REG_INDRECT_ACCESS_STATUS, &busyFlag);
        if ((retVal != RT_ERR_OK) || busyFlag)
        {
            checkCounter--;
            if (0 == checkCounter)
                return RT_ERR_BUSYWAIT_TIMEOUT;
        }
        else
        {
            checkCounter = 0;
        }
    }

    return retVal;
}
/* Function Name:
 *      rtl8367c_setAsicPHYReg
 * Description:
 *      Set PHY registers
 * Input:
 *      phyNo   - Physical port number (0~7)
 *      phyAddr - PHY address (0~31)
 *      phyData - Writing data
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK               - Success
 *      RT_ERR_SMI              - SMI access error
 *      RT_ERR_PHY_REG_ID       - invalid PHY address
 *      RT_ERR_PHY_ID           - invalid PHY no
 *      RT_ERR_BUSYWAIT_TIMEOUT - PHY access busy
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicPHYReg(uint32_t phyNo, uint32_t phyAddr, uint32_t phyData)
{
    uint32_t ocp_addr;

    if (phyAddr > RTL8367C_PHY_REGNOMAX)
        return RT_ERR_PHY_REG_ID;

    ocp_addr = 0xa400 + phyAddr * 2;

    return rtl8367c_setAsicPHYOCPReg(phyNo, ocp_addr, phyData);
}

int32_t rtl8367::_rtk_port_phyReg_set(rtk_port_t port, rtk_port_phy_reg_t reg, uint32_t regData)
{
    int32_t retVal;

    /* Check Port Valid */
    RTK_CHK_PORT_IS_UTP(port);

    if ((retVal = rtl8367c_setAsicPHYReg(rtk_switch_port_L2P_get(port), reg, regData)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_port_phyEnableAll_set(rtk_enable_t enable)
{
    int32_t retVal;
    uint32_t data;
    uint32_t port;

    if (enable >= RTK_ENABLE_END)
        return RT_ERR_ENABLE;

    if ((retVal = rtl8367c_setAsicPortEnableAll(enable)) != RT_ERR_OK)
        return retVal;

    RTK_SCAN_ALL_LOG_PORT(port)
    {
        if (rtk_switch_isUtpPort(port) == RT_ERR_OK)
        {
            if ((retVal = _rtk_port_phyReg_get((rtk_port_t)port, (rtk_port_phy_reg_t)PHY_CONTROL_REG, &data)) != RT_ERR_OK)
                return retVal;

            if (ENABLED == enable)
            {
                data &= 0xF7FF;
                data |= 0x0200;
            }
            else
            {
                data |= 0x0800;
            }

            if ((retVal = _rtk_port_phyReg_set((rtk_port_t)port, (rtk_port_phy_reg_t)PHY_CONTROL_REG, data)) != RT_ERR_OK)
                return retVal;
        }
    }

    return RT_ERR_OK;
}

/* Function Name:
 *      rtk_switch_isComboPort
 * Description:
 *      Check is logical port a Combo port
 * Input:
 *      logicalPort     - logical port ID
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Port ID is a combo port
 *      RT_ERR_FAILED   - Port ID is not a combo port
 *      RT_ERR_NOT_INIT - Not Initialize
 * Note:
 *
 */
int32_t rtl8367::rtk_switch_isComboPort(rtk_port_t logicalPort)
{
    if (logicalPort >= RTK_SWITCH_PORT_NUM)
        return RT_ERR_FAILED;

    if (halCtrl.combo_logical_port == logicalPort)
        return RT_ERR_OK;
    else
        return RT_ERR_FAILED;
}

int32_t rtl8367::_rtk_port_phyComboPortMedia_get(rtk_port_t port, rtk_port_media_t *pMedia)
{
    int32_t retVal;
    uint32_t regData;
    uint32_t data;

    /* Check Port Valid */
    RTK_CHK_PORT_IS_UTP(port);

    /* Check Combo Port ID */
    RTK_CHK_PORT_IS_COMBO(port);

    if ((retVal = rtl8367c_setAsicReg(0x13C2, 0x0249)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_getAsicReg(0x1300, &regData)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicReg(0x13C2, 0x0000)) != RT_ERR_OK)
        return retVal;

    if (regData != 0x6367)
    {
        *pMedia = PORT_MEDIA_COPPER;
    }
    else
    {
        if ((retVal = rtl8367c_getAsicRegBit(RTL8367C_REG_UTP_FIB_DET, RTL8367C_UTP_FIRST_OFFSET, &data)) != RT_ERR_OK)
            return retVal;

        if (data == 1)
            *pMedia = PORT_MEDIA_COPPER;
        else
            *pMedia = PORT_MEDIA_FIBER;
    }

    return RT_ERR_OK;
}

int32_t rtl8367::_rtk_port_FiberModeAbility_set(rtk_port_t port, rtk_port_phy_ability_t *pAbility)
{
    int32_t retVal;
    uint32_t regData;

    /* Check Combo port or not */
    RTK_CHK_PORT_IS_COMBO(port);

    /* Flow Control */
    if ((retVal = rtl8367c_getAsicReg(RTL8367C_REG_FIB0_CFG04, &regData)) != RT_ERR_OK)
        return retVal;

    if (pAbility->AsyFC == 1)
        regData |= (0x0001 << 8);
    else
        regData &= ~(0x0001 << 8);

    if (pAbility->FC == 1)
        regData |= (0x0001 << 7);
    else
        regData &= ~(0x0001 << 7);

    if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_FIB0_CFG04, regData)) != RT_ERR_OK)
        return retVal;

    /* Speed ability */
    if ((pAbility->Full_1000 == 1) && (pAbility->Full_100 == 1) && (pAbility->AutoNegotiation == 1))
    {
        if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_FIBER_CFG_1, RTL8367C_SDS_FRC_MODE_OFFSET, 0)) != RT_ERR_OK)
            return retVal;

        if ((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_FIBER_CFG_1, RTL8367C_SDS_MODE_MASK, 7)) != RT_ERR_OK)
            return retVal;

        if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_FIB0_CFG00, 0x1140)) != RT_ERR_OK)
            return retVal;
    }
    else if (pAbility->Full_1000 == 1)
    {
        if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_FIBER_CFG_1, RTL8367C_SDS_FRC_MODE_OFFSET, 1)) != RT_ERR_OK)
            return retVal;

        if ((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_FIBER_CFG_1, RTL8367C_SDS_MODE_MASK, 4)) != RT_ERR_OK)
            return retVal;

        if (pAbility->AutoNegotiation == 1)
        {
            if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_FIB0_CFG00, 0x1140)) != RT_ERR_OK)
                return retVal;
        }
        else
        {
            if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_FIB0_CFG00, 0x0140)) != RT_ERR_OK)
                return retVal;
        }
    }
    else if (pAbility->Full_100 == 1)
    {
        if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_FIBER_CFG_1, RTL8367C_SDS_FRC_MODE_OFFSET, 1)) != RT_ERR_OK)
            return retVal;

        if ((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_FIBER_CFG_1, RTL8367C_SDS_MODE_MASK, 5)) != RT_ERR_OK)
            return retVal;

        if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_FIB0_CFG00, 0x2100)) != RT_ERR_OK)
            return retVal;
    }

    /* Digital software reset */
    if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_ADR, 0x0003)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_CMD, 0x0080)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_getAsicReg(RTL8367C_REG_SDS_INDACS_DATA, &regData)) != RT_ERR_OK)
        return retVal;

    regData |= (0x0001 << 6);

    if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_DATA, regData)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_ADR, 0x0003)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_CMD, 0x00C0)) != RT_ERR_OK)
        return retVal;

    regData &= ~(0x0001 << 6);

    if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_DATA, regData)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_ADR, 0x0003)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_CMD, 0x00C0)) != RT_ERR_OK)
        return retVal;

    /* CDR reset */
    if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_DATA, 0x1401)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_ADR, 0x0000)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_CMD, 0x00C0)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_DATA, 0x1403)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_ADR, 0x0000)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_CMD, 0x00C0)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}
int32_t rtl8367::rtk_port_phyAutoNegoAbility_set(rtk_port_t port, rtk_port_phy_ability_t *pAbility)
{
    int32_t retVal;
    uint32_t phyData;
    uint32_t phyEnMsk0;
    uint32_t phyEnMsk4;
    uint32_t phyEnMsk9;
    rtk_port_media_t media_type;

    /* Check Port Valid */
    RTK_CHK_PORT_IS_UTP(port);

    if (NULL == pAbility)
        return RT_ERR_NULL_POINTER;

    if (pAbility->Half_10 >= RTK_ENABLE_END || pAbility->Full_10 >= RTK_ENABLE_END ||
        pAbility->Half_100 >= RTK_ENABLE_END || pAbility->Full_100 >= RTK_ENABLE_END ||
        pAbility->Full_1000 >= RTK_ENABLE_END || pAbility->AutoNegotiation >= RTK_ENABLE_END ||
        pAbility->AsyFC >= RTK_ENABLE_END || pAbility->FC >= RTK_ENABLE_END)
        return RT_ERR_INPUT;

    if (rtk_switch_isComboPort(port) == RT_ERR_OK)
    {
        if ((retVal = _rtk_port_phyComboPortMedia_get(port, &media_type)) != RT_ERR_OK)
            return retVal;

        if (media_type == PORT_MEDIA_FIBER)
        {
            return _rtk_port_FiberModeAbility_set(port, pAbility);
        }
    }

    /*for PHY auto mode setup*/
    pAbility->AutoNegotiation = 1;

    phyEnMsk0 = 0;
    phyEnMsk4 = 0;
    phyEnMsk9 = 0;

    if (1 == pAbility->Half_10)
    {
        /*10BASE-TX half duplex capable in reg 4.5*/
        phyEnMsk4 = phyEnMsk4 | (1 << 5);

        /*Speed selection [1:0] */
        /* 11=Reserved*/
        /* 10= 1000Mpbs*/
        /* 01= 100Mpbs*/
        /* 00= 10Mpbs*/
        phyEnMsk0 = phyEnMsk0 & (~(1 << 6));
        phyEnMsk0 = phyEnMsk0 & (~(1 << 13));
    }

    if (1 == pAbility->Full_10)
    {
        /*10BASE-TX full duplex capable in reg 4.6*/
        phyEnMsk4 = phyEnMsk4 | (1 << 6);
        /*Speed selection [1:0] */
        /* 11=Reserved*/
        /* 10= 1000Mpbs*/
        /* 01= 100Mpbs*/
        /* 00= 10Mpbs*/
        phyEnMsk0 = phyEnMsk0 & (~(1 << 6));
        phyEnMsk0 = phyEnMsk0 & (~(1 << 13));

        /*Full duplex mode in reg 0.8*/
        phyEnMsk0 = phyEnMsk0 | (1 << 8);
    }

    if (1 == pAbility->Half_100)
    {
        /*100BASE-TX half duplex capable in reg 4.7*/
        phyEnMsk4 = phyEnMsk4 | (1 << 7);
        /*Speed selection [1:0] */
        /* 11=Reserved*/
        /* 10= 1000Mpbs*/
        /* 01= 100Mpbs*/
        /* 00= 10Mpbs*/
        phyEnMsk0 = phyEnMsk0 & (~(1 << 6));
        phyEnMsk0 = phyEnMsk0 | (1 << 13);
    }

    if (1 == pAbility->Full_100)
    {
        /*100BASE-TX full duplex capable in reg 4.8*/
        phyEnMsk4 = phyEnMsk4 | (1 << 8);
        /*Speed selection [1:0] */
        /* 11=Reserved*/
        /* 10= 1000Mpbs*/
        /* 01= 100Mpbs*/
        /* 00= 10Mpbs*/
        phyEnMsk0 = phyEnMsk0 & (~(1 << 6));
        phyEnMsk0 = phyEnMsk0 | (1 << 13);
        /*Full duplex mode in reg 0.8*/
        phyEnMsk0 = phyEnMsk0 | (1 << 8);
    }

    if (1 == pAbility->Full_1000)
    {
        /*1000 BASE-T FULL duplex capable setting in reg 9.9*/
        phyEnMsk9 = phyEnMsk9 | (1 << 9);

        /*Speed selection [1:0] */
        /* 11=Reserved*/
        /* 10= 1000Mpbs*/
        /* 01= 100Mpbs*/
        /* 00= 10Mpbs*/
        phyEnMsk0 = phyEnMsk0 | (1 << 6);
        phyEnMsk0 = phyEnMsk0 & (~(1 << 13));

        /*Auto-Negotiation setting in reg 0.12*/
        phyEnMsk0 = phyEnMsk0 | (1 << 12);
    }

    if (1 == pAbility->AutoNegotiation)
    {
        /*Auto-Negotiation setting in reg 0.12*/
        phyEnMsk0 = phyEnMsk0 | (1 << 12);
    }

    if (1 == pAbility->AsyFC)
    {
        /*Asymetric flow control in reg 4.11*/
        phyEnMsk4 = phyEnMsk4 | (1 << 11);
    }
    if (1 == pAbility->FC)
    {
        /*Flow control in reg 4.10*/
        phyEnMsk4 = phyEnMsk4 | (1 << 10);
    }

    /*1000 BASE-T control register setting*/
    if ((retVal = rtl8367c_getAsicPHYReg(rtk_switch_port_L2P_get(port), PHY_1000_BASET_CONTROL_REG, &phyData)) != RT_ERR_OK)
        return retVal;

    phyData = (phyData & (~0x0200)) | phyEnMsk9;

    if ((retVal = rtl8367c_setAsicPHYReg(rtk_switch_port_L2P_get(port), PHY_1000_BASET_CONTROL_REG, phyData)) != RT_ERR_OK)
        return retVal;

    /*Auto-Negotiation control register setting*/
    if ((retVal = rtl8367c_getAsicPHYReg(rtk_switch_port_L2P_get(port), PHY_AN_ADVERTISEMENT_REG, &phyData)) != RT_ERR_OK)
        return retVal;

    phyData = (phyData & (~0x0DE0)) | phyEnMsk4;
    if ((retVal = rtl8367c_setAsicPHYReg(rtk_switch_port_L2P_get(port), PHY_AN_ADVERTISEMENT_REG, phyData)) != RT_ERR_OK)
        return retVal;

    /*Control register setting and restart auto*/
    if ((retVal = rtl8367c_getAsicPHYReg(rtk_switch_port_L2P_get(port), PHY_CONTROL_REG, &phyData)) != RT_ERR_OK)
        return retVal;

    phyData = (phyData & (~0x3140)) | phyEnMsk0;
    /*If have auto-negotiation capable, then restart auto negotiation*/
    if (1 == pAbility->AutoNegotiation)
    {
        phyData = phyData | (1 << 9);
    }

    if ((retVal = rtl8367c_setAsicPHYReg(rtk_switch_port_L2P_get(port), PHY_CONTROL_REG, phyData)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

int32_t rtl8367::_rtk_port_FiberModeAbility_get(rtk_port_t port, rtk_port_phy_ability_t *pAbility)
{
    int32_t retVal;
    uint32_t data, regData;

    /* Check Combo port or not */
    RTK_CHK_PORT_IS_COMBO(port);

    memset(pAbility, 0x00, sizeof(rtk_port_phy_ability_t));

    /* Flow Control */
    if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_FIBER_CFG_1, RTL8367C_SDS_FRC_REG4_OFFSET, 1)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_FIBER_CFG_1, RTL8367C_SDS_FRC_REG4_FIB100_OFFSET, 0)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_ADR, 0x0044)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_CMD, 0x0080)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_getAsicReg(RTL8367C_REG_SDS_INDACS_DATA, &regData)) != RT_ERR_OK)
        return retVal;

    if (regData & (0x0001 << 8))
        pAbility->AsyFC = 1;

    if (regData & (0x0001 << 7))
        pAbility->FC = 1;

    /* Speed ability */
    if ((retVal = rtl8367c_getAsicRegBit(RTL8367C_REG_FIBER_CFG_1, RTL8367C_SDS_FRC_MODE_OFFSET, &data)) != RT_ERR_OK)
        return retVal;

    if (data == 0)
    {
        pAbility->AutoNegotiation = 1;
        pAbility->Full_1000 = 1;
        pAbility->Full_100 = 1;
    }
    else
    {
        if ((retVal = rtl8367c_getAsicRegBits(RTL8367C_REG_FIBER_CFG_1, RTL8367C_SDS_MODE_MASK, &data)) != RT_ERR_OK)
            return retVal;

        if (data == 4)
        {
            pAbility->Full_1000 = 1;

            if ((retVal = rtl8367c_getAsicReg(RTL8367C_REG_FIB0_CFG00, &data)) != RT_ERR_OK)
                return retVal;

            if (data & 0x1000)
                pAbility->AutoNegotiation = 1;
            else
                pAbility->AutoNegotiation = 0;
        }
        else if (data == 5)
            pAbility->Full_100 = 1;
        else
            return RT_ERR_FAILED;
    }

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_port_phyAutoNegoAbility_get(rtk_port_t port, rtk_port_phy_ability_t *pAbility)
{
    int32_t retVal;
    uint32_t phyData0;
    uint32_t phyData4;
    uint32_t phyData9;
    rtk_port_media_t media_type;

    /* Check Port Valid */
    RTK_CHK_PORT_IS_UTP(port);

    if (NULL == pAbility)
        return RT_ERR_NULL_POINTER;

    if (rtk_switch_isComboPort(port) == RT_ERR_OK)
    {
        if ((retVal = _rtk_port_phyComboPortMedia_get(port, &media_type)) != RT_ERR_OK)
            return retVal;

        if (media_type == PORT_MEDIA_FIBER)
        {
            return _rtk_port_FiberModeAbility_get(port, pAbility);
        }
    }

    /*Control register setting and restart auto*/
    if ((retVal = rtl8367c_getAsicPHYReg(rtk_switch_port_L2P_get(port), PHY_CONTROL_REG, &phyData0)) != RT_ERR_OK)
        return retVal;

    /*Auto-Negotiation control register setting*/
    if ((retVal = rtl8367c_getAsicPHYReg(rtk_switch_port_L2P_get(port), PHY_AN_ADVERTISEMENT_REG, &phyData4)) != RT_ERR_OK)
        return retVal;

    /*1000 BASE-T control register setting*/
    if ((retVal = rtl8367c_getAsicPHYReg(rtk_switch_port_L2P_get(port), PHY_1000_BASET_CONTROL_REG, &phyData9)) != RT_ERR_OK)
        return retVal;

    if (phyData9 & (1 << 9))
        pAbility->Full_1000 = 1;
    else
        pAbility->Full_1000 = 0;

    if (phyData4 & (1 << 11))
        pAbility->AsyFC = 1;
    else
        pAbility->AsyFC = 0;

    if (phyData4 & (1 << 10))
        pAbility->FC = 1;
    else
        pAbility->FC = 0;

    if (phyData4 & (1 << 8))
        pAbility->Full_100 = 1;
    else
        pAbility->Full_100 = 0;

    if (phyData4 & (1 << 7))
        pAbility->Half_100 = 1;
    else
        pAbility->Half_100 = 0;

    if (phyData4 & (1 << 6))
        pAbility->Full_10 = 1;
    else
        pAbility->Full_10 = 0;

    if (phyData4 & (1 << 5))
        pAbility->Half_10 = 1;
    else
        pAbility->Half_10 = 0;

    if (phyData0 & (1 << 12))
        pAbility->AutoNegotiation = 1;
    else
        pAbility->AutoNegotiation = 0;

    return RT_ERR_OK;
}

// -------------------------- LED --------------------------

/* Function Name:
 *      rtk_switch_isCPUPort
 * Description:
 *      Check is logical port a CPU port
 * Input:
 *      logicalPort     - logical port ID
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Port ID is a CPU port
 *      RT_ERR_FAILED   - Port ID is not a CPU port
 *      RT_ERR_NOT_INIT - Not Initialize
 * Note:
 *
 */
int32_t rtl8367::rtk_switch_isCPUPort(uint32_t logicalPort)
{

    if (logicalPort >= RTK_SWITCH_PORT_NUM)
        return RT_ERR_FAILED;

    if (((0x01 << logicalPort) & halCtrl.valid_cpu_portmask) != 0)
        return RT_ERR_OK;
    else
        return RT_ERR_FAILED;
}

/*
@func int32_t | rtl8367c_setAsicLedGroupEnable | Turn on/off Led of all system ports
@parm uint32_t | group | LED group id.
@parm uint32_t | portmask | LED port mask.
@rvalue RT_ERR_OK | Success.
@rvalue RT_ERR_SMI | SMI access error.
@rvalue RT_ERR_PORT_ID | Invalid port number.
@rvalue RT_ERR_INPUT | Invalid input value.
@comm
    The API can turn on/off leds of dedicated port while indicated information configuration of LED group is set to force mode.
 */
int32_t rtl8367::rtl8367c_setAsicLedGroupEnable(uint32_t group, uint32_t portmask)
{
    int32_t retVal;
    uint32_t regAddr;
    uint32_t regDataMask;

    if (group >= RTL8367C_LEDGROUPNO)
        return RT_ERR_INPUT;

    regAddr = RTL8367C_REG_PARA_LED_IO_EN1 + group / 2;
    regDataMask = 0xFF << ((group % 2) * 8);
    retVal = rtl8367c_setAsicRegBits(regAddr, regDataMask, portmask & 0xff);
    if (retVal != RT_ERR_OK)
        return retVal;

    regAddr = RTL8367C_REG_PARA_LED_IO_EN3;
    regDataMask = 0x3 << (group * 2);
    retVal = rtl8367c_setAsicRegBits(regAddr, regDataMask, (portmask >> 8) & 0x7);
    if (retVal != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_led_enable_set(rtk_led_group_t group, rtk_portmask_t *pPortmask)
{
    int32_t retVal;
    uint32_t pmask;
    uint32_t port;

    if (group >= LED_GROUP_END)
        return RT_ERR_INPUT;

    RTK_CHK_PORTMASK_VALID(pPortmask);

    RTK_PORTMASK_SCAN((*pPortmask), port)
    {
        if (rtk_switch_isCPUPort(port) == RT_ERR_OK)
            return RT_ERR_PORT_MASK;
    }

    if ((retVal = rtk_switch_portmask_L2P_get(pPortmask, &pmask)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicLedGroupEnable(group, pmask)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

/*
@func int32_t | rtl8367c_setAsicLedOperationMode | Set LED operation mode
@parm uint32_t | mode | LED mode. 1:scan mode 1, 2:parallel mode, 3:mdx mode (serial mode)
@rvalue RT_ERR_OK | Success.
@rvalue RT_ERR_SMI | SMI access error.
@rvalue RT_ERR_INPUT | Invalid input value.
@comm
    The API can turn on/off led serial mode and set signal to active high/low.
 */
int32_t rtl8367::rtl8367c_setAsicLedOperationMode(uint32_t mode)
{
    int32_t retVal;

    /* Invalid input parameter */
    if (mode >= LEDOP_END)
        return RT_ERR_INPUT;

    switch (mode)
    {
    case LEDOP_PARALLEL:
        if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_LED_SYS_CONFIG, RTL8367C_LED_SELECT_OFFSET, 0)) != RT_ERR_OK)
            return retVal;
        /*Disable serial CLK mode*/
        if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_SCAN0_LED_IO_EN1, RTL8367C_LED_SERI_CLK_EN_OFFSET, 0)) != RT_ERR_OK)
            return retVal;
        /*Disable serial DATA mode*/
        if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_SCAN0_LED_IO_EN1, RTL8367C_LED_SERI_DATA_EN_OFFSET, 0)) != RT_ERR_OK)
            return retVal;
        break;
    case LEDOP_SERIAL:
        if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_LED_SYS_CONFIG, RTL8367C_LED_SELECT_OFFSET, 1)) != RT_ERR_OK)
            return retVal;
        /*Enable serial CLK mode*/
        if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_SCAN0_LED_IO_EN1, RTL8367C_LED_SERI_CLK_EN_OFFSET, 1)) != RT_ERR_OK)
            return retVal;
        /*Enable serial DATA mode*/
        if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_SCAN0_LED_IO_EN1, RTL8367C_LED_SERI_DATA_EN_OFFSET, 1)) != RT_ERR_OK)
            return retVal;
        break;
    default:
        return RT_ERR_INPUT;
        break;
    }

    return RT_ERR_OK;
}
int32_t rtl8367::rtk_led_operation_set(rtk_led_operation_t mode)
{
    int32_t retVal;
    uint32_t regData;

    if (mode >= LED_OP_END)
        return RT_ERR_INPUT;

    switch (mode)
    {
    case LED_OP_PARALLEL:
        regData = LEDOP_PARALLEL;
        break;
    case LED_OP_SERIAL:
        regData = LEDOP_SERIAL;
        break;
    default:
        return RT_ERR_CHIP_NOT_SUPPORTED;
        break;
    }

    if ((retVal = rtl8367c_setAsicLedOperationMode(regData)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsicLedBlinkRate
 * Description:
 *      Set led blinking rate at mode 0 to mode 3
 * Input:
 *      blinkRate   - Support 6 blink rates
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK           - Success
 *      RT_ERR_SMI          - SMI access error
 *      RT_ERR_OUT_OF_RANGE - input parameter out of range
 * Note:
 *      LED blink rate can be at 43ms, 84ms, 120ms, 170ms, 340ms and 670ms
 */
int32_t rtl8367::rtl8367c_setAsicLedBlinkRate(uint32_t blinkRate)
{
    if (blinkRate >= LEDBLINKRATE_END)
        return RT_ERR_OUT_OF_RANGE;

    return rtl8367c_setAsicRegBits(RTL8367C_REG_LED_MODE, RTL8367C_SEL_LEDRATE_MASK, blinkRate);
}

int32_t rtl8367::rtk_led_blinkRate_set(rtk_led_blink_rate_t blinkRate)
{
    int32_t retVal;

    if (blinkRate >= LED_BLINKRATE_END)
        return RT_ERR_FAILED;

    if ((retVal = rtl8367c_setAsicLedBlinkRate(blinkRate)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsicLedIndicateInfoConfig
 * Description:
 *      Set Leds indicated information mode
 * Input:
 *      ledno   - LED group number. There are 1 to 1 led mapping to each port in each led group
 *      config  - Support 16 types configuration
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK           - Success
 *      RT_ERR_SMI          - SMI access error
 *      RT_ERR_OUT_OF_RANGE - input parameter out of range
 * Note:
 *      The API can set LED indicated information configuration for each LED group with 1 to 1 led mapping to each port.
 *      Definition        LED Statuses            Description
 *      0000        LED_Off                LED pin Tri-State.
 *      0001        Dup/Col                Collision, Full duplex Indicator. Blinking every 43ms when collision happens. Low for full duplex, and high for half duplex mode.
 *      0010        Link/Act               Link, Activity Indicator. Low for link established. Link/Act Blinks every 43ms when the corresponding port is transmitting or receiving.
 *      0011        Spd1000                1000Mb/s Speed Indicator. Low for 1000Mb/s.
 *      0100        Spd100                 100Mb/s Speed Indicator. Low for 100Mb/s.
 *      0101        Spd10                  10Mb/s Speed Indicator. Low for 10Mb/s.
 *      0110        Spd1000/Act            1000Mb/s Speed/Activity Indicator. Low for 1000Mb/s. Blinks every 43ms when the corresponding port is transmitting or receiving.
 *      0111        Spd100/Act             100Mb/s Speed/Activity Indicator. Low for 100Mb/s. Blinks every 43ms when the corresponding port is transmitting or receiving.
 *      1000        Spd10/Act              10Mb/s Speed/Activity Indicator. Low for 10Mb/s. Blinks every 43ms when the corresponding port is transmitting or receiving.
 *      1001        Spd100 (10)/Act        10/100Mb/s Speed/Activity Indicator. Low for 10/100Mb/s. Blinks every 43ms when the corresponding port is transmitting or receiving.
 *      1010        Fiber                  Fiber link Indicator. Low for Fiber.
 *      1011        Fault                  Auto-negotiation     Fault Indicator. Low for Fault.
 *      1100        Link/Rx                Link, Activity Indicator. Low for link established. Link/Rx Blinks every 43ms when the corresponding port is transmitting.
 *      1101        Link/Tx                Link, Activity Indicator. Low for link established. Link/Tx Blinks every 43ms when the corresponding port is receiving.
 *      1110        Master                 Link on Master Indicator. Low for link Master established.
 *      1111        LED_Force              Force LED output, LED output value reference
 */
int32_t rtl8367::rtl8367c_setAsicLedIndicateInfoConfig(uint32_t ledno, uint32_t config)
{
    int32_t retVal;
    const uint16_t bits[RTL8367C_LEDGROUPNO] = {RTL8367C_LED0_CFG_MASK, RTL8367C_LED1_CFG_MASK, RTL8367C_LED2_CFG_MASK};

    if (ledno >= RTL8367C_LEDGROUPNO)
        return RT_ERR_OUT_OF_RANGE;

    if (config >= LEDCONF_END)
        return RT_ERR_OUT_OF_RANGE;

    retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_LED_CONFIGURATION, RTL8367C_LED_CONFIG_SEL_OFFSET, 0);
    if (retVal != RT_ERR_OK)
        return retVal;

    return rtl8367c_setAsicRegBits(RTL8367C_REG_LED_CONFIGURATION, bits[ledno], config);
}
int32_t rtl8367::rtk_led_groupConfig_set(rtk_led_group_t group, rtk_led_congig_t config)
{
    int32_t retVal;

    if (LED_GROUP_END <= group)
        return RT_ERR_FAILED;

    if (LED_CONFIG_END <= config)
        return RT_ERR_FAILED;

    if ((retVal = rtl8367c_setAsicLedIndicateInfoConfig(group, config)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

// ------------------------------- RMA -----------------------------------------

/* Function Name:
 *      rtl8367c_getAsicRma
 * Description:
 *      Get reserved multicast address for CPU trapping
 * Input:
 *      index     - reserved multicast LSB byte, 0x00~0x2F is available value
 *      rmacfg     - type of RMA for trapping frame type setting
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK         - Success
 *      RT_ERR_SMI      - SMI access error
 *      RT_ERR_RMA_ADDR - Invalid RMA address index
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicRma(uint32_t index, rtl8367c_rma_t *pRmacfg)
{
    int32_t retVal;
    uint32_t regData;

    if (index > RTL8367C_RMAMAX)
        return RT_ERR_RMA_ADDR;

    if ((index >= 0x4 && index <= 0x7) || (index >= 0x9 && index <= 0x0C) || (0x0F == index))
        index = 0x04;
    else if ((index >= 0x13 && index <= 0x17) || (0x19 == index) || (index >= 0x1B && index <= 0x1f))
        index = 0x13;
    else if (index >= 0x22 && index <= 0x2F)
        index = 0x22;

    retVal = rtl8367c_getAsicReg(RTL8367C_REG_RMA_CTRL00 + index, &regData);
    if (retVal != RT_ERR_OK)
        return retVal;

    pRmacfg->operation = ((regData >> 7) & 0x0003);
    pRmacfg->discard_storm_filter = ((regData >> 6) & 0x0001);
    pRmacfg->trap_priority = ((regData >> 3) & 0x0007);
    pRmacfg->keep_format = ((regData >> 2) & 0x0001);
    pRmacfg->vlan_leaky = ((regData >> 1) & 0x0001);
    pRmacfg->portiso_leaky = (regData & 0x0001);

    retVal = rtl8367c_getAsicRegBits(RTL8367C_REG_RMA_CTRL00, RTL8367C_TRAP_PRIORITY_MASK, &regData);
    if (retVal != RT_ERR_OK)
        return retVal;

    pRmacfg->trap_priority = regData;

    return RT_ERR_OK;
}
/* Function Name:
 *      rtl8367c_setAsicRma
 * Description:
 *      Set reserved multicast address for CPU trapping
 * Input:
 *      index     - reserved multicast LSB byte, 0x00~0x2F is available value
 *      pRmacfg     - type of RMA for trapping frame type setting
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK         - Success
 *      RT_ERR_SMI      - SMI access error
 *      RT_ERR_RMA_ADDR - Invalid RMA address index
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicRma(uint32_t index, rtl8367c_rma_t *pRmacfg)
{
    uint32_t regData = 0;
    int32_t retVal;

    if (index > RTL8367C_RMAMAX)
        return RT_ERR_RMA_ADDR;

    regData |= (pRmacfg->portiso_leaky & 0x0001);
    regData |= ((pRmacfg->vlan_leaky & 0x0001) << 1);
    regData |= ((pRmacfg->keep_format & 0x0001) << 2);
    regData |= ((pRmacfg->trap_priority & 0x0007) << 3);
    regData |= ((pRmacfg->discard_storm_filter & 0x0001) << 6);
    regData |= ((pRmacfg->operation & 0x0003) << 7);

    if ((index >= 0x4 && index <= 0x7) || (index >= 0x9 && index <= 0x0C) || (0x0F == index))
        index = 0x04;
    else if ((index >= 0x13 && index <= 0x17) || (0x19 == index) || (index >= 0x1B && index <= 0x1f))
        index = 0x13;
    else if (index >= 0x22 && index <= 0x2F)
        index = 0x22;

    retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_RMA_CTRL00, RTL8367C_TRAP_PRIORITY_MASK, pRmacfg->trap_priority);
    if (retVal != RT_ERR_OK)
        return retVal;

    return rtl8367c_setAsicReg(RTL8367C_REG_RMA_CTRL00 + index, regData);
}

/* Function Name:
 *      rtl8367c_getAsicRmaCdp
 * Description:
 *      Get CDP(Cisco Discovery Protocol) for CPU trapping
 * Input:
 *      None
 * Output:
 *      pRmacfg     - type of RMA for trapping frame type setting
 * Return:
 *      RT_ERR_OK         - Success
 *      RT_ERR_SMI      - SMI access error
 *      RT_ERR_RMA_ADDR - Invalid RMA address index
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicRmaCdp(rtl8367c_rma_t *pRmacfg)
{
    int32_t retVal;
    uint32_t regData;

    retVal = rtl8367c_getAsicReg(RTL8367C_REG_RMA_CTRL_CDP, &regData);
    if (retVal != RT_ERR_OK)
        return retVal;

    pRmacfg->operation = ((regData >> 7) & 0x0003);
    pRmacfg->discard_storm_filter = ((regData >> 6) & 0x0001);
    pRmacfg->trap_priority = ((regData >> 3) & 0x0007);
    pRmacfg->keep_format = ((regData >> 2) & 0x0001);
    pRmacfg->vlan_leaky = ((regData >> 1) & 0x0001);
    pRmacfg->portiso_leaky = (regData & 0x0001);

    retVal = rtl8367c_getAsicRegBits(RTL8367C_REG_RMA_CTRL00, RTL8367C_TRAP_PRIORITY_MASK, &regData);
    if (retVal != RT_ERR_OK)
        return retVal;

    pRmacfg->trap_priority = regData;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsicRmaCdp
 * Description:
 *      Set CDP(Cisco Discovery Protocol) for CPU trapping
 * Input:
 *      pRmacfg     - type of RMA for trapping frame type setting
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK         - Success
 *      RT_ERR_SMI      - SMI access error
 *      RT_ERR_RMA_ADDR - Invalid RMA address index
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicRmaCdp(rtl8367c_rma_t *pRmacfg)
{
    uint32_t regData = 0;
    int32_t retVal;

    if (pRmacfg->operation >= RMAOP_END)
        return RT_ERR_RMA_ACTION;

    if (pRmacfg->trap_priority > RTL8367C_PRIMAX)
        return RT_ERR_QOS_INT_PRIORITY;

    regData |= (pRmacfg->portiso_leaky & 0x0001);
    regData |= ((pRmacfg->vlan_leaky & 0x0001) << 1);
    regData |= ((pRmacfg->keep_format & 0x0001) << 2);
    regData |= ((pRmacfg->trap_priority & 0x0007) << 3);
    regData |= ((pRmacfg->discard_storm_filter & 0x0001) << 6);
    regData |= ((pRmacfg->operation & 0x0003) << 7);

    retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_RMA_CTRL00, RTL8367C_TRAP_PRIORITY_MASK, pRmacfg->trap_priority);
    if (retVal != RT_ERR_OK)
        return retVal;

    return rtl8367c_setAsicReg(RTL8367C_REG_RMA_CTRL_CDP, regData);
}

/* Function Name:
 *      rtl8367c_setAsicRmaCsstp
 * Description:
 *      Set CSSTP(Cisco Shared Spanning Tree Protocol) for CPU trapping
 * Input:
 *      pRmacfg     - type of RMA for trapping frame type setting
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK         - Success
 *      RT_ERR_SMI      - SMI access error
 *      RT_ERR_RMA_ADDR - Invalid RMA address index
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicRmaCsstp(rtl8367c_rma_t *pRmacfg)
{
    uint32_t regData = 0;
    int32_t retVal;

    if (pRmacfg->operation >= RMAOP_END)
        return RT_ERR_RMA_ACTION;

    if (pRmacfg->trap_priority > RTL8367C_PRIMAX)
        return RT_ERR_QOS_INT_PRIORITY;

    regData |= (pRmacfg->portiso_leaky & 0x0001);
    regData |= ((pRmacfg->vlan_leaky & 0x0001) << 1);
    regData |= ((pRmacfg->keep_format & 0x0001) << 2);
    regData |= ((pRmacfg->trap_priority & 0x0007) << 3);
    regData |= ((pRmacfg->discard_storm_filter & 0x0001) << 6);
    regData |= ((pRmacfg->operation & 0x0003) << 7);

    retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_RMA_CTRL00, RTL8367C_TRAP_PRIORITY_MASK, pRmacfg->trap_priority);
    if (retVal != RT_ERR_OK)
        return retVal;

    return rtl8367c_setAsicReg(RTL8367C_REG_RMA_CTRL_CSSTP, regData);
}
/* Function Name:
 *      rtl8367c_getAsicRmaCsstp
 * Description:
 *      Get CSSTP(Cisco Shared Spanning Tree Protocol) for CPU trapping
 * Input:
 *      None
 * Output:
 *      pRmacfg     - type of RMA for trapping frame type setting
 * Return:
 *      RT_ERR_OK         - Success
 *      RT_ERR_SMI      - SMI access error
 *      RT_ERR_RMA_ADDR - Invalid RMA address index
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicRmaCsstp(rtl8367c_rma_t *pRmacfg)
{
    int32_t retVal;
    uint32_t regData;

    retVal = rtl8367c_getAsicReg(RTL8367C_REG_RMA_CTRL_CSSTP, &regData);
    if (retVal != RT_ERR_OK)
        return retVal;

    pRmacfg->operation = ((regData >> 7) & 0x0003);
    pRmacfg->discard_storm_filter = ((regData >> 6) & 0x0001);
    pRmacfg->trap_priority = ((regData >> 3) & 0x0007);
    pRmacfg->keep_format = ((regData >> 2) & 0x0001);
    pRmacfg->vlan_leaky = ((regData >> 1) & 0x0001);
    pRmacfg->portiso_leaky = (regData & 0x0001);

    retVal = rtl8367c_getAsicRegBits(RTL8367C_REG_RMA_CTRL00, RTL8367C_TRAP_PRIORITY_MASK, &regData);
    if (retVal != RT_ERR_OK)
        return retVal;

    pRmacfg->trap_priority = regData;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsicRmaLldp
 * Description:
 *      Set LLDP for CPU trapping
 * Input:
 *      pRmacfg     - type of RMA for trapping frame type setting
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK         - Success
 *      RT_ERR_SMI      - SMI access error
 *      RT_ERR_RMA_ADDR - Invalid RMA address index
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicRmaLldp(uint32_t enabled, rtl8367c_rma_t *pRmacfg)
{
    uint32_t regData = 0;
    int32_t retVal;

    if (enabled > 1)
        return RT_ERR_ENABLE;

    if (pRmacfg->operation >= RMAOP_END)
        return RT_ERR_RMA_ACTION;

    if (pRmacfg->trap_priority > RTL8367C_PRIMAX)
        return RT_ERR_QOS_INT_PRIORITY;

    retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_RMA_LLDP_EN, RTL8367C_RMA_LLDP_EN_OFFSET, enabled);
    if (retVal != RT_ERR_OK)
        return retVal;

    regData |= (pRmacfg->portiso_leaky & 0x0001);
    regData |= ((pRmacfg->vlan_leaky & 0x0001) << 1);
    regData |= ((pRmacfg->keep_format & 0x0001) << 2);
    regData |= ((pRmacfg->trap_priority & 0x0007) << 3);
    regData |= ((pRmacfg->discard_storm_filter & 0x0001) << 6);
    regData |= ((pRmacfg->operation & 0x0003) << 7);

    retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_RMA_CTRL00, RTL8367C_TRAP_PRIORITY_MASK, pRmacfg->trap_priority);
    if (retVal != RT_ERR_OK)
        return retVal;

    return rtl8367c_setAsicReg(RTL8367C_REG_RMA_CTRL_LLDP, regData);
}
/* Function Name:
 *      rtl8367c_getAsicRmaLldp
 * Description:
 *      Get LLDP for CPU trapping
 * Input:
 *      None
 * Output:
 *      pRmacfg     - type of RMA for trapping frame type setting
 * Return:
 *      RT_ERR_OK         - Success
 *      RT_ERR_SMI      - SMI access error
 *      RT_ERR_RMA_ADDR - Invalid RMA address index
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicRmaLldp(uint32_t *pEnabled, rtl8367c_rma_t *pRmacfg)
{
    int32_t retVal;
    uint32_t regData;

    retVal = rtl8367c_getAsicRegBit(RTL8367C_REG_RMA_LLDP_EN, RTL8367C_RMA_LLDP_EN_OFFSET, pEnabled);
    if (retVal != RT_ERR_OK)
        return retVal;

    retVal = rtl8367c_getAsicReg(RTL8367C_REG_RMA_CTRL_LLDP, &regData);
    if (retVal != RT_ERR_OK)
        return retVal;

    pRmacfg->operation = ((regData >> 7) & 0x0003);
    pRmacfg->discard_storm_filter = ((regData >> 6) & 0x0001);
    pRmacfg->trap_priority = ((regData >> 3) & 0x0007);
    pRmacfg->keep_format = ((regData >> 2) & 0x0001);
    pRmacfg->vlan_leaky = ((regData >> 1) & 0x0001);
    pRmacfg->portiso_leaky = (regData & 0x0001);

    retVal = rtl8367c_getAsicRegBits(RTL8367C_REG_RMA_CTRL00, RTL8367C_TRAP_PRIORITY_MASK, &regData);
    if (retVal != RT_ERR_OK)
        return retVal;

    pRmacfg->trap_priority = regData;

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_trap_rmaAction_set(rtk_trap_type_t type, rtk_trap_rma_action_t rma_action)
{
    int32_t retVal;
    rtl8367c_rma_t rmacfg;
    uint32_t tmp;

    if (type >= TRAP_END)
        return RT_ERR_INPUT;

    if (rma_action >= RMA_ACTION_END)
        return RT_ERR_RMA_ACTION;

    if (type >= 0 && type <= TRAP_UNDEF_GARP_2F)
    {
        if ((retVal = rtl8367c_getAsicRma(type, &rmacfg)) != RT_ERR_OK)
            return retVal;

        rmacfg.operation = rma_action;

        if ((retVal = rtl8367c_setAsicRma(type, &rmacfg)) != RT_ERR_OK)
            return retVal;
    }
    else if (type == TRAP_CDP)
    {
        if ((retVal = rtl8367c_getAsicRmaCdp(&rmacfg)) != RT_ERR_OK)
            return retVal;

        rmacfg.operation = rma_action;

        if ((retVal = rtl8367c_setAsicRmaCdp(&rmacfg)) != RT_ERR_OK)
            return retVal;
    }
    else if (type == TRAP_CSSTP)
    {
        if ((retVal = rtl8367c_getAsicRmaCsstp(&rmacfg)) != RT_ERR_OK)
            return retVal;

        rmacfg.operation = rma_action;

        if ((retVal = rtl8367c_setAsicRmaCsstp(&rmacfg)) != RT_ERR_OK)
            return retVal;
    }
    else if (type == TRAP_LLDP)
    {
        if ((retVal = rtl8367c_getAsicRmaLldp(&tmp, &rmacfg)) != RT_ERR_OK)
            return retVal;

        rmacfg.operation = rma_action;

        if ((retVal = rtl8367c_setAsicRmaLldp(tmp, &rmacfg)) != RT_ERR_OK)
            return retVal;
    }
    else
        return RT_ERR_INPUT;

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_trap_rmaAction_get(rtk_trap_type_t type, rtk_trap_rma_action_t *pRma_action)
{
    int32_t retVal;
    rtl8367c_rma_t rmacfg;
    uint32_t tmp;

    if (type >= TRAP_END)
        return RT_ERR_INPUT;

    if (NULL == pRma_action)
        return RT_ERR_NULL_POINTER;

    if (type >= 0 && type <= TRAP_UNDEF_GARP_2F)
    {
        if ((retVal = rtl8367c_getAsicRma(type, &rmacfg)) != RT_ERR_OK)
            return retVal;

        *pRma_action = (rtk_trap_rma_action_t)rmacfg.operation;
    }
    else if (type == TRAP_CDP)
    {
        if ((retVal = rtl8367c_getAsicRmaCdp(&rmacfg)) != RT_ERR_OK)
            return retVal;

        *pRma_action = (rtk_trap_rma_action_t)rmacfg.operation;
    }
    else if (type == TRAP_CSSTP)
    {
        if ((retVal = rtl8367c_getAsicRmaCsstp(&rmacfg)) != RT_ERR_OK)
            return retVal;

        *pRma_action = (rtk_trap_rma_action_t)rmacfg.operation;
    }
    else if (type == TRAP_LLDP)
    {
        if ((retVal = rtl8367c_getAsicRmaLldp(&tmp, &rmacfg)) != RT_ERR_OK)
            return retVal;

        *pRma_action = (rtk_trap_rma_action_t)rmacfg.operation;
    }
    else
        return RT_ERR_INPUT;

    return RT_ERR_OK;
}

// ----------------------------- STORM FILTERING CONTROL -----------------------------

/* Function Name:
 *      rtl8367c_setAsicStormFilterUnknownUnicastEnable
 * Description:
 *      Set per-port unknown unicast storm filter enable/disable
 * Input:
 *      port    - Physical port number (0~7)
 *      enabled - 1: enabled, 0: disabled
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_SMI      - SMI access error
 *      RT_ERR_PORT_ID  - Invalid port number
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicStormFilterUnknownUnicastEnable(uint32_t port, uint32_t enabled)
{
    if (port >= RTL8367C_PORTNO)
        return RT_ERR_PORT_ID;

    return rtl8367c_setAsicRegBit(RTL8367C_STORM_UNKNOWN_UCAST_REG, port, enabled);
}

/* Function Name:
 *      rtl8367c_setAsicStormFilterUnknownMulticastEnable
 * Description:
 *      Set per-port unknown multicast storm filter enable/disable
 * Input:
 *      port    - Physical port number (0~7)
 *      enabled - 1: enabled, 0: disabled
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_SMI      - SMI access error
 *      RT_ERR_PORT_ID  - Invalid port number
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicStormFilterUnknownMulticastEnable(uint32_t port, uint32_t enabled)
{
    if (port >= RTL8367C_PORTNO)
        return RT_ERR_PORT_ID;

    return rtl8367c_setAsicRegBit(RTL8367C_STORM_UNKNOWN_MCAST_REG, port, enabled);
}

/* Function Name:
 *      rtl8367c_setAsicStormFilterMulticastEnable
 * Description:
 *      Set per-port multicast storm filter enable/disable
 * Input:
 *      port    - Physical port number (0~7)
 *      enabled - 1: enabled, 0: disabled
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_SMI      - SMI access error
 *      RT_ERR_PORT_ID  - Invalid port number
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicStormFilterMulticastEnable(uint32_t port, uint32_t enabled)
{
    if (port >= RTL8367C_PORTNO)
        return RT_ERR_PORT_ID;

    return rtl8367c_setAsicRegBit(RTL8367C_STORM_MCAST_REG, port, enabled);
}
/* Function Name:
 *      rtl8367c_setAsicStormFilterBroadcastEnable
 * Description:
 *      Set per-port broadcast storm filter enable/disable
 * Input:
 *      port    - Physical port number (0~7)
 *      enabled - 1: enabled, 0: disabled
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_SMI      - SMI access error
 *      RT_ERR_PORT_ID  - Invalid port number
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicStormFilterBroadcastEnable(uint32_t port, uint32_t enabled)
{
    if (port >= RTL8367C_PORTNO)
        return RT_ERR_PORT_ID;

    return rtl8367c_setAsicRegBit(RTL8367C_STORM_BCAST_REG, port, enabled);
}
int32_t rtl8367::rtk_rate_stormControlPortEnable_set(rtk_port_t port, rtk_rate_storm_group_t stormType, rtk_enable_t enable)
{
    int32_t retVal;

    /* Check Port Valid */
    RTK_CHK_PORT_VALID(port);

    if (stormType >= STORM_GROUP_END)
        return RT_ERR_SFC_UNKNOWN_GROUP;

    if (enable >= RTK_ENABLE_END)
        return RT_ERR_ENABLE;

    switch (stormType)
    {
    case STORM_GROUP_UNKNOWN_UNICAST:
        if ((retVal = rtl8367c_setAsicStormFilterUnknownUnicastEnable(rtk_switch_port_L2P_get(port), enable)) != RT_ERR_OK)
            return retVal;
        break;
    case STORM_GROUP_UNKNOWN_MULTICAST:
        if ((retVal = rtl8367c_setAsicStormFilterUnknownMulticastEnable(rtk_switch_port_L2P_get(port), enable)) != RT_ERR_OK)
            return retVal;
        break;
    case STORM_GROUP_MULTICAST:
        if ((retVal = rtl8367c_setAsicStormFilterMulticastEnable(rtk_switch_port_L2P_get(port), enable)) != RT_ERR_OK)
            return retVal;
        break;
    case STORM_GROUP_BROADCAST:
        if ((retVal = rtl8367c_setAsicStormFilterBroadcastEnable(rtk_switch_port_L2P_get(port), enable)) != RT_ERR_OK)
            return retVal;
        break;
    default:
        break;
    }

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_getAsicStormFilterUnknownUnicastEnable
 * Description:
 *      get per-port unknown unicast storm filter enable/disable
 * Input:
 *      port    - Physical port number (0~7)
 *      pEnabled - 1: enabled, 0: disabled
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_SMI      - SMI access error
 *      RT_ERR_PORT_ID  - Invalid port number
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicStormFilterUnknownUnicastEnable(uint32_t port, uint32_t *pEnabled)
{
    if (port >= RTL8367C_PORTNO)
        return RT_ERR_PORT_ID;

    return rtl8367c_getAsicRegBit(RTL8367C_STORM_UNKNOWN_UCAST_REG, port, pEnabled);
}
/* Function Name:
 *      rtl8367c_getAsicStormFilterUnknownMulticastEnable
 * Description:
 *      Get per-port unknown multicast storm filter enable/disable
 * Input:
 *      port    - Physical port number (0~7)
 *      pEnabled - 1: enabled, 0: disabled
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_SMI      - SMI access error
 *      RT_ERR_PORT_ID  - Invalid port number
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicStormFilterUnknownMulticastEnable(uint32_t port, uint32_t *pEnabled)
{
    if (port >= RTL8367C_PORTNO)
        return RT_ERR_PORT_ID;

    return rtl8367c_getAsicRegBit(RTL8367C_STORM_UNKNOWN_MCAST_REG, port, pEnabled);
}
/* Function Name:
 *      rtl8367c_getAsicStormFilterMulticastEnable
 * Description:
 *      Get per-port multicast storm filter enable/disable
 * Input:
 *      port    - Physical port number (0~7)
 *      pEnabled - 1: enabled, 0: disabled
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_SMI      - SMI access error
 *      RT_ERR_PORT_ID  - Invalid port number
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicStormFilterMulticastEnable(uint32_t port, uint32_t *pEnabled)
{
    if (port >= RTL8367C_PORTNO)
        return RT_ERR_PORT_ID;

    return rtl8367c_getAsicRegBit(RTL8367C_STORM_MCAST_REG, port, pEnabled);
}
/* Function Name:
 *      rtl8367c_getAsicStormFilterBroadcastEnable
 * Description:
 *      Get per-port broadcast storm filter enable/disable
 * Input:
 *      port    - Physical port number (0~7)
 *      pEnabled - 1: enabled, 0: disabled
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_SMI      - SMI access error
 *      RT_ERR_PORT_ID  - Invalid port number
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicStormFilterBroadcastEnable(uint32_t port, uint32_t *pEnabled)
{
    if (port >= RTL8367C_PORTNO)
        return RT_ERR_PORT_ID;

    return rtl8367c_getAsicRegBit(RTL8367C_STORM_BCAST_REG, port, pEnabled);
}
int32_t rtl8367::rtk_rate_stormControlPortEnable_get(rtk_port_t port, rtk_rate_storm_group_t stormType, rtk_enable_t *pEnable)
{
    int32_t retVal;

    /* Check Port Valid */
    RTK_CHK_PORT_VALID(port);

    if (stormType >= STORM_GROUP_END)
        return RT_ERR_SFC_UNKNOWN_GROUP;

    if (NULL == pEnable)
        return RT_ERR_ENABLE;

    switch (stormType)
    {
    case STORM_GROUP_UNKNOWN_UNICAST:
        if ((retVal = rtl8367c_getAsicStormFilterUnknownUnicastEnable(rtk_switch_port_L2P_get(port), (uint32_t *)pEnable)) != RT_ERR_OK)
            return retVal;
        break;
    case STORM_GROUP_UNKNOWN_MULTICAST:
        if ((retVal = rtl8367c_getAsicStormFilterUnknownMulticastEnable(rtk_switch_port_L2P_get(port), (uint32_t *)pEnable)) != RT_ERR_OK)
            return retVal;
        break;
    case STORM_GROUP_MULTICAST:
        if ((retVal = rtl8367c_getAsicStormFilterMulticastEnable(rtk_switch_port_L2P_get(port), (uint32_t *)pEnable)) != RT_ERR_OK)
            return retVal;
        break;
    case STORM_GROUP_BROADCAST:
        if ((retVal = rtl8367c_getAsicStormFilterBroadcastEnable(rtk_switch_port_L2P_get(port), (uint32_t *)pEnable)) != RT_ERR_OK)
            return retVal;
        break;
    default:
        break;
    }

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsicStormFilterUnknownUnicastMeter
 * Description:
 *      Set per-port unknown unicast storm filter meter
 * Input:
 *      port    - Physical port number (0~7)
 *      meter   - meter index (0~31)
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK               - Success
 *      RT_ERR_SMI              - SMI access error
 *      RT_ERR_PORT_ID          - Invalid port number
 *      RT_ERR_FILTER_METER_ID  - Invalid meter index
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicStormFilterUnknownUnicastMeter(uint32_t port, uint32_t meter)
{
    if (port >= RTL8367C_PORTNO)
        return RT_ERR_PORT_ID;

    if (meter > RTL8367C_METERMAX)
        return RT_ERR_FILTER_METER_ID;

    return rtl8367c_setAsicRegBits(RTL8367C_STORM_UNDA_METER_CTRL_REG(port), RTL8367C_STORM_UNDA_METER_CTRL_MASK(port), meter);
}
/* Function Name:
 *      rtl8367c_setAsicStormFilterUnknownMulticastMeter
 * Description:
 *      Set per-port unknown multicast storm filter meter
 * Input:
 *      port    - Physical port number (0~7)
 *      meter   - meter index (0~31)
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK               - Success
 *      RT_ERR_SMI              - SMI access error
 *      RT_ERR_PORT_ID          - Invalid port number
 *      RT_ERR_FILTER_METER_ID  - Invalid meter index
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicStormFilterUnknownMulticastMeter(uint32_t port, uint32_t meter)
{
    int32_t retVal;

    if (port >= RTL8367C_PORTNO)
        return RT_ERR_PORT_ID;

    if (meter > RTL8367C_METERMAX)
        return RT_ERR_FILTER_METER_ID;

    if (port < 8)
    {
        retVal = rtl8367c_setAsicRegBits(RTL8367C_STORM_UNMC_METER_CTRL_REG(port), RTL8367C_STORM_UNMC_METER_CTRL_MASK(port), meter);
        if (retVal != RT_ERR_OK)
            return retVal;
    }
    else
    {
        retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_STORM_UNMC_METER_CTRL4 + ((port - 8) >> 1), RTL8367C_STORM_UNMC_METER_CTRL_MASK(port), meter);
        if (retVal != RT_ERR_OK)
            return retVal;
    }

    return RT_ERR_OK;
}
/* Function Name:
 *      rtl8367c_setAsicStormFilterMulticastMeter
 * Description:
 *      Set per-port multicast storm filter meter
 * Input:
 *      port    - Physical port number (0~7)
 *      meter   - meter index (0~31)
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK               - Success
 *      RT_ERR_SMI              - SMI access error
 *      RT_ERR_PORT_ID          - Invalid port number
 *      RT_ERR_FILTER_METER_ID  - Invalid meter index
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicStormFilterMulticastMeter(uint32_t port, uint32_t meter)
{
    if (port >= RTL8367C_PORTNO)
        return RT_ERR_PORT_ID;

    if (meter > RTL8367C_METERMAX)
        return RT_ERR_FILTER_METER_ID;

    return rtl8367c_setAsicRegBits(RTL8367C_STORM_MCAST_METER_CTRL_REG(port), RTL8367C_STORM_MCAST_METER_CTRL_MASK(port), meter);
}
/* Function Name:
 *      rtl8367c_setAsicStormFilterBroadcastMeter
 * Description:
 *      Set per-port broadcast storm filter meter
 * Input:
 *      port    - Physical port number (0~7)
 *      meter   - meter index (0~31)
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK               - Success
 *      RT_ERR_SMI              - SMI access error
 *      RT_ERR_PORT_ID          - Invalid port number
 *      RT_ERR_FILTER_METER_ID  - Invalid meter index
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicStormFilterBroadcastMeter(uint32_t port, uint32_t meter)
{
    if (port >= RTL8367C_PORTNO)
        return RT_ERR_PORT_ID;

    if (meter > RTL8367C_METERMAX)
        return RT_ERR_FILTER_METER_ID;

    return rtl8367c_setAsicRegBits(RTL8367C_STORM_BCAST_METER_CTRL_REG(port), RTL8367C_STORM_BCAST_METER_CTRL_MASK(port), meter);
}
int32_t rtl8367::rtk_rate_stormControlMeterIdx_set(rtk_port_t port, rtk_rate_storm_group_t stormType, uint32_t index)
{
    int32_t retVal;

    /* Check Port Valid */
    RTK_CHK_PORT_VALID(port);

    if (stormType >= STORM_GROUP_END)
        return RT_ERR_SFC_UNKNOWN_GROUP;

    if (index > halCtrl.max_meter_id)
        return RT_ERR_FILTER_METER_ID;

    switch (stormType)
    {
    case STORM_GROUP_UNKNOWN_UNICAST:
        if ((retVal = rtl8367c_setAsicStormFilterUnknownUnicastMeter(rtk_switch_port_L2P_get(port), index)) != RT_ERR_OK)
            return retVal;
        break;
    case STORM_GROUP_UNKNOWN_MULTICAST:
        if ((retVal = rtl8367c_setAsicStormFilterUnknownMulticastMeter(rtk_switch_port_L2P_get(port), index)) != RT_ERR_OK)
            return retVal;
        break;
    case STORM_GROUP_MULTICAST:
        if ((retVal = rtl8367c_setAsicStormFilterMulticastMeter(rtk_switch_port_L2P_get(port), index)) != RT_ERR_OK)
            return retVal;
        break;
    case STORM_GROUP_BROADCAST:
        if ((retVal = rtl8367c_setAsicStormFilterBroadcastMeter(rtk_switch_port_L2P_get(port), index)) != RT_ERR_OK)
            return retVal;
        break;
    default:
        break;
    }

    return RT_ERR_OK;
}