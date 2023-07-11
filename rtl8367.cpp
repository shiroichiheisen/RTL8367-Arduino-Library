#include "rtl8367.h"
#include "i2cPart.h"

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
int32_t rtl8367::getPortStatus(uint8_t port, uint8_t &pLinkStatus, uint8_t &pSpeed, uint8_t &pDuplex)
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
    vlan4K.mbr = RTK_PHY_PORTMASK_ALL;
    vlan4K.untag = RTK_PHY_PORTMASK_ALL;
    vlan4K.fid_msti = 0;
    if ((retVal = rtl8367c_setAsicVlan4kEntry(&vlan4K)) != RT_ERR_OK)
        return retVal;

    /* Also set the default VLAN to 32 member configuration index 0 */
    memset(&vlanMC, 0, sizeof(rtl8367c_vlanconfiguser));
    vlanMC.evid = 1;
    vlanMC.mbr = RTK_PHY_PORTMASK_ALL;
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
    if (pVlanCfg->meteridx > RTK_MAX_METER_ID)
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

int32_t rtl8367::_rtk_vlan_get(uint32_t vid, rtk_vlan_cfg_t *pVlanCfg)
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

#define RTK_CHK_PORT_VALID(__port__)                            \
    do                                                          \
    {                                                           \
        if (rtk_switch_logicalPortCheck(__port__) != RT_ERR_OK) \
        {                                                       \
            return RT_ERR_PORT_ID;                              \
        }                                                       \
    } while (0)

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