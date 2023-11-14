#include "rtl8367.h"
#include "rtl8367c_i2cPart.h"

rtl8367::rtl8367(uint16_t usTransmissionDelay)
{
    this->usTransmissionDelay = usTransmissionDelay;
}

int32_t rtl8367::reset()
{
    int32_t retVal;
    retVal = rtl8367c_setAsicReg(RTL8367C_REG_CHIP_RESET, 1);
    if (retVal != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

int32_t rtl8367::resetWithDelay()
{
    int32_t retVal;
    retVal = rtl8367c_setAsicReg(RTL8367C_REG_CHIP_RESET, 1);
    if (retVal != RT_ERR_OK)
        return retVal;

    delay(2000);

    return RT_ERR_OK;
}

void rtl8367::setCommunicationPins(uint8_t sckPin, uint8_t sdaPin)
{
    this->sdaPin = sdaPin;
    this->sckPin = sckPin;

    pinMode(sdaPin, OUTPUT);
    pinMode(sckPin, OUTPUT);
}

void rtl8367::setCommunicationDelay(uint16_t usTransmissionDelay)
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

/* Function Name:
 *      rtl8367c_getAsicStormFilterUnknownUnicastMeter
 * Description:
 *      Get per-port unknown unicast storm filter meter
 * Input:
 *      port    - Physical port number (0~7)
 *      pMeter  - meter index (0~31)
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_SMI      - SMI access error
 *      RT_ERR_PORT_ID  - Invalid port number
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicStormFilterUnknownUnicastMeter(uint32_t port, uint32_t *pMeter)
{
    if (port >= RTL8367C_PORTNO)
        return RT_ERR_PORT_ID;

    return rtl8367c_getAsicRegBits(RTL8367C_STORM_UNDA_METER_CTRL_REG(port), RTL8367C_STORM_UNDA_METER_CTRL_MASK(port), pMeter);
}
/* Function Name:
 *      rtl8367c_getAsicStormFilterUnknownMulticastMeter
 * Description:
 *      Get per-port unknown multicast storm filter meter
 * Input:
 *      port    - Physical port number (0~7)
 *      pMeter  - meter index (0~31)
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_SMI      - SMI access error
 *      RT_ERR_PORT_ID  - Invalid port number
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicStormFilterUnknownMulticastMeter(uint32_t port, uint32_t *pMeter)
{
    int32_t retVal;

    if (port >= RTL8367C_PORTNO)
        return RT_ERR_PORT_ID;

    if (port < 8)
    {
        retVal = rtl8367c_getAsicRegBits(RTL8367C_STORM_UNMC_METER_CTRL_REG(port), RTL8367C_STORM_UNMC_METER_CTRL_MASK(port), pMeter);
        if (retVal != RT_ERR_OK)
            return retVal;
    }
    else
    {
        retVal = rtl8367c_getAsicRegBits(RTL8367C_REG_STORM_UNMC_METER_CTRL4 + ((port - 8) >> 1), RTL8367C_STORM_UNMC_METER_CTRL_MASK(port), pMeter);
        if (retVal != RT_ERR_OK)
            return retVal;
    }

    return RT_ERR_OK;
}
/* Function Name:
 *      rtl8367c_getAsicStormFilterMulticastMeter
 * Description:
 *      Get per-port multicast storm filter meter
 * Input:
 *      port    - Physical port number (0~7)
 *      pMeter  - meter index (0~31)
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_SMI      - SMI access error
 *      RT_ERR_PORT_ID  - Invalid port number
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicStormFilterMulticastMeter(uint32_t port, uint32_t *pMeter)
{
    if (port >= RTL8367C_PORTNO)
        return RT_ERR_PORT_ID;

    return rtl8367c_getAsicRegBits(RTL8367C_STORM_MCAST_METER_CTRL_REG(port), RTL8367C_STORM_MCAST_METER_CTRL_MASK(port), pMeter);
}
/* Function Name:
 *      rtl8367c_getAsicStormFilterBroadcastMeter
 * Description:
 *      Get per-port broadcast storm filter meter
 * Input:
 *      port    - Physical port number (0~7)
 *      pMeter  - meter index (0~31)
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_SMI      - SMI access error
 *      RT_ERR_PORT_ID  - Invalid port number
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicStormFilterBroadcastMeter(uint32_t port, uint32_t *pMeter)
{
    if (port >= RTL8367C_PORTNO)
        return RT_ERR_PORT_ID;

    return rtl8367c_getAsicRegBits(RTL8367C_STORM_BCAST_METER_CTRL_REG(port), RTL8367C_STORM_BCAST_METER_CTRL_MASK(port), pMeter);
}
int32_t rtl8367::rtk_rate_stormControlMeterIdx_get(rtk_port_t port, rtk_rate_storm_group_t stormType, uint32_t *pIndex)
{
    int32_t retVal;
    /* Check Port Valid */
    RTK_CHK_PORT_VALID(port);

    if (stormType >= STORM_GROUP_END)
        return RT_ERR_SFC_UNKNOWN_GROUP;

    if (NULL == pIndex)
        return RT_ERR_NULL_POINTER;

    switch (stormType)
    {
    case STORM_GROUP_UNKNOWN_UNICAST:
        if ((retVal = rtl8367c_getAsicStormFilterUnknownUnicastMeter(rtk_switch_port_L2P_get(port), pIndex)) != RT_ERR_OK)
            return retVal;
        break;
    case STORM_GROUP_UNKNOWN_MULTICAST:
        if ((retVal = rtl8367c_getAsicStormFilterUnknownMulticastMeter(rtk_switch_port_L2P_get(port), pIndex)) != RT_ERR_OK)
            return retVal;
        break;
    case STORM_GROUP_MULTICAST:
        if ((retVal = rtl8367c_getAsicStormFilterMulticastMeter(rtk_switch_port_L2P_get(port), pIndex)) != RT_ERR_OK)
            return retVal;
        break;
    case STORM_GROUP_BROADCAST:
        if ((retVal = rtl8367c_getAsicStormFilterBroadcastMeter(rtk_switch_port_L2P_get(port), pIndex)) != RT_ERR_OK)
            return retVal;
        break;
    default:
        break;
    }

    return RT_ERR_OK;
}

// --------------------------- PORT MIRROR ------------------------------------

/* Function Name:
 *      rtk_switch_maxLogicalPort_get
 * Description:
 *      Get Max logical port ID
 * Input:
 *      None
 * Output:
 *      None
 * Return:
 *      Max logical port
 * Note:
 *      This API can get max logical port
 */
rtk_port_t rtl8367::rtk_switch_maxLogicalPort_get()
{
    rtk_port_t port, maxLogicalPort = UTP_PORT0;

    for (uint8_t port1 = 0; port1 < RTK_SWITCH_PORT_NUM; port1++)
    {
        if ((halCtrl.log_port_type[port] == UTP_PORT) || (halCtrl.log_port_type[port] == EXT_PORT))
            maxLogicalPort = port;
    }

    return maxLogicalPort;
}
/* Function Name:
 *      rtl8367c_setAsicPortMirror
 * Description:
 *      Set port mirror function
 * Input:
 *      source  - Source port
 *      monitor - Monitor (destination) port
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_SMI      - SMI access error
 *      RT_ERR_PORT_ID  - Invalid port number
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicPortMirror(uint32_t source, uint32_t monitor)
{
    int32_t retVal;

    if ((source > RTL8367C_PORTIDMAX) || (monitor > RTL8367C_PORTIDMAX))
        return RT_ERR_PORT_ID;

    retVal = rtl8367c_setAsicRegBits(RTL8367C_MIRROR_CTRL_REG, RTL8367C_MIRROR_SOURCE_PORT_MASK, source);
    if (retVal != RT_ERR_OK)
        return retVal;

    return rtl8367c_setAsicRegBits(RTL8367C_MIRROR_CTRL_REG, RTL8367C_MIRROR_MONITOR_PORT_MASK, monitor);
}
/* Function Name:
 *      rtl8367c_setAsicPortMirrorMask
 * Description:
 *      Set mirror source port mask
 * Input:
 *      SourcePortmask  - Source Portmask
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_SMI      - SMI access error
 *      RT_ERR_PORT_MASK- Port Mask Error
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicPortMirrorMask(uint32_t SourcePortmask)
{
    if (SourcePortmask > RTL8367C_PORTMASK)
        return RT_ERR_PORT_MASK;

    return rtl8367c_setAsicRegBits(RTL8367C_REG_MIRROR_SRC_PMSK, RTL8367C_MIRROR_SRC_PMSK_MASK, SourcePortmask);
}
/* Function Name:
 *      rtl8367c_setAsicPortMirrorRxFunction
 * Description:
 *      Set the mirror function on RX of the mirrored
 * Input:
 *      enabled     - 1: enabled, 0: disabled
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_SMI      - SMI access error
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicPortMirrorRxFunction(uint32_t enabled)
{
    return rtl8367c_setAsicRegBit(RTL8367C_MIRROR_CTRL_REG, RTL8367C_MIRROR_RX_OFFSET, enabled);
}
/* Function Name:
 *      rtl8367c_setAsicPortMirrorTxFunction
 * Description:
 *      Set the mirror function on TX of the mirrored
 * Input:
 *      enabled     - 1: enabled, 0: disabled
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_SMI      - SMI access error
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicPortMirrorTxFunction(uint32_t enabled)
{
    return rtl8367c_setAsicRegBit(RTL8367C_MIRROR_CTRL_REG, RTL8367C_MIRROR_TX_OFFSET, enabled);
}
int32_t rtl8367::rtk_mirror_portBased_set(rtk_port_t mirroring_port, rtk_portmask_t *pMirrored_rx_portmask, rtk_portmask_t *pMirrored_tx_portmask)
{
    int32_t retVal;
    rtk_enable_t mirRx, mirTx;
    uint32_t i, pmask;
    uint32_t source_port;

    /* Check port valid */
    RTK_CHK_PORT_VALID(mirroring_port);

    if (NULL == pMirrored_rx_portmask)
        return RT_ERR_NULL_POINTER;

    if (NULL == pMirrored_tx_portmask)
        return RT_ERR_NULL_POINTER;

    RTK_CHK_PORTMASK_VALID(pMirrored_rx_portmask);

    RTK_CHK_PORTMASK_VALID(pMirrored_tx_portmask);

    /*Mirror Sorce Port Mask Check*/
    if (pMirrored_tx_portmask->bits[0] != pMirrored_rx_portmask->bits[0] && pMirrored_tx_portmask->bits[0] != 0 && pMirrored_rx_portmask->bits[0] != 0)
        return RT_ERR_PORT_MASK;

    /*mirror port != source port*/
    if (RTK_PORTMASK_IS_PORT_SET((*pMirrored_tx_portmask), mirroring_port) || RTK_PORTMASK_IS_PORT_SET((*pMirrored_rx_portmask), mirroring_port))
        return RT_ERR_PORT_MASK;

    source_port = rtk_switch_maxLogicalPort_get();

    RTK_SCAN_ALL_LOG_PORT(i)
    {
        if (pMirrored_tx_portmask->bits[0] & (1 << i))
        {
            source_port = i;
            break;
        }

        if (pMirrored_rx_portmask->bits[0] & (1 << i))
        {
            source_port = i;
            break;
        }
    }

    if ((retVal = rtl8367c_setAsicPortMirror(rtk_switch_port_L2P_get(source_port), rtk_switch_port_L2P_get(mirroring_port))) != RT_ERR_OK)
        return retVal;
    if (pMirrored_rx_portmask->bits[0] != 0)
    {
        if ((retVal = rtk_switch_portmask_L2P_get(pMirrored_rx_portmask, &pmask)) != RT_ERR_OK)
            return retVal;
        if ((retVal = rtl8367c_setAsicPortMirrorMask(pmask)) != RT_ERR_OK)
            return retVal;
    }
    else
    {
        if ((retVal = rtk_switch_portmask_L2P_get(pMirrored_tx_portmask, &pmask)) != RT_ERR_OK)
            return retVal;
        if ((retVal = rtl8367c_setAsicPortMirrorMask(pmask)) != RT_ERR_OK)
            return retVal;
    }

    if (pMirrored_rx_portmask->bits[0])
        mirRx = ENABLED;
    else
        mirRx = DISABLED_RTK;

    if ((retVal = rtl8367c_setAsicPortMirrorRxFunction(mirRx)) != RT_ERR_OK)
        return retVal;

    if (pMirrored_tx_portmask->bits[0])
        mirTx = ENABLED;
    else
        mirTx = DISABLED_RTK;

    if ((retVal = rtl8367c_setAsicPortMirrorTxFunction(mirTx)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

// ---------------------------- FORCE EXTERNAL INTERFACE ----------------------------

int32_t rtl8367::rtk_switch_isExtPort(rtk_port_t logicalPort)
{

    if (logicalPort >= RTK_SWITCH_PORT_NUM)
        return RT_ERR_FAILED;

    if (halCtrl.log_port_type[logicalPort] == EXT_PORT)
        return RT_ERR_OK;
    else
        return RT_ERR_FAILED;
}

/* Function Name:
 *      rtk_switch_isHsgPort
 * Description:
 *      Check is logical port a HSG port
 * Input:
 *      logicalPort     - logical port ID
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Port ID is a HSG port
 *      RT_ERR_FAILED   - Port ID is not a HSG port
 *      RT_ERR_NOT_INIT - Not Initialize
 * Note:
 *
 */
int32_t rtl8367::rtk_switch_isHsgPort(rtk_port_t logicalPort)
{

    if (logicalPort >= RTK_SWITCH_PORT_NUM)
        return RT_ERR_FAILED;

    if (logicalPort == halCtrl.hsg_logical_port)
        return RT_ERR_OK;
    else
        return RT_ERR_FAILED;
}

/* Function Name:
 *      rtl8367c_getAiscSdsReg
 * Description:
 *      Get Serdes registers
 * Input:
 *      sdsId   - sdsid (0~1)
 *      sdsReg - reg address (0~31)
 *      sdsPage - Writing data
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK               - Success

 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicSdsReg(uint32_t sdsId, uint32_t sdsReg, uint32_t sdsPage, uint32_t *value)
{
    uint32_t retVal, busy;

    if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_ADR, (sdsPage << 5) | sdsReg)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_CMD, 0x0080 | sdsId)) != RT_ERR_OK)
        return retVal;

    while (1)
    {
        if ((retVal = rtl8367c_getAsicReg(RTL8367C_REG_SDS_INDACS_CMD, &busy)) != RT_ERR_OK)
            return retVal;

        if ((busy & 0x100) == 0)
            break;
    }

    if ((retVal = rtl8367c_getAsicReg(RTL8367C_REG_SDS_INDACS_DATA, value)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsicSdsReg
 * Description:
 *      Set Serdes registers
 * Input:
 *      sdsId   - sdsid (0~1)
 *      sdsReg - reg address (0~31)
 *      sdsPage - Writing data
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK               - Success

 * Note:
 *      None
 */

int32_t rtl8367::rtl8367c_setAsicSdsReg(uint32_t sdsId, uint32_t sdsReg, uint32_t sdsPage, uint32_t value)
{
    uint32_t retVal;

    if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_DATA, value)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_ADR, (sdsPage << 5) | sdsReg)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_CMD, 0x00C0 | sdsId)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}
/* Function Name:
 *      rtl8367c_setAsicPortForceLinkExt
 * Description:
 *      Set external interface force linking configuration
 * Input:
 *      id          - external interface id (0~2)
 *      portAbility - port ability configuration
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK           - Success
 *      RT_ERR_SMI          - SMI access error
 *      RT_ERR_OUT_OF_RANGE - input parameter out of range
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicPortForceLinkExt(uint32_t id, rtl8367c_port_ability_t *pPortAbility)
{
    uint32_t retVal, regValue, regValue2, type, sgmiibit, hisgmiibit;
    uint32_t reg_data = 0;
    uint32_t i = 0;

    /* Invalid input parameter */
    if (id >= RTL8367C_EXTNO)
        return RT_ERR_OUT_OF_RANGE;

    reg_data |= pPortAbility->forcemode << 12;
    reg_data |= pPortAbility->mstfault << 9;
    reg_data |= pPortAbility->mstmode << 8;
    reg_data |= pPortAbility->nway << 7;
    reg_data |= pPortAbility->txpause << 6;
    reg_data |= pPortAbility->rxpause << 5;
    reg_data |= pPortAbility->link << 4;
    reg_data |= pPortAbility->duplex << 2;
    reg_data |= pPortAbility->speed;

    if ((retVal = rtl8367c_setAsicReg(0x13C2, 0x0249)) != RT_ERR_OK)
        return retVal;
    /*get chip ID */
    if ((retVal = rtl8367c_getAsicReg(0x1300, &regValue)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicReg(0x13C2, 0x0000)) != RT_ERR_OK)
        return retVal;

    type = 0;

    switch (regValue)
    {
    case 0x0276:
    case 0x0597:
    case 0x6367:
        type = 1;
        break;
    case 0x0652:
    case 0x6368:
        type = 2;
        break;
    case 0x0801:
    case 0x6511:
        type = 3;
        break;
    default:
        return RT_ERR_FAILED;
    }

    if (1 == type)
    {
        if (1 == id)
        {
            if ((retVal = rtl8367c_getAsicReg(RTL8367C_REG_REG_TO_ECO4, &regValue)) != RT_ERR_OK)
                return retVal;

            if ((regValue & (0x0001 << 5)) && (regValue & (0x0001 << 7)))
            {
                return RT_ERR_OK;
            }

            if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_SGMII_FDUP_OFFSET, pPortAbility->duplex)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_SGMII_SPD_MASK, pPortAbility->speed)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_SGMII_LINK_OFFSET, pPortAbility->link)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_SGMII_TXFC_OFFSET, pPortAbility->txpause)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_SGMII_RXFC_OFFSET, pPortAbility->rxpause)) != RT_ERR_OK)
                return retVal;
        }

        if (0 == id || 1 == id)
            return rtl8367c_setAsicReg(RTL8367C_REG_DIGITAL_INTERFACE0_FORCE + id, reg_data);
        else
            return rtl8367c_setAsicReg(RTL8367C_REG_DIGITAL_INTERFACE2_FORCE, reg_data);
    }
    else if (2 == type)
    {
        if (1 == id)
        {
            if ((retVal = rtl8367c_setAsicRegBit(0x1311, 2, pPortAbility->duplex)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicRegBits(0x1311, 0x3, pPortAbility->speed)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicRegBit(0x1311, 4, pPortAbility->link)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicRegBit(0x1311, 6, pPortAbility->txpause)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicRegBit(0x1311, 5, pPortAbility->rxpause)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicRegBit(0x1311, 12, pPortAbility->forcemode)) != RT_ERR_OK)
                return retVal;

            if (pPortAbility->link == 1)
            {
                if ((retVal = rtl8367c_setAsicRegBit(0x1311, 4, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1311, 4, 1)) != RT_ERR_OK)
                    return retVal;
            }
            else
            {
                if ((retVal = rtl8367c_setAsicRegBits(0x1311, 0x3, 2)) != RT_ERR_OK)
                    return retVal;
            }

            if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_SGMII_FDUP_OFFSET, pPortAbility->duplex)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_SGMII_SPD_MASK, pPortAbility->speed)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_SGMII_LINK_OFFSET, pPortAbility->link)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_SGMII_TXFC_OFFSET, pPortAbility->txpause)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_SGMII_RXFC_OFFSET, pPortAbility->rxpause)) != RT_ERR_OK)
                return retVal;
        }
        else if (2 == id)
        {
            if ((retVal = rtl8367c_setAsicRegBit(0x13c4, 2, pPortAbility->duplex)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicRegBits(0x13c4, 0x3, pPortAbility->speed)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicRegBit(0x13c4, 4, pPortAbility->link)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicRegBit(0x13c4, 6, pPortAbility->txpause)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicRegBit(0x13c4, 5, pPortAbility->rxpause)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicRegBit(0x13c4, 12, pPortAbility->forcemode)) != RT_ERR_OK)
                return retVal;

            if (pPortAbility->link == 1)
            {
                if ((retVal = rtl8367c_setAsicRegBit(0x13c4, 4, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x13c4, 4, 1)) != RT_ERR_OK)
                    return retVal;
            }
            else
            {
                if ((retVal = rtl8367c_setAsicRegBits(0x13c4, 0x3, 2)) != RT_ERR_OK)
                    return retVal;
            }

            if ((retVal = rtl8367c_setAsicRegBit(0x1dc1, RTL8367C_CFG_SGMII_FDUP_OFFSET, pPortAbility->duplex)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicRegBits(0x1dc1, RTL8367C_CFG_SGMII_SPD_MASK, pPortAbility->speed)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicRegBit(0x1dc1, RTL8367C_CFG_SGMII_LINK_OFFSET, pPortAbility->link)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicRegBit(0x1dc1, RTL8367C_CFG_SGMII_TXFC_OFFSET, pPortAbility->txpause)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicRegBit(0x1dc1, RTL8367C_CFG_SGMII_RXFC_OFFSET, pPortAbility->rxpause)) != RT_ERR_OK)
                return retVal;
        }
    }
    else if (3 == type)
    {
        if (1 == id)
        {
            if ((retVal = rtl8367c_getAsicRegBit(0x1d11, 6, &sgmiibit)) != RT_ERR_OK)
                return retVal;
            if ((retVal = rtl8367c_getAsicRegBit(0x1d11, 11, &hisgmiibit)) != RT_ERR_OK)
                return retVal;

            if ((sgmiibit == 1) || (hisgmiibit == 1))
            {
                /*for 1000x/100fx/1000x_100fx, param has to be set to serdes registers*/
                if ((retVal = rtl8367c_getAsicReg(0x1d41, &regValue)) != RT_ERR_OK)
                    return retVal;

                if ((regValue & 0xa0) == 0xa0)
                {

                    if ((retVal = rtl8367c_getAsicRegBits(0x1d95, 0x1f00, &regValue2)) != RT_ERR_OK)
                        return retVal;

                    /*1000X*/
                    if (regValue2 == 0x4)
                    {
                        /* Enable new sds mode config */
                        if ((retVal = rtl8367c_setAsicRegBit(0x1d95, 13, 1)) != RT_ERR_OK)
                            return retVal;

                        /* 0 4 0  bit 12  set 1,  bit15~13 = 4*/
                        if ((retVal = rtl8367c_getAsicSdsReg(0, 4, 0, &reg_data)) != RT_ERR_OK)
                            return retVal;
                        reg_data &= 0xFFFF0FFF;
                        reg_data |= 0x9000;
                        if ((retVal = rtl8367c_setAsicSdsReg(0, 4, 0, reg_data)) != RT_ERR_OK)
                            return retVal;

                        /* 0 0 2  bit 6  set 1,  bit13 set to 0, bit12 nway_en*/
                        if ((retVal = rtl8367c_getAsicSdsReg(0, 0, 2, &reg_data)) != RT_ERR_OK)
                            return retVal;
                        reg_data &= 0xFFFFDFFF;
                        reg_data |= 0x40;
                        if (pPortAbility->forcemode)
                            reg_data &= 0xffffefff;
                        else
                            reg_data |= 0x1000;

                        if ((retVal = rtl8367c_setAsicSdsReg(0, 0, 2, reg_data)) != RT_ERR_OK)
                            return retVal;

                        /* 0 4 2  bit 8  rx pause,  bit7 tx pause*/
                        if ((retVal = rtl8367c_getAsicSdsReg(0, 4, 2, &reg_data)) != RT_ERR_OK)
                            return retVal;

                        if (pPortAbility->txpause)
                            reg_data |= 0x80;
                        else
                            reg_data &= (~0x80);

                        if (pPortAbility->rxpause)
                            reg_data |= 0x100;
                        else
                            reg_data &= (~0x100);

                        if ((retVal = rtl8367c_setAsicSdsReg(0, 4, 2, reg_data)) != RT_ERR_OK)
                            return retVal;

                        /* 0 4 0  bit 12  set 0*/
                        if ((retVal = rtl8367c_getAsicSdsReg(0, 4, 0, &reg_data)) != RT_ERR_OK)
                            return retVal;
                        reg_data &= 0xFFFFEFFF;
                        if ((retVal = rtl8367c_setAsicSdsReg(0, 4, 0, reg_data)) != RT_ERR_OK)
                            return retVal;

                        /*new_cfg_sds_mode=1000x*/
                        if ((retVal = rtl8367c_setAsicRegBits(0x1d95, 0x1f00, 0x4)) != RT_ERR_OK)
                            return retVal;
                    }
                    else if (regValue2 == 0x5)
                    {

                        if ((retVal = rtl8367c_setAsicRegBit(0x1d95, 13, 1)) != RT_ERR_OK)
                            return retVal;

                        /* 0 4 0  bit 12  set 1,  bit15~13 = 5*/
                        if ((retVal = rtl8367c_getAsicSdsReg(0, 4, 0, &reg_data)) != RT_ERR_OK)
                            return retVal;
                        reg_data &= 0xFFFF0FFF;
                        reg_data |= 0xB000;
                        if ((retVal = rtl8367c_setAsicSdsReg(0, 4, 0, reg_data)) != RT_ERR_OK)
                            return retVal;

                        /* 0 0 2  bit 6  set 0,  bit13 set to 1, bit12 0*/
                        if ((retVal = rtl8367c_getAsicSdsReg(0, 0, 2, &reg_data)) != RT_ERR_OK)
                            return retVal;
                        reg_data &= 0xFFFFFFBF;
                        reg_data |= 0x2000;
                        reg_data &= 0xffffefff;

                        if ((retVal = rtl8367c_setAsicSdsReg(0, 0, 2, reg_data)) != RT_ERR_OK)
                            return retVal;

                        /* 0 4 2  bit 8  rx pause,  bit7 tx pause*/
                        if ((retVal = rtl8367c_getAsicSdsReg(0, 4, 2, &reg_data)) != RT_ERR_OK)
                            return retVal;
                        if (pPortAbility->txpause)
                            reg_data |= 0x80;
                        else
                            reg_data &= (~0x80);
                        if (pPortAbility->rxpause)
                            reg_data |= 0x100;
                        else
                            reg_data &= (~0x100);
                        if ((retVal = rtl8367c_setAsicSdsReg(0, 4, 2, reg_data)) != RT_ERR_OK)
                            return retVal;

                        /* 0 4 0  bit 12  set 0*/
                        if ((retVal = rtl8367c_getAsicSdsReg(0, 4, 0, &reg_data)) != RT_ERR_OK)
                            return retVal;
                        reg_data &= 0xFFFFEFFF;
                        if ((retVal = rtl8367c_setAsicSdsReg(0, 4, 0, reg_data)) != RT_ERR_OK)
                            return retVal;
                        /* new_cfg_sds_mode=1000x */
                        if ((retVal = rtl8367c_setAsicRegBits(0x1d95, 0x1f00, 0x5)) != RT_ERR_OK)
                            return retVal;
                    }
                    else if (regValue2 == 0x7)
                    {
                        if ((retVal = rtl8367c_setAsicRegBit(0x1d95, 13, 1)) != RT_ERR_OK)
                            return retVal;

                        /* 0 4 0  bit 12  set 1,  bit15~13 = 4*/
                        if ((retVal = rtl8367c_getAsicSdsReg(0, 4, 0, &reg_data)) != RT_ERR_OK)
                            return retVal;
                        reg_data &= 0xFFFF0FFF;
                        reg_data |= 0x9000;
                        if ((retVal = rtl8367c_setAsicSdsReg(0, 4, 0, reg_data)) != RT_ERR_OK)
                            return retVal;

                        /* 0 0 2  bit 6  set 1,  bit13 set to 0, bit12 nway_en*/
                        if ((retVal = rtl8367c_getAsicSdsReg(0, 0, 2, &reg_data)) != RT_ERR_OK)
                            return retVal;
                        reg_data &= 0xFFFFDFFF;
                        reg_data |= 0x40;
                        if (pPortAbility->forcemode)
                            reg_data &= 0xffffefff;
                        else
                            reg_data |= 0x1000;

                        if ((retVal = rtl8367c_setAsicSdsReg(0, 0, 2, reg_data)) != RT_ERR_OK)
                            return retVal;

                        /* 0 4 2  bit 8  rx pause,  bit7 tx pause*/
                        if ((retVal = rtl8367c_getAsicSdsReg(0, 4, 2, &reg_data)) != RT_ERR_OK)
                            return retVal;
                        if (pPortAbility->txpause)
                            reg_data |= 0x80;
                        else
                            reg_data &= (~0x80);
                        if (pPortAbility->rxpause)
                            reg_data |= 0x100;
                        else
                            reg_data &= (~0x100);
                        if ((retVal = rtl8367c_setAsicSdsReg(0, 4, 2, reg_data)) != RT_ERR_OK)
                            return retVal;

                        /* 0 4 0  bit 12  set 0*/
                        if ((retVal = rtl8367c_getAsicSdsReg(0, 4, 0, &reg_data)) != RT_ERR_OK)
                            return retVal;
                        reg_data &= 0xFFFFEFFF;
                        if ((retVal = rtl8367c_setAsicSdsReg(0, 4, 0, reg_data)) != RT_ERR_OK)
                            return retVal;

                        /* 0 4 0  bit 12  set 1,  bit15~13 = 5*/
                        if ((retVal = rtl8367c_getAsicSdsReg(0, 4, 0, &reg_data)) != RT_ERR_OK)
                            return retVal;
                        reg_data &= 0xFFFF0FFF;
                        reg_data |= 0xB000;
                        if ((retVal = rtl8367c_setAsicSdsReg(0, 4, 0, reg_data)) != RT_ERR_OK)
                            return retVal;

                        /* 0 0 2  bit 6  set 0,  bit13 set to 1, bit12 0*/
                        if ((retVal = rtl8367c_getAsicSdsReg(0, 0, 2, &reg_data)) != RT_ERR_OK)
                            return retVal;
                        reg_data &= 0xFFFFFFBF;
                        reg_data |= 0x2000;
                        reg_data &= 0xffffefff;

                        if ((retVal = rtl8367c_setAsicSdsReg(0, 0, 2, reg_data)) != RT_ERR_OK)
                            return retVal;

                        /* 0 4 2  bit 8  rx pause,  bit7 tx pause*/
                        if ((retVal = rtl8367c_getAsicSdsReg(0, 4, 2, &reg_data)) != RT_ERR_OK)
                            return retVal;
                        if (pPortAbility->txpause)
                            reg_data |= 0x80;
                        else
                            reg_data &= 0xffffff7f;
                        if (pPortAbility->rxpause)
                            reg_data |= 0x100;
                        else
                            reg_data &= 0xfffffeff;
                        if ((retVal = rtl8367c_setAsicSdsReg(0, 4, 2, reg_data)) != RT_ERR_OK)
                            return retVal;

                        /* 0 4 0  bit 12  set 0*/
                        if ((retVal = rtl8367c_getAsicSdsReg(0, 4, 0, &reg_data)) != RT_ERR_OK)
                            return retVal;
                        reg_data &= 0xFFFFEFFF;
                        if ((retVal = rtl8367c_setAsicSdsReg(0, 4, 0, reg_data)) != RT_ERR_OK)
                            return retVal;
                        /*sds_mode:*/
                        if ((retVal = rtl8367c_setAsicRegBits(0x1d95, 0x1f00, 0x7)) != RT_ERR_OK)
                            return retVal;
                    }

                    /*disable force ability   ---      */
                    if ((retVal = rtl8367c_setAsicRegBit(0x137c, 12, 0)) != RT_ERR_OK)
                        return retVal;
                    return RT_ERR_OK;
                }

                /* new_cfg_sds_mode */
                if ((retVal = rtl8367c_getAsicRegBits(0x1d95, 0x1f00, &regValue2)) != RT_ERR_OK)
                    return retVal;
                if (regValue2 == 0x2)
                {
                    if ((retVal = rtl8367c_setAsicRegBit(0x1d95, 13, 1)) != RT_ERR_OK)
                        return retVal;

                    for (i = 0; i < 0xfff; i++)
                        ;

                    if ((retVal = rtl8367c_setAsicRegBits(0x1d95, 0x1f00, 0x2)) != RT_ERR_OK)
                        return retVal;

                    for (i = 0; i < 0xfff; i++)
                        ;

                    if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_SGMII_FDUP_OFFSET, pPortAbility->duplex)) != RT_ERR_OK)
                        return retVal;

                    if ((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_SGMII_SPD_MASK, pPortAbility->speed)) != RT_ERR_OK)
                        return retVal;

                    if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_SGMII_TXFC_OFFSET, pPortAbility->txpause)) != RT_ERR_OK)
                        return retVal;

                    if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_SGMII_RXFC_OFFSET, pPortAbility->rxpause)) != RT_ERR_OK)
                        return retVal;

                    if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_SGMII_LINK_OFFSET, pPortAbility->link)) != RT_ERR_OK)
                        return retVal;

                    /*disable force ability   ---      */
                    if ((retVal = rtl8367c_setAsicRegBit(0x137c, 12, 0)) != RT_ERR_OK)
                        return retVal;
                    return RT_ERR_OK;
                }
                else if (regValue2 == 0x12)
                {
                    if ((retVal = rtl8367c_setAsicRegBit(0x1d95, 13, 1)) != RT_ERR_OK)
                        return retVal;

                    for (i = 0; i < 0xfff; i++)
                        ;

                    if ((retVal = rtl8367c_setAsicRegBits(0x1d95, 0x1f00, 0x12)) != RT_ERR_OK)
                        return retVal;

                    for (i = 0; i < 0xfff; i++)
                        ;

                    if ((retVal = rtl8367c_setAsicRegBit(0x1d11, 11, 0x1)) != RT_ERR_OK)
                        return retVal;

                    if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_SGMII_FDUP_OFFSET, pPortAbility->duplex)) != RT_ERR_OK)
                        return retVal;

                    if ((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_SGMII_SPD_MASK, pPortAbility->speed)) != RT_ERR_OK)
                        return retVal;

                    if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_SGMII_TXFC_OFFSET, pPortAbility->txpause)) != RT_ERR_OK)
                        return retVal;

                    if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_SGMII_RXFC_OFFSET, pPortAbility->rxpause)) != RT_ERR_OK)
                        return retVal;

                    if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_SGMII_LINK_OFFSET, pPortAbility->link)) != RT_ERR_OK)
                        return retVal;

                    /*disable force ability   ---      */
                    if ((retVal = rtl8367c_setAsicRegBit(0x137c, 12, 0)) != RT_ERR_OK)
                        return retVal;
                    return RT_ERR_OK;
                }
            }
            else
            {
                if ((retVal = rtl8367c_getAsicRegBits(0x1d3d, 10, &regValue2)) != RT_ERR_OK)
                    return retVal;
                if (regValue2 == 0)
                {
                    /*ext1_force_ablty*/
                    if ((retVal = rtl8367c_setAsicRegBit(0x1311, 2, pPortAbility->duplex)) != RT_ERR_OK)
                        return retVal;

                    if ((retVal = rtl8367c_setAsicRegBits(0x1311, 0x3, pPortAbility->speed)) != RT_ERR_OK)
                        return retVal;

                    if ((retVal = rtl8367c_setAsicRegBit(0x1311, 4, pPortAbility->link)) != RT_ERR_OK)
                        return retVal;

                    if ((retVal = rtl8367c_setAsicRegBit(0x1311, 6, pPortAbility->txpause)) != RT_ERR_OK)
                        return retVal;

                    if ((retVal = rtl8367c_setAsicRegBit(0x1311, 5, pPortAbility->rxpause)) != RT_ERR_OK)
                        return retVal;

                    /*force mode for ext1*/
                    if ((retVal = rtl8367c_setAsicRegBit(0x1311, 12, pPortAbility->forcemode)) != RT_ERR_OK)
                        return retVal;

                    if (pPortAbility->link == 1)
                    {
                        if ((retVal = rtl8367c_setAsicRegBit(0x1311, 4, 0)) != RT_ERR_OK)
                            return retVal;

                        if ((retVal = rtl8367c_setAsicRegBit(0x1311, 4, 1)) != RT_ERR_OK)
                            return retVal;
                    }
                    else
                    {
                        if ((retVal = rtl8367c_setAsicRegBits(0x1311, 0x3, 2)) != RT_ERR_OK)
                            return retVal;
                    }

                    /*disable force ability   ---      */
                    if ((retVal = rtl8367c_setAsicRegBit(0x137c, 12, 0)) != RT_ERR_OK)
                        return retVal;
                    return RT_ERR_OK;
                }
            }
        }
        else if (2 == id)
        {

            if ((retVal = rtl8367c_getAsicRegBit(0x1d95, 0, &sgmiibit)) != RT_ERR_OK)
                return retVal;
            if (sgmiibit == 1)
            {
                /*for 1000x/100fx/1000x_100fx, param has to bet set to serdes registers*/
                if ((retVal = rtl8367c_getAsicReg(0x1d95, &regValue)) != RT_ERR_OK)
                    return retVal;
                /*cfg_mac7_sel_sgmii=1 & cfg_mac7_fib =1*/
                if ((regValue & 0x3) == 0x3)
                {
                    if ((retVal = rtl8367c_getAsicRegBits(0x1d95, 0x1f00, &regValue2)) != RT_ERR_OK)
                        return retVal;

                    if (regValue2 == 0x4)
                    {
                        if ((retVal = rtl8367c_setAsicRegBit(0x1d95, 13, 1)) != RT_ERR_OK)
                            return retVal;

                        /* 0 4 0  bit 12  set 1,  bit15~13 = 4*/
                        if ((retVal = rtl8367c_getAsicSdsReg(0, 4, 0, &reg_data)) != RT_ERR_OK)
                            return retVal;
                        reg_data &= 0xFFFF0FFF;
                        reg_data |= 0x9000;
                        if ((retVal = rtl8367c_setAsicSdsReg(0, 4, 0, reg_data)) != RT_ERR_OK)
                            return retVal;

                        /* 0 0 2  bit 6  set 1,  bit13 set to 0, bit12 nway_en*/
                        if ((retVal = rtl8367c_getAsicSdsReg(0, 0, 2, &reg_data)) != RT_ERR_OK)
                            return retVal;
                        reg_data &= 0xFFFFDFFF;
                        reg_data |= 0x40;
                        if (pPortAbility->forcemode)
                            reg_data &= 0xffffefff;
                        else
                            reg_data |= 0x1000;

                        if ((retVal = rtl8367c_setAsicSdsReg(0, 0, 2, reg_data)) != RT_ERR_OK)
                            return retVal;

                        /* 0 4 2  bit 8  rx pause,  bit7 tx pause*/
                        if ((retVal = rtl8367c_getAsicSdsReg(0, 4, 2, &reg_data)) != RT_ERR_OK)
                            return retVal;
                        if (pPortAbility->txpause)
                            reg_data |= 0x80;
                        else
                            reg_data &= 0xffffff7f;
                        if (pPortAbility->rxpause)
                            reg_data |= 0x100;
                        else
                            reg_data &= 0xfffffeff;
                        if ((retVal = rtl8367c_setAsicSdsReg(0, 4, 2, reg_data)) != RT_ERR_OK)
                            return retVal;

                        /* 0 4 0  bit 12  set 0*/
                        if ((retVal = rtl8367c_getAsicSdsReg(0, 4, 0, &reg_data)) != RT_ERR_OK)
                            return retVal;
                        reg_data &= 0xFFFFEFFF;
                        if ((retVal = rtl8367c_setAsicSdsReg(0, 4, 0, reg_data)) != RT_ERR_OK)
                            return retVal;

                        if ((retVal = rtl8367c_setAsicRegBits(0x1d95, 0x1f00, 0x4)) != RT_ERR_OK)
                            return retVal;
                    }
                    else if (regValue2 == 0x5)
                    {
                        if ((retVal = rtl8367c_setAsicRegBit(0x1d95, 13, 1)) != RT_ERR_OK)
                            return retVal;

                        /* 0 4 0  bit 12  set 1,  bit15~13 = 5*/
                        if ((retVal = rtl8367c_getAsicSdsReg(0, 4, 0, &reg_data)) != RT_ERR_OK)
                            return retVal;
                        reg_data &= 0xFFFF0FFF;
                        reg_data |= 0xB000;
                        if ((retVal = rtl8367c_setAsicSdsReg(0, 4, 0, reg_data)) != RT_ERR_OK)
                            return retVal;

                        /* 0 0 2  bit 6  set 0,  bit13 set to 1, bit12 0*/
                        if ((retVal = rtl8367c_getAsicSdsReg(0, 0, 2, &reg_data)) != RT_ERR_OK)
                            return retVal;
                        reg_data &= 0xFFFFFFBF;
                        reg_data |= 0x2000;
                        reg_data &= 0xffffefff;

                        if ((retVal = rtl8367c_setAsicSdsReg(0, 0, 2, reg_data)) != RT_ERR_OK)
                            return retVal;

                        /* 0 4 2  bit 8  rx pause,  bit7 tx pause*/
                        if ((retVal = rtl8367c_getAsicSdsReg(0, 4, 2, &reg_data)) != RT_ERR_OK)
                            return retVal;
                        if (pPortAbility->txpause)
                            reg_data |= 0x80;
                        else
                            reg_data &= 0xffffff7f;
                        if (pPortAbility->rxpause)
                            reg_data |= 0x100;
                        else
                            reg_data &= 0xfffffeff;
                        if ((retVal = rtl8367c_setAsicSdsReg(0, 4, 2, reg_data)) != RT_ERR_OK)
                            return retVal;

                        /* 0 4 0  bit 12  set 0*/
                        if ((retVal = rtl8367c_getAsicSdsReg(0, 4, 0, &reg_data)) != RT_ERR_OK)
                            return retVal;
                        reg_data &= 0xFFFFEFFF;
                        if ((retVal = rtl8367c_setAsicSdsReg(0, 4, 0, reg_data)) != RT_ERR_OK)
                            return retVal;

                        if ((retVal = rtl8367c_setAsicRegBits(0x1d95, 0x1f00, 0x5)) != RT_ERR_OK)
                            return retVal;
                    }
                    else if (regValue2 == 0x7)
                    {
                        if ((retVal = rtl8367c_setAsicRegBit(0x1d95, 13, 1)) != RT_ERR_OK)
                            return retVal;

                        /* 0 4 0  bit 12  set 1,  bit15~13 = 4*/
                        if ((retVal = rtl8367c_getAsicSdsReg(0, 4, 0, &reg_data)) != RT_ERR_OK)
                            return retVal;
                        reg_data &= 0xFFFF0FFF;
                        reg_data |= 0x9000;
                        if ((retVal = rtl8367c_setAsicSdsReg(0, 4, 0, reg_data)) != RT_ERR_OK)
                            return retVal;

                        /* 0 0 2  bit 6  set 1,  bit13 set to 0, bit12 nway_en*/
                        if ((retVal = rtl8367c_getAsicSdsReg(0, 0, 2, &reg_data)) != RT_ERR_OK)
                            return retVal;
                        reg_data &= 0xFFFFDFFF;
                        reg_data |= 0x40;
                        if (pPortAbility->forcemode)
                            reg_data &= 0xffffefff;
                        else
                            reg_data |= 0x1000;

                        if ((retVal = rtl8367c_setAsicSdsReg(0, 0, 2, reg_data)) != RT_ERR_OK)
                            return retVal;

                        /* 0 4 2  bit 8  rx pause,  bit7 tx pause*/
                        if ((retVal = rtl8367c_getAsicSdsReg(0, 4, 2, &reg_data)) != RT_ERR_OK)
                            return retVal;
                        if (pPortAbility->txpause)
                            reg_data |= 0x80;
                        else
                            reg_data &= 0xffffff7f;
                        if (pPortAbility->rxpause)
                            reg_data |= 0x100;
                        else
                            reg_data &= 0xfffffeff;
                        if ((retVal = rtl8367c_setAsicSdsReg(0, 4, 2, reg_data)) != RT_ERR_OK)
                            return retVal;

                        /* 0 4 0  bit 12  set 0*/
                        if ((retVal = rtl8367c_getAsicSdsReg(0, 4, 0, &reg_data)) != RT_ERR_OK)
                            return retVal;
                        reg_data &= 0xFFFFEFFF;
                        if ((retVal = rtl8367c_setAsicSdsReg(0, 4, 0, reg_data)) != RT_ERR_OK)
                            return retVal;

                        /* 0 4 0  bit 12  set 1,  bit15~13 = 5*/
                        if ((retVal = rtl8367c_getAsicSdsReg(0, 4, 0, &reg_data)) != RT_ERR_OK)
                            return retVal;
                        reg_data &= 0xFFFF0FFF;
                        reg_data |= 0xB000;
                        if ((retVal = rtl8367c_setAsicSdsReg(0, 4, 0, reg_data)) != RT_ERR_OK)
                            return retVal;

                        /* 0 0 2  bit 6  set 0,  bit13 set to 1, bit12 0*/
                        if ((retVal = rtl8367c_getAsicSdsReg(0, 0, 2, &reg_data)) != RT_ERR_OK)
                            return retVal;
                        reg_data &= 0xFFFFFFBF;
                        reg_data |= 0x2000;
                        reg_data &= 0xffffefff;

                        if ((retVal = rtl8367c_setAsicSdsReg(0, 0, 2, reg_data)) != RT_ERR_OK)
                            return retVal;

                        /* 0 4 2  bit 8  rx pause,  bit7 tx pause*/
                        if ((retVal = rtl8367c_getAsicSdsReg(0, 4, 2, &reg_data)) != RT_ERR_OK)
                            return retVal;
                        if (pPortAbility->txpause)
                            reg_data |= 0x80;
                        else
                            reg_data &= 0xffffff7f;
                        if (pPortAbility->rxpause)
                            reg_data |= 0x100;
                        else
                            reg_data &= 0xfffffeff;
                        if ((retVal = rtl8367c_setAsicSdsReg(0, 4, 2, reg_data)) != RT_ERR_OK)
                            return retVal;

                        /* 0 4 0  bit 12  set 0*/
                        if ((retVal = rtl8367c_getAsicSdsReg(0, 4, 0, &reg_data)) != RT_ERR_OK)
                            return retVal;
                        reg_data &= 0xFFFFEFFF;
                        if ((retVal = rtl8367c_setAsicSdsReg(0, 4, 0, reg_data)) != RT_ERR_OK)
                            return retVal;

                        if ((retVal = rtl8367c_setAsicRegBits(0x1d95, 0x1f00, 0x7)) != RT_ERR_OK)
                            return retVal;
                    }

                    if ((retVal = rtl8367c_setAsicRegBit(0x137d, 12, 0)) != RT_ERR_OK)
                        return retVal;
                    return RT_ERR_OK;
                }

                if ((retVal = rtl8367c_getAsicRegBits(0x1d95, 0x1f00, &regValue2)) != RT_ERR_OK)
                    return retVal;
                if (regValue2 == 0x2)
                {
                    if ((retVal = rtl8367c_setAsicRegBit(0x1d95, 13, 1)) != RT_ERR_OK)
                        return retVal;

                    for (i = 0; i < 0xfff; i++)
                        ;

                    /* 0 2 0  bit 8-9  nway*/
                    if ((retVal = rtl8367c_getAsicSdsReg(0, 2, 0, &reg_data)) != RT_ERR_OK)
                        return retVal;
                    reg_data &= 0xfffffcff;
                    if (pPortAbility->nway)
                        reg_data &= 0xfffffcff;
                    else
                        reg_data |= 0x100;
                    if ((retVal = rtl8367c_setAsicSdsReg(0, 2, 0, reg_data)) != RT_ERR_OK)
                        return retVal;

                    if ((retVal = rtl8367c_setAsicRegBits(0x1d95, 0x1f00, 0x2)) != RT_ERR_OK)
                        return retVal;

                    for (i = 0; i < 0xfff; i++)
                        ;

                    if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_SGMII_FDUP_OFFSET, pPortAbility->duplex)) != RT_ERR_OK)
                        return retVal;

                    if ((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_SGMII_SPD_MASK, pPortAbility->speed)) != RT_ERR_OK)
                        return retVal;

                    if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_SGMII_TXFC_OFFSET, pPortAbility->txpause)) != RT_ERR_OK)
                        return retVal;

                    if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_SGMII_RXFC_OFFSET, pPortAbility->rxpause)) != RT_ERR_OK)
                        return retVal;

                    if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_SGMII_LINK_OFFSET, pPortAbility->link)) != RT_ERR_OK)
                        return retVal;

                    if ((retVal = rtl8367c_setAsicRegBit(0x137d, 12, 0)) != RT_ERR_OK)
                        return retVal;
                    return RT_ERR_OK;
                }
            }
            else
            {

                /*ext2_force_ablty*/
                if ((retVal = rtl8367c_setAsicRegBit(0x13c4, 2, pPortAbility->duplex)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBits(0x13c4, 0x3, pPortAbility->speed)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x13c4, 4, pPortAbility->link)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x13c4, 6, pPortAbility->txpause)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x13c4, 5, pPortAbility->rxpause)) != RT_ERR_OK)
                    return retVal;

                /*force mode for ext2*/
                if ((retVal = rtl8367c_setAsicRegBit(0x13c4, 12, pPortAbility->forcemode)) != RT_ERR_OK)
                    return retVal;

                if (pPortAbility->link == 1)
                {
                    if ((retVal = rtl8367c_setAsicRegBit(0x13c4, 4, 0)) != RT_ERR_OK)
                        return retVal;

                    if ((retVal = rtl8367c_setAsicRegBit(0x13c4, 4, 1)) != RT_ERR_OK)
                        return retVal;
                }
                else
                {
                    if ((retVal = rtl8367c_setAsicRegBits(0x13c4, 0x3, 2)) != RT_ERR_OK)
                        return retVal;
                }

                if ((retVal = rtl8367c_getAsicRegBit(0x1d3d, 10, &reg_data)) != RT_ERR_OK)
                    return retVal;
                if (reg_data == 1)
                {
                    if ((retVal = rtl8367c_setAsicRegBit(0x1311, 2, pPortAbility->duplex)) != RT_ERR_OK)
                        return retVal;

                    if ((retVal = rtl8367c_setAsicRegBits(0x1311, 0x3, pPortAbility->speed)) != RT_ERR_OK)
                        return retVal;

                    if ((retVal = rtl8367c_setAsicRegBit(0x1311, 4, pPortAbility->link)) != RT_ERR_OK)
                        return retVal;

                    if ((retVal = rtl8367c_setAsicRegBit(0x1311, 6, pPortAbility->txpause)) != RT_ERR_OK)
                        return retVal;

                    if ((retVal = rtl8367c_setAsicRegBit(0x1311, 5, pPortAbility->rxpause)) != RT_ERR_OK)
                        return retVal;

                    /*force mode for ext1*/
                    if ((retVal = rtl8367c_setAsicRegBit(0x1311, 12, pPortAbility->forcemode)) != RT_ERR_OK)
                        return retVal;

                    if (pPortAbility->link == 1)
                    {
                        if ((retVal = rtl8367c_setAsicRegBit(0x1311, 4, 0)) != RT_ERR_OK)
                            return retVal;

                        if ((retVal = rtl8367c_setAsicRegBit(0x1311, 4, 1)) != RT_ERR_OK)
                            return retVal;
                    }
                    else
                    {
                        if ((retVal = rtl8367c_setAsicRegBits(0x1311, 0x3, 2)) != RT_ERR_OK)
                            return retVal;
                    }
                }
            }

            /*disable force ability   ---      */
            if ((retVal = rtl8367c_setAsicRegBit(0x137d, 12, 0)) != RT_ERR_OK)
                return retVal;
        }
    }

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsicPortExtMode
 * Description:
 *      Set external interface mode configuration
 * Input:
 *      id      - external interface id (0~2)
 *      mode    - external interface mode
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK           - Success
 *      RT_ERR_SMI          - SMI access error
 *      RT_ERR_OUT_OF_RANGE - input parameter out of range
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicPortExtMode(uint32_t id, uint32_t mode)
{
    int32_t retVal;
    uint32_t i, regValue, type, option, reg_data;
    uint32_t idx;
    uint32_t redData[][2] = {{0x04D7, 0x0480}, {0xF994, 0x0481}, {0x21A2, 0x0482}, {0x6960, 0x0483}, {0x9728, 0x0484}, {0x9D85, 0x0423}, {0xD810, 0x0424}, {0x83F2, 0x002E}};
    uint32_t redDataSB[][2] = {{0x04D7, 0x0480}, {0xF994, 0x0481}, {0x31A2, 0x0482}, {0x6960, 0x0483}, {0x9728, 0x0484}, {0x9D85, 0x0423}, {0xD810, 0x0424}, {0x83F2, 0x002E}};
    uint32_t redData1[][2] = {{0x82F1, 0x0500}, {0xF195, 0x0501}, {0x31A2, 0x0502}, {0x796C, 0x0503}, {0x9728, 0x0504}, {0x9D85, 0x0423}, {0xD810, 0x0424}, {0x0F80, 0x0001}, {0x83F2, 0x002E}};
    uint32_t redData5[][2] = {{0x82F1, 0x0500}, {0xF195, 0x0501}, {0x31A2, 0x0502}, {0x796C, 0x0503}, {0x9728, 0x0504}, {0x9D85, 0x0423}, {0xD810, 0x0424}, {0x0F80, 0x0001}, {0x83F2, 0x002E}};
    uint32_t redData6[][2] = {{0x82F1, 0x0500}, {0xF195, 0x0501}, {0x31A2, 0x0502}, {0x796C, 0x0503}, {0x9728, 0x0504}, {0x9D85, 0x0423}, {0xD810, 0x0424}, {0x0F80, 0x0001}, {0x83F2, 0x002E}};
    uint32_t redData8[][2] = {{0x82F1, 0x0500}, {0xF995, 0x0501}, {0x31A2, 0x0502}, {0x796C, 0x0503}, {0x9728, 0x0504}, {0x9D85, 0x0423}, {0xD810, 0x0424}, {0x0F80, 0x0001}, {0x83F2, 0x002E}};
    uint32_t redData9[][2] = {{0x82F1, 0x0500}, {0xF995, 0x0501}, {0x31A2, 0x0502}, {0x796C, 0x0503}, {0x9728, 0x0504}, {0x9D85, 0x0423}, {0xD810, 0x0424}, {0x0F80, 0x0001}, {0x83F2, 0x002E}};
    uint32_t redDataHB[][2] = {{0x82F0, 0x0500}, {0xF195, 0x0501}, {0x31A2, 0x0502}, {0x7960, 0x0503}, {0x9728, 0x0504}, {0x9D85, 0x0423}, {0xD810, 0x0424}, {0x0F80, 0x0001}, {0x83F2, 0x002E}};

    if (id >= RTL8367C_EXTNO)
        return RT_ERR_OUT_OF_RANGE;

    if (mode >= EXT_END)
        return RT_ERR_OUT_OF_RANGE;

    if ((retVal = rtl8367c_setAsicReg(0x13C2, 0x0249)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_getAsicReg(0x1300, &regValue)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicReg(0x13C2, 0x0000)) != RT_ERR_OK)
        return retVal;

    type = 0;

    switch (regValue)
    {
    case 0x0276:
    case 0x0597:
    case 0x6367:
        type = 1;
        break;
    case 0x0652:
    case 0x6368:
        type = 2;
        break;
    case 0x0801:
    case 0x6511:
        type = 3;
        break;
    default:
        return RT_ERR_FAILED;
    }

    if (1 == type)
    {
        if ((mode == EXT_1000X_100FX) || (mode == EXT_1000X) || (mode == EXT_100FX))
        {
            if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_REG_TO_ECO4, 5, 1)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_REG_TO_ECO4, 7, 1)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_CHIP_RESET, RTL8367C_DW8051_RST_OFFSET, 1)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_MISCELLANEOUS_CONFIGURE0, RTL8367C_DW8051_EN_OFFSET, 1)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_DW8051_RDY, RTL8367C_ACS_IROM_ENABLE_OFFSET, 1)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_DW8051_RDY, RTL8367C_IROM_MSB_OFFSET, 0)) != RT_ERR_OK)
                return retVal;

            if (mode == EXT_1000X_100FX)
            {
                for (idx = 0; idx < FIBER2_AUTO_INIT_SIZE; idx++)
                {
                    if ((retVal = rtl8367c_setAsicReg(0xE000 + idx, (uint32_t)Fiber2_Auto[idx])) != RT_ERR_OK)
                        return retVal;
                }
            }

            if (mode == EXT_1000X)
            {
                for (idx = 0; idx < FIBER2_1G_INIT_SIZE; idx++)
                {
                    if ((retVal = rtl8367c_setAsicReg(0xE000 + idx, (uint32_t)Fiber2_1G[idx])) != RT_ERR_OK)
                        return retVal;
                }
            }

            if (mode == EXT_100FX)
            {
                for (idx = 0; idx < FIBER2_100M_INIT_SIZE; idx++)
                {
                    if ((retVal = rtl8367c_setAsicReg(0xE000 + idx, (uint32_t)Fiber2_100M[idx])) != RT_ERR_OK)
                        return retVal;
                }
            }

            if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_DW8051_RDY, RTL8367C_IROM_MSB_OFFSET, 0)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_DW8051_RDY, RTL8367C_ACS_IROM_ENABLE_OFFSET, 0)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_CHIP_RESET, RTL8367C_DW8051_RST_OFFSET, 0)) != RT_ERR_OK)
                return retVal;
        }

        if (mode == EXT_GMII)
        {
            if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_EXT0_RGMXF, RTL8367C_EXT0_RGTX_INV_OFFSET, 1)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_EXT1_RGMXF, RTL8367C_EXT1_RGTX_INV_OFFSET, 1)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_EXT_TXC_DLY, RTL8367C_EXT1_GMII_TX_DELAY_MASK, 5)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_EXT_TXC_DLY, RTL8367C_EXT0_GMII_TX_DELAY_MASK, 6)) != RT_ERR_OK)
                return retVal;
        }

        /* Serdes reset */
        if ((mode == EXT_TMII_MAC) || (mode == EXT_TMII_PHY))
        {
            if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_BYPASS_LINE_RATE, id, 1)) != RT_ERR_OK)
                return retVal;
        }
        else
        {
            if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_BYPASS_LINE_RATE, id, 0)) != RT_ERR_OK)
                return retVal;
        }

        if ((mode == EXT_SGMII) || (mode == EXT_HSGMII))
        {
            if (id != 1)
                return RT_ERR_PORT_ID;

            if ((retVal = rtl8367c_setAsicReg(0x13C0, 0x0249)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_getAsicReg(0x13C1, &option)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicReg(0x13C0, 0x0000)) != RT_ERR_OK)
                return retVal;
        }

        if (mode == EXT_SGMII)
        {
            if (option == 0)
            {
                for (i = 0; i <= 7; i++)
                {
                    if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_DATA, redData[i][0])) != RT_ERR_OK)
                        return retVal;

                    if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_ADR, redData[i][1])) != RT_ERR_OK)
                        return retVal;

                    if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_CMD, 0x00C0)) != RT_ERR_OK)
                        return retVal;
                }
            }
            else
            {
                for (i = 0; i <= 7; i++)
                {
                    if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_DATA, redDataSB[i][0])) != RT_ERR_OK)
                        return retVal;

                    if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_ADR, redDataSB[i][1])) != RT_ERR_OK)
                        return retVal;

                    if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_CMD, 0x00C0)) != RT_ERR_OK)
                        return retVal;
                }
            }
        }

        if (mode == EXT_HSGMII)
        {
            if (option == 0)
            {
                if ((retVal = rtl8367c_setAsicReg(0x13c2, 0x0249)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_getAsicReg(0x1301, &regValue)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicReg(0x13c2, 0x0000)) != RT_ERR_OK)
                    return retVal;

                if (((regValue & 0x00F0) >> 4) == 0x0001)
                {
                    for (i = 0; i <= 8; i++)
                    {
                        if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_DATA, redData1[i][0])) != RT_ERR_OK)
                            return retVal;

                        if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_ADR, redData1[i][1])) != RT_ERR_OK)
                            return retVal;

                        if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_CMD, 0x00C0)) != RT_ERR_OK)
                            return retVal;
                    }
                }
                else if (((regValue & 0x00F0) >> 4) == 0x0005)
                {
                    for (i = 0; i <= 8; i++)
                    {
                        if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_DATA, redData5[i][0])) != RT_ERR_OK)
                            return retVal;

                        if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_ADR, redData5[i][1])) != RT_ERR_OK)
                            return retVal;

                        if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_CMD, 0x00C0)) != RT_ERR_OK)
                            return retVal;
                    }
                }
                else if (((regValue & 0x00F0) >> 4) == 0x0006)
                {
                    for (i = 0; i <= 8; i++)
                    {
                        if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_DATA, redData6[i][0])) != RT_ERR_OK)
                            return retVal;

                        if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_ADR, redData6[i][1])) != RT_ERR_OK)
                            return retVal;

                        if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_CMD, 0x00C0)) != RT_ERR_OK)
                            return retVal;
                    }
                }
                else if (((regValue & 0x00F0) >> 4) == 0x0008)
                {
                    for (i = 0; i <= 8; i++)
                    {
                        if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_DATA, redData8[i][0])) != RT_ERR_OK)
                            return retVal;

                        if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_ADR, redData8[i][1])) != RT_ERR_OK)
                            return retVal;

                        if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_CMD, 0x00C0)) != RT_ERR_OK)
                            return retVal;
                    }
                }
                else if (((regValue & 0x00F0) >> 4) == 0x0009)
                {
                    for (i = 0; i <= 8; i++)
                    {
                        if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_DATA, redData9[i][0])) != RT_ERR_OK)
                            return retVal;

                        if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_ADR, redData9[i][1])) != RT_ERR_OK)
                            return retVal;

                        if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_CMD, 0x00C0)) != RT_ERR_OK)
                            return retVal;
                    }
                }
            }
            else
            {
                for (i = 0; i <= 8; i++)
                {
                    if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_DATA, redDataHB[i][0])) != RT_ERR_OK)
                        return retVal;

                    if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_ADR, redDataHB[i][1])) != RT_ERR_OK)
                        return retVal;

                    if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_CMD, 0x00C0)) != RT_ERR_OK)
                        return retVal;
                }
            }
        }

        /* Only one ext port should care SGMII setting */
        if (id == 1)
        {

            if (mode == EXT_SGMII)
            {
                if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_MAC8_SEL_SGMII_OFFSET, 1)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_MAC8_SEL_HSGMII_OFFSET, 0)) != RT_ERR_OK)
                    return retVal;
            }
            else if (mode == EXT_HSGMII)
            {
                if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_MAC8_SEL_SGMII_OFFSET, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_MAC8_SEL_HSGMII_OFFSET, 1)) != RT_ERR_OK)
                    return retVal;
            }
            else
            {

                if ((mode != EXT_1000X_100FX) && (mode != EXT_1000X) && (mode != EXT_100FX))
                {
                    if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_MAC8_SEL_SGMII_OFFSET, 0)) != RT_ERR_OK)
                        return retVal;

                    if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_MAC8_SEL_HSGMII_OFFSET, 0)) != RT_ERR_OK)
                        return retVal;
                }
            }
        }

        if (0 == id || 1 == id)
        {
            if ((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_DIGITAL_INTERFACE_SELECT, RTL8367C_SELECT_GMII_0_MASK << (id * RTL8367C_SELECT_GMII_1_OFFSET), mode)) != RT_ERR_OK)
                return retVal;
        }
        else
        {
            if ((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_DIGITAL_INTERFACE_SELECT_1, RTL8367C_SELECT_GMII_2_MASK, mode)) != RT_ERR_OK)
                return retVal;
        }

        /* Serdes not reset */
        if ((mode == EXT_SGMII) || (mode == EXT_HSGMII))
        {
            if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_DATA, 0x7106)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_ADR, 0x0003)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_CMD, 0x00C0)) != RT_ERR_OK)
                return retVal;
        }

        if ((mode == EXT_SGMII) || (mode == EXT_HSGMII))
        {
            if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_CHIP_RESET, RTL8367C_DW8051_RST_OFFSET, 1)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_MISCELLANEOUS_CONFIGURE0, RTL8367C_DW8051_EN_OFFSET, 1)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_DW8051_RDY, RTL8367C_ACS_IROM_ENABLE_OFFSET, 1)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_DW8051_RDY, RTL8367C_IROM_MSB_OFFSET, 0)) != RT_ERR_OK)
                return retVal;

            for (idx = 0; idx < SGMII_INIT_SIZE; idx++)
            {
                if ((retVal = rtl8367c_setAsicReg(0xE000 + idx, (uint32_t)Sgmii_Init[idx])) != RT_ERR_OK)
                    return retVal;
            }

            if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_DW8051_RDY, RTL8367C_IROM_MSB_OFFSET, 0)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_DW8051_RDY, RTL8367C_ACS_IROM_ENABLE_OFFSET, 0)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_CHIP_RESET, RTL8367C_DW8051_RST_OFFSET, 0)) != RT_ERR_OK)
                return retVal;
        }
    }
    else if (2 == type)
    {

        if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_CHIP_RESET, RTL8367C_DW8051_RST_OFFSET, 1)) != RT_ERR_OK)
            return retVal;

        if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_MISCELLANEOUS_CONFIGURE0, RTL8367C_DW8051_EN_OFFSET, 1)) != RT_ERR_OK)
            return retVal;

        if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_DW8051_RDY, RTL8367C_ACS_IROM_ENABLE_OFFSET, 1)) != RT_ERR_OK)
            return retVal;

        if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_DW8051_RDY, RTL8367C_IROM_MSB_OFFSET, 0)) != RT_ERR_OK)
            return retVal;

        for (idx = 0; idx < FIBER1_2_INIT_SIZE; idx++)
        {
            if ((retVal = rtl8367c_setAsicReg(0xE000 + idx, (uint32_t)Fiber1_2_Init[idx])) != RT_ERR_OK)
                return retVal;
        }

        if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_DW8051_RDY, RTL8367C_IROM_MSB_OFFSET, 0)) != RT_ERR_OK)
            return retVal;

        if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_DW8051_RDY, RTL8367C_ACS_IROM_ENABLE_OFFSET, 0)) != RT_ERR_OK)
            return retVal;

        if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_CHIP_RESET, RTL8367C_DW8051_RST_OFFSET, 0)) != RT_ERR_OK)
            return retVal;

        if ((mode == EXT_TMII_MAC) || (mode == EXT_TMII_PHY))
        {
            if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_BYPASS_LINE_RATE, id + 2, 1)) != RT_ERR_OK)
                return retVal;
        }
        else
        {
            if ((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_BYPASS_LINE_RATE, id + 2, 0)) != RT_ERR_OK)
                return retVal;
        }

        if (id == 1)
        {
            if (mode == EXT_HSGMII)
                return RT_ERR_PORT_ID;

            if (mode == EXT_SGMII)
            {

                if ((retVal = rtl8367c_setAsicRegBits(0x1305, 0xf0, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d92, 14, 1)) != RT_ERR_OK)
                    return retVal;
            }
            else if (mode == EXT_1000X || mode == EXT_100FX || mode == EXT_1000X_100FX)
            {

                if ((retVal = rtl8367c_setAsicRegBits(0x1305, 0xf0, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d92, 14, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x6210, 11, 0)) != RT_ERR_OK)
                    return retVal;
            }
            else
            {

                if ((retVal = rtl8367c_setAsicRegBits(0x1305, 0xf0, mode)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d92, 14, 0)) != RT_ERR_OK)
                    return retVal;
            }

            if ((retVal = rtl8367c_setAsicRegBits(0x1d92, 0x1f00, 0x1f)) != RT_ERR_OK)
                return retVal;
        }
        else if (id == 2)
        {
            if (mode == EXT_HSGMII)
            {
                if ((retVal = rtl8367c_setAsicReg(0x130, 7)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicReg(0x39f, 7)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicReg(0x3fa, 7)) != RT_ERR_OK)
                    return retVal;
            }
            else
            {
                if ((retVal = rtl8367c_setAsicReg(0x130, 1)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicReg(0x39f, 1)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicReg(0x3fa, 4)) != RT_ERR_OK)
                    return retVal;
            }

            if (mode == EXT_SGMII)
            {

                if ((retVal = rtl8367c_setAsicRegBits(0x13c3, 0xf, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d92, 6, 1)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d92, 7, 0)) != RT_ERR_OK)
                    return retVal;
            }
            else if (mode == EXT_HSGMII)
            {

                if ((retVal = rtl8367c_setAsicRegBits(0x13c3, 0xf, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d92, 6, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d92, 7, 1)) != RT_ERR_OK)
                    return retVal;
            }
            else if (mode == EXT_1000X || mode == EXT_100FX || mode == EXT_1000X_100FX)
            {

                if ((retVal = rtl8367c_setAsicRegBits(0x13c3, 0xf, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d92, 6, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d92, 7, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x6200, 11, 0)) != RT_ERR_OK)
                    return retVal;
            }
            else
            {

                if ((retVal = rtl8367c_setAsicRegBits(0x13c3, 0xf, mode)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d92, 6, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d92, 7, 0)) != RT_ERR_OK)
                    return retVal;
            }

            if ((retVal = rtl8367c_setAsicRegBits(0x1d92, 0x1f, 0x1f)) != RT_ERR_OK)
                return retVal;
        }

        if (mode == EXT_RGMII)
        {

            if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_PARA_LED_IO_EN3, 0)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_PARA_LED_IO_EN1, 0)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_PARA_LED_IO_EN2, 0)) != RT_ERR_OK)
                return retVal;

            if (id == 1)
            {

                if ((retVal = rtl8367c_setAsicRegBit(0x1303, 9, 1)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1303, 6, 1)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1303, 4, 1)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1303, 1, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1307, 3, 1)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBits(0x13f9, 0x38, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBits(0x1307, 0x7, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBits(0x1304, 0x7000, 4)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBits(0x13f9, 0x700, 4)) != RT_ERR_OK)
                    return retVal;
            }
            else if (id == 2)
            {

                if ((retVal = rtl8367c_setAsicRegBit(0x1303, 10, 1)) != RT_ERR_OK)
                    return retVal;

                /*drving 1*/
                if ((retVal = rtl8367c_setAsicRegBit(0x13e2, 2, 1)) != RT_ERR_OK)
                    return retVal;

                /*drving 1*/
                if ((retVal = rtl8367c_setAsicRegBit(0x13e2, 1, 1)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x13e2, 0, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x13c5, 3, 1)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBits(0x13f9, 0x1c0, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBits(0x13c5, 0x7, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBits(0x13e2, 0x1c0, 4)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBits(0x13e2, 0x38, 4)) != RT_ERR_OK)
                    return retVal;
            }
        }
        else if (mode == EXT_SGMII)
        {
            if (id == 1)
            {
                /*sds 1     reg 1    page 0x21     write value  0xec91*/
                if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_DATA, 0xec91)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_ADR, (0x21 << 5) | 1)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_CMD, 0x00C1)) != RT_ERR_OK)
                    return retVal;

                /*sds 1     reg 5    page 0x24     write value  0x5825*/
                if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_DATA, 0x5825)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_ADR, (0x24 << 5) | 5)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_CMD, 0x00C1)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBits(0x1d92, 0x1f00, 2)) != RT_ERR_OK)
                    return retVal;

                /*?????????????????*/
            }
            else if (id == 2)
            {
                /*sds 0     reg 0    page 0x28     write value  0x942c*/
                if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_DATA, 0x942c)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_ADR, (0x28 << 5) | 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_CMD, 0x00C0)) != RT_ERR_OK)
                    return retVal;

                /*sds 0     reg 0    page 0x24     write value  0x942c*/
                if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_DATA, 0x942c)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_ADR, (0x24 << 5) | 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_CMD, 0x00C0)) != RT_ERR_OK)
                    return retVal;

                /*sds 0     reg 5    page 0x21     write value  0x8dc3*/
                if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_DATA, 0x8dc3)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_ADR, (0x21 << 5) | 5)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_CMD, 0x00C0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBits(0x1d92, 0x1f, 2)) != RT_ERR_OK)
                    return retVal;

                /*?????????????????*/
            }
        }
        else if (mode == EXT_HSGMII)
        {
            if (id == 2)
            {
                /*sds 0     reg 0    page 0x28     write value  0x942c*/
                if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_DATA, 0x942c)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_ADR, (0x28 << 5) | 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_CMD, 0x00C0)) != RT_ERR_OK)
                    return retVal;

                /*sds 0     reg 0    page 0x24     write value  0x942c*/
                if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_DATA, 0x942c)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_ADR, (0x24 << 5) | 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_CMD, 0x00C0)) != RT_ERR_OK)
                    return retVal;

                /*sds 0     reg 5    page 0x21     write value  0x8dc3*/
                if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_DATA, 0x8dc3)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_ADR, (0x21 << 5) | 5)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_CMD, 0x00C0)) != RT_ERR_OK)
                    return retVal;

                /* optimizing HISGMII performance while RGMII used & */
                /*sds 0     reg 9     page 0x21     write value 0x3931*/
                if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_DATA, 0x3931)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_ADR, (0x21 << 5) | 9)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicReg(RTL8367C_REG_SDS_INDACS_CMD, 0x00C0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBits(0x1d92, 0x1f, 0x12)) != RT_ERR_OK)
                    return retVal;

                /*?????????????????*/
            }
        }
        else if (mode == EXT_1000X)
        {
            if (id == 1)
            {

                if ((retVal = rtl8367c_setAsicSdsReg(1, 1, 0x21, 0xec91)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_setAsicSdsReg(1, 5, 0x24, 0x5825)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_setAsicRegBits(0x1d92, 0x1f00, 4)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBits(0x1d92, 0x1f00, 0x1f)) != RT_ERR_OK)
                    return retVal;

                /*patch speed change sds1 1000M*/
                if ((retVal = rtl8367c_getAsicSdsReg(1, 4, 0, &regValue)) != RT_ERR_OK)
                    return retVal;
                regValue &= 0xFFFF0FFF;
                regValue |= 0x9000;
                if ((retVal = rtl8367c_setAsicSdsReg(1, 4, 0, regValue)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_getAsicSdsReg(1, 0, 2, &regValue)) != RT_ERR_OK)
                    return retVal;
                regValue &= 0xFFFFdFFF;
                regValue |= 0x40;
                if ((retVal = rtl8367c_setAsicSdsReg(1, 0, 2, regValue)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_getAsicSdsReg(1, 4, 0, &regValue)) != RT_ERR_OK)
                    return retVal;
                regValue &= 0xFFFFEFFF;
                if ((retVal = rtl8367c_setAsicSdsReg(1, 4, 0, regValue)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBits(0x1d92, 0x1f00, 4)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_setAsicRegBits(0x1d92, 0x6000, 0)) != RT_ERR_OK)
                    return retVal;
            }
            else if (id == 2)
            {
                if ((retVal = rtl8367c_setAsicSdsReg(0, 0, 0x28, 0x942c)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_setAsicSdsReg(0, 0, 0x24, 0x942c)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_setAsicSdsReg(0, 5, 0x21, 0x8dc3)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBits(0x1d92, 0x1f, 4)) != RT_ERR_OK)
                    return retVal;

                /*patch speed change sds0 1000M*/
                if ((retVal = rtl8367c_setAsicRegBits(0x1d92, 0x1f, 0x1f)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_getAsicSdsReg(0, 4, 0, &regValue)) != RT_ERR_OK)
                    return retVal;
                regValue &= 0xFFFF0FFF;
                regValue |= 0x9000;
                if ((retVal = rtl8367c_setAsicSdsReg(0, 4, 0, regValue)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_getAsicSdsReg(0, 0, 2, &regValue)) != RT_ERR_OK)
                    return retVal;
                regValue &= 0xFFFFDFFF;
                regValue |= 0x40;
                if ((retVal = rtl8367c_setAsicSdsReg(0, 0, 2, regValue)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_getAsicSdsReg(0, 4, 0, &regValue)) != RT_ERR_OK)
                    return retVal;
                regValue &= 0xFFFFEFFF;
                if ((retVal = rtl8367c_setAsicSdsReg(0, 4, 0, regValue)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBits(0x1d92, 0x1f, 4)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_setAsicRegBits(0x1d92, 0xe0, 0)) != RT_ERR_OK)
                    return retVal;
            }
        }
        else if (mode == EXT_100FX)
        {
            if (id == 1)
            {
                if ((retVal = rtl8367c_setAsicSdsReg(1, 1, 0x21, 0xec91)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_setAsicSdsReg(1, 5, 0x24, 0x5825)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_setAsicRegBits(0x1d92, 0x1f00, 5)) != RT_ERR_OK)
                    return retVal;

                /*patch speed change sds1 100M*/
                if ((retVal = rtl8367c_setAsicRegBits(0x1d92, 0x1f00, 0x1f)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_getAsicSdsReg(1, 4, 0, &regValue)) != RT_ERR_OK)
                    return retVal;
                regValue &= 0xFFFF0FFF;
                regValue |= 0xb000;
                if ((retVal = rtl8367c_setAsicSdsReg(1, 4, 0, regValue)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_getAsicSdsReg(1, 0, 2, &regValue)) != RT_ERR_OK)
                    return retVal;
                regValue &= 0xFFFFFFBF;
                regValue |= 0x2000;
                if ((retVal = rtl8367c_setAsicSdsReg(1, 0, 2, regValue)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_getAsicSdsReg(1, 4, 0, &regValue)) != RT_ERR_OK)
                    return retVal;
                regValue &= 0xFFFFEFFF;
                if ((retVal = rtl8367c_setAsicSdsReg(1, 4, 0, regValue)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBits(0x1d92, 0x1f00, 5)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_setAsicRegBits(0x1d92, 0x6000, 0)) != RT_ERR_OK)
                    return retVal;
            }
            else if (id == 2)
            {
                if ((retVal = rtl8367c_setAsicSdsReg(0, 0, 0x28, 0x942c)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_setAsicSdsReg(0, 0, 0x24, 0x942c)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_setAsicSdsReg(0, 5, 0x21, 0x8dc3)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_setAsicRegBits(0x1d92, 0x1f, 5)) != RT_ERR_OK)
                    return retVal;

                /*patch speed change sds0 100M*/
                if ((retVal = rtl8367c_setAsicRegBits(0x1d92, 0x1f, 0x1f)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_getAsicSdsReg(0, 4, 0, &regValue)) != RT_ERR_OK)
                    return retVal;
                regValue &= 0xFFFF0FFF;
                regValue |= 0xb000;
                if ((retVal = rtl8367c_setAsicSdsReg(0, 4, 0, regValue)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_getAsicSdsReg(0, 0, 2, &regValue)) != RT_ERR_OK)
                    return retVal;
                regValue &= 0xFFFFFFBF;
                regValue |= 0x2000;
                if ((retVal = rtl8367c_setAsicSdsReg(0, 0, 2, regValue)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_getAsicSdsReg(0, 4, 0, &regValue)) != RT_ERR_OK)
                    return retVal;
                regValue &= 0xFFFFEFFF;
                if ((retVal = rtl8367c_setAsicSdsReg(0, 4, 0, regValue)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBits(0x1d92, 0x1f, 5)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_setAsicRegBits(0x1d92, 0xe0, 0)) != RT_ERR_OK)
                    return retVal;
            }
        }
        else if (mode == EXT_1000X_100FX)
        {
            if (id == 1)
            {
                if ((retVal = rtl8367c_setAsicSdsReg(1, 1, 0x21, 0xec91)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_setAsicSdsReg(1, 5, 0x24, 0x5825)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_setAsicSdsReg(1, 13, 0, 0x4616)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_setAsicSdsReg(1, 1, 0, 0xf20)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_setAsicRegBits(0x1d92, 0x1f00, 7)) != RT_ERR_OK)
                    return retVal;
            }
            else if (id == 2)
            {
                if ((retVal = rtl8367c_setAsicSdsReg(0, 0, 0x28, 0x942c)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_setAsicSdsReg(0, 0, 0x24, 0x942c)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_setAsicSdsReg(0, 5, 0x21, 0x8dc3)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_setAsicSdsReg(0, 13, 0, 0x4616)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_setAsicSdsReg(0, 1, 0, 0xf20)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_setAsicRegBits(0x1d92, 0x1f, 7)) != RT_ERR_OK)
                    return retVal;
            }
        }
    }
    else if (3 == type)
    {

        /*restore patch, by designer. patch Tx FIFO issue, when not HSGMII 2.5G mode
         #sds0, page 1, reg 1, bit4=0*/
        if ((retVal = rtl8367c_getAsicSdsReg(0, 1, 1, &regValue)) != RT_ERR_OK)
            return retVal;
        regValue &= 0xFFFFFFEF;
        if ((retVal = rtl8367c_setAsicSdsReg(0, 1, 1, regValue)) != RT_ERR_OK)
            return retVal;

        /*set for mac 6*/
        if (1 == id)
        {

            if ((retVal = rtl8367c_setAsicReg(0x137c, 0x1000)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_getAsicRegBit(0x1d9d, 6, &reg_data)) != RT_ERR_OK)
                return retVal;
            while (reg_data == 0)
            {
                if ((retVal = rtl8367c_getAsicRegBit(0x1d9d, 6, &reg_data)) != RT_ERR_OK)
                    return retVal;
            }

            if (mode == EXT_SGMII)
            {

                if ((retVal = rtl8367c_getAsicRegBit(0x1d3d, 10, &reg_data)) != RT_ERR_OK)
                    return retVal;
                if (reg_data == 0)
                {
                    if ((retVal = rtl8367c_setAsicRegBits(0x1305, 0xf0, 0)) != RT_ERR_OK)
                        return retVal;
                }

                if ((retVal = rtl8367c_setAsicRegBit(0x3f7, 1, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d41, 5, 0)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_setAsicRegBit(0x1d41, 7, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d95, 1, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d95, 0, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d11, 9, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d11, 11, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d95, 13, 1)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBits(0x1d95, 0x1f00, 0x1f)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d11, 6, 1)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBits(0x1d95, 0x1f00, 0x2)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d95, 2, 0)) != RT_ERR_OK)
                    return retVal;
            }
            else if (mode == EXT_HSGMII)
            {

                /*restore patch, by designer. patch Tx FIFO issue, when  HSGMII 2.5G mode
                 #sds0, page 1, reg 1, bit4=1*/
                if ((retVal = rtl8367c_getAsicSdsReg(0, 1, 1, &regValue)) != RT_ERR_OK)
                    return retVal;
                regValue |= 0x10;
                if ((retVal = rtl8367c_setAsicSdsReg(0, 1, 1, regValue)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_getAsicRegBit(0x1d3d, 10, &reg_data)) != RT_ERR_OK)
                    return retVal;
                if (reg_data == 0)
                {
                    if ((retVal = rtl8367c_setAsicRegBits(0x1305, 0xf0, 0)) != RT_ERR_OK)
                        return retVal;
                }

                if ((retVal = rtl8367c_setAsicRegBit(0x3f7, 1, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d41, 5, 0)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_setAsicRegBit(0x1d41, 7, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d95, 1, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d95, 0, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d11, 9, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d11, 11, 1)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d11, 6, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicReg(0xd0, 7)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_setAsicReg(0x399, 7)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_setAsicReg(0x3fa, 7)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d95, 13, 1)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBits(0x1d95, 0x1f00, 0x12)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d95, 2, 0)) != RT_ERR_OK)
                    return retVal;
            }
            else if (mode == EXT_1000X)
            {

                if ((retVal = rtl8367c_getAsicSdsReg(0, 2, 0, &reg_data)) != RT_ERR_OK)
                    return retVal;
                reg_data &= 0xFFFFFCFF;
                if ((retVal = rtl8367c_setAsicSdsReg(0, 2, 0, reg_data)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_getAsicRegBit(0x1d3d, 10, &reg_data)) != RT_ERR_OK)
                    return retVal;
                if (reg_data == 0)
                {
                    if ((retVal = rtl8367c_setAsicRegBits(0x1305, 0xf0, 0)) != RT_ERR_OK)
                        return retVal;
                }

                if ((retVal = rtl8367c_setAsicRegBit(0x3f7, 1, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicReg(0x1d11, 0x1500)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBits(0x1d95, 0x1f00, 0x1f)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_setAsicRegBit(0x1d95, 13, 1)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_setAsicRegBits(0x1d95, 3, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicReg(0x13eb, 0x15bb)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicReg(0x13e7, 0xc)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d41, 5, 1)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_setAsicRegBit(0x1d41, 7, 1)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d11, 11, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d11, 6, 1)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBits(0x1d95, 0x1f00, 0x4)) != RT_ERR_OK)
                    return retVal;
            }
            else if (mode == EXT_100FX)
            {

                if ((retVal = rtl8367c_getAsicSdsReg(0, 2, 0, &reg_data)) != RT_ERR_OK)
                    return retVal;
                reg_data &= 0xFFFFFCFF;
                if ((retVal = rtl8367c_setAsicSdsReg(0, 2, 0, reg_data)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_getAsicRegBit(0x1d3d, 10, &reg_data)) != RT_ERR_OK)
                    return retVal;
                if (reg_data == 0)
                {
                    if ((retVal = rtl8367c_setAsicRegBits(0x1305, 0xf0, 0)) != RT_ERR_OK)
                        return retVal;
                }

                if ((retVal = rtl8367c_setAsicRegBit(0x3f7, 1, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicReg(0x1d11, 0x1500)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBits(0x1d95, 0x1f00, 0x1f)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_setAsicRegBit(0x1d95, 13, 1)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_setAsicRegBits(0x1d95, 3, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicReg(0x13eb, 0x15bb)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicReg(0x13e7, 0xc)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d41, 5, 1)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_setAsicRegBit(0x1d41, 7, 1)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d11, 11, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d11, 6, 1)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBits(0x1d95, 0x1f00, 0x5)) != RT_ERR_OK)
                    return retVal;
            }
            else if (mode == EXT_1000X_100FX)
            {
                /* 0 2 0  bit 8~9  set 0, force n-way*/
                if ((retVal = rtl8367c_getAsicSdsReg(0, 2, 0, &reg_data)) != RT_ERR_OK)
                    return retVal;
                reg_data &= 0xFFFFFCFF;
                if ((retVal = rtl8367c_setAsicSdsReg(0, 2, 0, reg_data)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_getAsicRegBit(0x1d3d, 10, &reg_data)) != RT_ERR_OK)
                    return retVal;
                if (reg_data == 0)
                {
                    if ((retVal = rtl8367c_setAsicRegBits(0x1305, 0xf0, 0)) != RT_ERR_OK)
                        return retVal;
                }

                if ((retVal = rtl8367c_setAsicRegBit(0x3f7, 1, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicReg(0x1d11, 0x1500)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBits(0x1d95, 0x1f00, 0x1f)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_setAsicRegBit(0x1d95, 13, 0)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_setAsicRegBits(0x1d95, 3, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicReg(0x13eb, 0x15bb)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicReg(0x13e7, 0xc)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d41, 5, 1)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_setAsicRegBit(0x1d41, 7, 1)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d11, 11, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d11, 6, 1)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBits(0x1d95, 0x1f00, 0x7)) != RT_ERR_OK)
                    return retVal;
            }
            else if (mode < EXT_SGMII)
            {
                if ((retVal = rtl8367c_setAsicRegBit(0x1d3d, 10, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x3f7, 1, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d11, 11, 0)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_setAsicRegBit(0x1d11, 6, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d41, 5, 0)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_setAsicRegBit(0x1d41, 7, 0)) != RT_ERR_OK)
                    return retVal;

                if (mode < EXT_GMII)
                {
                    /* set mac6 mode*/
                    if ((retVal = rtl8367c_setAsicRegBits(0x1305, 0xf0, mode)) != RT_ERR_OK)
                        return retVal;
                }
                else if (mode == EXT_RMII_MAC)
                {

                    if ((retVal = rtl8367c_setAsicRegBits(0x1305, 0xf0, 7)) != RT_ERR_OK)
                        return retVal;
                }
                else if (mode == EXT_RMII_PHY)
                {
                    if ((retVal = rtl8367c_setAsicRegBits(0x1305, 0xf0, 8)) != RT_ERR_OK)
                        return retVal;
                }

                if ((mode == EXT_TMII_MAC) || (mode == EXT_TMII_PHY))
                {
                    if ((retVal = rtl8367c_setAsicRegBit(0x3f7, 1, 1)) != RT_ERR_OK)
                        return retVal;
                }
            }
        }
        else if (2 == id)
        {

            /*force port7 linkdown*/
            if ((retVal = rtl8367c_setAsicReg(0x137d, 0x1000)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_getAsicRegBit(0x1d9d, 7, &reg_data)) != RT_ERR_OK)
                return retVal;
            while (reg_data == 0)
            {
                if ((retVal = rtl8367c_getAsicRegBit(0x1d9d, 7, &reg_data)) != RT_ERR_OK)
                    return retVal;
            }

            if (mode == EXT_SGMII)
            {

                if ((retVal = rtl8367c_setAsicRegBits(0x13c3, 0xf, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicReg(0x13c4, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x3f7, 2, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d41, 5, 0)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_setAsicRegBit(0x1d41, 7, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBits(0x1d95, 3, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d11, 11, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d11, 6, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d95, 13, 1)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBits(0x1d95, 0x1f00, 0x1f)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d95, 0, 1)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBits(0x1d95, 0x1f00, 0x2)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d95, 2, 0)) != RT_ERR_OK)
                    return retVal;
            }
            else if (mode == EXT_1000X)
            {

                if ((retVal = rtl8367c_setAsicRegBits(0x13c3, 0xf, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicReg(0x13c4, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_getAsicSdsReg(0, 2, 0, &reg_data)) != RT_ERR_OK)
                    return retVal;
                reg_data &= 0xFFFFFCFF;
                if ((retVal = rtl8367c_setAsicSdsReg(0, 2, 0, reg_data)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x3f7, 2, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicReg(0x1d11, 0x1500)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d95, 13, 1)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBits(0x1d95, 0x1f00, 0x1f)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d41, 5, 0)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_setAsicRegBit(0x1d41, 7, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBits(0x1d95, 3, 3)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBits(0x1d95, 0x1f00, 0x4)) != RT_ERR_OK)
                    return retVal;
            }
            else if (mode == EXT_100FX)
            {

                if ((retVal = rtl8367c_setAsicRegBits(0x13c3, 0xf, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicReg(0x13c4, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_getAsicSdsReg(0, 2, 0, &reg_data)) != RT_ERR_OK)
                    return retVal;
                reg_data &= 0xFFFFFCFF;
                if ((retVal = rtl8367c_setAsicSdsReg(0, 2, 0, reg_data)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x3f7, 2, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicReg(0x1d11, 0x1500)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d95, 13, 1)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBits(0x1d95, 0x1f00, 0x1f)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d41, 5, 0)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_setAsicRegBit(0x1d41, 7, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBits(0x1d95, 3, 3)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBits(0x1d95, 0x1f00, 0x5)) != RT_ERR_OK)
                    return retVal;
            }
            else if (mode == EXT_1000X_100FX)
            {
                /*  disable mac7 MII/TMM/RMII/GMII/RGMII mode, mode_ext2 = disable  */
                if ((retVal = rtl8367c_setAsicRegBits(0x13c3, 0xf, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicReg(0x13c4, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_getAsicSdsReg(0, 2, 0, &reg_data)) != RT_ERR_OK)
                    return retVal;
                reg_data &= 0xFFFFFCFF;
                if ((retVal = rtl8367c_setAsicSdsReg(0, 2, 0, reg_data)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x3f7, 2, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicReg(0x1d11, 0x1500)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d95, 13, 1)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBits(0x1d95, 0x1f00, 0x1f)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d41, 5, 0)) != RT_ERR_OK)
                    return retVal;
                if ((retVal = rtl8367c_setAsicRegBit(0x1d41, 7, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBits(0x1d95, 3, 3)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBits(0x1d95, 0x1f00, 0x7)) != RT_ERR_OK)
                    return retVal;
            }
            else if (mode < EXT_SGMII)
            {
                if ((retVal = rtl8367c_setAsicRegBit(0x1d3d, 10, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x3f7, 2, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicReg(0x1d95, 0x1f00)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBits(0x1d95, 3, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBits(0x13c3, 0xf, mode)) != RT_ERR_OK)
                    return retVal;

                if ((mode == EXT_TMII_MAC) || (mode == EXT_TMII_PHY))
                {
                    if ((retVal = rtl8367c_setAsicRegBit(0x3f7, 2, 1)) != RT_ERR_OK)
                        return retVal;
                }
            }
            else if ((mode < EXT_END) && (mode > EXT_100FX))
            {
                if ((retVal = rtl8367c_setAsicRegBits(0x13C3, 0xf, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x3f7, 2, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBits(0x1d95, 3, 0)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_setAsicRegBit(0x1d3d, 10, 1)) != RT_ERR_OK)
                    return retVal;

                if ((retVal = rtl8367c_getAsicRegBit(0x1d11, 11, &reg_data)) != RT_ERR_OK)
                    return retVal;
                if (reg_data == 0)
                {
                    if ((retVal = rtl8367c_setAsicRegBit(0x1d11, 6, 1)) != RT_ERR_OK)
                        return retVal;
                }

                if (mode < EXT_RMII_MAC_2)
                {
                    if ((retVal = rtl8367c_setAsicRegBits(0x1305, 0xf0, (mode - 13))) != RT_ERR_OK)
                        return retVal;
                }
                else
                {
                    if ((retVal = rtl8367c_setAsicRegBits(0x1305, 0xf0, (mode - 12))) != RT_ERR_OK)
                        return retVal;
                }

                if ((mode == EXT_TMII_MAC_2) || (mode == EXT_TMII_PHY_2))
                {
                    if ((retVal = rtl8367c_setAsicRegBit(0x3f7, 2, 1)) != RT_ERR_OK)
                        return retVal;
                }
            }
        }
    }
    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_getAsicPortExtMode
 * Description:
 *      Get external interface mode configuration
 * Input:
 *      id      - external interface id (0~1)
 *      pMode   - external interface mode
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK           - Success
 *      RT_ERR_SMI          - SMI access error
 *      RT_ERR_OUT_OF_RANGE - input parameter out of range
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicPortExtMode(uint32_t id, uint32_t *pMode)
{
    int32_t retVal;
    uint32_t regData, regValue, type;

    if (id >= RTL8367C_EXTNO)
        return RT_ERR_OUT_OF_RANGE;

    if ((retVal = rtl8367c_setAsicReg(0x13C2, 0x0249)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_getAsicReg(0x1300, &regValue)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicReg(0x13C2, 0x0000)) != RT_ERR_OK)
        return retVal;

    type = 0;

    switch (regValue)
    {
    case 0x0276:
    case 0x0597:
    case 0x6367:
        type = 1;
        break;
    case 0x0652:
    case 0x6368:
        type = 2;
        break;
    case 0x0801:
    case 0x6511:
        type = 3;
        break;
    default:
        return RT_ERR_FAILED;
    }

    if (1 == type)
    {

        if (1 == id)
        {
            if ((retVal = rtl8367c_getAsicRegBit(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_MAC8_SEL_SGMII_OFFSET, &regData)) != RT_ERR_OK)
                return retVal;

            if (1 == regData)
            {
                *pMode = EXT_SGMII;
                return RT_ERR_OK;
            }

            if ((retVal = rtl8367c_getAsicRegBit(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_MAC8_SEL_HSGMII_OFFSET, &regData)) != RT_ERR_OK)
                return retVal;

            if (1 == regData)
            {
                *pMode = EXT_HSGMII;
                return RT_ERR_OK;
            }
        }

        if (0 == id || 1 == id)
            return rtl8367c_getAsicRegBits(RTL8367C_REG_DIGITAL_INTERFACE_SELECT, RTL8367C_SELECT_GMII_0_MASK << (id * RTL8367C_SELECT_GMII_1_OFFSET), pMode);
        else
            return rtl8367c_getAsicRegBits(RTL8367C_REG_DIGITAL_INTERFACE_SELECT_1, RTL8367C_SELECT_GMII_2_MASK, pMode);
    }
    else if (2 == type)
    {
        if (1 == id)
        {
            if ((retVal = rtl8367c_getAsicReg(0x1d92, &regData)) != RT_ERR_OK)
                return retVal;

            if (regData & 0x4000)
            {
                *pMode = EXT_SGMII;
                return RT_ERR_OK;
            }

            else if (((regData >> 8) & 0x1f) == 4)
            {
                *pMode = EXT_1000X;
                return RT_ERR_OK;
            }
            else if (((regData >> 8) & 0x1f) == 5)
            {
                *pMode = EXT_100FX;
                return RT_ERR_OK;
            }
            else if (((regData >> 8) & 0x1f) == 7)
            {
                *pMode = EXT_1000X_100FX;
                return RT_ERR_OK;
            }

            return rtl8367c_getAsicRegBits(0x1305, 0xf0, pMode);
        }
        else if (2 == id)
        {
            if ((retVal = rtl8367c_getAsicReg(0x1d92, &regData)) != RT_ERR_OK)
                return retVal;

            if (regData & 0x40)
            {
                *pMode = EXT_SGMII;
                return RT_ERR_OK;
            }
            else if (regData & 0x80)
            {
                *pMode = EXT_HSGMII;
                return RT_ERR_OK;
            }
            else if ((regData & 0x1f) == 4)
            {
                *pMode = EXT_1000X;
                return RT_ERR_OK;
            }
            else if ((regData & 0x1f) == 5)
            {
                *pMode = EXT_100FX;
                return RT_ERR_OK;
            }
            else if ((regData & 0x1f) == 7)
            {
                *pMode = EXT_1000X_100FX;
                return RT_ERR_OK;
            }

            return rtl8367c_getAsicRegBits(0x1305, 0xf, pMode);
        }
    }
    else if (3 == type)
    {
        if (1 == id)
        {
            /* SDS_CFG_NEW */
            if ((retVal = rtl8367c_getAsicReg(0x1d95, &regData)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_getAsicReg(0x1d41, &regValue)) != RT_ERR_OK)
                return retVal;

            if ((regValue & 0xa0) == 0xa0)
            {

                regData = regData >> 8;
                if ((regData & 0x1f) == 4)
                {
                    *pMode = EXT_1000X;
                    return RT_ERR_OK;
                }
                else if ((regData & 0x1f) == 5)
                {
                    *pMode = EXT_100FX;
                    return RT_ERR_OK;
                }
                else if ((regData & 0x1f) == 7)
                {
                    *pMode = EXT_1000X_100FX;
                    return RT_ERR_OK;
                }
            }

            if ((retVal = rtl8367c_getAsicReg(0x1d11, &regData)) != RT_ERR_OK)
                return retVal;

            /* check cfg_mac6_sel_sgmii */
            if ((regData >> 6) & 1)
            {
                *pMode = EXT_SGMII;
                return RT_ERR_OK;
            }
            else if ((regData >> 11) & 1)
            {
                *pMode = EXT_HSGMII;
                return RT_ERR_OK;
            }
            else
            {
                /* check port6 MAC mode */
                if ((retVal = rtl8367c_getAsicRegBits(0x1305, 0xf0, &regData)) != RT_ERR_OK)
                    return retVal;

                *pMode = regData;
                return RT_ERR_OK;
            }
        }
        else if (2 == id)
        {
            if ((retVal = rtl8367c_getAsicReg(0x1d95, &regData)) != RT_ERR_OK)
                return retVal;

            if (((regData & 0x3) == 3) && (((regData >> 8) & 0x1f) == 0x4))
            {
                *pMode = EXT_1000X;
                return RT_ERR_OK;
            }
            else if (((regData & 0x3) == 3) && (((regData >> 8) & 0x1f) == 0x5))
            {
                *pMode = EXT_100FX;
                return RT_ERR_OK;
            }
            else if (((regData & 0x3) == 3) && (((regData >> 8) & 0x1f) == 0x7))
            {
                *pMode = EXT_1000X_100FX;
                return RT_ERR_OK;
            }
            else if (regData & 1)
            {
                *pMode = EXT_SGMII;
                return RT_ERR_OK;
            }
            else
            {

                if ((retVal = rtl8367c_getAsicRegBits(0x13c3, 0xf, &regData)) != RT_ERR_OK)
                    return retVal;

                *pMode = regData;

                return RT_ERR_OK;
            }
        }
    }

    return RT_ERR_OK;
}
/* Function Name:
 *      rtl8367c_getAsicPortForceLinkExt
 * Description:
 *      Get external interface force linking configuration
 * Input:
 *      id          - external interface id (0~1)
 *      pPortAbility - port ability configuration
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK           - Success
 *      RT_ERR_SMI          - SMI access error
 *      RT_ERR_OUT_OF_RANGE - input parameter out of range
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicPortForceLinkExt(uint32_t id, rtl8367c_port_ability_t *pPortAbility)
{
    uint32_t reg_data, regValue, type;
    uint32_t sgmiiSel;
    uint32_t hsgmiiSel;
    uint32_t Mode;
    int32_t retVal;

    if (id >= RTL8367C_EXTNO)
        return RT_ERR_OUT_OF_RANGE;

    if ((retVal = rtl8367c_setAsicReg(0x13C2, 0x0249)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_getAsicReg(0x1300, &regValue)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicReg(0x13C2, 0x0000)) != RT_ERR_OK)
        return retVal;

    type = 0;

    switch (regValue)
    {
    case 0x0276:
    case 0x0597:
    case 0x6367:
        type = 1;
        break;
    case 0x0652:
    case 0x6368:
        type = 2;
        break;
    case 0x0801:
    case 0x6511:
        type = 3;
        break;
    default:
        return RT_ERR_FAILED;
    }

    if (1 == type)
    {
        if (1 == id)
        {
            if ((retVal = rtl8367c_getAsicRegBit(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_MAC8_SEL_SGMII_OFFSET, &sgmiiSel)) != RT_ERR_OK)
                return retVal;

            if ((retVal = rtl8367c_getAsicRegBit(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_MAC8_SEL_HSGMII_OFFSET, &hsgmiiSel)) != RT_ERR_OK)
                return retVal;

            if ((sgmiiSel == 1) || (hsgmiiSel == 1))
            {
                memset(pPortAbility, 0x00, sizeof(rtl8367c_port_ability_t));
                pPortAbility->forcemode = 1;

                if ((retVal = rtl8367c_getAsicRegBit(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_SGMII_FDUP_OFFSET, &reg_data)) != RT_ERR_OK)
                    return retVal;

                pPortAbility->duplex = reg_data;

                if ((retVal = rtl8367c_getAsicRegBits(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_SGMII_SPD_MASK, &reg_data)) != RT_ERR_OK)
                    return retVal;

                pPortAbility->speed = reg_data;

                if ((retVal = rtl8367c_getAsicRegBit(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_SGMII_LINK_OFFSET, &reg_data)) != RT_ERR_OK)
                    return retVal;

                pPortAbility->link = reg_data;

                if ((retVal = rtl8367c_getAsicRegBit(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_SGMII_TXFC_OFFSET, &reg_data)) != RT_ERR_OK)
                    return retVal;

                pPortAbility->txpause = reg_data;

                if ((retVal = rtl8367c_getAsicRegBit(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_SGMII_RXFC_OFFSET, &reg_data)) != RT_ERR_OK)
                    return retVal;

                pPortAbility->rxpause = reg_data;

                return RT_ERR_OK;
            }
        }

        if (0 == id || 1 == id)
            retVal = rtl8367c_getAsicReg(RTL8367C_REG_DIGITAL_INTERFACE0_FORCE + id, &reg_data);
        else
            retVal = rtl8367c_getAsicReg(RTL8367C_REG_DIGITAL_INTERFACE2_FORCE, &reg_data);

        if (retVal != RT_ERR_OK)
            return retVal;

        pPortAbility->forcemode = (reg_data >> 12) & 0x0001;
        pPortAbility->mstfault = (reg_data >> 9) & 0x0001;
        pPortAbility->mstmode = (reg_data >> 8) & 0x0001;
        pPortAbility->nway = (reg_data >> 7) & 0x0001;
        pPortAbility->txpause = (reg_data >> 6) & 0x0001;
        pPortAbility->rxpause = (reg_data >> 5) & 0x0001;
        pPortAbility->link = (reg_data >> 4) & 0x0001;
        pPortAbility->duplex = (reg_data >> 2) & 0x0001;
        pPortAbility->speed = reg_data & 0x0003;
    }
    else if (2 == type)
    {
        if (id == 1)
        {
            if ((retVal = rtl8367c_getAsicReg(0x1311, &reg_data)) != RT_ERR_OK)
                return retVal;

            pPortAbility->forcemode = (reg_data >> 12) & 1;
            pPortAbility->duplex = (reg_data >> 2) & 1;
            pPortAbility->link = (reg_data >> 4) & 1;
            pPortAbility->speed = reg_data & 3;
            pPortAbility->rxpause = (reg_data >> 5) & 1;
            pPortAbility->txpause = (reg_data >> 6) & 1;
        }
        else if (2 == id)
        {
            if ((retVal = rtl8367c_getAsicReg(0x13c4, &reg_data)) != RT_ERR_OK)
                return retVal;

            pPortAbility->forcemode = (reg_data >> 12) & 1;
            pPortAbility->duplex = (reg_data >> 2) & 1;
            pPortAbility->link = (reg_data >> 4) & 1;
            pPortAbility->speed = reg_data & 3;
            pPortAbility->rxpause = (reg_data >> 5) & 1;
            pPortAbility->txpause = (reg_data >> 6) & 1;
        }
    }
    else if (3 == type)
    {
        if (id == 1)
        {

            if ((retVal = rtl8367c_getAsicPortExtMode(id, &Mode)) != RT_ERR_OK)
                return retVal;
            if (Mode < EXT_SGMII)
            {

                if ((retVal = rtl8367c_getAsicReg(0x1311, &reg_data)) != RT_ERR_OK)
                    return retVal;

                pPortAbility->forcemode = (reg_data >> 12) & 1;
                pPortAbility->duplex = (reg_data >> 2) & 1;
                pPortAbility->link = (reg_data >> 4) & 1;
                pPortAbility->speed = reg_data & 3;
                pPortAbility->rxpause = (reg_data >> 5) & 1;
                pPortAbility->txpause = (reg_data >> 6) & 1;
            }
            else if (Mode < EXT_1000X_100FX)
            {
                if ((retVal = rtl8367c_getAsicReg(0x1d11, &reg_data)) != RT_ERR_OK)
                    return retVal;

                // pPortAbility->forcemode = (reg_data >> 12) & 1;
                pPortAbility->duplex = (reg_data >> 10) & 1;
                pPortAbility->link = (reg_data >> 9) & 1;
                pPortAbility->speed = (reg_data >> 7) & 3;
                pPortAbility->rxpause = (reg_data >> 14) & 1;
                pPortAbility->txpause = (reg_data >> 13) & 1;
            }
            else if (Mode < EXT_RGMII_2)
            {
                if ((retVal = rtl8367c_getAsicReg(0x1358, &reg_data)) != RT_ERR_OK)
                    return retVal;

                // pPortAbility->forcemode = (reg_data >> 12) & 1;
                pPortAbility->duplex = (reg_data >> 2) & 1;
                pPortAbility->link = (reg_data >> 4) & 1;
                pPortAbility->speed = reg_data & 3;
                pPortAbility->rxpause = (reg_data >> 5) & 1;
                pPortAbility->txpause = (reg_data >> 6) & 1;
            }
        }
        else if (2 == id)
        {
            if ((retVal = rtl8367c_getAsicPortExtMode(id, &Mode)) != RT_ERR_OK)
                return retVal;
            if (Mode < EXT_SGMII)
            {

                if ((retVal = rtl8367c_getAsicReg(0x13c4, &reg_data)) != RT_ERR_OK)
                    return retVal;

                pPortAbility->forcemode = (reg_data >> 12) & 1;
                pPortAbility->duplex = (reg_data >> 2) & 1;
                pPortAbility->link = (reg_data >> 4) & 1;
                pPortAbility->speed = reg_data & 3;
                pPortAbility->rxpause = (reg_data >> 5) & 1;
                pPortAbility->txpause = (reg_data >> 6) & 1;
            }
            else if (Mode < EXT_1000X_100FX)
            {
                if ((retVal = rtl8367c_getAsicReg(0x1d11, &reg_data)) != RT_ERR_OK)
                    return retVal;

                // pPortAbility->forcemode = (reg_data >> 12) & 1;
                pPortAbility->duplex = (reg_data >> 10) & 1;
                pPortAbility->link = (reg_data >> 9) & 1;
                pPortAbility->speed = (reg_data >> 7) & 3;
                pPortAbility->rxpause = (reg_data >> 14) & 1;
                pPortAbility->txpause = (reg_data >> 13) & 1;
            }
            else if (Mode < EXT_RGMII_2)
            {
                if ((retVal = rtl8367c_getAsicReg(0x1359, &reg_data)) != RT_ERR_OK)
                    return retVal;

                // pPortAbility->forcemode = (reg_data >> 12) & 1;
                pPortAbility->duplex = (reg_data >> 2) & 1;
                pPortAbility->link = (reg_data >> 4) & 1;
                pPortAbility->speed = reg_data & 3;
                pPortAbility->rxpause = (reg_data >> 5) & 1;
                pPortAbility->txpause = (reg_data >> 6) & 1;
            }
            else if (Mode < EXT_END)
            {

                if ((retVal = rtl8367c_getAsicReg(0x1311, &reg_data)) != RT_ERR_OK)
                    return retVal;

                pPortAbility->forcemode = (reg_data >> 12) & 1;
                pPortAbility->duplex = (reg_data >> 2) & 1;
                pPortAbility->link = (reg_data >> 4) & 1;
                pPortAbility->speed = reg_data & 3;
                pPortAbility->rxpause = (reg_data >> 5) & 1;
                pPortAbility->txpause = (reg_data >> 6) & 1;
            }
        }
    }
    return RT_ERR_OK;
}

int32_t rtl8367::rtk_port_macForceLinkExt_set(rtk_port_t port, rtk_mode_ext_t mode, rtk_port_mac_ability_t *pPortability)
{
    int32_t retVal;
    rtl8367c_port_ability_t ability;
    uint32_t ext_id;

    /* Check Port Valid */
    RTK_CHK_PORT_IS_EXT(port);

    if (NULL == pPortability)
        return RT_ERR_NULL_POINTER;

    if (mode >= MODE_EXT_END)
        return RT_ERR_INPUT;

    if (mode == MODE_EXT_HSGMII)
    {
        if (pPortability->forcemode > 1 || pPortability->speed != PORT_SPEED_2500M || pPortability->duplex != PORT_FULL_DUPLEX ||
            pPortability->link >= PORT_LINKSTATUS_END || pPortability->nway > 1 || pPortability->txpause > 1 || pPortability->rxpause > 1)
            return RT_ERR_INPUT;

        if (rtk_switch_isHsgPort(port) != RT_ERR_OK)
            return RT_ERR_PORT_ID;
    }
    else
    {
        if (pPortability->forcemode > 1 || pPortability->speed > PORT_SPEED_1000M || pPortability->duplex >= PORT_DUPLEX_END ||
            pPortability->link >= PORT_LINKSTATUS_END || pPortability->nway > 1 || pPortability->txpause > 1 || pPortability->rxpause > 1)
            return RT_ERR_INPUT;
    }

    ext_id = port - 15;

    if (mode == MODE_EXT_DISABLE)
    {
        memset(&ability, 0x00, sizeof(rtl8367c_port_ability_t));
        if ((retVal = rtl8367c_setAsicPortForceLinkExt(ext_id, &ability)) != RT_ERR_OK)
            return retVal;

        if ((retVal = rtl8367c_setAsicPortExtMode(ext_id, mode)) != RT_ERR_OK)
            return retVal;
    }
    else
    {
        if ((retVal = rtl8367c_setAsicPortExtMode(ext_id, mode)) != RT_ERR_OK)
            return retVal;

        if ((retVal = rtl8367c_getAsicPortForceLinkExt(ext_id, &ability)) != RT_ERR_OK)
            return retVal;

        ability.forcemode = pPortability->forcemode;
        ability.speed = (mode == MODE_EXT_HSGMII) ? PORT_SPEED_1000M : pPortability->speed;
        ability.duplex = pPortability->duplex;
        ability.link = pPortability->link;
        ability.nway = pPortability->nway;
        ability.txpause = pPortability->txpause;
        ability.rxpause = pPortability->rxpause;

        if ((retVal = rtl8367c_setAsicPortForceLinkExt(ext_id, &ability)) != RT_ERR_OK)
            return retVal;
    }

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_port_macForceLinkExt_get(rtk_port_t port, rtk_mode_ext_t *pMode, rtk_port_mac_ability_t *pPortability)
{
    int32_t retVal;
    rtl8367c_port_ability_t ability;
    uint32_t ext_id;

    /* Check Port Valid */
    RTK_CHK_PORT_IS_EXT(port);

    if (NULL == pMode)
        return RT_ERR_NULL_POINTER;

    if (NULL == pPortability)
        return RT_ERR_NULL_POINTER;

    ext_id = port - 15;

    if ((retVal = rtl8367c_getAsicPortExtMode(ext_id, (uint32_t *)pMode)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_getAsicPortForceLinkExt(ext_id, &ability)) != RT_ERR_OK)
        return retVal;

    pPortability->forcemode = ability.forcemode;
    pPortability->speed = (*pMode == MODE_EXT_HSGMII) ? PORT_SPEED_2500M : ability.speed;
    pPortability->duplex = ability.duplex;
    pPortability->link = ability.link;
    pPortability->nway = ability.nway;
    pPortability->txpause = ability.txpause;
    pPortability->rxpause = ability.rxpause;

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_port_rgmiiDelayExt_set(rtk_port_t port, uint32_t txDelay, uint32_t rxDelay)
{
    int32_t retVal;
    uint32_t regAddr, regData;

    /* Check Port Valid */
    RTK_CHK_PORT_IS_EXT(port);

    if ((txDelay > 1) || (rxDelay > 7))
        return RT_ERR_INPUT;

    if (port == EXT_PORT0)
        regAddr = RTL8367C_REG_EXT1_RGMXF;
    else if (port == EXT_PORT1)
        regAddr = RTL8367C_REG_EXT2_RGMXF;
    else
        return RT_ERR_INPUT;

    if ((retVal = rtl8367c_getAsicReg(regAddr, &regData)) != RT_ERR_OK)
        return retVal;

    regData = (regData & 0xFFF0) | ((txDelay << 3) & 0x0008) | (rxDelay & 0x0007);

    if ((retVal = rtl8367c_setAsicReg(regAddr, regData)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_port_rgmiiDelayExt_get(rtk_port_t port, uint32_t *pTxDelay, uint32_t *pRxDelay)
{
    int32_t retVal;
    uint32_t regAddr, regData;

    /* Check Port Valid */
    RTK_CHK_PORT_IS_EXT(port);

    if ((NULL == pTxDelay) || (NULL == pRxDelay))
        return RT_ERR_NULL_POINTER;

    if (port == EXT_PORT0)
        regAddr = RTL8367C_REG_EXT1_RGMXF;
    else if (port == EXT_PORT1)
        regAddr = RTL8367C_REG_EXT2_RGMXF;
    else
        return RT_ERR_INPUT;

    if ((retVal = rtl8367c_getAsicReg(regAddr, &regData)) != RT_ERR_OK)
        return retVal;

    *pTxDelay = (regData & 0x0008) >> 3;
    *pRxDelay = regData & 0x0007;

    return RT_ERR_OK;
}

// --------------------------------- PORT ISOLATION ---------------------------------

/* Function Name:
 *      rtl8367c_setAsicPortIsolationPermittedPortmask
 * Description:
 *      Set permitted port isolation portmask
 * Input:
 *      port            - Physical port number (0~10)
 *      permitPortmask  - port mask
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK           - Success
 *      RT_ERR_SMI          - SMI access error
 *      RT_ERR_PORT_ID      - Invalid port number
 *      RT_ERR_PORT_MASK    - Invalid portmask
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicPortIsolationPermittedPortmask(uint32_t port, uint32_t permitPortmask)
{
    if (port >= RTL8367C_PORTNO)
        return RT_ERR_PORT_ID;

    if (permitPortmask > RTL8367C_PORTMASK)
        return RT_ERR_PORT_MASK;

    return rtl8367c_setAsicReg(RTL8367C_PORT_ISOLATION_PORT_MASK_REG(port), permitPortmask);
}
/* Function Name:
 *      rtl8367c_getAsicPortIsolationPermittedPortmask
 * Description:
 *      Get permitted port isolation portmask
 * Input:
 *      port                - Physical port number (0~10)
 *      pPermitPortmask     - port mask
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK           - Success
 *      RT_ERR_SMI          - SMI access error
 *      RT_ERR_PORT_ID      - Invalid port number
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicPortIsolationPermittedPortmask(uint32_t port, uint32_t *pPermitPortmask)
{
    if (port >= RTL8367C_PORTNO)
        return RT_ERR_PORT_ID;

    return rtl8367c_getAsicReg(RTL8367C_PORT_ISOLATION_PORT_MASK_REG(port), pPermitPortmask);
}
int32_t rtl8367::rtk_port_isolation_set(rtk_port_t port, rtk_portmask_t *pPortmask)
{
    int32_t retVal;
    uint32_t pmask;

    /* Check Port Valid */
    RTK_CHK_PORT_VALID(port);

    if (NULL == pPortmask)
        return RT_ERR_NULL_POINTER;

    /* check port mask */
    RTK_CHK_PORTMASK_VALID(pPortmask);

    if ((retVal = rtk_switch_portmask_L2P_get(pPortmask, &pmask)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicPortIsolationPermittedPortmask(rtk_switch_port_L2P_get(port), pmask)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_port_isolation_get(rtk_port_t port, rtk_portmask_t *pPortmask)
{
    int32_t retVal;
    uint32_t pmask;

    /* Check Port Valid */
    RTK_CHK_PORT_VALID(port);

    if (NULL == pPortmask)
        return RT_ERR_NULL_POINTER;

    if ((retVal = rtl8367c_getAsicPortIsolationPermittedPortmask(rtk_switch_port_L2P_get(port), &pmask)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtk_switch_portmask_P2L_get(pmask, pPortmask)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

// ------------------------------- MAC LEARNING LIMIT -------------------------------

/* Function Name:
 *      rtl8367c_setAsicLutLearnLimitNo
 * Description:
 *      Set per-Port auto learning limit number
 * Input:
 *      port    - Physical port number (0~7)
 *      number  - ASIC auto learning entries limit number
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK                   - Success
 *      RT_ERR_SMI                  - SMI access error
 *      RT_ERR_PORT_ID              - Invalid port number
 *      RT_ERR_LIMITED_L2ENTRY_NUM  - Invalid auto learning limit number
 * Note:
 *      None
 */
/*�޸�: RTL8367C_PORTIDMAX, RTL8367C_LUT_LEARNLIMITMAX, RTL8367C_LUT_PORT_LEARN_LIMITNO_REG*/
int32_t rtl8367::rtl8367c_setAsicLutLearnLimitNo(uint32_t port, uint32_t number)
{
    if (port > RTL8367C_PORTIDMAX)
        return RT_ERR_PORT_ID;

    if (number > RTL8367C_LUT_LEARNLIMITMAX)
        return RT_ERR_LIMITED_L2ENTRY_NUM;

    if (port < 8)
        return rtl8367c_setAsicReg(RTL8367C_LUT_PORT_LEARN_LIMITNO_REG(port), number);
    else
        return rtl8367c_setAsicReg(RTL8367C_REG_LUT_PORT8_LEARN_LIMITNO + port - 8, number);
}
/* Function Name:
 *      rtl8367c_getAsicLutLearnLimitNo
 * Description:
 *      Get per-Port auto learning limit number
 * Input:
 *      port    - Physical port number (0~7)
 *      pNumber     - ASIC auto learning entries limit number
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_SMI      - SMI access error
 *      RT_ERR_PORT_ID  - Invalid port number
 * Note:
 *      None
 */
/*�޸�: RTL8367C_PORTIDMAX, RTL8367C_LUT_PORT_LEARN_LIMITNO_REG*/
int32_t rtl8367::rtl8367c_getAsicLutLearnLimitNo(uint32_t port, uint32_t *pNumber)
{
    if (port > RTL8367C_PORTIDMAX)
        return RT_ERR_PORT_ID;

    if (port < 8)
        return rtl8367c_getAsicReg(RTL8367C_LUT_PORT_LEARN_LIMITNO_REG(port), pNumber);
    else
        return rtl8367c_getAsicReg(RTL8367C_REG_LUT_PORT8_LEARN_LIMITNO + port - 8, pNumber);
}

int32_t rtl8367::rtk_l2_limitLearningCnt_set(rtk_port_t port, uint32_t mac_cnt)
{
    int32_t retVal;

    /* check port valid */
    RTK_CHK_PORT_VALID(port);

    if (mac_cnt > halCtrl.max_lut_addr_num)
        return RT_ERR_LIMITED_L2ENTRY_NUM;

    if ((retVal = rtl8367c_setAsicLutLearnLimitNo(rtk_switch_port_L2P_get(port), mac_cnt)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_l2_limitLearningCnt_get(rtk_port_t port, uint32_t *pMac_cnt)
{
    int32_t retVal;

    /* check port valid */
    RTK_CHK_PORT_VALID(port);

    if (NULL == pMac_cnt)
        return RT_ERR_NULL_POINTER;

    if ((retVal = rtl8367c_getAsicLutLearnLimitNo(rtk_switch_port_L2P_get(port), pMac_cnt)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_getAsicLutLearnNo
 * Description:
 *      Get per-Port auto learning number
 * Input:
 *      port    - Physical port number (0~7)
 *      pNumber     - ASIC auto learning entries number
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_SMI      - SMI access error
 *      RT_ERR_PORT_ID  - Invalid port number
 * Note:
 *      None
 */
/*�޸�RTL8367C_PORTIDMAX, RTL8367C_REG_L2_LRN_CNT_REG, port10 reg is not contnious, wait for updating of base.h*/
int32_t rtl8367::rtl8367c_getAsicLutLearnNo(uint32_t port, uint32_t *pNumber)
{
    int32_t retVal;

    if (port > RTL8367C_PORTIDMAX)
        return RT_ERR_PORT_ID;

    if (port < 10)
    {
        retVal = rtl8367c_getAsicReg(RTL8367C_REG_L2_LRN_CNT_REG(port), pNumber);
        if (retVal != RT_ERR_OK)
            return retVal;
    }
    else
    {
        retVal = rtl8367c_getAsicReg(RTL8367C_REG_L2_LRN_CNT_CTRL10, pNumber);
        if (retVal != RT_ERR_OK)
            return retVal;
    }

    return RT_ERR_OK;
}
int32_t rtl8367::rtk_l2_learningCnt_get(rtk_port_t port, uint32_t *pMac_cnt)
{
    int32_t retVal;

    /* check port valid */
    RTK_CHK_PORT_VALID(port);

    if (NULL == pMac_cnt)
        return RT_ERR_NULL_POINTER;

    if ((retVal = rtl8367c_getAsicLutLearnNo(rtk_switch_port_L2P_get(port), pMac_cnt)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

// -------------------------------- ACL ---------------------------------------

/* Function Name:
 *      rtl8367c_setAsicAclActCtrl
 * Description:
 *      Set ACL rule matched Action Control Bits
 * Input:
 *      index       - ACL rule index (0-95) of 96 ACL rules
 *      aclActCtrl  - 6 ACL Control Bits
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK               - Success
 *      RT_ERR_SMI              - SMI access error
 *      RT_ERR_OUT_OF_RANGE     - Invalid ACL rule index (0-95)
 * Note:
 *      ACL Action Control Bits Indicate which actions will be take when a rule matches
 */
int32_t rtl8367::rtl8367c_setAsicAclActCtrl(uint32_t index, uint32_t aclActCtrl)
{
    int32_t retVal;

    if (index > RTL8367C_ACLRULEMAX)
        return RT_ERR_OUT_OF_RANGE;

    if (index >= 64)
        retVal = rtl8367c_setAsicRegBits(RTL8367C_ACL_ACTION_CTRL2_REG(index), RTL8367C_ACL_OP_ACTION_MASK(index), aclActCtrl);
    else
        retVal = rtl8367c_setAsicRegBits(RTL8367C_ACL_ACTION_CTRL_REG(index), RTL8367C_ACL_OP_ACTION_MASK(index), aclActCtrl);

    return retVal;
}
/* Function Name:
 *      rtl8367c_setAsicAclNot
 * Description:
 *      Set rule comparison result inversion / no inversion
 * Input:
 *      index   - ACL rule index (0-95) of 96 ACL rules
 *      not     - 1: inverse, 0: don't inverse
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK               - Success
 *      RT_ERR_SMI              - SMI access error
 *      RT_ERR_OUT_OF_RANGE     - Invalid ACL rule index (0-95)
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicAclNot(uint32_t index, uint32_t nott)
{
    if (index > RTL8367C_ACLRULEMAX)
        return RT_ERR_OUT_OF_RANGE;

    if (index < 64)
        return rtl8367c_setAsicRegBit(RTL8367C_ACL_ACTION_CTRL_REG(index), RTL8367C_ACL_OP_NOT_OFFSET(index), nott);
    else
        return rtl8367c_setAsicRegBit(RTL8367C_ACL_ACTION_CTRL2_REG(index), RTL8367C_ACL_OP_NOT_OFFSET(index), nott);
}
int32_t rtl8367::_rtk_filter_igrAcl_cfg_delAll()
{
    uint32_t i;
    int32_t ret;

    for (i = 0; i < RTL8367C_ACLRULENO; i++)
    {
        if ((ret = rtl8367c_setAsicAclActCtrl(i, FILTER_ENACT_INIT_MASK)) != RT_ERR_OK)
            return ret;
        if ((ret = rtl8367c_setAsicAclNot(i, DISABLED)) != RT_ERR_OK)
            return ret;
    }

    return rtl8367c_setAsicRegBit(RTL8367C_REG_ACL_RESET_CFG, RTL8367C_ACL_RESET_CFG_OFFSET, 1);
}

/* Function Name:
 *      rtl8367c_setAsicAclTemplate
 * Description:
 *      Set fields of a ACL Template
 * Input:
 *      index   - ACL template index(0~4)
 *      pAclType - ACL type stucture for setting
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK               - Success
 *      RT_ERR_SMI              - SMI access error
 *      RT_ERR_OUT_OF_RANGE     - Invalid ACL template index(0~4)
 * Note:
 *      The API can set type field of the 5 ACL rule templates.
 *      Each type has 8 fields. One field means what data in one field of a ACL rule means
 *      8 fields of ACL rule 0~95 is descripted by one type in ACL group
 */
int32_t rtl8367::rtl8367c_setAsicAclTemplate(uint32_t index, rtl8367c_acltemplate_t *pAclType)
{
    int32_t retVal;
    uint32_t i;
    uint32_t regAddr, regData;

    if (index >= RTL8367C_ACLTEMPLATENO)
        return RT_ERR_OUT_OF_RANGE;

    regAddr = RTL8367C_ACL_RULE_TEMPLATE_CTRL_REG(index);

    for (i = 0; i < (RTL8367C_ACLRULEFIELDNO / 2); i++)
    {
        regData = pAclType->field[i * 2 + 1];
        regData = regData << 8 | pAclType->field[i * 2];

        retVal = rtl8367c_setAsicReg(regAddr + i, regData);

        if (retVal != RT_ERR_OK)
            return retVal;
    }

    return retVal;
}

/* Function Name:
 *      rtl8367c_setAsicFieldSelector
 * Description:
 *      Set user defined field selectors in HSB
 * Input:
 *      index       - index of field selector 0-15
 *      format      - Format of field selector
 *      offset      - Retrieving data offset
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK           - Success
 *      RT_ERR_SMI          - SMI access error
 *      RT_ERR_OUT_OF_RANGE - input parameter out of range
 * Note:
 *      System support 16 user defined field selctors.
 *      Each selector can be enabled or disable. User can defined retrieving 16-bits in many predefiend
 *      standard l2/l3/l4 payload.
 */
int32_t rtl8367::rtl8367c_setAsicFieldSelector(uint32_t index, uint32_t format, uint32_t offset)
{
    uint32_t regData;

    if (index > RTL8367C_FIELDSEL_FORMAT_NUMBER)
        return RT_ERR_OUT_OF_RANGE;

    if (format >= FIELDSEL_FORMAT_END)
        return RT_ERR_OUT_OF_RANGE;

    regData = (((format << RTL8367C_FIELD_SELECTOR_FORMAT_OFFSET) & RTL8367C_FIELD_SELECTOR_FORMAT_MASK) |
               ((offset << RTL8367C_FIELD_SELECTOR_OFFSET_OFFSET) & RTL8367C_FIELD_SELECTOR_OFFSET_MASK));

    return rtl8367c_setAsicReg(RTL8367C_FIELD_SELECTOR_REG(index), regData);
}
/* Function Name:
 *      rtl8367c_setAsicAcl
 * Description:
 *      Set port acl function enable/disable
 * Input:
 *      port    - Physical port number (0~10)
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
int32_t rtl8367::rtl8367c_setAsicAcl(uint32_t port, uint32_t enabled)
{
    if (port > RTL8367C_PORTIDMAX)
        return RT_ERR_PORT_ID;

    return rtl8367c_setAsicRegBit(RTL8367C_ACL_ENABLE_REG, port, enabled);
}
/* Function Name:
 *      rtl8367c_setAsicAclUnmatchedPermit
 * Description:
 *      Set port acl function unmatched permit action
 * Input:
 *      port    - Physical port number (0~10)
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
int32_t rtl8367::rtl8367c_setAsicAclUnmatchedPermit(uint32_t port, uint32_t enabled)
{
    if (port > RTL8367C_PORTIDMAX)
        return RT_ERR_PORT_ID;

    return rtl8367c_setAsicRegBit(RTL8367C_ACL_UNMATCH_PERMIT_REG, port, enabled);
}
int32_t rtl8367::rtk_filter_igrAcl_init()
{
    rtl8367c_acltemplate_t aclTemp;
    uint32_t i, j;
    int32_t ret;

    if ((ret = _rtk_filter_igrAcl_cfg_delAll()) != RT_ERR_OK)
        return ret;

    for (i = 0; i < RTL8367C_ACLTEMPLATENO; i++)
    {
        for (j = 0; j < RTL8367C_ACLRULEFIELDNO; j++)
            aclTemp.field[j] = filter_templateField[i][j];

        if ((ret = rtl8367c_setAsicAclTemplate(i, &aclTemp)) != RT_ERR_OK)
            return ret;
    }

    for (i = 0; i < RTL8367C_FIELDSEL_FORMAT_NUMBER; i++)
    {
        if ((ret = rtl8367c_setAsicFieldSelector(i, field_selector[i][0], field_selector[i][1])) != RT_ERR_OK)
            return ret;
    }

    RTK_SCAN_ALL_PHY_PORTMASK(i)
    {
        if ((ret = rtl8367c_setAsicAcl(i, 1)) != RT_ERR_OK)
            return ret;

        if ((ret = rtl8367c_setAsicAclUnmatchedPermit(i, 1)) != RT_ERR_OK)
            return ret;
    }

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_getAsicAclTemplate
 * Description:
 *      Get fields of a ACL Template
 * Input:
 *      index   - ACL template index(0~4)
 *      pAclType - ACL type stucture for setting
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK               - Success
 *      RT_ERR_SMI              - SMI access error
 *      RT_ERR_OUT_OF_RANGE     - Invalid ACL template index(0~4)
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicAclTemplate(uint32_t index, rtl8367c_acltemplate_t *pAclType)
{
    int32_t retVal;
    uint32_t i;
    uint32_t regData, regAddr;

    if (index >= RTL8367C_ACLTEMPLATENO)
        return RT_ERR_OUT_OF_RANGE;

    regAddr = RTL8367C_ACL_RULE_TEMPLATE_CTRL_REG(index);

    for (i = 0; i < (RTL8367C_ACLRULEFIELDNO / 2); i++)
    {
        retVal = rtl8367c_getAsicReg(regAddr + i, &regData);
        if (retVal != RT_ERR_OK)
            return retVal;

        pAclType->field[i * 2] = regData & 0xFF;
        pAclType->field[i * 2 + 1] = (regData >> 8) & 0xFF;
    }

    return RT_ERR_OK;
}
int32_t rtl8367::rtk_filter_igrAcl_template_set(rtk_filter_template_t *aclTemplate)
{
    int32_t retVal;
    uint32_t idxField;
    rtl8367c_acltemplate_t aclType;

    if (aclTemplate->index >= RTK_MAX_NUM_OF_FILTER_TYPE)
        return RT_ERR_INPUT;

    for (idxField = 0; idxField < RTK_MAX_NUM_OF_FILTER_FIELD; idxField++)
    {
        if (aclTemplate->fieldType[idxField] < FILTER_FIELD_RAW_DMAC_15_0 ||
            (aclTemplate->fieldType[idxField] > FILTER_FIELD_RAW_CTAG && aclTemplate->fieldType[idxField] < FILTER_FIELD_RAW_IPV4_SIP_15_0) ||
            (aclTemplate->fieldType[idxField] > FILTER_FIELD_RAW_IPV4_DIP_31_16 && aclTemplate->fieldType[idxField] < FILTER_FIELD_RAW_IPV6_SIP_15_0) ||
            (aclTemplate->fieldType[idxField] > FILTER_FIELD_RAW_IPV6_DIP_31_16 && aclTemplate->fieldType[idxField] < FILTER_FIELD_RAW_VIDRANGE) ||
            (aclTemplate->fieldType[idxField] > FILTER_FIELD_RAW_FIELD_VALID && aclTemplate->fieldType[idxField] < FILTER_FIELD_RAW_FIELD_SELECT00) ||
            aclTemplate->fieldType[idxField] >= FILTER_FIELD_RAW_END)
        {
            return RT_ERR_INPUT;
        }
    }

    for (idxField = 0; idxField < RTK_MAX_NUM_OF_FILTER_FIELD; idxField++)
    {
        aclType.field[idxField] = aclTemplate->fieldType[idxField];
    }

    if ((retVal = rtl8367c_setAsicAclTemplate(aclTemplate->index, &aclType)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_filter_igrAcl_template_get(rtk_filter_template_t *aclTemplate)
{
    int32_t ret;
    uint32_t idxField;
    rtl8367c_acltemplate_t aclType;

    if (NULL == aclTemplate)
        return RT_ERR_NULL_POINTER;

    if (aclTemplate->index >= RTK_MAX_NUM_OF_FILTER_TYPE)
        return RT_ERR_INPUT;

    if ((ret = rtl8367c_getAsicAclTemplate(aclTemplate->index, &aclType)) != RT_ERR_OK)
        return ret;

    for (idxField = 0; idxField < RTK_MAX_NUM_OF_FILTER_FIELD; idxField++)
    {
        aclTemplate->fieldType[idxField] = (rtk_filter_field_type_raw_t)aclType.field[idxField];
    }

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_filter_igrAcl_field_add(rtk_filter_cfg_t *pFilter_cfg, rtk_filter_field_t *pFilter_field)
{
    uint32_t i;
    rtk_filter_field_t *tailPtr;

    if (NULL == pFilter_cfg || NULL == pFilter_field)
        return RT_ERR_NULL_POINTER;

    if (pFilter_field->fieldType >= FILTER_FIELD_END)
        return RT_ERR_ENTRY_INDEX;

    if (0 == pFilter_field->fieldTemplateNo)
    {
        pFilter_field->fieldTemplateNo = filter_fieldSize[pFilter_field->fieldType];

        for (i = 0; i < pFilter_field->fieldTemplateNo; i++)
        {
            pFilter_field->fieldTemplateIdx[i] = filter_fieldTemplateIndex[pFilter_field->fieldType][i];
        }
    }

    if (NULL == pFilter_cfg->fieldHead)
    {
        pFilter_cfg->fieldHead = pFilter_field;
    }
    else
    {
        if (pFilter_cfg->fieldHead->next == NULL)
        {
            pFilter_cfg->fieldHead->next = pFilter_field;
        }
        else
        {
            tailPtr = pFilter_cfg->fieldHead->next;
            while (tailPtr->next != NULL)
            {
                tailPtr = tailPtr->next;
            }
            tailPtr->next = pFilter_field;
        }
    }

    return RT_ERR_OK;
}

int32_t rtl8367::_rtk_filter_igrAcl_writeDataField(rtl8367c_aclrule *aclRule, rtk_filter_field_t *fieldPtr)
{
    uint32_t i, tempIdx, fieldIdx, ipValue, ipMask;
    uint32_t ip6addr[RTK_IPV6_ADDR_WORD_LENGTH];
    uint32_t ip6mask[RTK_IPV6_ADDR_WORD_LENGTH];

    for (i = 0; i < fieldPtr->fieldTemplateNo; i++)
    {
        tempIdx = (fieldPtr->fieldTemplateIdx[i] & 0xF0) >> 4;

        aclRule[tempIdx].valid = 1;
    }

    switch (fieldPtr->fieldType)
    {
    /* use DMAC structure as representative for mac structure */
    case FILTER_FIELD_DMAC:
    case FILTER_FIELD_SMAC:

        for (i = 0; i < fieldPtr->fieldTemplateNo; i++)
        {
            tempIdx = (fieldPtr->fieldTemplateIdx[i] & 0xF0) >> 4;
            fieldIdx = fieldPtr->fieldTemplateIdx[i] & 0x0F;

            aclRule[tempIdx].data_bits.field[fieldIdx] = fieldPtr->filter_pattern_union.mac.value.octet[5 - i * 2] | (fieldPtr->filter_pattern_union.mac.value.octet[5 - (i * 2 + 1)] << 8);
            aclRule[tempIdx].care_bits.field[fieldIdx] = fieldPtr->filter_pattern_union.mac.mask.octet[5 - i * 2] | (fieldPtr->filter_pattern_union.mac.mask.octet[5 - (i * 2 + 1)] << 8);
        }
        break;
    case FILTER_FIELD_ETHERTYPE:
        for (i = 0; i < fieldPtr->fieldTemplateNo; i++)
        {
            tempIdx = (fieldPtr->fieldTemplateIdx[i] & 0xF0) >> 4;
            fieldIdx = fieldPtr->fieldTemplateIdx[i] & 0x0F;

            aclRule[tempIdx].data_bits.field[fieldIdx] = fieldPtr->filter_pattern_union.etherType.value;
            aclRule[tempIdx].care_bits.field[fieldIdx] = fieldPtr->filter_pattern_union.etherType.mask;
        }
        break;
    case FILTER_FIELD_IPV4_SIP:
    case FILTER_FIELD_IPV4_DIP:

        ipValue = fieldPtr->filter_pattern_union.sip.value;
        ipMask = fieldPtr->filter_pattern_union.sip.mask;

        for (i = 0; i < fieldPtr->fieldTemplateNo; i++)
        {
            tempIdx = (fieldPtr->fieldTemplateIdx[i] & 0xF0) >> 4;
            fieldIdx = fieldPtr->fieldTemplateIdx[i] & 0x0F;

            aclRule[tempIdx].data_bits.field[fieldIdx] = (0xFFFF & (ipValue >> (i * 16)));
            aclRule[tempIdx].care_bits.field[fieldIdx] = (0xFFFF & (ipMask >> (i * 16)));
        }
        break;
    case FILTER_FIELD_IPV4_TOS:
        for (i = 0; i < fieldPtr->fieldTemplateNo; i++)
        {
            tempIdx = (fieldPtr->fieldTemplateIdx[i] & 0xF0) >> 4;
            fieldIdx = fieldPtr->fieldTemplateIdx[i] & 0x0F;

            aclRule[tempIdx].data_bits.field[fieldIdx] = fieldPtr->filter_pattern_union.ipTos.value & 0xFF;
            aclRule[tempIdx].care_bits.field[fieldIdx] = fieldPtr->filter_pattern_union.ipTos.mask & 0xFF;
        }
        break;
    case FILTER_FIELD_IPV4_PROTOCOL:
        for (i = 0; i < fieldPtr->fieldTemplateNo; i++)
        {
            tempIdx = (fieldPtr->fieldTemplateIdx[i] & 0xF0) >> 4;
            fieldIdx = fieldPtr->fieldTemplateIdx[i] & 0x0F;

            aclRule[tempIdx].data_bits.field[fieldIdx] = fieldPtr->filter_pattern_union.protocol.value & 0xFF;
            aclRule[tempIdx].care_bits.field[fieldIdx] = fieldPtr->filter_pattern_union.protocol.mask & 0xFF;
        }
        break;
    case FILTER_FIELD_IPV6_SIPV6:
    case FILTER_FIELD_IPV6_DIPV6:
        for (i = 0; i < RTK_IPV6_ADDR_WORD_LENGTH; i++)
        {
            ip6addr[i] = fieldPtr->filter_pattern_union.sipv6.value.addr[i];
            ip6mask[i] = fieldPtr->filter_pattern_union.sipv6.mask.addr[i];
        }

        for (i = 0; i < fieldPtr->fieldTemplateNo; i++)
        {
            tempIdx = (fieldPtr->fieldTemplateIdx[i] & 0xF0) >> 4;
            fieldIdx = fieldPtr->fieldTemplateIdx[i] & 0x0F;

            if (i < 2)
            {
                aclRule[tempIdx].data_bits.field[fieldIdx] = ((ip6addr[0] & (0xFFFF << (i * 16))) >> (i * 16));
                aclRule[tempIdx].care_bits.field[fieldIdx] = ((ip6mask[0] & (0xFFFF << (i * 16))) >> (i * 16));
            }
            else
            {
                /*default acl template for ipv6 address supports MSB 32-bits and LSB 32-bits only*/
                aclRule[tempIdx].data_bits.field[fieldIdx] = ((ip6addr[3] & (0xFFFF << ((i & 1) * 16))) >> ((i & 1) * 16));
                aclRule[tempIdx].care_bits.field[fieldIdx] = ((ip6mask[3] & (0xFFFF << ((i & 1) * 16))) >> ((i & 1) * 16));
            }
        }

        break;
    case FILTER_FIELD_CTAG:
    case FILTER_FIELD_STAG:

        for (i = 0; i < fieldPtr->fieldTemplateNo; i++)
        {
            tempIdx = (fieldPtr->fieldTemplateIdx[i] & 0xF0) >> 4;
            fieldIdx = fieldPtr->fieldTemplateIdx[i] & 0x0F;

            aclRule[tempIdx].data_bits.field[fieldIdx] = (fieldPtr->filter_pattern_union.l2tag.pri.value << 13) | (fieldPtr->filter_pattern_union.l2tag.cfi.value << 12) | fieldPtr->filter_pattern_union.l2tag.vid.value;
            aclRule[tempIdx].care_bits.field[fieldIdx] = (fieldPtr->filter_pattern_union.l2tag.pri.mask << 13) | (fieldPtr->filter_pattern_union.l2tag.cfi.mask << 12) | fieldPtr->filter_pattern_union.l2tag.vid.mask;
        }
        break;
    case FILTER_FIELD_IPV4_FLAG:

        for (i = 0; i < fieldPtr->fieldTemplateNo; i++)
        {
            tempIdx = (fieldPtr->fieldTemplateIdx[i] & 0xF0) >> 4;
            fieldIdx = fieldPtr->fieldTemplateIdx[i] & 0x0F;

            aclRule[tempIdx].data_bits.field[fieldIdx] &= 0x1FFF;
            aclRule[tempIdx].data_bits.field[fieldIdx] |= (fieldPtr->filter_pattern_union.ipFlag.xf.value << 15);
            aclRule[tempIdx].data_bits.field[fieldIdx] |= (fieldPtr->filter_pattern_union.ipFlag.df.value << 14);
            aclRule[tempIdx].data_bits.field[fieldIdx] |= (fieldPtr->filter_pattern_union.ipFlag.mf.value << 13);

            aclRule[tempIdx].care_bits.field[fieldIdx] &= 0x1FFF;
            aclRule[tempIdx].care_bits.field[fieldIdx] |= (fieldPtr->filter_pattern_union.ipFlag.xf.mask << 15);
            aclRule[tempIdx].care_bits.field[fieldIdx] |= (fieldPtr->filter_pattern_union.ipFlag.df.mask << 14);
            aclRule[tempIdx].care_bits.field[fieldIdx] |= (fieldPtr->filter_pattern_union.ipFlag.mf.mask << 13);
        }

        break;
    case FILTER_FIELD_IPV4_OFFSET:

        for (i = 0; i < fieldPtr->fieldTemplateNo; i++)
        {
            tempIdx = (fieldPtr->fieldTemplateIdx[i] & 0xF0) >> 4;
            fieldIdx = fieldPtr->fieldTemplateIdx[i] & 0x0F;

            aclRule[tempIdx].data_bits.field[fieldIdx] &= 0xE000;
            aclRule[tempIdx].data_bits.field[fieldIdx] |= fieldPtr->filter_pattern_union.inData.value;

            aclRule[tempIdx].care_bits.field[fieldIdx] &= 0xE000;
            aclRule[tempIdx].care_bits.field[fieldIdx] |= fieldPtr->filter_pattern_union.inData.mask;
        }

        break;

    case FILTER_FIELD_IPV6_TRAFFIC_CLASS:
        for (i = 0; i < fieldPtr->fieldTemplateNo; i++)
        {
            tempIdx = (fieldPtr->fieldTemplateIdx[i] & 0xF0) >> 4;
            fieldIdx = fieldPtr->fieldTemplateIdx[i] & 0x0F;

            aclRule[tempIdx].data_bits.field[fieldIdx] = (fieldPtr->filter_pattern_union.inData.value << 4) & 0x0FF0;
            aclRule[tempIdx].care_bits.field[fieldIdx] = (fieldPtr->filter_pattern_union.inData.mask << 4) & 0x0FF0;
        }
        break;
    case FILTER_FIELD_IPV6_NEXT_HEADER:
        for (i = 0; i < fieldPtr->fieldTemplateNo; i++)
        {
            tempIdx = (fieldPtr->fieldTemplateIdx[i] & 0xF0) >> 4;
            fieldIdx = fieldPtr->fieldTemplateIdx[i] & 0x0F;

            aclRule[tempIdx].data_bits.field[fieldIdx] = fieldPtr->filter_pattern_union.inData.value << 8;
            aclRule[tempIdx].care_bits.field[fieldIdx] = fieldPtr->filter_pattern_union.inData.mask << 8;
        }
        break;
    case FILTER_FIELD_TCP_SPORT:
        for (i = 0; i < fieldPtr->fieldTemplateNo; i++)
        {
            tempIdx = (fieldPtr->fieldTemplateIdx[i] & 0xF0) >> 4;
            fieldIdx = fieldPtr->fieldTemplateIdx[i] & 0x0F;

            aclRule[tempIdx].data_bits.field[fieldIdx] = fieldPtr->filter_pattern_union.tcpSrcPort.value;
            aclRule[tempIdx].care_bits.field[fieldIdx] = fieldPtr->filter_pattern_union.tcpSrcPort.mask;
        }
        break;
    case FILTER_FIELD_TCP_DPORT:
        for (i = 0; i < fieldPtr->fieldTemplateNo; i++)
        {
            tempIdx = (fieldPtr->fieldTemplateIdx[i] & 0xF0) >> 4;
            fieldIdx = fieldPtr->fieldTemplateIdx[i] & 0x0F;

            aclRule[tempIdx].data_bits.field[fieldIdx] = fieldPtr->filter_pattern_union.tcpDstPort.value;
            aclRule[tempIdx].care_bits.field[fieldIdx] = fieldPtr->filter_pattern_union.tcpDstPort.mask;
        }
        break;
    case FILTER_FIELD_TCP_FLAG:

        for (i = 0; i < fieldPtr->fieldTemplateNo; i++)
        {
            tempIdx = (fieldPtr->fieldTemplateIdx[i] & 0xF0) >> 4;
            fieldIdx = fieldPtr->fieldTemplateIdx[i] & 0x0F;

            aclRule[tempIdx].data_bits.field[fieldIdx] |= (fieldPtr->filter_pattern_union.tcpFlag.cwr.value << 7);
            aclRule[tempIdx].data_bits.field[fieldIdx] |= (fieldPtr->filter_pattern_union.tcpFlag.ece.value << 6);
            aclRule[tempIdx].data_bits.field[fieldIdx] |= (fieldPtr->filter_pattern_union.tcpFlag.urg.value << 5);
            aclRule[tempIdx].data_bits.field[fieldIdx] |= (fieldPtr->filter_pattern_union.tcpFlag.ack.value << 4);
            aclRule[tempIdx].data_bits.field[fieldIdx] |= (fieldPtr->filter_pattern_union.tcpFlag.psh.value << 3);
            aclRule[tempIdx].data_bits.field[fieldIdx] |= (fieldPtr->filter_pattern_union.tcpFlag.rst.value << 2);
            aclRule[tempIdx].data_bits.field[fieldIdx] |= (fieldPtr->filter_pattern_union.tcpFlag.syn.value << 1);
            aclRule[tempIdx].data_bits.field[fieldIdx] |= fieldPtr->filter_pattern_union.tcpFlag.fin.value;

            aclRule[tempIdx].care_bits.field[fieldIdx] |= (fieldPtr->filter_pattern_union.tcpFlag.cwr.mask << 7);
            aclRule[tempIdx].care_bits.field[fieldIdx] |= (fieldPtr->filter_pattern_union.tcpFlag.ece.mask << 6);
            aclRule[tempIdx].care_bits.field[fieldIdx] |= (fieldPtr->filter_pattern_union.tcpFlag.urg.mask << 5);
            aclRule[tempIdx].care_bits.field[fieldIdx] |= (fieldPtr->filter_pattern_union.tcpFlag.ack.mask << 4);
            aclRule[tempIdx].care_bits.field[fieldIdx] |= (fieldPtr->filter_pattern_union.tcpFlag.psh.mask << 3);
            aclRule[tempIdx].care_bits.field[fieldIdx] |= (fieldPtr->filter_pattern_union.tcpFlag.rst.mask << 2);
            aclRule[tempIdx].care_bits.field[fieldIdx] |= (fieldPtr->filter_pattern_union.tcpFlag.syn.mask << 1);
            aclRule[tempIdx].care_bits.field[fieldIdx] |= fieldPtr->filter_pattern_union.tcpFlag.fin.mask;
        }
        break;
    case FILTER_FIELD_UDP_SPORT:
        for (i = 0; i < fieldPtr->fieldTemplateNo; i++)
        {
            tempIdx = (fieldPtr->fieldTemplateIdx[i] & 0xF0) >> 4;
            fieldIdx = fieldPtr->fieldTemplateIdx[i] & 0x0F;

            aclRule[tempIdx].data_bits.field[fieldIdx] = fieldPtr->filter_pattern_union.udpSrcPort.value;
            aclRule[tempIdx].care_bits.field[fieldIdx] = fieldPtr->filter_pattern_union.udpSrcPort.mask;
        }
        break;
    case FILTER_FIELD_UDP_DPORT:
        for (i = 0; i < fieldPtr->fieldTemplateNo; i++)
        {
            tempIdx = (fieldPtr->fieldTemplateIdx[i] & 0xF0) >> 4;
            fieldIdx = fieldPtr->fieldTemplateIdx[i] & 0x0F;

            aclRule[tempIdx].data_bits.field[fieldIdx] = fieldPtr->filter_pattern_union.udpDstPort.value;
            aclRule[tempIdx].care_bits.field[fieldIdx] = fieldPtr->filter_pattern_union.udpDstPort.mask;
        }
        break;
    case FILTER_FIELD_ICMP_CODE:
        for (i = 0; i < fieldPtr->fieldTemplateNo; i++)
        {
            tempIdx = (fieldPtr->fieldTemplateIdx[i] & 0xF0) >> 4;
            fieldIdx = fieldPtr->fieldTemplateIdx[i] & 0x0F;

            aclRule[tempIdx].data_bits.field[fieldIdx] &= 0xFF00;
            aclRule[tempIdx].data_bits.field[fieldIdx] |= fieldPtr->filter_pattern_union.icmpCode.value;
            aclRule[tempIdx].care_bits.field[fieldIdx] &= 0xFF00;
            aclRule[tempIdx].care_bits.field[fieldIdx] |= fieldPtr->filter_pattern_union.icmpCode.mask;
        }
        break;
    case FILTER_FIELD_ICMP_TYPE:
        for (i = 0; i < fieldPtr->fieldTemplateNo; i++)
        {
            tempIdx = (fieldPtr->fieldTemplateIdx[i] & 0xF0) >> 4;
            fieldIdx = fieldPtr->fieldTemplateIdx[i] & 0x0F;

            aclRule[tempIdx].data_bits.field[fieldIdx] &= 0x00FF;
            aclRule[tempIdx].data_bits.field[fieldIdx] |= (fieldPtr->filter_pattern_union.icmpType.value << 8);
            aclRule[tempIdx].care_bits.field[fieldIdx] &= 0x00FF;
            aclRule[tempIdx].care_bits.field[fieldIdx] |= (fieldPtr->filter_pattern_union.icmpType.mask << 8);
        }
        break;
    case FILTER_FIELD_IGMP_TYPE:
        for (i = 0; i < fieldPtr->fieldTemplateNo; i++)
        {
            tempIdx = (fieldPtr->fieldTemplateIdx[i] & 0xF0) >> 4;
            fieldIdx = fieldPtr->fieldTemplateIdx[i] & 0x0F;

            aclRule[tempIdx].data_bits.field[fieldIdx] = (fieldPtr->filter_pattern_union.igmpType.value << 8);
            aclRule[tempIdx].care_bits.field[fieldIdx] = (fieldPtr->filter_pattern_union.igmpType.mask << 8);
        }
        break;
    case FILTER_FIELD_PATTERN_MATCH:
        for (i = 0; i < fieldPtr->fieldTemplateNo; i++)
        {
            tempIdx = (fieldPtr->fieldTemplateIdx[i] & 0xF0) >> 4;
            fieldIdx = fieldPtr->fieldTemplateIdx[i] & 0x0F;

            aclRule[tempIdx].data_bits.field[fieldIdx] = ((fieldPtr->filter_pattern_union.pattern.value[i / 2] >> (16 * (i % 2))) & 0x0000FFFF);
            aclRule[tempIdx].care_bits.field[fieldIdx] = ((fieldPtr->filter_pattern_union.pattern.mask[i / 2] >> (16 * (i % 2))) & 0x0000FFFF);
        }
        break;
    case FILTER_FIELD_VID_RANGE:
    case FILTER_FIELD_IP_RANGE:
    case FILTER_FIELD_PORT_RANGE:
    default:
        tempIdx = (fieldPtr->fieldTemplateIdx[0] & 0xF0) >> 4;
        fieldIdx = fieldPtr->fieldTemplateIdx[0] & 0x0F;

        aclRule[tempIdx].data_bits.field[fieldIdx] = fieldPtr->filter_pattern_union.inData.value;
        aclRule[tempIdx].care_bits.field[fieldIdx] = fieldPtr->filter_pattern_union.inData.mask;
        break;
    }

    return RT_ERR_OK;
}

/* Function Name:
 *      rtk_svlan_checkAndCreateMbr
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
 *      RT_ERR_TBL_FULL     - Member Configuration table full
 * Note:
 *
 */
int32_t rtl8367::rtk_svlan_checkAndCreateMbr(uint32_t vid, uint32_t *pIndex)
{
    int32_t retVal;
    uint32_t svidx;
    uint32_t empty_idx = 0xFFFF;
    rtl8367c_svlan_memconf_t svlan_cfg;

    /* vid must be 0~4095 */
    if (vid > RTL8367C_VIDMAX)
        return RT_ERR_VLAN_VID;

    /* Null pointer check */
    if (NULL == pIndex)
        return RT_ERR_NULL_POINTER;

    /* Search exist entry */
    for (svidx = 0; svidx <= RTL8367C_SVIDXMAX; svidx++)
    {
        if (svlan_mbrCfgUsage[svidx] == 1)
        {
            if (svlan_mbrCfgVid[svidx] == vid)
            {
                /* Found! return index */
                *pIndex = svidx;
                return RT_ERR_OK;
            }
        }
        else if (empty_idx == 0xFFFF)
        {
            empty_idx = svidx;
        }
    }

    if (empty_idx == 0xFFFF)
    {
        /* No empty index */
        return RT_ERR_TBL_FULL;
    }

    svlan_mbrCfgUsage[empty_idx] = 1;
    svlan_mbrCfgVid[empty_idx] = vid;

    memset(&svlan_cfg, 0, sizeof(rtl8367c_svlan_memconf_t));

    svlan_cfg.vs_svid = vid;
    /*for create check*/
    if (vid == 0)
    {
        svlan_cfg.vs_efid = 1;
    }

    if ((retVal = rtl8367c_setAsicSvlanMemberConfiguration(empty_idx, &svlan_cfg)) != RT_ERR_OK)
        return retVal;

    *pIndex = empty_idx;
    return RT_ERR_OK;
}

void rtl8367::_rtl8367c_aclRuleStSmi2User(rtl8367c_aclrule *pAclUser, rtl8367c_aclrulesmi *pAclSmi)
{
    uint8_t *care_ptr, *data_ptr;
    uint8_t care_tmp, data_tmp;
    uint32_t i;

    pAclUser->data_bits.active_portmsk = (((pAclSmi->data_bits_ext.rule_info >> 1) & 0x0007) << 8) | ((pAclSmi->data_bits.rule_info >> 8) & 0x00FF);
    pAclUser->data_bits.type = (pAclSmi->data_bits.rule_info & 0x0007);
    pAclUser->data_bits.tag_exist = (pAclSmi->data_bits.rule_info & 0x00F8) >> 3;

    care_ptr = (uint8_t *)&pAclSmi->care_bits;
    data_ptr = (uint8_t *)&pAclSmi->data_bits;

    for (i = 0; i < sizeof(struct acl_rule_smi_st); i++)
    {
        care_tmp = *(care_ptr + i) ^ (*(data_ptr + i));
        data_tmp = *(data_ptr + i);

        *(care_ptr + i) = care_tmp;
        *(data_ptr + i) = data_tmp;
    }

    care_ptr = (uint8_t *)&pAclSmi->care_bits_ext;
    data_ptr = (uint8_t *)&pAclSmi->data_bits_ext;
    care_tmp = (*care_ptr) ^ (*data_ptr);
    data_tmp = (*data_ptr);
    *care_ptr = care_tmp;
    *data_ptr = data_tmp;

    for (i = 0; i < RTL8367C_ACLRULEFIELDNO; i++)
        pAclUser->data_bits.field[i] = pAclSmi->data_bits.field[i];

    pAclUser->valid = pAclSmi->valid;

    pAclUser->care_bits.active_portmsk = (((pAclSmi->care_bits_ext.rule_info >> 1) & 0x0007) << 8) | ((pAclSmi->care_bits.rule_info >> 8) & 0x00FF);
    pAclUser->care_bits.type = (pAclSmi->care_bits.rule_info & 0x0007);
    pAclUser->care_bits.tag_exist = (pAclSmi->care_bits.rule_info & 0x00F8) >> 3;

    for (i = 0; i < RTL8367C_ACLRULEFIELDNO; i++)
        pAclUser->care_bits.field[i] = pAclSmi->care_bits.field[i];
}
/* Function Name:
 *      rtl8367c_getAsicAclRule
 * Description:
 *      Get acl rule content
 * Input:
 *      index   - ACL rule index (0-63) of 64 ACL rules
 *      pAclRule - ACL rule stucture for setting
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK               - Success
 *      RT_ERR_SMI              - SMI access error
 *      RT_ERR_OUT_OF_RANGE     - Invalid ACL rule index (0-63)
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicAclRule(uint32_t index, rtl8367c_aclrule *pAclRule)
{
    rtl8367c_aclrulesmi aclRuleSmi;
    uint32_t regAddr, regData;
    int32_t retVal;
    uint16_t *tableAddr;
    uint32_t i;

    if (index > RTL8367C_ACLRULEMAX)
        return RT_ERR_OUT_OF_RANGE;

    memset(&aclRuleSmi, 0x00, sizeof(rtl8367c_aclrulesmi));

    /* Write ACS_ADR register for data bits */
    regAddr = RTL8367C_TABLE_ACCESS_ADDR_REG;
    if (index >= 64)
        regData = RTL8367C_ACLRULETBADDR2(DATABITS, index);
    else
        regData = RTL8367C_ACLRULETBADDR(DATABITS, index);

    retVal = rtl8367c_setAsicReg(regAddr, regData);
    if (retVal != RT_ERR_OK)
        return retVal;

    /* Write ACS_CMD register */
    regAddr = RTL8367C_TABLE_ACCESS_CTRL_REG;
    regData = RTL8367C_TABLE_ACCESS_REG_DATA(TB_OP_READ, TB_TARGET_ACLRULE);
    retVal = rtl8367c_setAsicRegBits(regAddr, RTL8367C_TABLE_TYPE_MASK | RTL8367C_COMMAND_TYPE_MASK, regData);
    if (retVal != RT_ERR_OK)
        return retVal;

    /* Read Data Bits */
    regAddr = RTL8367C_TABLE_ACCESS_RDDATA_BASE;
    tableAddr = (uint16_t *)&aclRuleSmi.data_bits;
    for (i = 0; i < RTL8367C_ACLRULETBLEN; i++)
    {
        retVal = rtl8367c_getAsicReg(regAddr, &regData);
        if (retVal != RT_ERR_OK)
            return retVal;

        *tableAddr = regData;

        regAddr++;
        tableAddr++;
    }

    /* Read Valid Bit */
    retVal = rtl8367c_getAsicRegBit(RTL8367C_TABLE_ACCESS_RDDATA_REG(RTL8367C_ACLRULETBLEN), 0, &regData);
    if (retVal != RT_ERR_OK)
        return retVal;
    aclRuleSmi.valid = regData & 0x1;
    /* Read active_portmsk_ext Bits */
    retVal = rtl8367c_getAsicRegBits(RTL8367C_TABLE_ACCESS_RDDATA_REG(RTL8367C_ACLRULETBLEN), 0x7 << 1, &regData);
    if (retVal != RT_ERR_OK)
        return retVal;
    aclRuleSmi.data_bits_ext.rule_info = (regData % 0x0007) << 1;

    /* Write ACS_ADR register for carebits*/
    regAddr = RTL8367C_TABLE_ACCESS_ADDR_REG;
    if (index >= 64)
        regData = RTL8367C_ACLRULETBADDR2(CAREBITS, index);
    else
        regData = RTL8367C_ACLRULETBADDR(CAREBITS, index);

    retVal = rtl8367c_setAsicReg(regAddr, regData);
    if (retVal != RT_ERR_OK)
        return retVal;

    /* Write ACS_CMD register */
    regAddr = RTL8367C_TABLE_ACCESS_CTRL_REG;
    regData = RTL8367C_TABLE_ACCESS_REG_DATA(TB_OP_READ, TB_TARGET_ACLRULE);
    retVal = rtl8367c_setAsicRegBits(regAddr, RTL8367C_TABLE_TYPE_MASK | RTL8367C_COMMAND_TYPE_MASK, regData);
    if (retVal != RT_ERR_OK)
        return retVal;

    /* Read Care Bits */
    regAddr = RTL8367C_TABLE_ACCESS_RDDATA_BASE;
    tableAddr = (uint16_t *)&aclRuleSmi.care_bits;
    for (i = 0; i < RTL8367C_ACLRULETBLEN; i++)
    {
        retVal = rtl8367c_getAsicReg(regAddr, &regData);
        if (retVal != RT_ERR_OK)
            return retVal;

        *tableAddr = regData;

        regAddr++;
        tableAddr++;
    }
    /* Read active_portmsk_ext care Bits */
    retVal = rtl8367c_getAsicRegBits(RTL8367C_TABLE_ACCESS_RDDATA_REG(RTL8367C_ACLRULETBLEN), 0x7 << 1, &regData);
    if (retVal != RT_ERR_OK)
        return retVal;
    aclRuleSmi.care_bits_ext.rule_info = (regData & 0x0007) << 1;

    _rtl8367c_aclRuleStSmi2User(pAclRule, &aclRuleSmi);

    return RT_ERR_OK;
}

void rtl8367::_rtl8367c_aclActStUser2Smi(rtl8367c_acl_act_t *pAclUser, uint16_t *pAclSmi)
{
    pAclSmi[0] |= (pAclUser->cvidx_cact & 0x003F);
    pAclSmi[0] |= (pAclUser->cact & 0x0003) << 6;
    pAclSmi[0] |= (pAclUser->svidx_sact & 0x003F) << 8;
    pAclSmi[0] |= (pAclUser->sact & 0x0003) << 14;

    pAclSmi[1] |= (pAclUser->aclmeteridx & 0x003F);
    pAclSmi[1] |= (pAclUser->fwdpmask & 0x00FF) << 6;
    pAclSmi[1] |= (pAclUser->fwdact & 0x0003) << 14;

    pAclSmi[2] |= (pAclUser->pridx & 0x003F);
    pAclSmi[2] |= (pAclUser->priact & 0x0003) << 6;
    pAclSmi[2] |= (pAclUser->gpio_pin & 0x000F) << 8;
    pAclSmi[2] |= (pAclUser->gpio_en & 0x0001) << 12;
    pAclSmi[2] |= (pAclUser->aclint & 0x0001) << 13;
    pAclSmi[2] |= (pAclUser->cact_ext & 0x0003) << 14;

    pAclSmi[3] |= (pAclUser->tag_fmt & 0x0003);
    pAclSmi[3] |= (pAclUser->fwdact_ext & 0x0001) << 2;
    pAclSmi[3] |= ((pAclUser->cvidx_cact & 0x0040) >> 6) << 3;
    pAclSmi[3] |= ((pAclUser->svidx_sact & 0x0040) >> 6) << 4;
    pAclSmi[3] |= ((pAclUser->aclmeteridx & 0x0040) >> 6) << 5;
    pAclSmi[3] |= ((pAclUser->fwdpmask & 0x0700) >> 8) << 6;
    pAclSmi[3] |= ((pAclUser->pridx & 0x0040) >> 6) << 9;
}

/* Function Name:
 *      rtl8367c_setAsicAclAct
 * Description:
 *      Set ACL rule matched Action
 * Input:
 *      index   - ACL rule index (0-95) of 96 ACL rules
 *      pAclAct     - ACL action stucture for setting
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK               - Success
 *      RT_ERR_SMI              - SMI access error
 *      RT_ERR_OUT_OF_RANGE     - Invalid ACL rule index (0-95)
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicAclAct(uint32_t index, rtl8367c_acl_act_t *pAclAct)
{
    uint16_t aclActSmi[RTL8367C_ACL_ACT_TABLE_LEN];
    int32_t retVal;
    uint32_t regAddr, regData;
    uint16_t *tableAddr;
    uint32_t i;

    if (index > RTL8367C_ACLRULEMAX)
        return RT_ERR_OUT_OF_RANGE;

    memset(aclActSmi, 0x00, sizeof(uint16_t) * RTL8367C_ACL_ACT_TABLE_LEN);
    _rtl8367c_aclActStUser2Smi(pAclAct, aclActSmi);

    /* Write ACS_ADR register for data bits */
    regAddr = RTL8367C_TABLE_ACCESS_ADDR_REG;
    regData = index;
    retVal = rtl8367c_setAsicReg(regAddr, regData);
    if (retVal != RT_ERR_OK)
        return retVal;

    /* Write Data Bits to ACS_DATA registers */
    tableAddr = aclActSmi;
    regAddr = RTL8367C_TABLE_ACCESS_WRDATA_BASE;

    for (i = 0; i < RTL8367C_ACLACTTBLEN; i++)
    {
        regData = *tableAddr;
        retVal = rtl8367c_setAsicReg(regAddr, regData);
        if (retVal != RT_ERR_OK)
            return retVal;

        regAddr++;
        tableAddr++;
    }

    /* Write ACS_CMD register for care bits*/
    regAddr = RTL8367C_TABLE_ACCESS_CTRL_REG;
    regData = RTL8367C_TABLE_ACCESS_REG_DATA(TB_OP_WRITE, TB_TARGET_ACLACT);
    retVal = rtl8367c_setAsicRegBits(regAddr, RTL8367C_TABLE_TYPE_MASK | RTL8367C_COMMAND_TYPE_MASK, regData);
    if (retVal != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

void rtl8367::_rtl8367c_aclRuleStUser2Smi(rtl8367c_aclrule *pAclUser, rtl8367c_aclrulesmi *pAclSmi)
{
    uint8_t *care_ptr, *data_ptr;
    uint8_t care_tmp, data_tmp;
    uint32_t i;

    pAclSmi->data_bits_ext.rule_info = ((pAclUser->data_bits.active_portmsk >> 8) & 0x7) << 1;
    pAclSmi->data_bits.rule_info = ((pAclUser->data_bits.active_portmsk & 0xff) << 8) | ((pAclUser->data_bits.tag_exist & 0x1F) << 3) | (pAclUser->data_bits.type & 0x07);

    for (i = 0; i < RTL8367C_ACLRULEFIELDNO; i++)
        pAclSmi->data_bits.field[i] = pAclUser->data_bits.field[i];

    pAclSmi->valid = pAclUser->valid;

    pAclSmi->care_bits_ext.rule_info = ((pAclUser->care_bits.active_portmsk >> 8) & 0x7) << 1;
    pAclSmi->care_bits.rule_info = ((pAclUser->care_bits.active_portmsk & 0xff) << 8) | ((pAclUser->care_bits.tag_exist & 0x1F) << 3) | (pAclUser->care_bits.type & 0x07);

    for (i = 0; i < RTL8367C_ACLRULEFIELDNO; i++)
        pAclSmi->care_bits.field[i] = pAclUser->care_bits.field[i];

    care_ptr = (uint8_t *)&pAclSmi->care_bits;
    data_ptr = (uint8_t *)&pAclSmi->data_bits;

    for (i = 0; i < sizeof(struct acl_rule_smi_st); i++)
    {
        care_tmp = *(care_ptr + i) & ~(*(data_ptr + i));
        data_tmp = *(care_ptr + i) & *(data_ptr + i);

        *(care_ptr + i) = care_tmp;
        *(data_ptr + i) = data_tmp;
    }

    care_ptr = (uint8_t *)&pAclSmi->care_bits_ext;
    data_ptr = (uint8_t *)&pAclSmi->data_bits_ext;
    care_tmp = *care_ptr & ~(*data_ptr);
    data_tmp = *care_ptr & *data_ptr;

    *care_ptr = care_tmp;
    *data_ptr = data_tmp;
}

/* Function Name:
 *      rtl8367c_setAsicAclRule
 * Description:
 *      Set acl rule content
 * Input:
 *      index   - ACL rule index (0-95) of 96 ACL rules
 *      pAclRule - ACL rule stucture for setting
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK               - Success
 *      RT_ERR_SMI              - SMI access error
 *      RT_ERR_OUT_OF_RANGE     - Invalid ACL rule index (0-95)
 * Note:
 *      System supported 95 shared 289-bit ACL ingress rule. Index was available at range 0-95 only.
 *      If software want to modify ACL rule, the ACL function should be disable at first or unspecify
 *      acl action will be executed.
 *      One ACL rule structure has three parts setting:
 *      Bit 0-147       Data Bits of this Rule
 *      Bit 148     Valid Bit
 *      Bit 149-296 Care Bits of this Rule
 *      There are four kinds of field in Data Bits and Care Bits: Active Portmask, Type, Tag Exist, and 8 fields
 */
int32_t rtl8367::rtl8367c_setAsicAclRule(uint32_t index, rtl8367c_aclrule *pAclRule)
{
    rtl8367c_aclrulesmi aclRuleSmi;
    uint16_t *tableAddr;
    uint32_t regAddr;
    uint32_t regData;
    uint32_t i;
    int32_t retVal;

    if (index > RTL8367C_ACLRULEMAX)
        return RT_ERR_OUT_OF_RANGE;

    memset(&aclRuleSmi, 0x00, sizeof(rtl8367c_aclrulesmi));

    _rtl8367c_aclRuleStUser2Smi(pAclRule, &aclRuleSmi);

    /* Write valid bit = 0 */
    regAddr = RTL8367C_TABLE_ACCESS_ADDR_REG;
    if (index >= 64)
        regData = RTL8367C_ACLRULETBADDR2(DATABITS, index);
    else
        regData = RTL8367C_ACLRULETBADDR(DATABITS, index);
    retVal = rtl8367c_setAsicReg(regAddr, regData);
    if (retVal != RT_ERR_OK)
        return retVal;

    retVal = rtl8367c_setAsicRegBits(RTL8367C_TABLE_ACCESS_WRDATA_REG(RTL8367C_ACLRULETBLEN), 0x1, 0);
    if (retVal != RT_ERR_OK)
        return retVal;

    regAddr = RTL8367C_TABLE_ACCESS_CTRL_REG;
    regData = RTL8367C_TABLE_ACCESS_REG_DATA(TB_OP_WRITE, TB_TARGET_ACLRULE);
    retVal = rtl8367c_setAsicReg(regAddr, regData);
    if (retVal != RT_ERR_OK)
        return retVal;

    /* Write ACS_ADR register */
    regAddr = RTL8367C_TABLE_ACCESS_ADDR_REG;
    if (index >= 64)
        regData = RTL8367C_ACLRULETBADDR2(CAREBITS, index);
    else
        regData = RTL8367C_ACLRULETBADDR(CAREBITS, index);
    retVal = rtl8367c_setAsicReg(regAddr, regData);
    if (retVal != RT_ERR_OK)
        return retVal;

    /* Write Care Bits to ACS_DATA registers */
    tableAddr = (uint16_t *)&aclRuleSmi.care_bits;
    regAddr = RTL8367C_TABLE_ACCESS_WRDATA_BASE;

    for (i = 0; i < RTL8367C_ACLRULETBLEN; i++)
    {
        regData = *tableAddr;
        retVal = rtl8367c_setAsicReg(regAddr, regData);
        if (retVal != RT_ERR_OK)
            return retVal;

        regAddr++;
        tableAddr++;
    }
    retVal = rtl8367c_setAsicRegBits(RTL8367C_TABLE_ACCESS_WRDATA_REG(RTL8367C_ACLRULETBLEN), (0x0007 << 1), (aclRuleSmi.care_bits_ext.rule_info >> 1) & 0x0007);
    if (retVal != RT_ERR_OK)
        return retVal;

    /* Write ACS_CMD register */
    regAddr = RTL8367C_TABLE_ACCESS_CTRL_REG;
    regData = RTL8367C_TABLE_ACCESS_REG_DATA(TB_OP_WRITE, TB_TARGET_ACLRULE);
    retVal = rtl8367c_setAsicRegBits(regAddr, RTL8367C_TABLE_TYPE_MASK | RTL8367C_COMMAND_TYPE_MASK, regData);
    if (retVal != RT_ERR_OK)
        return retVal;

    /* Write ACS_ADR register for data bits */
    regAddr = RTL8367C_TABLE_ACCESS_ADDR_REG;
    if (index >= 64)
        regData = RTL8367C_ACLRULETBADDR2(DATABITS, index);
    else
        regData = RTL8367C_ACLRULETBADDR(DATABITS, index);

    retVal = rtl8367c_setAsicReg(regAddr, regData);
    if (retVal != RT_ERR_OK)
        return retVal;

    /* Write Data Bits to ACS_DATA registers */
    tableAddr = (uint16_t *)&aclRuleSmi.data_bits;
    regAddr = RTL8367C_TABLE_ACCESS_WRDATA_BASE;

    for (i = 0; i < RTL8367C_ACLRULETBLEN; i++)
    {
        regData = *tableAddr;
        retVal = rtl8367c_setAsicReg(regAddr, regData);
        if (retVal != RT_ERR_OK)
            return retVal;

        regAddr++;
        tableAddr++;
    }

    retVal = rtl8367c_setAsicRegBit(RTL8367C_TABLE_ACCESS_WRDATA_REG(RTL8367C_ACLRULETBLEN), 0, aclRuleSmi.valid);
    if (retVal != RT_ERR_OK)
        return retVal;
    retVal = rtl8367c_setAsicRegBits(RTL8367C_TABLE_ACCESS_WRDATA_REG(RTL8367C_ACLRULETBLEN), (0x0007 << 1), (aclRuleSmi.data_bits_ext.rule_info >> 1) & 0x0007);
    if (retVal != RT_ERR_OK)
        return retVal;

    /* Write ACS_CMD register for care bits*/
    regAddr = RTL8367C_TABLE_ACCESS_CTRL_REG;
    regData = RTL8367C_TABLE_ACCESS_REG_DATA(TB_OP_WRITE, TB_TARGET_ACLRULE);
    retVal = rtl8367c_setAsicRegBits(regAddr, RTL8367C_TABLE_TYPE_MASK | RTL8367C_COMMAND_TYPE_MASK, regData);
    if (retVal != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_filter_igrAcl_cfg_add(rtk_filter_id_t filter_id, rtk_filter_cfg_t *pFilter_cfg, rtk_filter_action_t *pFilter_action, uint32_t *ruleNum)
{
    int32_t retVal;
    uint32_t careTagData, careTagMask;
    uint32_t i, vidx, svidx, actType, ruleId;
    uint32_t aclActCtrl;
    uint32_t cpuPort;
    rtk_filter_field_t *fieldPtr;
    rtl8367c_aclrule aclRule[RTL8367C_ACLTEMPLATENO];
    rtl8367c_aclrule tempRule;
    rtl8367c_acl_act_t aclAct;
    uint32_t noRulesAdd;
    uint32_t portmask;

    if (filter_id > RTL8367C_ACLRULEMAX)
        return RT_ERR_ENTRY_INDEX;

    if ((NULL == pFilter_cfg) || (NULL == pFilter_action) || (NULL == ruleNum))
        return RT_ERR_NULL_POINTER;

    fieldPtr = pFilter_cfg->fieldHead;

    /* init RULE */
    for (i = 0; i < RTL8367C_ACLTEMPLATENO; i++)
    {
        memset(&aclRule[i], 0, sizeof(rtl8367c_aclrule));

        aclRule[i].data_bits.type = i;
        aclRule[i].care_bits.type = 0x7;
    }

    while (NULL != fieldPtr)
    {
        _rtk_filter_igrAcl_writeDataField(aclRule, fieldPtr);

        fieldPtr = fieldPtr->next;
    }

    /*set care tag mask in User Defined Field 15*/
    /*Follow care tag should not be used while ACL template and User defined fields are fully control by system designer*/
    /*those advanced packet type care tag is used in default template design structure only*/
    careTagData = 0;
    careTagMask = 0;

    for (i = CARE_TAG_TCP; i < CARE_TAG_END; i++)
    {
        if (pFilter_cfg->careTag.tagType[i].mask)
            careTagMask = careTagMask | (1 << (i - CARE_TAG_TCP));

        if (pFilter_cfg->careTag.tagType[i].value)
            careTagData = careTagData | (1 << (i - CARE_TAG_TCP));
    }

    if (careTagData || careTagMask)
    {
        i = 0;
        while (i < RTL8367C_ACLTEMPLATENO)
        {
            if (aclRule[i].valid == 1 && filter_advanceCaretagField[i][0] == 1)
            {

                aclRule[i].data_bits.field[filter_advanceCaretagField[i][1]] = careTagData & 0xFFFF;
                aclRule[i].care_bits.field[filter_advanceCaretagField[i][1]] = careTagMask & 0xFFFF;
                break;
            }
            i++;
        }
        /*none of previous used template containing field 15*/
        if (i == RTL8367C_ACLTEMPLATENO)
        {
            i = 0;
            while (i < RTL8367C_ACLTEMPLATENO)
            {
                if (filter_advanceCaretagField[i][0] == 1)
                {
                    aclRule[i].data_bits.field[filter_advanceCaretagField[i][1]] = careTagData & 0xFFFF;
                    aclRule[i].care_bits.field[filter_advanceCaretagField[i][1]] = careTagMask & 0xFFFF;
                    aclRule[i].valid = 1;
                    break;
                }
                i++;
            }
        }
    }

    /*Check rule number*/
    noRulesAdd = 0;
    for (i = 0; i < RTL8367C_ACLTEMPLATENO; i++)
    {
        if (1 == aclRule[i].valid)
        {
            noRulesAdd++;
        }
    }

    *ruleNum = noRulesAdd;

    if ((filter_id + noRulesAdd - 1) > RTL8367C_ACLRULEMAX)
    {
        return RT_ERR_ENTRY_INDEX;
    }

    /*set care tag mask in TAG Indicator*/
    careTagData = 0;
    careTagMask = 0;

    for (i = 0; i <= CARE_TAG_IPV6; i++)
    {
        if (0 == pFilter_cfg->careTag.tagType[i].mask)
        {
            careTagMask &= ~(1 << i);
        }
        else
        {
            careTagMask |= (1 << i);
            if (0 == pFilter_cfg->careTag.tagType[i].value)
                careTagData &= ~(1 << i);
            else
                careTagData |= (1 << i);
        }
    }

    for (i = 0; i < RTL8367C_ACLTEMPLATENO; i++)
    {
        aclRule[i].data_bits.tag_exist = (careTagData)&ACL_RULE_CARETAG_MASK;
        aclRule[i].care_bits.tag_exist = (careTagMask)&ACL_RULE_CARETAG_MASK;
    }

    RTK_CHK_PORTMASK_VALID(&pFilter_cfg->activeport.value);
    RTK_CHK_PORTMASK_VALID(&pFilter_cfg->activeport.mask);

    for (i = 0; i < RTL8367C_ACLTEMPLATENO; i++)
    {
        if (1 == aclRule[i].valid)
        {
            if (rtk_switch_portmask_L2P_get(&pFilter_cfg->activeport.value, &portmask) != RT_ERR_OK)
                return RT_ERR_PORT_MASK;

            aclRule[i].data_bits.active_portmsk = portmask;

            if (rtk_switch_portmask_L2P_get(&pFilter_cfg->activeport.mask, &portmask) != RT_ERR_OK)
                return RT_ERR_PORT_MASK;

            aclRule[i].care_bits.active_portmsk = portmask;
        }
    }

    if (pFilter_cfg->invert >= FILTER_INVERT_END)
        return RT_ERR_INPUT;

    /*Last action gets high priority if actions are the same*/
    memset(&aclAct, 0, sizeof(rtl8367c_acl_act_t));
    aclActCtrl = 0;
    for (actType = 0; actType < FILTER_ENACT_END; actType++)
    {
        if (pFilter_action->actEnable[actType])
        {
            switch (actType)
            {
            case FILTER_ENACT_CVLAN_INGRESS:
                if (pFilter_action->filterCvlanVid > RTL8367C_EVIDMAX)
                    return RT_ERR_INPUT;

                if ((retVal = rtk_vlan_checkAndCreateMbr(pFilter_action->filterCvlanVid, &vidx)) != RT_ERR_OK)
                {
                    return retVal;
                }
                aclAct.cact = FILTER_ENACT_CVLAN_TYPE(actType);
                aclAct.cvidx_cact = vidx;

                if (aclActCtrl & (FILTER_ENACT_CVLAN_MASK))
                {
                    if (aclAct.cact_ext == FILTER_ENACT_CACTEXT_TAGONLY)
                        aclAct.cact_ext = FILTER_ENACT_CACTEXT_BOTHVLANTAG;
                }
                else
                {
                    aclAct.cact_ext = FILTER_ENACT_CACTEXT_VLANONLY;
                }

                aclActCtrl |= FILTER_ENACT_CVLAN_MASK;
                break;
            case FILTER_ENACT_CVLAN_EGRESS:
                if (pFilter_action->filterCvlanVid > RTL8367C_EVIDMAX)
                    return RT_ERR_INPUT;

                if ((retVal = rtk_vlan_checkAndCreateMbr(pFilter_action->filterCvlanVid, &vidx)) != RT_ERR_OK)
                    return retVal;

                aclAct.cact = FILTER_ENACT_CVLAN_TYPE(actType);
                aclAct.cvidx_cact = vidx;

                if (aclActCtrl & (FILTER_ENACT_CVLAN_MASK))
                {
                    if (aclAct.cact_ext == FILTER_ENACT_CACTEXT_TAGONLY)
                        aclAct.cact_ext = FILTER_ENACT_CACTEXT_BOTHVLANTAG;
                }
                else
                {
                    aclAct.cact_ext = FILTER_ENACT_CACTEXT_VLANONLY;
                }

                aclActCtrl |= FILTER_ENACT_CVLAN_MASK;
                break;
            case FILTER_ENACT_CVLAN_SVID:

                aclAct.cact = FILTER_ENACT_CVLAN_TYPE(actType);

                if (aclActCtrl & (FILTER_ENACT_CVLAN_MASK))
                {
                    if (aclAct.cact_ext == FILTER_ENACT_CACTEXT_TAGONLY)
                        aclAct.cact_ext = FILTER_ENACT_CACTEXT_BOTHVLANTAG;
                }
                else
                {
                    aclAct.cact_ext = FILTER_ENACT_CACTEXT_VLANONLY;
                }

                aclActCtrl |= FILTER_ENACT_CVLAN_MASK;
                break;
            case FILTER_ENACT_POLICING_1:
                if (pFilter_action->filterPolicingIdx[1] >= ((halCtrl.max_meter_id + 1) + RTL8367C_MAX_LOG_CNT_NUM))
                    return RT_ERR_INPUT;

                aclAct.cact = FILTER_ENACT_CVLAN_TYPE(actType);
                aclAct.cvidx_cact = pFilter_action->filterPolicingIdx[1];

                if (aclActCtrl & (FILTER_ENACT_CVLAN_MASK))
                {
                    if (aclAct.cact_ext == FILTER_ENACT_CACTEXT_TAGONLY)
                        aclAct.cact_ext = FILTER_ENACT_CACTEXT_BOTHVLANTAG;
                }
                else
                {
                    aclAct.cact_ext = FILTER_ENACT_CACTEXT_VLANONLY;
                }

                aclActCtrl |= FILTER_ENACT_CVLAN_MASK;
                break;

            case FILTER_ENACT_SVLAN_INGRESS:
            case FILTER_ENACT_SVLAN_EGRESS:

                if ((retVal = rtk_svlan_checkAndCreateMbr(pFilter_action->filterSvlanVid, &svidx)) != RT_ERR_OK)
                    return retVal;

                aclAct.sact = FILTER_ENACT_SVLAN_TYPE(actType);
                aclAct.svidx_sact = svidx;
                aclActCtrl |= FILTER_ENACT_SVLAN_MASK;
                break;
            case FILTER_ENACT_SVLAN_CVID:

                aclAct.sact = FILTER_ENACT_SVLAN_TYPE(actType);
                aclActCtrl |= FILTER_ENACT_SVLAN_MASK;
                break;
            case FILTER_ENACT_POLICING_2:
                if (pFilter_action->filterPolicingIdx[2] >= ((halCtrl.max_meter_id + 1) + RTL8367C_MAX_LOG_CNT_NUM))
                    return RT_ERR_INPUT;

                aclAct.sact = FILTER_ENACT_SVLAN_TYPE(actType);
                aclAct.svidx_sact = pFilter_action->filterPolicingIdx[2];
                aclActCtrl |= FILTER_ENACT_SVLAN_MASK;
                break;
            case FILTER_ENACT_POLICING_0:
                if (pFilter_action->filterPolicingIdx[0] >= ((halCtrl.max_meter_id + 1) + RTL8367C_MAX_LOG_CNT_NUM))
                    return RT_ERR_INPUT;

                aclAct.aclmeteridx = pFilter_action->filterPolicingIdx[0];
                aclActCtrl |= FILTER_ENACT_POLICING_MASK;
                break;
            case FILTER_ENACT_PRIORITY:
            case FILTER_ENACT_1P_REMARK:
                if (pFilter_action->filterPriority > RTL8367C_PRIMAX)
                    return RT_ERR_INPUT;

                aclAct.priact = FILTER_ENACT_PRI_TYPE(actType);
                aclAct.pridx = pFilter_action->filterPriority;
                aclActCtrl |= FILTER_ENACT_PRIORITY_MASK;
                break;
            case FILTER_ENACT_DSCP_REMARK:
                if (pFilter_action->filterPriority > RTL8367C_DSCPMAX)
                    return RT_ERR_INPUT;

                aclAct.priact = FILTER_ENACT_PRI_TYPE(actType);
                aclAct.pridx = pFilter_action->filterPriority;
                aclActCtrl |= FILTER_ENACT_PRIORITY_MASK;
                break;
            case FILTER_ENACT_POLICING_3:
                if (pFilter_action->filterPriority >= ((halCtrl.max_meter_id + 1) + RTL8367C_MAX_LOG_CNT_NUM))
                    return RT_ERR_INPUT;

                aclAct.priact = FILTER_ENACT_PRI_TYPE(actType);
                aclAct.pridx = pFilter_action->filterPolicingIdx[3];
                aclActCtrl |= FILTER_ENACT_PRIORITY_MASK;
                break;
            case FILTER_ENACT_DROP:

                aclAct.fwdact = FILTER_ENACT_FWD_TYPE(FILTER_ENACT_REDIRECT);
                aclAct.fwdact_ext = 0;

                aclAct.fwdpmask = 0;
                aclActCtrl |= FILTER_ENACT_FWD_MASK;
                break;
            case FILTER_ENACT_REDIRECT:
                RTK_CHK_PORTMASK_VALID(&pFilter_action->filterPortmask);

                aclAct.fwdact = FILTER_ENACT_FWD_TYPE(actType);
                aclAct.fwdact_ext = 0;

                if (rtk_switch_portmask_L2P_get(&pFilter_action->filterPortmask, &portmask) != RT_ERR_OK)
                    return RT_ERR_PORT_MASK;
                aclAct.fwdpmask = portmask;

                aclActCtrl |= FILTER_ENACT_FWD_MASK;
                break;

            case FILTER_ENACT_ADD_DSTPORT:
                RTK_CHK_PORTMASK_VALID(&pFilter_action->filterPortmask);

                aclAct.fwdact = FILTER_ENACT_FWD_TYPE(actType);
                aclAct.fwdact_ext = 0;

                if (rtk_switch_portmask_L2P_get(&pFilter_action->filterPortmask, &portmask) != RT_ERR_OK)
                    return RT_ERR_PORT_MASK;
                aclAct.fwdpmask = portmask;

                aclActCtrl |= FILTER_ENACT_FWD_MASK;
                break;
            case FILTER_ENACT_MIRROR:
                RTK_CHK_PORTMASK_VALID(&pFilter_action->filterPortmask);

                aclAct.fwdact = FILTER_ENACT_FWD_TYPE(actType);
                aclAct.cact_ext = 0;

                if (rtk_switch_portmask_L2P_get(&pFilter_action->filterPortmask, &portmask) != RT_ERR_OK)
                    return RT_ERR_PORT_MASK;
                aclAct.fwdpmask = portmask;

                aclActCtrl |= FILTER_ENACT_FWD_MASK;
                break;
            case FILTER_ENACT_TRAP_CPU:

                aclAct.fwdact = FILTER_ENACT_FWD_TYPE(actType);
                aclAct.fwdact_ext = 0;

                aclActCtrl |= FILTER_ENACT_FWD_MASK;
                break;
            case FILTER_ENACT_COPY_CPU:
                if ((retVal = rtl8367c_getAsicCputagTrapPort(&cpuPort)) != RT_ERR_OK)
                    return retVal;

                aclAct.fwdact = FILTER_ENACT_FWD_TYPE(FILTER_ENACT_MIRROR);
                aclAct.fwdact_ext = 0;

                aclAct.fwdpmask = 1 << cpuPort;
                aclActCtrl |= FILTER_ENACT_FWD_MASK;
                break;
            case FILTER_ENACT_ISOLATION:
                RTK_CHK_PORTMASK_VALID(&pFilter_action->filterPortmask);

                aclAct.fwdact_ext = 1;

                if (rtk_switch_portmask_L2P_get(&pFilter_action->filterPortmask, &portmask) != RT_ERR_OK)
                    return RT_ERR_PORT_MASK;
                aclAct.fwdpmask = portmask;

                aclActCtrl |= FILTER_ENACT_FWD_MASK;
                break;

            case FILTER_ENACT_INTERRUPT:

                aclAct.aclint = 1;
                aclActCtrl |= FILTER_ENACT_INTGPIO_MASK;
                break;
            case FILTER_ENACT_GPO:

                aclAct.gpio_en = 1;
                aclAct.gpio_pin = pFilter_action->filterPin;
                aclActCtrl |= FILTER_ENACT_INTGPIO_MASK;
                break;
            case FILTER_ENACT_EGRESSCTAG_TAG:

                if (aclActCtrl & (FILTER_ENACT_CVLAN_MASK))
                {
                    if (aclAct.cact_ext == FILTER_ENACT_CACTEXT_VLANONLY)
                        aclAct.cact_ext = FILTER_ENACT_CACTEXT_BOTHVLANTAG;
                }
                else
                {
                    aclAct.cact_ext = FILTER_ENACT_CACTEXT_TAGONLY;
                }
                aclAct.tag_fmt = FILTER_CTAGFMT_TAG;
                aclActCtrl |= FILTER_ENACT_CVLAN_MASK;
                break;
            case FILTER_ENACT_EGRESSCTAG_UNTAG:

                if (aclActCtrl & (FILTER_ENACT_CVLAN_MASK))
                {
                    if (aclAct.cact_ext == FILTER_ENACT_CACTEXT_VLANONLY)
                        aclAct.cact_ext = FILTER_ENACT_CACTEXT_BOTHVLANTAG;
                }
                else
                {
                    aclAct.cact_ext = FILTER_ENACT_CACTEXT_TAGONLY;
                }
                aclAct.tag_fmt = FILTER_CTAGFMT_UNTAG;
                aclActCtrl |= FILTER_ENACT_CVLAN_MASK;
                break;
            case FILTER_ENACT_EGRESSCTAG_KEEP:

                if (aclActCtrl & (FILTER_ENACT_CVLAN_MASK))
                {
                    if (aclAct.cact_ext == FILTER_ENACT_CACTEXT_VLANONLY)
                        aclAct.cact_ext = FILTER_ENACT_CACTEXT_BOTHVLANTAG;
                }
                else
                {
                    aclAct.cact_ext = FILTER_ENACT_CACTEXT_TAGONLY;
                }
                aclAct.tag_fmt = FILTER_CTAGFMT_KEEP;
                aclActCtrl |= FILTER_ENACT_CVLAN_MASK;
                break;
            case FILTER_ENACT_EGRESSCTAG_KEEPAND1PRMK:

                if (aclActCtrl & (FILTER_ENACT_CVLAN_MASK))
                {
                    if (aclAct.cact_ext == FILTER_ENACT_CACTEXT_VLANONLY)
                        aclAct.cact_ext = FILTER_ENACT_CACTEXT_BOTHVLANTAG;
                }
                else
                {
                    aclAct.cact_ext = FILTER_ENACT_CACTEXT_TAGONLY;
                }
                aclAct.tag_fmt = FILTER_CTAGFMT_KEEP1PRMK;
                aclActCtrl |= FILTER_ENACT_CVLAN_MASK;
                break;
            default:
                return RT_ERR_FILTER_INACL_ACT_NOT_SUPPORT;
            }
        }
    }

    /*check if free ACL rules are enough*/
    for (i = filter_id; i < (filter_id + noRulesAdd); i++)
    {
        if ((retVal = rtl8367c_getAsicAclRule(i, &tempRule)) != RT_ERR_OK)
            return retVal;

        if (tempRule.valid == 1)
        {
            return RT_ERR_TBL_FULL;
        }
    }

    ruleId = 0;
    for (i = 0; i < RTL8367C_ACLTEMPLATENO; i++)
    {
        if (aclRule[i].valid == 1)
        {
            /* write ACL action control */
            if ((retVal = rtl8367c_setAsicAclActCtrl(filter_id + ruleId, aclActCtrl)) != RT_ERR_OK)
                return retVal;
            /* write ACL action */
            if ((retVal = rtl8367c_setAsicAclAct(filter_id + ruleId, &aclAct)) != RT_ERR_OK)
                return retVal;

            /* write ACL not */
            if ((retVal = rtl8367c_setAsicAclNot(filter_id + ruleId, pFilter_cfg->invert)) != RT_ERR_OK)
                return retVal;
            /* write ACL rule */
            if ((retVal = rtl8367c_setAsicAclRule(filter_id + ruleId, &aclRule[i])) != RT_ERR_OK)
                return retVal;

            /* only the first rule will be written with input action control, aclActCtrl of other rules will be zero */
            aclActCtrl = 0;
            memset(&aclAct, 0, sizeof(rtl8367c_acl_act_t));

            ruleId++;
        }
    }

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_filter_igrAcl_cfg_del(rtk_filter_id_t filter_id)
{
    rtl8367c_aclrule initRule;
    rtl8367c_acl_act_t initAct;
    int32_t ret;

    if (filter_id > RTL8367C_ACLRULEMAX)
        return RT_ERR_FILTER_ENTRYIDX;

    memset(&initRule, 0, sizeof(rtl8367c_aclrule));
    memset(&initAct, 0, sizeof(rtl8367c_acl_act_t));

    if ((ret = rtl8367c_setAsicAclRule(filter_id, &initRule)) != RT_ERR_OK)
        return ret;
    if ((ret = rtl8367c_setAsicAclActCtrl(filter_id, FILTER_ENACT_INIT_MASK)) != RT_ERR_OK)
        return ret;
    if ((ret = rtl8367c_setAsicAclAct(filter_id, &initAct)) != RT_ERR_OK)
        return ret;
    if ((ret = rtl8367c_setAsicAclNot(filter_id, DISABLED)) != RT_ERR_OK)
        return ret;

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_filter_igrAcl_cfg_delAll()
{
    uint32_t i;
    int32_t ret;

    for (i = 0; i < RTL8367C_ACLRULENO; i++)
    {
        if ((ret = rtl8367c_setAsicAclActCtrl(i, FILTER_ENACT_INIT_MASK)) != RT_ERR_OK)
            return ret;
        if ((ret = rtl8367c_setAsicAclNot(i, DISABLED)) != RT_ERR_OK)
            return ret;
    }

    return rtl8367c_setAsicRegBit(RTL8367C_REG_ACL_RESET_CFG, RTL8367C_ACL_RESET_CFG_OFFSET, 1);
}

/* Function Name:
 *      rtl8367c_getAsicAcl
 * Description:
 *      Get rule comparison result inversion / no inversion
 * Input:
 *      index   - ACL rule index (0-95) of 95 ACL rules
 *      pNot    - 1: inverse, 0: don't inverse
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK               - Success
 *      RT_ERR_SMI              - SMI access error
 *      RT_ERR_OUT_OF_RANGE     - Invalid ACL rule index (0-95)
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicAclNot(uint32_t index, uint32_t *pNot)
{
    if (index > RTL8367C_ACLRULEMAX)
        return RT_ERR_OUT_OF_RANGE;

    if (index < 64)
        return rtl8367c_getAsicRegBit(RTL8367C_ACL_ACTION_CTRL_REG(index), RTL8367C_ACL_OP_NOT_OFFSET(index), pNot);
    else
        return rtl8367c_getAsicRegBit(RTL8367C_ACL_ACTION_CTRL2_REG(index), RTL8367C_ACL_OP_NOT_OFFSET(index), pNot);
}

void rtl8367::_rtl8367c_aclActStSmi2User(rtl8367c_acl_act_t *pAclUser, uint16_t *pAclSmi)
{
    pAclUser->cact = (pAclSmi[0] & 0x00C0) >> 6;
    pAclUser->cvidx_cact = (pAclSmi[0] & 0x003F) | (((pAclSmi[3] & 0x0008) >> 3) << 6);

    pAclUser->sact = (pAclSmi[0] & 0xC000) >> 14;
    pAclUser->svidx_sact = ((pAclSmi[0] & 0x3F00) >> 8) | (((pAclSmi[3] & 0x0010) >> 4) << 6);

    pAclUser->aclmeteridx = (pAclSmi[1] & 0x003F) | (((pAclSmi[3] & 0x0020) >> 5) << 6);

    pAclUser->fwdact = (pAclSmi[1] & 0xC000) >> 14;
    pAclUser->fwdpmask = ((pAclSmi[1] & 0x3FC0) >> 6) | (((pAclSmi[3] & 0x01C0) >> 6) << 8);

    pAclUser->priact = (pAclSmi[2] & 0x00C0) >> 6;
    pAclUser->pridx = (pAclSmi[2] & 0x003F) | (((pAclSmi[3] & 0x0200) >> 9) << 6);

    pAclUser->aclint = (pAclSmi[2] & 0x2000) >> 13;
    pAclUser->gpio_en = (pAclSmi[2] & 0x1000) >> 12;
    pAclUser->gpio_pin = (pAclSmi[2] & 0x0F00) >> 8;

    pAclUser->cact_ext = (pAclSmi[2] & 0xC000) >> 14;
    pAclUser->tag_fmt = (pAclSmi[3] & 0x0003);
    pAclUser->fwdact_ext = (pAclSmi[3] & 0x0004) >> 2;
}
/* Function Name:
 *      rtl8367c_getAsicAclAct
 * Description:
 *      Get ACL rule matched Action
 * Input:
 *      index   - ACL rule index (0-95) of 96 ACL rules
 *      pAclAct     - ACL action stucture for setting
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK               - Success
 *      RT_ERR_SMI              - SMI access error
 *      RT_ERR_OUT_OF_RANGE     - Invalid ACL rule index (0-95)
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicAclAct(uint32_t index, rtl8367c_acl_act_t *pAclAct)
{
    uint16_t aclActSmi[RTL8367C_ACL_ACT_TABLE_LEN];
    int32_t retVal;
    uint32_t regAddr, regData;
    uint16_t *tableAddr;
    uint32_t i;

    if (index > RTL8367C_ACLRULEMAX)
        return RT_ERR_OUT_OF_RANGE;

    memset(aclActSmi, 0x00, sizeof(uint16_t) * RTL8367C_ACL_ACT_TABLE_LEN);

    /* Write ACS_ADR register for data bits */
    regAddr = RTL8367C_TABLE_ACCESS_ADDR_REG;
    regData = index;
    retVal = rtl8367c_setAsicReg(regAddr, regData);
    if (retVal != RT_ERR_OK)
        return retVal;

    /* Write ACS_CMD register */
    regAddr = RTL8367C_TABLE_ACCESS_CTRL_REG;
    regData = RTL8367C_TABLE_ACCESS_REG_DATA(TB_OP_READ, TB_TARGET_ACLACT);
    retVal = rtl8367c_setAsicRegBits(regAddr, RTL8367C_TABLE_TYPE_MASK | RTL8367C_COMMAND_TYPE_MASK, regData);
    if (retVal != RT_ERR_OK)
        return retVal;

    /* Read Data Bits */
    regAddr = RTL8367C_TABLE_ACCESS_RDDATA_BASE;
    tableAddr = aclActSmi;
    for (i = 0; i < RTL8367C_ACLACTTBLEN; i++)
    {
        retVal = rtl8367c_getAsicReg(regAddr, &regData);
        if (retVal != RT_ERR_OK)
            return retVal;

        *tableAddr = regData;

        regAddr++;
        tableAddr++;
    }

    _rtl8367c_aclActStSmi2User(pAclAct, aclActSmi);

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_getAsicAclActCtrl
 * Description:
 *      Get ACL rule matched Action Control Bits
 * Input:
 *      index       - ACL rule index (0-95) of 96 ACL rules
 *      pAclActCtrl     - 6 ACL Control Bits
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK               - Success
 *      RT_ERR_SMI              - SMI access error
 *      RT_ERR_OUT_OF_RANGE     - Invalid ACL rule index (0-95)
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicAclActCtrl(uint32_t index, uint32_t *pAclActCtrl)
{
    int32_t retVal;
    uint32_t regData;

    if (index > RTL8367C_ACLRULEMAX)
        return RT_ERR_OUT_OF_RANGE;

    if (index >= 64)
        retVal = rtl8367c_getAsicRegBits(RTL8367C_ACL_ACTION_CTRL2_REG(index), RTL8367C_ACL_OP_ACTION_MASK(index), &regData);
    else
        retVal = rtl8367c_getAsicRegBits(RTL8367C_ACL_ACTION_CTRL_REG(index), RTL8367C_ACL_OP_ACTION_MASK(index), &regData);

    if (retVal != RT_ERR_OK)
        return retVal;

    *pAclActCtrl = regData;

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_filter_igrAcl_cfg_get(rtk_filter_id_t filter_id, rtk_filter_cfg_raw_t *pFilter_cfg, rtk_filter_action_t *pAction)
{
    int32_t retVal;
    uint32_t i, tmp;
    rtl8367c_aclrule aclRule;
    rtl8367c_acl_act_t aclAct;
    uint32_t cpuPort;
    rtl8367c_acltemplate_t type;
    rtl8367c_svlan_memconf_t svlan_cfg;
    rtl8367c_vlanconfiguser vlanMC;
    uint32_t phyPmask;

    if (NULL == pFilter_cfg || NULL == pAction)
        return RT_ERR_NULL_POINTER;

    if (filter_id > RTL8367C_ACLRULEMAX)
        return RT_ERR_ENTRY_INDEX;

    if ((retVal = rtl8367c_getAsicAclRule(filter_id, &aclRule)) != RT_ERR_OK)
        return retVal;

    /* Check valid */
    if (aclRule.valid == 0)
    {
        pFilter_cfg->valid = DISABLED_RTK;
        return RT_ERR_OK;
    }

    phyPmask = aclRule.data_bits.active_portmsk;
    if (rtk_switch_portmask_P2L_get(phyPmask, &(pFilter_cfg->activeport.value)) != RT_ERR_OK)
        return RT_ERR_FAILED;

    phyPmask = aclRule.care_bits.active_portmsk;
    if (rtk_switch_portmask_P2L_get(phyPmask, &(pFilter_cfg->activeport.mask)) != RT_ERR_OK)
        return RT_ERR_FAILED;

    for (i = 0; i <= CARE_TAG_IPV6; i++)
    {
        if (aclRule.data_bits.tag_exist & (1 << i))
            pFilter_cfg->careTag.tagType[i].value = 1;
        else
            pFilter_cfg->careTag.tagType[i].value = 0;

        if (aclRule.care_bits.tag_exist & (1 << i))
            pFilter_cfg->careTag.tagType[i].mask = 1;
        else
            pFilter_cfg->careTag.tagType[i].mask = 0;
    }

    if (filter_advanceCaretagField[aclRule.data_bits.type][0] == 1)
    {
        /* Advanced Care tag setting */
        for (i = CARE_TAG_TCP; i < CARE_TAG_END; i++)
        {
            if (aclRule.data_bits.field[filter_advanceCaretagField[aclRule.data_bits.type][1]] & (0x0001 << (i - CARE_TAG_TCP)))
                pFilter_cfg->careTag.tagType[i].value = 1;
            else
                pFilter_cfg->careTag.tagType[i].value = 0;

            if (aclRule.care_bits.field[filter_advanceCaretagField[aclRule.care_bits.type][1]] & (0x0001 << (i - CARE_TAG_TCP)))
                pFilter_cfg->careTag.tagType[i].mask = 1;
            else
                pFilter_cfg->careTag.tagType[i].mask = 0;
        }
    }

    for (i = 0; i < RTL8367C_ACLRULEFIELDNO; i++)
    {
        pFilter_cfg->careFieldRaw[i] = aclRule.care_bits.field[i];
        pFilter_cfg->dataFieldRaw[i] = aclRule.data_bits.field[i];
    }

    if ((retVal = rtl8367c_getAsicAclNot(filter_id, &tmp)) != RT_ERR_OK)
        return retVal;

    pFilter_cfg->invert = (rtk_filter_invert_t)tmp;

    pFilter_cfg->valid = (rtk_enable_t)aclRule.valid;

    memset(pAction, 0, sizeof(rtk_filter_action_t));

    if ((retVal = rtl8367c_getAsicAclActCtrl(filter_id, &tmp)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_getAsicAclAct(filter_id, &aclAct)) != RT_ERR_OK)
        return retVal;

    if (tmp & FILTER_ENACT_FWD_MASK)
    {
        if (1 == aclAct.fwdact_ext)
        {
            pAction->actEnable[FILTER_ENACT_ISOLATION] = (rtk_filter_act_enable_t)1;

            phyPmask = aclAct.fwdpmask;
            if (rtk_switch_portmask_P2L_get(phyPmask, &(pAction->filterPortmask)) != RT_ERR_OK)
                return RT_ERR_FAILED;
        }
        else if (aclAct.fwdact == RTL8367C_ACL_FWD_TRAP)
        {
            pAction->actEnable[FILTER_ENACT_TRAP_CPU] = (rtk_filter_act_enable_t)1;
        }
        else if (aclAct.fwdact == RTL8367C_ACL_FWD_MIRRORFUNTION)
        {
            pAction->actEnable[FILTER_ENACT_MIRROR] = (rtk_filter_act_enable_t)1;

            phyPmask = aclAct.fwdpmask;
            if (rtk_switch_portmask_P2L_get(phyPmask, &(pAction->filterPortmask)) != RT_ERR_OK)
                return RT_ERR_FAILED;
        }
        else if (aclAct.fwdact == RTL8367C_ACL_FWD_REDIRECT)
        {
            if (aclAct.fwdpmask == 0)
                pAction->actEnable[FILTER_ENACT_DROP] = (rtk_filter_act_enable_t)1;
            else
            {
                pAction->actEnable[FILTER_ENACT_REDIRECT] = (rtk_filter_act_enable_t)1;

                phyPmask = aclAct.fwdpmask;
                if (rtk_switch_portmask_P2L_get(phyPmask, &(pAction->filterPortmask)) != RT_ERR_OK)
                    return RT_ERR_FAILED;
            }
        }
        else if (aclAct.fwdact == RTL8367C_ACL_FWD_MIRROR)
        {
            if ((retVal = rtl8367c_getAsicCputagTrapPort(&cpuPort)) != RT_ERR_OK)
                return retVal;
            if (aclAct.fwdpmask == (1 << cpuPort))
            {
                pAction->actEnable[FILTER_ENACT_COPY_CPU] = (rtk_filter_act_enable_t)1;
            }
            else
            {
                pAction->actEnable[FILTER_ENACT_ADD_DSTPORT] = (rtk_filter_act_enable_t)1;

                phyPmask = aclAct.fwdpmask;
                if (rtk_switch_portmask_P2L_get(phyPmask, &(pAction->filterPortmask)) != RT_ERR_OK)
                    return RT_ERR_FAILED;
            }
        }
        else
        {
            return RT_ERR_FAILED;
        }
    }

    if (tmp & FILTER_ENACT_POLICING_MASK)
    {
        pAction->actEnable[FILTER_ENACT_POLICING_0] = (rtk_filter_act_enable_t)1;
        pAction->filterPolicingIdx[0] = aclAct.aclmeteridx;
    }

    if (tmp & FILTER_ENACT_PRIORITY_MASK)
    {
        if (aclAct.priact == FILTER_ENACT_PRI_TYPE(FILTER_ENACT_PRIORITY))
        {
            pAction->actEnable[FILTER_ENACT_PRIORITY] = (rtk_filter_act_enable_t)1;
            pAction->filterPriority = aclAct.pridx;
        }
        else if (aclAct.priact == FILTER_ENACT_PRI_TYPE(FILTER_ENACT_1P_REMARK))
        {
            pAction->actEnable[FILTER_ENACT_1P_REMARK] = (rtk_filter_act_enable_t)1;
            pAction->filterPriority = aclAct.pridx;
        }
        else if (aclAct.priact == FILTER_ENACT_PRI_TYPE(FILTER_ENACT_DSCP_REMARK))
        {
            pAction->actEnable[FILTER_ENACT_DSCP_REMARK] = (rtk_filter_act_enable_t)1;
            pAction->filterPriority = aclAct.pridx;
        }
        else if (aclAct.priact == FILTER_ENACT_PRI_TYPE(FILTER_ENACT_POLICING_3))
        {
            pAction->actEnable[FILTER_ENACT_POLICING_3] = (rtk_filter_act_enable_t)1;
            pAction->filterPolicingIdx[3] = aclAct.pridx;
        }
    }

    if (tmp & FILTER_ENACT_SVLAN_MASK)
    {
        if (aclAct.sact == FILTER_ENACT_SVLAN_TYPE(FILTER_ENACT_SVLAN_INGRESS))
        {
            if ((retVal = rtl8367c_getAsicSvlanMemberConfiguration(aclAct.svidx_sact, &svlan_cfg)) != RT_ERR_OK)
                return retVal;

            pAction->actEnable[FILTER_ENACT_SVLAN_INGRESS] = (rtk_filter_act_enable_t)1;
            pAction->filterSvlanIdx = aclAct.svidx_sact;
            pAction->filterSvlanVid = svlan_cfg.vs_svid;
        }
        else if (aclAct.sact == FILTER_ENACT_SVLAN_TYPE(FILTER_ENACT_SVLAN_EGRESS))
        {
            if ((retVal = rtl8367c_getAsicSvlanMemberConfiguration(aclAct.svidx_sact, &svlan_cfg)) != RT_ERR_OK)
                return retVal;

            pAction->actEnable[FILTER_ENACT_SVLAN_EGRESS] = (rtk_filter_act_enable_t)1;
            pAction->filterSvlanIdx = aclAct.svidx_sact;
            pAction->filterSvlanVid = svlan_cfg.vs_svid;
        }
        else if (aclAct.sact == FILTER_ENACT_SVLAN_TYPE(FILTER_ENACT_SVLAN_CVID))
            pAction->actEnable[FILTER_ENACT_SVLAN_CVID] = (rtk_filter_act_enable_t)1;
        else if (aclAct.sact == FILTER_ENACT_SVLAN_TYPE(FILTER_ENACT_POLICING_2))
        {
            pAction->actEnable[FILTER_ENACT_POLICING_2] = (rtk_filter_act_enable_t)1;
            pAction->filterPolicingIdx[2] = aclAct.svidx_sact;
        }
    }

    if (tmp & FILTER_ENACT_CVLAN_MASK)
    {
        if (FILTER_ENACT_CACTEXT_TAGONLY == aclAct.cact_ext ||
            FILTER_ENACT_CACTEXT_BOTHVLANTAG == aclAct.cact_ext)
        {
            if (FILTER_CTAGFMT_UNTAG == aclAct.tag_fmt)
            {
                pAction->actEnable[FILTER_ENACT_EGRESSCTAG_UNTAG] = (rtk_filter_act_enable_t)1;
            }
            else if (FILTER_CTAGFMT_TAG == aclAct.tag_fmt)
            {
                pAction->actEnable[FILTER_ENACT_EGRESSCTAG_TAG] = (rtk_filter_act_enable_t)1;
            }
            else if (FILTER_CTAGFMT_KEEP == aclAct.tag_fmt)
            {
                pAction->actEnable[FILTER_ENACT_EGRESSCTAG_KEEP] = (rtk_filter_act_enable_t)1;
            }
            else if (FILTER_CTAGFMT_KEEP1PRMK == aclAct.tag_fmt)
            {
                pAction->actEnable[FILTER_ENACT_EGRESSCTAG_KEEPAND1PRMK] = (rtk_filter_act_enable_t)1;
            }
        }

        if (FILTER_ENACT_CACTEXT_VLANONLY == aclAct.cact_ext ||
            FILTER_ENACT_CACTEXT_BOTHVLANTAG == aclAct.cact_ext)
        {
            if (aclAct.cact == FILTER_ENACT_CVLAN_TYPE(FILTER_ENACT_CVLAN_INGRESS))
            {
                if ((retVal = rtl8367c_getAsicVlanMemberConfig(aclAct.cvidx_cact, &vlanMC)) != RT_ERR_OK)
                    return retVal;

                pAction->actEnable[FILTER_ENACT_CVLAN_INGRESS] = (rtk_filter_act_enable_t)1;
                pAction->filterCvlanIdx = aclAct.cvidx_cact;
                pAction->filterCvlanVid = vlanMC.evid;
            }
            else if (aclAct.cact == FILTER_ENACT_CVLAN_TYPE(FILTER_ENACT_CVLAN_EGRESS))
            {
                if ((retVal = rtl8367c_getAsicVlanMemberConfig(aclAct.cvidx_cact, &vlanMC)) != RT_ERR_OK)
                    return retVal;

                pAction->actEnable[FILTER_ENACT_CVLAN_EGRESS] = (rtk_filter_act_enable_t)1;
                pAction->filterCvlanIdx = aclAct.cvidx_cact;
                pAction->filterCvlanVid = vlanMC.evid;
            }
            else if (aclAct.cact == FILTER_ENACT_CVLAN_TYPE(FILTER_ENACT_CVLAN_SVID))
            {
                pAction->actEnable[FILTER_ENACT_CVLAN_SVID] = (rtk_filter_act_enable_t)1;
            }
            else if (aclAct.cact == FILTER_ENACT_CVLAN_TYPE(FILTER_ENACT_POLICING_1))
            {
                pAction->actEnable[FILTER_ENACT_POLICING_1] = (rtk_filter_act_enable_t)1;
                pAction->filterPolicingIdx[1] = aclAct.cvidx_cact;
            }
        }
    }

    if (tmp & FILTER_ENACT_INTGPIO_MASK)
    {
        if (1 == aclAct.aclint)
        {
            pAction->actEnable[FILTER_ENACT_INTERRUPT] = (rtk_filter_act_enable_t)1;
        }

        if (1 == aclAct.gpio_en)
        {
            pAction->actEnable[FILTER_ENACT_GPO] = (rtk_filter_act_enable_t)1;
            pAction->filterPin = aclAct.gpio_pin;
        }
    }

    /* Get field type of RAW data */
    if ((retVal = rtl8367c_getAsicAclTemplate(aclRule.data_bits.type, &type)) != RT_ERR_OK)
        return retVal;

    for (i = 0; i < RTL8367C_ACLRULEFIELDNO; i++)
    {
        pFilter_cfg->fieldRawType[i] = (rtk_filter_field_type_raw_t)type.field[i];
    } /* end of for(i...) */

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_filter_igrAcl_state_set(rtk_port_t port, rtk_filter_state_t state)
{
    int32_t ret;

    /* Check port valid */
    RTK_CHK_PORT_VALID(port);

    if (state >= RTK_ENABLE_END)
        return RT_ERR_INPUT;

    if ((ret = rtl8367c_setAsicAcl(rtk_switch_port_L2P_get(port), state)) != RT_ERR_OK)
        return ret;

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_filter_igrAcl_field_sel_set(uint32_t index, rtk_field_sel_t format, uint32_t offset)
{
    int32_t ret;

    if (index >= RTL8367C_FIELDSEL_FORMAT_NUMBER)
        return RT_ERR_OUT_OF_RANGE;

    if (format >= FORMAT_END)
        return RT_ERR_OUT_OF_RANGE;

    if (offset > RTL8367C_FIELDSEL_MAX_OFFSET)
        return RT_ERR_OUT_OF_RANGE;

    if ((ret = rtl8367c_setAsicFieldSelector(index, (uint32_t)format, offset)) != RT_ERR_OK)
        return ret;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsicAclIpRange
 * Description:
 *      Set ACL IP range check
 * Input:
 *      index       - ACL IP range check index(0~15)
 *      type        - Range check type
 *      upperIp     - IP range upper bound
 *      lowerIp     - IP range lower bound
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK               - Success
 *      RT_ERR_SMI              - SMI access error
 *      RT_ERR_OUT_OF_RANGE     - Invalid ACL IP range check index(0~15)
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicAclIpRange(uint32_t index, uint32_t type, uint32_t upperIp, uint32_t lowerIp)
{
    int32_t retVal;
    uint32_t regData;
    uint32_t ipData;

    if (index > RTL8367C_ACLRANGEMAX)
        return RT_ERR_OUT_OF_RANGE;

    retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_ACL_IP_RANGE_ENTRY0_CTRL4 + index * 5, RTL8367C_ACL_IP_RANGE_ENTRY0_CTRL4_MASK, type);
    if (retVal != RT_ERR_OK)
        return retVal;

    ipData = upperIp;

    regData = ipData & 0xFFFF;
    retVal = rtl8367c_setAsicReg(RTL8367C_REG_ACL_IP_RANGE_ENTRY0_CTRL2 + index * 5, regData);
    if (retVal != RT_ERR_OK)
        return retVal;

    regData = (ipData >> 16) & 0xFFFF;
    retVal = rtl8367c_setAsicReg(RTL8367C_REG_ACL_IP_RANGE_ENTRY0_CTRL3 + index * 5, regData);
    if (retVal != RT_ERR_OK)
        return retVal;

    ipData = lowerIp;

    regData = ipData & 0xFFFF;
    retVal = rtl8367c_setAsicReg(RTL8367C_REG_ACL_IP_RANGE_ENTRY0_CTRL0 + index * 5, regData);
    if (retVal != RT_ERR_OK)
        return retVal;

    regData = (ipData >> 16) & 0xFFFF;
    retVal = rtl8367c_setAsicReg(RTL8367C_REG_ACL_IP_RANGE_ENTRY0_CTRL1 + index * 5, regData);
    if (retVal != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}
int32_t rtl8367::rtk_filter_iprange_set(uint32_t index, rtk_filter_iprange_t type, uint32_t upperIp, uint32_t lowerIp)
{
    int32_t ret;

    if (index > RTL8367C_ACLRANGEMAX)
        return RT_ERR_OUT_OF_RANGE;

    if (type >= IPRANGE_END)
        return RT_ERR_OUT_OF_RANGE;

    if (lowerIp > upperIp)
        return RT_ERR_INPUT;

    if ((ret = rtl8367c_setAsicAclIpRange(index, type, upperIp, lowerIp)) != RT_ERR_OK)
        return ret;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsicAclVidRange
 * Description:
 *      Set ACL VID range check
 * Input:
 *      index       - ACL VID range check index(0~15)
 *      type        - Range check type
 *      upperVid    - VID range upper bound
 *      lowerVid    - VID range lower bound
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK               - Success
 *      RT_ERR_SMI              - SMI access error
 *      RT_ERR_OUT_OF_RANGE     - Invalid ACL  VID range check index(0~15)
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicAclVidRange(uint32_t index, uint32_t type, uint32_t upperVid, uint32_t lowerVid)
{
    int32_t retVal;
    uint32_t regData;

    if (index > RTL8367C_ACLRANGEMAX)
        return RT_ERR_OUT_OF_RANGE;

    regData = ((type << RTL8367C_ACL_VID_RANGE_ENTRY0_CTRL1_CHECK0_TYPE_OFFSET) & RTL8367C_ACL_VID_RANGE_ENTRY0_CTRL1_CHECK0_TYPE_MASK) |
              (upperVid & RTL8367C_ACL_VID_RANGE_ENTRY0_CTRL1_CHECK0_HIGH_MASK);

    retVal = rtl8367c_setAsicReg(RTL8367C_REG_ACL_VID_RANGE_ENTRY0_CTRL1 + index * 2, regData);
    if (retVal != RT_ERR_OK)
        return retVal;

    retVal = rtl8367c_setAsicReg(RTL8367C_REG_ACL_VID_RANGE_ENTRY0_CTRL0 + index * 2, lowerVid);
    if (retVal != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_filter_vidrange_set(uint32_t index, rtk_filter_vidrange_t type, uint32_t upperVid, uint32_t lowerVid)
{
    int32_t ret;

    if (index > RTL8367C_ACLRANGEMAX)
        return RT_ERR_OUT_OF_RANGE;

    if (type >= VIDRANGE_END)
        return RT_ERR_OUT_OF_RANGE;

    if (lowerVid > upperVid)
        return RT_ERR_INPUT;

    if ((upperVid > RTL8367C_VIDMAX) || (lowerVid > RTL8367C_VIDMAX))
        return RT_ERR_OUT_OF_RANGE;

    if ((ret = rtl8367c_setAsicAclVidRange(index, type, upperVid, lowerVid)) != RT_ERR_OK)
        return ret;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsicAclPortRange
 * Description:
 *      Set ACL TCP/UDP range check
 * Input:
 *      index       - TCP/UDP port range check table index
 *      type        - Range check type
 *      upperPort   - TCP/UDP port range upper bound
 *      lowerPort   - TCP/UDP port range lower bound
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK               - Success
 *      RT_ERR_SMI              - SMI access error
 *      RT_ERR_OUT_OF_RANGE     - Invalid TCP/UDP port range check table index
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicAclPortRange(uint32_t index, uint32_t type, uint32_t upperPort, uint32_t lowerPort)
{
    int32_t retVal;

    if (index > RTL8367C_ACLRANGEMAX)
        return RT_ERR_OUT_OF_RANGE;

    retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_ACL_SDPORT_RANGE_ENTRY0_CTRL2 + index * 3, RTL8367C_ACL_SDPORT_RANGE_ENTRY0_CTRL2_MASK, type);
    if (retVal != RT_ERR_OK)
        return retVal;

    retVal = rtl8367c_setAsicReg(RTL8367C_REG_ACL_SDPORT_RANGE_ENTRY0_CTRL1 + index * 3, upperPort);
    if (retVal != RT_ERR_OK)
        return retVal;

    retVal = rtl8367c_setAsicReg(RTL8367C_REG_ACL_SDPORT_RANGE_ENTRY0_CTRL0 + index * 3, lowerPort);
    if (retVal != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_filter_portrange_set(uint32_t index, rtk_filter_portrange_t type, uint32_t upperPort, uint32_t lowerPort)
{
    int32_t ret;

    if (index > RTL8367C_ACLRANGEMAX)
        return RT_ERR_OUT_OF_RANGE;

    if (type >= PORTRANGE_END)
        return RT_ERR_OUT_OF_RANGE;

    if (lowerPort > upperPort)
        return RT_ERR_INPUT;

    if (upperPort > RTL8367C_ACL_PORTRANGEMAX)
        return RT_ERR_INPUT;

    if (lowerPort > RTL8367C_ACL_PORTRANGEMAX)
        return RT_ERR_INPUT;

    if ((ret = rtl8367c_setAsicAclPortRange(index, type, upperPort, lowerPort)) != RT_ERR_OK)
        return ret;

    return RT_ERR_OK;
}

// ------------------------- EEE ---------------------------------

int32_t rtl8367::rtk_eee_init()
{
    int32_t retVal;

    if ((retVal = rtl8367c_setAsicRegBit(0x0018, 10, 1)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicRegBit(0x0018, 11, 1)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

/*
@func int32_t | rtl8367c_setAsicEee100M | Set eee force mode function enable/disable.
@parm uint32_t | port | The port number.
@parm uint32_t | enabled | 1: enabled, 0: disabled.
@rvalue RT_ERR_OK | Success.
@rvalue RT_ERR_SMI | SMI access error.
@rvalue RT_ERR_INPUT | Invalid input parameter.
@comm
    This API set the 100M EEE enable function.

*/
int32_t rtl8367::rtl8367c_setAsicEee100M(uint32_t port, uint32_t enable)
{
    int32_t retVal;
    uint32_t regData;

    if (port >= RTL8367C_PORTNO)
        return RT_ERR_PORT_ID;

    if (enable > 1)
        return RT_ERR_INPUT;

    if ((retVal = rtl8367c_getAsicPHYOCPReg(port, EEE_OCP_PHY_ADDR, &regData)) != RT_ERR_OK)
        return retVal;

    if (enable)
        regData |= (0x0001 << 1);
    else
        regData &= ~(0x0001 << 1);

    if ((retVal = rtl8367c_setAsicPHYOCPReg(port, EEE_OCP_PHY_ADDR, regData)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_getAsicReg(RTL8367C_PORT_EEE_CFG_REG(port), &regData)) != RT_ERR_OK)
        return retVal;

    if (enable)
        regData |= (0x0001 << 11);
    else
        regData &= ~(0x0001 << 11);

    if ((retVal = rtl8367c_setAsicReg(RTL8367C_PORT_EEE_CFG_REG(port), regData)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

/*
@func int32_t | rtl8367c_setAsicEeeGiga | Set eee force mode function enable/disable.
@parm uint32_t | port | The port number.
@parm uint32_t | enabled | 1: enabled, 0: disabled.
@rvalue RT_ERR_OK | Success.
@rvalue RT_ERR_SMI | SMI access error.
@rvalue RT_ERR_INPUT | Invalid input parameter.
@comm
    This API set the 100M EEE enable function.

*/
int32_t rtl8367::rtl8367c_setAsicEeeGiga(uint32_t port, uint32_t enable)
{
    int32_t retVal;
    uint32_t regData;

    if (port >= RTL8367C_PORTNO)
        return RT_ERR_PORT_ID;

    if (enable > 1)
        return RT_ERR_INPUT;

    if ((retVal = rtl8367c_getAsicPHYOCPReg(port, EEE_OCP_PHY_ADDR, &regData)) != RT_ERR_OK)
        return retVal;

    if (enable)
        regData |= (0x0001 << 2);
    else
        regData &= ~(0x0001 << 2);

    if ((retVal = rtl8367c_setAsicPHYOCPReg(port, EEE_OCP_PHY_ADDR, regData)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_getAsicReg(RTL8367C_PORT_EEE_CFG_REG(port), &regData)) != RT_ERR_OK)
        return retVal;

    if (enable)
        regData |= (0x0001 << 10);
    else
        regData &= ~(0x0001 << 10);

    if ((retVal = rtl8367c_setAsicReg(RTL8367C_PORT_EEE_CFG_REG(port), regData)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_eee_portEnable_set(rtk_port_t port, rtk_enable_t enable)
{
    int32_t retVal;
    uint32_t regData;
    uint32_t phy_port;

    /* Check port is UTP port */
    RTK_CHK_PORT_IS_UTP(port);

    if (enable >= RTK_ENABLE_END)
        return RT_ERR_INPUT;

    phy_port = rtk_switch_port_L2P_get(port);

    if ((retVal = rtl8367c_setAsicEee100M(phy_port, enable)) != RT_ERR_OK)
        return retVal;
    if ((retVal = rtl8367c_setAsicEeeGiga(phy_port, enable)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicPHYReg(phy_port, RTL8367C_PHY_PAGE_ADDRESS, 0)) != RT_ERR_OK)
        return retVal;
    if ((retVal = rtl8367c_getAsicPHYReg(phy_port, 0, &regData)) != RT_ERR_OK)
        return retVal;
    regData |= 0x0200;
    if ((retVal = rtl8367c_setAsicPHYReg(phy_port, 0, regData)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

// ------------------------ 802.1X --------------------------------

int32_t rtl8367::rtk_dot1x_eapolFrame2CpuEnable_set(rtk_enable_t enable)
{
    int32_t retVal;
    rtl8367c_rma_t rmacfg;

    if (enable >= RTK_ENABLE_END)
        return RT_ERR_ENABLE;

    if ((retVal = rtl8367c_getAsicRma(3, &rmacfg)) != RT_ERR_OK)
        return retVal;

    if (ENABLED == enable)
        rmacfg.operation = RMAOP_TRAP_TO_CPU;
    else if (DISABLED == enable)
        rmacfg.operation = RMAOP_FORWARD;

    if ((retVal = rtl8367c_setAsicRma(3, &rmacfg)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsic1xPBEnConfig
 * Description:
 *      Set 802.1x port-based port enable configuration
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
int32_t rtl8367::rtl8367c_setAsic1xPBEnConfig(uint32_t port, uint32_t enabled)
{
    if (port >= RTL8367C_PORTNO)
        return RT_ERR_PORT_ID;

    return rtl8367c_setAsicRegBit(RTL8367C_DOT1X_PORT_ENABLE_REG, port, enabled);
}
int32_t rtl8367::rtk_dot1x_portBasedEnable_set(rtk_port_t port, rtk_enable_t enable)
{
    int32_t retVal;

    /* Check port Valid */
    RTK_CHK_PORT_VALID(port);

    if (enable >= RTK_ENABLE_END)
        return RT_ERR_ENABLE;

    if ((retVal = rtl8367c_setAsic1xPBEnConfig(rtk_switch_port_L2P_get(port), enable)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsic1xPBAuthConfig
 * Description:
 *      Set 802.1x port-based authorised port configuration
 * Input:
 *      port    - Physical port number (0~7)
 *      auth    - 1: authorised, 0: non-authorised
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_SMI      - SMI access error
 *      RT_ERR_PORT_ID  - Invalid port number
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsic1xPBAuthConfig(uint32_t port, uint32_t auth)
{
    if (port >= RTL8367C_PORTNO)
        return RT_ERR_PORT_ID;

    return rtl8367c_setAsicRegBit(RTL8367C_DOT1X_PORT_AUTH_REG, port, auth);
}
int32_t rtl8367::rtk_dot1x_portBasedAuthStatus_set(rtk_port_t port, rtk_dot1x_auth_status_t port_auth)
{
    int32_t retVal;

    /* Check port Valid */
    RTK_CHK_PORT_VALID(port);

    if (port_auth >= AUTH_STATUS_END)
        return RT_ERR_DOT1X_PORTBASEDAUTH;

    if ((retVal = rtl8367c_setAsic1xPBAuthConfig(rtk_switch_port_L2P_get(port), port_auth)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsic1xPBOpdirConfig
 * Description:
 *      Set 802.1x port-based operational direction
 * Input:
 *      port    - Physical port number (0~7)
 *      opdir   - Operation direction 1: IN, 0:BOTH
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_SMI      - SMI access error
 *      RT_ERR_PORT_ID  - Invalid port number
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsic1xPBOpdirConfig(uint32_t port, uint32_t opdir)
{
    if (port >= RTL8367C_PORTNO)
        return RT_ERR_PORT_ID;

    return rtl8367c_setAsicRegBit(RTL8367C_DOT1X_PORT_OPDIR_REG, port, opdir);
}
int32_t rtl8367::rtk_dot1x_portBasedDirection_set(rtk_port_t port, rtk_dot1x_direction_t port_direction)
{
    int32_t retVal;

    /* Check port Valid */
    RTK_CHK_PORT_VALID(port);

    if (port_direction >= DIRECTION_END)
        return RT_ERR_DOT1X_PORTBASEDOPDIR;

    if ((retVal = rtl8367c_setAsic1xPBOpdirConfig(rtk_switch_port_L2P_get(port), port_direction)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsic1xProcConfig
 * Description:
 *      Set 802.1x unauth. behavior configuration
 * Input:
 *      port    - Physical port number (0~7)
 *      proc    - 802.1x unauth. behavior configuration 0:drop 1:trap to CPU 2:Guest VLAN
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK           - Success
 *      RT_ERR_SMI          - SMI access error
 *      RT_ERR_PORT_ID      - Invalid port number
 *      RT_ERR_DOT1X_PROC   - Unauthorized behavior error
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsic1xProcConfig(uint32_t port, uint32_t proc)
{
    if (port >= RTL8367C_PORTNO)
        return RT_ERR_PORT_ID;

    if (proc >= DOT1X_UNAUTH_END)
        return RT_ERR_DOT1X_PROC;

    if (port < 8)
    {
        return rtl8367c_setAsicRegBits(RTL8367C_DOT1X_UNAUTH_ACT_BASE, RTL8367C_DOT1X_UNAUTH_ACT_MASK(port), proc);
    }
    else
    {
        return rtl8367c_setAsicRegBits(RTL8367C_REG_DOT1X_UNAUTH_ACT_W1, RTL8367C_DOT1X_UNAUTH_ACT_MASK(port), proc);
    }
}
int32_t rtl8367::rtk_dot1x_unauthPacketOper_set(rtk_port_t port, rtk_dot1x_unauth_action_t unauth_action)
{
    int32_t retVal;

    /* Check port Valid */
    RTK_CHK_PORT_VALID(port);

    if (unauth_action >= DOT1X_ACTION_END)
        return RT_ERR_DOT1X_PROC;

    if ((retVal = rtl8367c_setAsic1xProcConfig(rtk_switch_port_L2P_get(port), unauth_action)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

// ------------------------- SHARE METER ---------------------------------

/* Function Name:
 *      rtl8367c_setAsicShareMeter
 * Description:
 *      Set meter configuration
 * Input:
 *      index   - hared meter index (0-31)
 *      rate    - 17-bits rate of share meter, unit is 8Kpbs
 *      ifg     - Including IFG in rate calculation, 1:include 0:exclude
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK               - Success
 *      RT_ERR_SMI              - SMI access error
 *      RT_ERR_FILTER_METER_ID  - Invalid meter
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicShareMeter(uint32_t index, uint32_t rate, uint32_t ifg)
{
    int32_t retVal;

    if (index > RTL8367C_METERMAX)
        return RT_ERR_FILTER_METER_ID;

    if (index < 32)
    {
        /*19-bits Rate*/
        retVal = rtl8367c_setAsicReg(RTL8367C_METER_RATE_REG(index), rate & 0xFFFF);
        if (retVal != RT_ERR_OK)
            return retVal;

        retVal = rtl8367c_setAsicReg(RTL8367C_METER_RATE_REG(index) + 1, (rate & 0x70000) >> 16);
        if (retVal != RT_ERR_OK)
            return retVal;

        retVal = rtl8367c_setAsicRegBit(RTL8367C_METER_IFG_CTRL_REG(index), RTL8367C_METER_IFG_OFFSET(index), ifg);
        if (retVal != RT_ERR_OK)
            return retVal;
    }
    else
    {
        /*19-bits Rate*/
        retVal = rtl8367c_setAsicReg(RTL8367C_REG_METER32_RATE_CTRL0 + ((index - 32) << 1), rate & 0xFFFF);
        if (retVal != RT_ERR_OK)
            return retVal;

        retVal = rtl8367c_setAsicReg(RTL8367C_REG_METER32_RATE_CTRL0 + ((index - 32) << 1) + 1, (rate & 0x70000) >> 16);
        if (retVal != RT_ERR_OK)
            return retVal;

        retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_METER_IFG_CTRL2 + ((index - 32) >> 4), RTL8367C_METER_IFG_OFFSET(index), ifg);
        if (retVal != RT_ERR_OK)
            return retVal;
    }

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsicShareMeterType
 * Description:
 *      Set meter Type
 * Input:
 *      index       - shared meter index (0-31)
 *      Type        - 0: kbps, 1: pps
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK               - Success
 *      RT_ERR_SMI              - SMI access error
 *      RT_ERR_FILTER_METER_ID  - Invalid meter
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicShareMeterType(uint32_t index, uint32_t type)
{
    uint32_t reg;

    if (index > RTL8367C_METERMAX)
        return RT_ERR_FILTER_METER_ID;

    if (index < 32)
        reg = RTL8367C_REG_METER_MODE_SETTING0 + (index / 16);
    else
        reg = RTL8367C_REG_METER_MODE_SETTING2 + ((index - 32) / 16);
    return rtl8367c_setAsicRegBit(reg, index % 16, type);
}
int32_t rtl8367::rtk_rate_shareMeter_set(uint32_t index, rtk_meter_type_t type, uint32_t rate, rtk_enable_t ifg_include)
{
    int32_t retVal;

    if (index > halCtrl.max_meter_id)
        return RT_ERR_FILTER_METER_ID;

    if (type >= METER_TYPE_END)
        return RT_ERR_INPUT;

    if (ifg_include >= RTK_ENABLE_END)
        return RT_ERR_INPUT;

    switch (type)
    {
    case METER_TYPE_KBPS:
        if (rate > RTL8367C_QOS_RATE_INPUT_MAX_HSG || rate < RTL8367C_QOS_RATE_INPUT_MIN)
            return RT_ERR_RATE;

        if ((retVal = rtl8367c_setAsicShareMeter(index, rate >> 3, ifg_include)) != RT_ERR_OK)
            return retVal;

        break;
    case METER_TYPE_PPS:
        if (rate > RTL8367C_QOS_PPS_INPUT_MAX || rate < RTL8367C_QOS_PPS_INPUT_MIN)
            return RT_ERR_RATE;

        if ((retVal = rtl8367c_setAsicShareMeter(index, rate, ifg_include)) != RT_ERR_OK)
            return retVal;

        break;
    default:
        return RT_ERR_INPUT;
    }

    /* Set Type */
    if ((retVal = rtl8367c_setAsicShareMeterType(index, (uint32_t)type)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_getAsicShareMeter
 * Description:
 *      Get meter configuration
 * Input:
 *      index   - hared meter index (0-31)
 *      pRate   - 17-bits rate of share meter, unit is 8Kpbs
 *      pIfg    - Including IFG in rate calculation, 1:include 0:exclude
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK               - Success
 *      RT_ERR_SMI              - SMI access error
 *      RT_ERR_FILTER_METER_ID  - Invalid meter
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicShareMeter(uint32_t index, uint32_t *pRate, uint32_t *pIfg)
{
    uint32_t regData;
    uint32_t regData2;
    int32_t retVal;

    if (index > RTL8367C_METERMAX)
        return RT_ERR_FILTER_METER_ID;

    if (index < 32)
    {
        /*17-bits Rate*/
        retVal = rtl8367c_getAsicReg(RTL8367C_METER_RATE_REG(index), &regData);
        if (retVal != RT_ERR_OK)
            return retVal;

        retVal = rtl8367c_getAsicReg(RTL8367C_METER_RATE_REG(index) + 1, &regData2);
        if (retVal != RT_ERR_OK)
            return retVal;

        *pRate = ((regData2 << 16) & 0x70000) | regData;
        /*IFG*/
        retVal = rtl8367c_getAsicRegBit(RTL8367C_METER_IFG_CTRL_REG(index), RTL8367C_METER_IFG_OFFSET(index), pIfg);

        return retVal;
    }
    else
    {
        /*17-bits Rate*/
        retVal = rtl8367c_getAsicReg(RTL8367C_REG_METER32_RATE_CTRL0 + ((index - 32) << 1), &regData);
        if (retVal != RT_ERR_OK)
            return retVal;

        retVal = rtl8367c_getAsicReg(RTL8367C_REG_METER32_RATE_CTRL0 + ((index - 32) << 1) + 1, &regData2);
        if (retVal != RT_ERR_OK)
            return retVal;

        *pRate = ((regData2 << 16) & 0x70000) | regData;
        /*IFG*/
        retVal = rtl8367c_getAsicRegBit(RTL8367C_REG_METER_IFG_CTRL2 + ((index - 32) >> 4), RTL8367C_METER_IFG_OFFSET(index), pIfg);

        return retVal;
    }
}
/* Function Name:
 *      rtl8367c_getAsicShareMeterType
 * Description:
 *      Get meter Type
 * Input:
 *      index       - shared meter index (0-31)
 * Output:
 *      pType       - 0: kbps, 1: pps
 * Return:
 *      RT_ERR_OK               - Success
 *      RT_ERR_SMI              - SMI access error
 *      RT_ERR_FILTER_METER_ID  - Invalid meter
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicShareMeterType(uint32_t index, uint32_t *pType)
{
    uint32_t reg;

    if (index > RTL8367C_METERMAX)
        return RT_ERR_FILTER_METER_ID;

    if (NULL == pType)
        return RT_ERR_NULL_POINTER;

    if (index < 32)
        reg = RTL8367C_REG_METER_MODE_SETTING0 + (index / 16);
    else
        reg = RTL8367C_REG_METER_MODE_SETTING2 + ((index - 32) / 16);
    return rtl8367c_getAsicRegBit(reg, index % 16, pType);
}
int32_t rtl8367::rtk_rate_shareMeter_get(uint32_t index, rtk_meter_type_t *pType, uint32_t *pRate, rtk_enable_t *pIfg_include)
{
    int32_t retVal;
    uint32_t regData;

    if (index > halCtrl.max_meter_id)
        return RT_ERR_FILTER_METER_ID;

    if (NULL == pType)
        return RT_ERR_NULL_POINTER;

    if (NULL == pRate)
        return RT_ERR_NULL_POINTER;

    if (NULL == pIfg_include)
        return RT_ERR_NULL_POINTER;

    if ((retVal = rtl8367c_getAsicShareMeter(index, &regData, (uint32_t *)pIfg_include)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_getAsicShareMeterType(index, (uint32_t *)pType)) != RT_ERR_OK)
        return retVal;

    if (*pType == METER_TYPE_KBPS)
        *pRate = regData << 3;
    else
        *pRate = regData;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsicShareMeterBucketSize
 * Description:
 *      Set meter related leaky bucket threshold
 * Input:
 *      index       - hared meter index (0-31)
 *      lbthreshold - Leaky bucket threshold of meter
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK               - Success
 *      RT_ERR_SMI              - SMI access error
 *      RT_ERR_FILTER_METER_ID  - Invalid meter
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicShareMeterBucketSize(uint32_t index, uint32_t lbthreshold)
{

    if (index > RTL8367C_METERMAX)
        return RT_ERR_FILTER_METER_ID;

    if (index < 32)
        return rtl8367c_setAsicReg(RTL8367C_METER_BUCKET_SIZE_REG(index), lbthreshold);
    else
        return rtl8367c_setAsicReg(RTL8367C_REG_METER32_BUCKET_SIZE + index - 32, lbthreshold);
}
/* Function Name:
 *      rtl8367c_getAsicShareMeterBucketSize
 * Description:
 *      Get meter related leaky bucket threshold
 * Input:
 *      index       - hared meter index (0-31)
 *      pLbthreshold - Leaky bucket threshold of meter
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK               - Success
 *      RT_ERR_SMI              - SMI access error
 *      RT_ERR_FILTER_METER_ID  - Invalid meter
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicShareMeterBucketSize(uint32_t index, uint32_t *pLbthreshold)
{
    if (index > RTL8367C_METERMAX)
        return RT_ERR_FILTER_METER_ID;

    if (index < 32)
        return rtl8367c_getAsicReg(RTL8367C_METER_BUCKET_SIZE_REG(index), pLbthreshold);
    else
        return rtl8367c_getAsicReg(RTL8367C_REG_METER32_BUCKET_SIZE + index - 32, pLbthreshold);
}
int32_t rtl8367::rtk_rate_shareMeterBucket_set(uint32_t index, uint32_t bucket_size)
{
    int32_t retVal;

    if (index > halCtrl.max_meter_id)
        return RT_ERR_FILTER_METER_ID;

    if (bucket_size > RTL8367C_METERBUCKETSIZEMAX)
        return RT_ERR_INPUT;

    if ((retVal = rtl8367c_setAsicShareMeterBucketSize(index, bucket_size)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_rate_shareMeterBucket_get(uint32_t index, uint32_t *pBucket_size)
{
    int32_t retVal;

    if (index > halCtrl.max_meter_id)
        return RT_ERR_FILTER_METER_ID;

    if (NULL == pBucket_size)
        return RT_ERR_NULL_POINTER;

    if ((retVal = rtl8367c_getAsicShareMeterBucketSize(index, pBucket_size)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

// -------------------------------------------- IGMP --------------------------------------------

/* Function Name:
 *      rtl8367c_setAsicLutIpMulticastLookup
 * Description:
 *      Set Lut IP multicast lookup function
 * Input:
 *      enabled - 1: enabled, 0: disabled
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK   - Success
 *      RT_ERR_SMI  - SMI access error
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicLutIpMulticastLookup(uint32_t enabled)
{
    return rtl8367c_setAsicRegBit(RTL8367C_REG_LUT_CFG, RTL8367C_LUT_IPMC_HASH_OFFSET, enabled);
}

/* Function Name:
 *      rtl8367c_setAsicLutIpLookupMethod
 * Description:
 *      Set Lut IP lookup hash with DIP or {DIP,SIP} pair
 * Input:
 *      type - 1: When DIP can be found in IPMC_GROUP_TABLE, use DIP+SIP Hash, otherwise, use DIP+(SIP=0.0.0.0) Hash.
 *             0: When DIP can be found in IPMC_GROUP_TABLE, use DIP+(SIP=0.0.0.0) Hash, otherwise use DIP+SIP Hash.
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK   - Success
 *      RT_ERR_SMI  - SMI access error
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicLutIpLookupMethod(uint32_t type)
{
    return rtl8367c_setAsicRegBit(RTL8367C_REG_LUT_CFG, RTL8367C_LUT_IPMC_LOOKUP_OP_OFFSET, type);
}

/* Function Name:
 *      rtl8367c_setAsicIGMPv1Opeartion
 * Description:
 *      Set port-based IGMPv1 Control packet action
 * Input:
 *      port            - port number
 *      igmpv1_op       - IGMPv1 control packet action
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_PORT_ID  - Error PORT ID
 *      RT_ERR_SMI      - SMI access error
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicIGMPv1Opeartion(uint32_t port, uint32_t igmpv1_op)
{
    int32_t retVal;

    if (port > RTL8367C_PORTIDMAX)
        return RT_ERR_PORT_ID;

    if (igmpv1_op >= PROTOCOL_OP_END)
        return RT_ERR_INPUT;

    /* IGMPv1 operation */
    if (port < 8)
    {
        retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_IGMP_PORT0_CONTROL + port, RTL8367C_IGMP_PORT0_CONTROL_IGMPV1_OP_MASK, igmpv1_op);
        if (retVal != RT_ERR_OK)
            return retVal;
    }
    else
    {
        retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_IGMP_PORT8_CONTROL + port - 8, RTL8367C_IGMP_PORT0_CONTROL_IGMPV1_OP_MASK, igmpv1_op);
        if (retVal != RT_ERR_OK)
            return retVal;
    }

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsicIGMPv2Opeartion
 * Description:
 *      Set port-based IGMPv2 Control packet action
 * Input:
 *      port            - port number
 *      igmpv2_op       - IGMPv2 control packet action
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_PORT_ID  - Error PORT ID
 *      RT_ERR_SMI      - SMI access error
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicIGMPv2Opeartion(uint32_t port, uint32_t igmpv2_op)
{
    int32_t retVal;

    if (port > RTL8367C_PORTIDMAX)
        return RT_ERR_PORT_ID;

    if (igmpv2_op >= PROTOCOL_OP_END)
        return RT_ERR_INPUT;

    /* IGMPv2 operation */
    if (port < 8)
    {
        retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_IGMP_PORT0_CONTROL + port, RTL8367C_IGMP_PORT0_CONTROL_IGMPV2_OP_MASK, igmpv2_op);
        if (retVal != RT_ERR_OK)
            return retVal;
    }
    else
    {
        retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_IGMP_PORT8_CONTROL + port - 8, RTL8367C_IGMP_PORT0_CONTROL_IGMPV2_OP_MASK, igmpv2_op);
        if (retVal != RT_ERR_OK)
            return retVal;
    }

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsicIGMPv3Opeartion
 * Description:
 *      Set port-based IGMPv3 Control packet action
 * Input:
 *      port            - port number
 *      igmpv3_op       - IGMPv3 control packet action
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_PORT_ID  - Error PORT ID
 *      RT_ERR_SMI      - SMI access error
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicIGMPv3Opeartion(uint32_t port, uint32_t igmpv3_op)
{
    int32_t retVal;

    if (port > RTL8367C_PORTIDMAX)
        return RT_ERR_PORT_ID;

    if (igmpv3_op >= PROTOCOL_OP_END)
        return RT_ERR_INPUT;

    /* IGMPv3 operation */
    if (port < 8)
    {
        retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_IGMP_PORT0_CONTROL + port, RTL8367C_IGMP_PORT0_CONTROL_IGMPV3_OP_MASK, igmpv3_op);
        if (retVal != RT_ERR_OK)
            return retVal;
    }
    else
    {
        retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_IGMP_PORT8_CONTROL + port - 8, RTL8367C_IGMP_PORT0_CONTROL_IGMPV3_OP_MASK, igmpv3_op);
        if (retVal != RT_ERR_OK)
            return retVal;
    }

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsicMLDv1Opeartion
 * Description:
 *      Set port-based MLDv1 Control packet action
 * Input:
 *      port            - port number
 *      mldv1_op        - MLDv1 control packet action
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_PORT_ID  - Error PORT ID
 *      RT_ERR_SMI      - SMI access error
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicMLDv1Opeartion(uint32_t port, uint32_t mldv1_op)
{
    int32_t retVal;

    if (port > RTL8367C_PORTIDMAX)
        return RT_ERR_PORT_ID;

    if (mldv1_op >= PROTOCOL_OP_END)
        return RT_ERR_INPUT;

    /* MLDv1 operation */
    if (port < 8)
    {
        retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_IGMP_PORT0_CONTROL + port, RTL8367C_IGMP_PORT0_CONTROL_MLDv1_OP_MASK, mldv1_op);
        if (retVal != RT_ERR_OK)
            return retVal;
    }
    else
    {
        retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_IGMP_PORT8_CONTROL + port - 8, RTL8367C_IGMP_PORT0_CONTROL_MLDv1_OP_MASK, mldv1_op);
        if (retVal != RT_ERR_OK)
            return retVal;
    }

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsicMLDv2Opeartion
 * Description:
 *      Set port-based MLDv2 Control packet action
 * Input:
 *      port            - port number
 *      mldv2_op        - MLDv2 control packet action
 * Output:
 *      none
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_PORT_ID  - Error PORT ID
 *      RT_ERR_SMI      - SMI access error
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicMLDv2Opeartion(uint32_t port, uint32_t mldv2_op)
{
    int32_t retVal;

    if (port > RTL8367C_PORTIDMAX)
        return RT_ERR_PORT_ID;

    if (mldv2_op >= PROTOCOL_OP_END)
        return RT_ERR_INPUT;

    /* MLDv2 operation */
    if (port < 8)
    {
        retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_IGMP_PORT0_CONTROL + port, RTL8367C_IGMP_PORT0_CONTROL_MLDv2_OP_MASK, mldv2_op);
        if (retVal != RT_ERR_OK)
            return retVal;
    }
    else
    {
        retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_IGMP_PORT8_CONTROL + port - 8, RTL8367C_IGMP_PORT0_CONTROL_MLDv2_OP_MASK, mldv2_op);
        if (retVal != RT_ERR_OK)
            return retVal;
    }

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsicIGMPAllowDynamicRouterPort
 * Description:
 *      Set IGMP dynamic router port allow mask
 * Input:
 *      pmsk    - Allow dynamic router port mask
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK           - Success
 *      RT_ERR_SMI          - SMI access error
 *      RT_ERR_PORT_MASK    - Invalid port mask
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicIGMPAllowDynamicRouterPort(uint32_t pmsk)
{
    return rtl8367c_setAsicReg(RTL8367C_REG_IGMP_MLD_CFG4, pmsk);
}

/* Function Name:
 *      rtl8367c_setAsicIGMPFastLeaveEn
 * Description:
 *      Enable/Disable Fast Leave
 * Input:
 *      enabled - 1:enable Fast Leave; 0:disable Fast Leave
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_SMI      - SMI access error
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicIGMPFastLeaveEn(uint32_t enabled)
{
    int32_t retVal;

    /* Fast Leave */
    retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_IGMP_MLD_CFG0, RTL8367C_FAST_LEAVE_EN_MASK, (enabled >= 1) ? 1 : 0);
    if (retVal != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsicIGMPReportLeaveFlood
 * Description:
 *      Set IGMP/MLD Report/Leave flood
 * Input:
 *      flood   - 0: Reserved, 1: flooding to router ports, 2: flooding to all ports, 3: flooding to router port or to all ports if there is no router port
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_SMI      - SMI access error
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicIGMPReportLeaveFlood(uint32_t flood)
{
    int32_t retVal;

    retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_IGMP_MLD_CFG3, RTL8367C_REPORT_LEAVE_FORWARD_MASK, flood);
    if (retVal != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsicIgmp
 * Description:
 *      Set IGMP/MLD state
 * Input:
 *      enabled     - 1: enabled, 0: disabled
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK   - Success
 *      RT_ERR_SMI  - SMI access error
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicIgmp(uint32_t enabled)
{
    int32_t retVal;

    /* Enable/Disable H/W IGMP/MLD */
    retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_IGMP_MLD_CFG0, RTL8367C_IGMP_MLD_EN_OFFSET, enabled);

    return retVal;
}

/* Function Name:
 *      rtl8367c_getAsicIgmp
 * Description:
 *      Get IGMP/MLD state
 * Input:
 *      enabled     - 1: enabled, 0: disabled
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK   - Success
 *      RT_ERR_SMI  - SMI access error
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicIgmp(uint32_t *ptr_enabled)
{
    int32_t retVal;

    retVal = rtl8367c_getAsicRegBit(RTL8367C_REG_IGMP_MLD_CFG0, RTL8367C_IGMP_MLD_EN_OFFSET, ptr_enabled);
    return retVal;
}
int32_t rtl8367::rtk_igmp_init()
{
    int32_t retVal;
    uint32_t port;

    if ((retVal = rtl8367c_setAsicLutIpMulticastLookup(ENABLED)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicLutIpLookupMethod(1)) != RT_ERR_OK)
        return retVal;

    RTK_SCAN_ALL_PHY_PORTMASK(port)
    {
        if ((retVal = rtl8367c_setAsicIGMPv1Opeartion(port, PROTOCOL_OP_ASIC)) != RT_ERR_OK)
            return retVal;

        if ((retVal = rtl8367c_setAsicIGMPv2Opeartion(port, PROTOCOL_OP_ASIC)) != RT_ERR_OK)
            return retVal;

        if ((retVal = rtl8367c_setAsicIGMPv3Opeartion(port, PROTOCOL_OP_FLOOD)) != RT_ERR_OK)
            return retVal;

        if ((retVal = rtl8367c_setAsicMLDv1Opeartion(port, PROTOCOL_OP_ASIC)) != RT_ERR_OK)
            return retVal;

        if ((retVal = rtl8367c_setAsicMLDv2Opeartion(port, PROTOCOL_OP_FLOOD)) != RT_ERR_OK)
            return retVal;
    }

    if ((retVal = rtl8367c_setAsicIGMPAllowDynamicRouterPort(halCtrl.phy_portmask)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicIGMPFastLeaveEn(ENABLED)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicIGMPReportLeaveFlood(1)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicIgmp(ENABLED)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_igmp_state_set(rtk_enable_t enabled)
{
    int32_t retVal;

    if (enabled >= RTK_ENABLE_END)
        return RT_ERR_INPUT;

    if ((retVal = rtl8367c_setAsicIgmp(enabled)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_igmp_state_get(rtk_enable_t *pEnabled)
{
    int32_t retVal;

    if (pEnabled == NULL)
        return RT_ERR_NULL_POINTER;

    if ((retVal = rtl8367c_getAsicIgmp((uint32_t *)pEnabled)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsicIGMPStaticRouterPort
 * Description:
 *      Set IGMP static router port mask
 * Input:
 *      pmsk    - Static portmask
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK           - Success
 *      RT_ERR_SMI          - SMI access error
 *      RT_ERR_PORT_MASK    - Invalid port mask
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicIGMPStaticRouterPort(uint32_t pmsk)
{
    if (pmsk > RTL8367C_PORTMASK)
        return RT_ERR_PORT_MASK;

    return rtl8367c_setAsicRegBits(RTL8367C_REG_IGMP_STATIC_ROUTER_PORT, RTL8367C_IGMP_STATIC_ROUTER_PORT_MASK, pmsk);
}

int32_t rtl8367::rtk_igmp_static_router_port_set(rtk_portmask_t *pPortmask)
{
    int32_t retVal;
    uint32_t pmask;

    /* Check Valid port mask */
    if (pPortmask == NULL)
        return RT_ERR_NULL_POINTER;

    RTK_CHK_PORTMASK_VALID(pPortmask);

    if ((retVal = rtk_switch_portmask_L2P_get(pPortmask, &pmask)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicIGMPStaticRouterPort(pmask)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_getAsicIGMPStaticRouterPort
 * Description:
 *      Get IGMP static router port mask
 * Input:
 *      pmsk    - Static portmask
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK   - Success
 *      RT_ERR_SMI  - SMI access error
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicIGMPStaticRouterPort(uint32_t *pmsk)
{
    return rtl8367c_getAsicRegBits(RTL8367C_REG_IGMP_STATIC_ROUTER_PORT, RTL8367C_IGMP_STATIC_ROUTER_PORT_MASK, pmsk);
}

int32_t rtl8367::rtk_igmp_static_router_port_get(rtk_portmask_t *pPortmask)
{
    int32_t retVal;
    uint32_t pmask;

    if (pPortmask == NULL)
        return RT_ERR_NULL_POINTER;

    if ((retVal = rtl8367c_getAsicIGMPStaticRouterPort(&pmask)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtk_switch_portmask_P2L_get(pmask, pPortmask)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_igmp_protocol_set(rtk_port_t port, rtk_igmp_protocol_t protocol, rtk_igmp_action_t action)
{
    uint32_t operation;
    int32_t retVal;

    /* Check port valid */
    RTK_CHK_PORT_VALID(port);

    if (protocol >= PROTOCOL_END)
        return RT_ERR_INPUT;

    if (action >= IGMP_ACTION_END)
        return RT_ERR_INPUT;

    switch (action)
    {
    case IGMP_ACTION_FORWARD:
        operation = PROTOCOL_OP_FLOOD;
        break;
    case IGMP_ACTION_TRAP2CPU:
        operation = PROTOCOL_OP_TRAP;
        break;
    case IGMP_ACTION_DROP:
        operation = PROTOCOL_OP_DROP;
        break;
    case IGMP_ACTION_ASIC:
        operation = PROTOCOL_OP_ASIC;
        break;
    default:
        return RT_ERR_INPUT;
    }

    switch (protocol)
    {
    case PROTOCOL_IGMPv1:
        if ((retVal = rtl8367c_setAsicIGMPv1Opeartion(rtk_switch_port_L2P_get(port), operation)) != RT_ERR_OK)
            return retVal;

        break;
    case PROTOCOL_IGMPv2:
        if ((retVal = rtl8367c_setAsicIGMPv2Opeartion(rtk_switch_port_L2P_get(port), operation)) != RT_ERR_OK)
            return retVal;

        break;
    case PROTOCOL_IGMPv3:
        if ((retVal = rtl8367c_setAsicIGMPv3Opeartion(rtk_switch_port_L2P_get(port), operation)) != RT_ERR_OK)
            return retVal;

        break;
    case PROTOCOL_MLDv1:
        if ((retVal = rtl8367c_setAsicMLDv1Opeartion(rtk_switch_port_L2P_get(port), operation)) != RT_ERR_OK)
            return retVal;

        break;
    case PROTOCOL_MLDv2:
        if ((retVal = rtl8367c_setAsicMLDv2Opeartion(rtk_switch_port_L2P_get(port), operation)) != RT_ERR_OK)
            return retVal;

        break;
    default:
        return RT_ERR_INPUT;
    }

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_getAsicIGMPv1Opeartion
 * Description:
 *      Get port-based IGMPv1 Control packet action
 * Input:
 *      port            - port number
 * Output:
 *      igmpv1_op       - IGMPv1 control packet action
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_PORT_ID  - Error PORT ID
 *      RT_ERR_SMI      - SMI access error
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicIGMPv1Opeartion(uint32_t port, uint32_t *igmpv1_op)
{
    int32_t retVal;
    uint32_t value;

    if (port > RTL8367C_PORTIDMAX)
        return RT_ERR_PORT_ID;

    /* IGMPv1 operation */
    if (port < 8)
    {
        retVal = rtl8367c_getAsicRegBits(RTL8367C_REG_IGMP_PORT0_CONTROL + port, RTL8367C_IGMP_PORT0_CONTROL_IGMPV1_OP_MASK, &value);
        if (retVal != RT_ERR_OK)
            return retVal;
    }
    else
    {
        retVal = rtl8367c_getAsicRegBits(RTL8367C_REG_IGMP_PORT8_CONTROL + port - 8, RTL8367C_IGMP_PORT0_CONTROL_IGMPV1_OP_MASK, &value);
        if (retVal != RT_ERR_OK)
            return retVal;
    }

    *igmpv1_op = value;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_getAsicIGMPv2Opeartion
 * Description:
 *      Get port-based IGMPv2 Control packet action
 * Input:
 *      port            - port number
 * Output:
 *      igmpv2_op       - IGMPv2 control packet action
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_PORT_ID  - Error PORT ID
 *      RT_ERR_SMI      - SMI access error
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicIGMPv2Opeartion(uint32_t port, uint32_t *igmpv2_op)
{
    int32_t retVal;
    uint32_t value;

    if (port > RTL8367C_PORTIDMAX)
        return RT_ERR_PORT_ID;

    /* IGMPv2 operation */
    if (port < 8)
    {
        retVal = rtl8367c_getAsicRegBits(RTL8367C_REG_IGMP_PORT0_CONTROL + port, RTL8367C_IGMP_PORT0_CONTROL_IGMPV2_OP_MASK, &value);
        if (retVal != RT_ERR_OK)
            return retVal;
    }
    else
    {
        retVal = rtl8367c_getAsicRegBits(RTL8367C_REG_IGMP_PORT8_CONTROL + port - 8, RTL8367C_IGMP_PORT0_CONTROL_IGMPV2_OP_MASK, &value);
        if (retVal != RT_ERR_OK)
            return retVal;
    }

    *igmpv2_op = value;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_getAsicIGMPv3Opeartion
 * Description:
 *      Get port-based IGMPv3 Control packet action
 * Input:
 *      port            - port number
 * Output:
 *      igmpv3_op       - IGMPv3 control packet action
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_PORT_ID  - Error PORT ID
 *      RT_ERR_SMI      - SMI access error
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicIGMPv3Opeartion(uint32_t port, uint32_t *igmpv3_op)
{
    int32_t retVal;
    uint32_t value;

    if (port > RTL8367C_PORTIDMAX)
        return RT_ERR_PORT_ID;

    /* IGMPv3 operation */
    if (port < 8)
    {
        retVal = rtl8367c_getAsicRegBits(RTL8367C_REG_IGMP_PORT0_CONTROL + port, RTL8367C_IGMP_PORT0_CONTROL_IGMPV3_OP_MASK, &value);
        if (retVal != RT_ERR_OK)
            return retVal;
    }
    else
    {
        retVal = rtl8367c_getAsicRegBits(RTL8367C_REG_IGMP_PORT8_CONTROL + port - 8, RTL8367C_IGMP_PORT0_CONTROL_IGMPV3_OP_MASK, &value);
        if (retVal != RT_ERR_OK)
            return retVal;
    }

    *igmpv3_op = value;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_getAsicMLDv1Opeartion
 * Description:
 *      Get port-based MLDv1 Control packet action
 * Input:
 *      port            - port number
 * Output:
 *      mldv1_op        - MLDv1 control packet action
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_PORT_ID  - Error PORT ID
 *      RT_ERR_SMI      - SMI access error
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicMLDv1Opeartion(uint32_t port, uint32_t *mldv1_op)
{
    int32_t retVal;
    uint32_t value;

    if (port > RTL8367C_PORTIDMAX)
        return RT_ERR_PORT_ID;

    /* MLDv1 operation */
    if (port < 8)
    {
        retVal = rtl8367c_getAsicRegBits(RTL8367C_REG_IGMP_PORT0_CONTROL + port, RTL8367C_IGMP_PORT0_CONTROL_MLDv1_OP_MASK, &value);
        if (retVal != RT_ERR_OK)
            return retVal;
    }
    else
    {
        retVal = rtl8367c_getAsicRegBits(RTL8367C_REG_IGMP_PORT8_CONTROL + port - 8, RTL8367C_IGMP_PORT0_CONTROL_MLDv1_OP_MASK, &value);
        if (retVal != RT_ERR_OK)
            return retVal;
    }

    *mldv1_op = value;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_getAsicMLDv2Opeartion
 * Description:
 *      Get port-based MLDv2 Control packet action
 * Input:
 *      port            - port number
 * Output:
 *      mldv2_op        - MLDv2 control packet action
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_PORT_ID  - Error PORT ID
 *      RT_ERR_SMI      - SMI access error
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicMLDv2Opeartion(uint32_t port, uint32_t *mldv2_op)
{
    int32_t retVal;
    uint32_t value;

    if (port > RTL8367C_PORTIDMAX)
        return RT_ERR_PORT_ID;

    /* MLDv2 operation */
    if (port < 8)
    {
        retVal = rtl8367c_getAsicRegBits(RTL8367C_REG_IGMP_PORT0_CONTROL + port, RTL8367C_IGMP_PORT0_CONTROL_MLDv2_OP_MASK, &value);
        if (retVal != RT_ERR_OK)
            return retVal;
    }
    else
    {
        retVal = rtl8367c_getAsicRegBits(RTL8367C_REG_IGMP_PORT8_CONTROL + port - 8, RTL8367C_IGMP_PORT0_CONTROL_MLDv2_OP_MASK, &value);
        if (retVal != RT_ERR_OK)
            return retVal;
    }

    *mldv2_op = value;

    return RT_ERR_OK;
}
int32_t rtl8367::rtk_igmp_protocol_get(rtk_port_t port, rtk_igmp_protocol_t protocol, rtk_igmp_action_t *pAction)
{
    uint32_t operation;
    int32_t retVal;

    /* Check port valid */
    RTK_CHK_PORT_VALID(port);

    if (protocol >= PROTOCOL_END)
        return RT_ERR_INPUT;

    if (pAction == NULL)
        return RT_ERR_NULL_POINTER;

    switch (protocol)
    {
    case PROTOCOL_IGMPv1:
        if ((retVal = rtl8367c_getAsicIGMPv1Opeartion(rtk_switch_port_L2P_get(port), &operation)) != RT_ERR_OK)
            return retVal;

        break;
    case PROTOCOL_IGMPv2:
        if ((retVal = rtl8367c_getAsicIGMPv2Opeartion(rtk_switch_port_L2P_get(port), &operation)) != RT_ERR_OK)
            return retVal;

        break;
    case PROTOCOL_IGMPv3:
        if ((retVal = rtl8367c_getAsicIGMPv3Opeartion(rtk_switch_port_L2P_get(port), &operation)) != RT_ERR_OK)
            return retVal;

        break;
    case PROTOCOL_MLDv1:
        if ((retVal = rtl8367c_getAsicMLDv1Opeartion(rtk_switch_port_L2P_get(port), &operation)) != RT_ERR_OK)
            return retVal;

        break;
    case PROTOCOL_MLDv2:
        if ((retVal = rtl8367c_getAsicMLDv2Opeartion(rtk_switch_port_L2P_get(port), &operation)) != RT_ERR_OK)
            return retVal;

        break;
    default:
        return RT_ERR_INPUT;
    }

    switch (operation)
    {
    case PROTOCOL_OP_FLOOD:
        *pAction = IGMP_ACTION_FORWARD;
        break;
    case PROTOCOL_OP_TRAP:
        *pAction = IGMP_ACTION_TRAP2CPU;
        break;
    case PROTOCOL_OP_DROP:
        *pAction = IGMP_ACTION_DROP;
        break;
    case PROTOCOL_OP_ASIC:
        *pAction = IGMP_ACTION_ASIC;
        break;
    default:
        return RT_ERR_FAILED;
    }

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_igmp_fastLeave_set(rtk_enable_t state)
{
    int32_t retVal;

    if (state >= RTK_ENABLE_END)
        return RT_ERR_INPUT;

    if ((retVal = rtl8367c_setAsicIGMPFastLeaveEn((uint32_t)state)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_getAsicIGMPFastLeaveEn
 * Description:
 *      Get Fast Leave state
 * Input:
 *      None
 * Output:
 *      penabled        - 1:enable Fast Leave; 0:disable Fast Leave
 * Return:
 *      RT_ERR_OK       - Success
 *      RT_ERR_SMI      - SMI access error
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicIGMPFastLeaveEn(uint32_t *penabled)
{
    int32_t retVal;
    uint32_t value;

    /* Fast Leave */
    retVal = rtl8367c_getAsicRegBits(RTL8367C_REG_IGMP_MLD_CFG0, RTL8367C_FAST_LEAVE_EN_MASK, &value);
    if (retVal != RT_ERR_OK)
        return retVal;

    *penabled = value;

    return RT_ERR_OK;
}
int32_t rtl8367::rtk_igmp_fastLeave_get(rtk_enable_t *pState)
{
    uint32_t fast_leave;
    int32_t retVal;

    if (pState == NULL)
        return RT_ERR_NULL_POINTER;

    if ((retVal = rtl8367c_getAsicIGMPFastLeaveEn(&fast_leave)) != RT_ERR_OK)
        return retVal;

    *pState = (rtk_enable_t)((fast_leave == 1) ? ENABLED : DISABLED);
    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsicIGMPPortMAXGroup
 * Description:
 *      Set per-port Max group number
 * Input:
 *      port        - Physical port number (0~7)
 *      max_group   - max IGMP group
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK           - Success
 *      RT_ERR_SMI          - SMI access error
 *      RT_ERR_PORT_ID      - Invalid port number
 *      RT_ERR_OUT_OF_RANGE - input parameter out of range
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicIGMPPortMAXGroup(uint32_t port, uint32_t max_group)
{
    int32_t retVal;

    if (port > RTL8367C_PORTIDMAX)
        return RT_ERR_PORT_ID;

    if (max_group > RTL8367C_IGMP_MAX_GOUP)
        return RT_ERR_OUT_OF_RANGE;

    if (port < 8)
    {
        retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_IGMP_PORT01_MAX_GROUP + (port / 2), RTL8367C_PORT0_MAX_GROUP_MASK << (RTL8367C_PORT1_MAX_GROUP_OFFSET * (port % 2)), max_group);
        if (retVal != RT_ERR_OK)
            return retVal;
    }
    else
    {
        retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_IGMP_PORT89_MAX_GROUP + (port / 2), RTL8367C_PORT0_MAX_GROUP_MASK << (RTL8367C_PORT1_MAX_GROUP_OFFSET * (port % 2)), max_group);
        if (retVal != RT_ERR_OK)
            return retVal;
    }

    return RT_ERR_OK;
}
int32_t rtl8367::rtk_igmp_maxGroup_set(rtk_port_t port, uint32_t group)
{
    int32_t retVal;

    /* Check port valid */
    RTK_CHK_PORT_VALID(port);

    if (group > RTL8367C_IGMP_MAX_GOUP)
        return RT_ERR_OUT_OF_RANGE;

    if ((retVal = rtl8367c_setAsicIGMPPortMAXGroup(rtk_switch_port_L2P_get(port), group)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_getAsicIGMPPortMAXGroup
 * Description:
 *      Get per-port Max group number
 * Input:
 *      port        - Physical port number (0~7)
 *      max_group   - max IGMP group
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK           - Success
 *      RT_ERR_SMI          - SMI access error
 *      RT_ERR_PORT_ID      - Invalid port number
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicIGMPPortMAXGroup(uint32_t port, uint32_t *max_group)
{
    int32_t retVal;
    uint32_t value;

    if (port > RTL8367C_PORTIDMAX)
        return RT_ERR_PORT_ID;

    if (port < 8)
    {
        retVal = rtl8367c_getAsicRegBits(RTL8367C_REG_IGMP_PORT01_MAX_GROUP + (port / 2), RTL8367C_PORT0_MAX_GROUP_MASK << (RTL8367C_PORT1_MAX_GROUP_OFFSET * (port % 2)), &value);
        if (retVal != RT_ERR_OK)
            return retVal;
    }
    else
    {
        retVal = rtl8367c_getAsicRegBits(RTL8367C_REG_IGMP_PORT89_MAX_GROUP + (port / 2), RTL8367C_PORT0_MAX_GROUP_MASK << (RTL8367C_PORT1_MAX_GROUP_OFFSET * (port % 2)), &value);
        if (retVal != RT_ERR_OK)
            return retVal;
    }

    *max_group = value;
    return RT_ERR_OK;
}
int32_t rtl8367::rtk_igmp_maxGroup_get(rtk_port_t port, uint32_t *pGroup)
{
    int32_t retVal;

    /* Check port valid */
    RTK_CHK_PORT_VALID(port);

    if (pGroup == NULL)
        return RT_ERR_NULL_POINTER;

    if ((retVal = rtl8367c_getAsicIGMPPortMAXGroup(rtk_switch_port_L2P_get(port), pGroup)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_getAsicIGMPPortCurrentGroup
 * Description:
 *      Get per-port current group number
 * Input:
 *      port            - Physical port number (0~7)
 *      current_group   - current IGMP group
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK           - Success
 *      RT_ERR_SMI          - SMI access error
 *      RT_ERR_PORT_ID      - Invalid port number
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_getAsicIGMPPortCurrentGroup(uint32_t port, uint32_t *current_group)
{
    int32_t retVal;
    uint32_t value;

    if (port > RTL8367C_PORTIDMAX)
        return RT_ERR_PORT_ID;

    if (port < 8)
    {
        retVal = rtl8367c_getAsicRegBits(RTL8367C_REG_IGMP_PORT01_CURRENT_GROUP + (port / 2), RTL8367C_PORT0_CURRENT_GROUP_MASK << (RTL8367C_PORT1_CURRENT_GROUP_OFFSET * (port % 2)), &value);
        if (retVal != RT_ERR_OK)
            return retVal;
    }
    else
    {
        retVal = rtl8367c_getAsicRegBits(RTL8367C_REG_IGMP_PORT89_CURRENT_GROUP + ((port - 8) / 2), RTL8367C_PORT0_CURRENT_GROUP_MASK << (RTL8367C_PORT1_CURRENT_GROUP_OFFSET * (port % 2)), &value);
        if (retVal != RT_ERR_OK)
            return retVal;
    }

    *current_group = value;
    return RT_ERR_OK;
}
int32_t rtl8367::rtk_igmp_currentGroup_get(rtk_port_t port, uint32_t *pGroup)
{
    int32_t retVal;

    /* Check port valid */
    RTK_CHK_PORT_VALID(port);

    if (pGroup == NULL)
        return RT_ERR_NULL_POINTER;

    if ((retVal = rtl8367c_getAsicIGMPPortCurrentGroup(rtk_switch_port_L2P_get(port), pGroup)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

int32_t rtl8367::clearVlan(uint16_t vlanId)
{
    rtk_vlan_cfg_t vlan1;
    memset(&vlan1, 0x00, sizeof(rtk_vlan_cfg_t));
    RTK_PORTMASK_CLEAR(vlan1.mbr);
    RTK_PORTMASK_CLEAR(vlan1.untag);
    return rtk_vlan_set(vlanId, &vlan1);
}

/* Function Name:
 *      rtl8367c_setAsicPortIngressBandwidth
 * Description:
 *      Set per-port total ingress bandwidth
 * Input:
 *      port        - Physical port number (0~7)
 *      bandwidth   - The total ingress bandwidth (unit: 8Kbps), 0x1FFFF:disable
 *      preifg      - Include preamble and IFG, 0:Exclude, 1:Include
 *      enableFC    - Action when input rate exceeds. 0: Drop   1: Flow Control
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK           - Success
 *      RT_ERR_SMI          - SMI access error
 *      RT_ERR_PORT_ID      - Invalid port number
 *      RT_ERR_OUT_OF_RANGE - input parameter out of range
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicPortIngressBandwidth(uint32_t port, uint32_t bandwidth, uint32_t preifg, uint32_t enableFC)
{
    int32_t retVal;
    uint32_t regData;
    uint32_t regAddr;

    /* Invalid input parameter */
    if (port >= RTL8367C_PORTNO)
        return RT_ERR_PORT_ID;

    if (bandwidth > RTL8367C_QOS_GRANULARTY_MAX)
        return RT_ERR_OUT_OF_RANGE;

    regAddr = RTL8367C_INGRESSBW_PORT_RATE_LSB_REG(port);
    regData = bandwidth & RTL8367C_QOS_GRANULARTY_LSB_MASK;
    retVal = rtl8367c_setAsicReg(regAddr, regData);
    if (retVal != RT_ERR_OK)
        return retVal;

    regAddr += 1;
    regData = (bandwidth & RTL8367C_QOS_GRANULARTY_MSB_MASK) >> RTL8367C_QOS_GRANULARTY_MSB_OFFSET;
    retVal = rtl8367c_setAsicRegBits(regAddr, RTL8367C_INGRESSBW_PORT0_RATE_CTRL1_INGRESSBW_RATE16_MASK, regData);
    if (retVal != RT_ERR_OK)
        return retVal;

    regAddr = RTL8367C_PORT_MISC_CFG_REG(port);
    retVal = rtl8367c_setAsicRegBit(regAddr, RTL8367C_PORT0_MISC_CFG_INGRESSBW_IFG_OFFSET, preifg);
    if (retVal != RT_ERR_OK)
        return retVal;

    regAddr = RTL8367C_PORT_MISC_CFG_REG(port);
    retVal = rtl8367c_setAsicRegBit(regAddr, RTL8367C_PORT0_MISC_CFG_INGRESSBW_FLOWCTRL_OFFSET, enableFC);
    if (retVal != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtk_rate_igrBandwidthCtrlRate_set
 * Description:
 *      Set port ingress bandwidth control
 * Input:
 *      port        - Port id
 *      rate        - Rate of share meter
 *      ifg_include - include IFG or not, ENABLE:include DISABLE:exclude
 *      fc_enable   - enable flow control or not, ENABLE:use flow control DISABLE:drop
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK           - OK
 *      RT_ERR_FAILED       - Failed
 *      RT_ERR_SMI          - SMI access error
 *      RT_ERR_PORT_ID      - Invalid port number.
 *      RT_ERR_ENABLE       - Invalid IFG parameter.
 *      RT_ERR_INBW_RATE    - Invalid ingress rate parameter.
 * Note:
 *      The rate unit is 1 kbps and the range is from 8k to 1048568k. The granularity of rate is 8 kbps.
 *      The ifg_include parameter is used for rate calculation with/without inter-frame-gap and preamble.
 */
int32_t rtl8367::rtk_rate_igrBandwidthCtrlRate_set(rtk_port_t port, uint32_t rate, rtk_enable_t ifg_include, rtk_enable_t fc_enable)
{
    int32_t retVal;

    /* Check Port Valid */
    RTK_CHK_PORT_VALID(port);

    if (ifg_include >= RTK_ENABLE_END)
        return RT_ERR_INPUT;

    if (fc_enable >= RTK_ENABLE_END)
        return RT_ERR_INPUT;

    if (rtk_switch_isHsgPort(port) == RT_ERR_OK)
    {
        if ((rate > RTL8367C_QOS_RATE_INPUT_MAX_HSG) || (rate < RTL8367C_QOS_RATE_INPUT_MIN))
            return RT_ERR_QOS_EBW_RATE;
    }
    else
    {
        if ((rate > RTL8367C_QOS_RATE_INPUT_MAX) || (rate < RTL8367C_QOS_RATE_INPUT_MIN))
            return RT_ERR_QOS_EBW_RATE;
    }

    if ((retVal = rtl8367c_setAsicPortIngressBandwidth(rtk_switch_port_L2P_get(port), rate >> 3, ifg_include, fc_enable)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtl8367c_setAsicPortEgressRate
 * Description:
 *      Set per-port egress rate
 * Input:
 *      port        - Physical port number (0~10)
 *      rate        - Egress rate
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK           - Success
 *      RT_ERR_SMI          - SMI access error
 *      RT_ERR_PORT_ID      - Invalid port number
 *      RT_ERR_QOS_EBW_RATE - Invalid bandwidth/rate
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicPortEgressRate(uint32_t port, uint32_t rate)
{
    int32_t retVal;
    uint32_t regAddr, regData;

    if (port > RTL8367C_PORTIDMAX)
        return RT_ERR_PORT_ID;

    if (rate > RTL8367C_QOS_GRANULARTY_MAX)
        return RT_ERR_QOS_EBW_RATE;

    regAddr = RTL8367C_PORT_EGRESSBW_LSB_REG(port);
    regData = RTL8367C_QOS_GRANULARTY_LSB_MASK & rate;

    retVal = rtl8367c_setAsicReg(regAddr, regData);

    if (retVal != RT_ERR_OK)
        return retVal;

    regAddr = RTL8367C_PORT_EGRESSBW_MSB_REG(port);
    regData = (RTL8367C_QOS_GRANULARTY_MSB_MASK & rate) >> RTL8367C_QOS_GRANULARTY_MSB_OFFSET;

    retVal = rtl8367c_setAsicRegBits(regAddr, RTL8367C_PORT6_EGRESSBW_CTRL1_MASK, regData);

    return retVal;
}

/* Function Name:
 *      rtl8367c_setAsicPortEgressRateIfg
 * Description:
 *      Set per-port egress rate calculate include/exclude IFG
 * Input:
 *      ifg     - 1:include IFG 0:exclude IFG
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK   - Success
 *      RT_ERR_SMI  - SMI access error
 * Note:
 *      None
 */
int32_t rtl8367::rtl8367c_setAsicPortEgressRateIfg(uint32_t ifg)
{
    int32_t retVal;

    retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_SCHEDULE_WFQ_CTRL, RTL8367C_SCHEDULE_WFQ_CTRL_OFFSET, ifg);

    return retVal;
}

/* Function Name:
 *      rtk_rate_egrBandwidthCtrlRate_set
 * Description:
 *      Set port egress bandwidth control
 * Input:
 *      port        - Port id
 *      rate        - Rate of egress bandwidth
 *      ifg_include - include IFG or not, ENABLE:include DISABLE:exclude
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK           - OK
 *      RT_ERR_FAILED       - Failed
 *      RT_ERR_SMI          - SMI access error
 *      RT_ERR_PORT_ID      - Invalid port number.
 *      RT_ERR_INPUT        - Invalid input parameters.
 *      RT_ERR_QOS_EBW_RATE - Invalid egress bandwidth/rate
 * Note:
 *     The rate unit is 1 kbps and the range is from 8k to 1048568k. The granularity of rate is 8 kbps.
 *     The ifg_include parameter is used for rate calculation with/without inter-frame-gap and preamble.
 */
int32_t rtl8367::rtk_rate_egrBandwidthCtrlRate_set(rtk_port_t port, uint32_t rate, rtk_enable_t ifg_include)
{
    int32_t retVal;

    /* Check Port Valid */
    RTK_CHK_PORT_VALID(port);

    if (rtk_switch_isHsgPort(port) == RT_ERR_OK)
    {
        if ((rate > RTL8367C_QOS_RATE_INPUT_MAX_HSG) || (rate < RTL8367C_QOS_RATE_INPUT_MIN))
            return RT_ERR_QOS_EBW_RATE;
    }
    else
    {
        if ((rate > RTL8367C_QOS_RATE_INPUT_MAX) || (rate < RTL8367C_QOS_RATE_INPUT_MIN))
            return RT_ERR_QOS_EBW_RATE;
    }

    if (ifg_include >= RTK_ENABLE_END)
        return RT_ERR_ENABLE;

    if ((retVal = rtl8367c_setAsicPortEgressRate(rtk_switch_port_L2P_get(port), rate >> 3)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicPortEgressRateIfg(ifg_include)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

int32_t rtl8367::rtk_switch_isSgmiiPort(rtk_port_t logicalPort)
{
    if (logicalPort >= RTK_SWITCH_PORT_NUM)
        return RT_ERR_FAILED;

    if (((0x01 << logicalPort) & halCtrl.sg_logical_portmask) != 0)
        return RT_ERR_OK;
    else
        return RT_ERR_FAILED;
}

/* Function Name:
 *      rtl8367c_getSdsLinkStatus
 * Description:
 *      Get SGMII status
 * Input:
 *      id  - EXT ID
 * Output:
 *      None.
 * Return:
 *      RT_ERR_OK                   - Success
 *      RT_ERR_SMI                  - SMI access error
 * Note:
 *      None.
 */
int32_t rtl8367::rtl8367c_getSdsLinkStatus(uint32_t ext_id, uint8_t *pSignalDetect, uint8_t *pSync, uint8_t *pLink)
{
    uint32_t retVal, regValue, type, running = 0, retVal2;

    if ((retVal = rtl8367c_setAsicReg(0x13C2, 0x0249)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_getAsicReg(0x1300, &regValue)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicReg(0x13C2, 0x0000)) != RT_ERR_OK)
        return retVal;

    switch (regValue)
    {
    case 0x0276:
    case 0x0597:
    case 0x6367:
        type = 0;
        break;
    case 0x0652:
    case 0x6368:
        type = 1;
        break;
    case 0x0801:
    case 0x6511:
        type = 2;
        break;
    default:
        return RT_ERR_FAILED;
    }

    if (type == 0)
    {
        if (1 == ext_id)
        {
            if ((retVal = rtl8367c_getAsicRegBit(0x130c, 5, &running)) != RT_ERR_OK)
                return retVal;

            if (running == 1)
            {
                if ((retVal = rtl8367c_setAsicRegBit(0x130c, 5, 0)) != RT_ERR_OK)
                    return retVal;
            }

            retVal = rtl8367c_setAsicReg(0x6601, 0x003D);

            if (retVal == RT_ERR_OK)
                retVal = rtl8367c_setAsicReg(0x6600, 0x0080);

            if (retVal == RT_ERR_OK)
                retVal = rtl8367c_getAsicReg(0x6602, &regValue);

            if (running == 1)
            {
                if ((retVal2 = rtl8367c_setAsicRegBit(0x130c, 5, 1)) != RT_ERR_OK)
                    return retVal2;
            }

            if (retVal != RT_ERR_OK)
                return retVal;

            *pSignalDetect = (regValue & 0x0100) ? 1 : 0;
            *pSync = (regValue & 0x0001) ? 1 : 0;
            *pLink = (regValue & 0x0010) ? 1 : 0;
        }
        else
            return RT_ERR_PORT_ID;
    }
    else if (type == 1)
    {
        if (1 == ext_id)
        {
            if ((retVal = rtl8367c_setAsicReg(0x6601, 0x003D)) != RT_ERR_OK)
                return retVal;
            if ((retVal = rtl8367c_setAsicReg(0x6600, 0x0081)) != RT_ERR_OK)
                return retVal;
            if ((retVal = rtl8367c_getAsicReg(0x6602, &regValue)) != RT_ERR_OK)
                return retVal;

            *pSignalDetect = (regValue & 0x0100) ? 1 : 0;
            *pSync = (regValue & 0x0001) ? 1 : 0;
            *pLink = (regValue & 0x0010) ? 1 : 0;
        }
        else if (2 == ext_id)
        {
            if ((retVal = rtl8367c_setAsicReg(0x6601, 0x003D)) != RT_ERR_OK)
                return retVal;
            if ((retVal = rtl8367c_setAsicReg(0x6600, 0x0080)) != RT_ERR_OK)
                return retVal;
            if ((retVal = rtl8367c_getAsicReg(0x6602, &regValue)) != RT_ERR_OK)
                return retVal;

            *pSignalDetect = (regValue & 0x0100) ? 1 : 0;
            *pSync = (regValue & 0x0001) ? 1 : 0;
            *pLink = (regValue & 0x0010) ? 1 : 0;
        }
        else
            return RT_ERR_PORT_ID;
    }
    else if (type == 2)
    {
        if ((retVal = rtl8367c_getAsicSdsReg(0, 30, 1, &regValue)) != RT_ERR_OK)
            return retVal;
        if ((retVal = rtl8367c_getAsicSdsReg(0, 30, 1, &regValue)) != RT_ERR_OK)
            return retVal;

        *pSignalDetect = (regValue & 0x0100) ? 1 : 0;
        *pSync = (regValue & 0x0001) ? 1 : 0;
        *pLink = (regValue & 0x0010) ? 1 : 0;
    }

    return RT_ERR_OK;
}

/* Function Name:
 *      rtk_port_sgmiiLinkStatus_get
 * Description:
 *      Get SGMII status
 * Input:
 *      port        - Port ID
 * Output:
 *      pSignalDetect   - Signal detect
 *      pSync           - Sync
 *      pLink           - Link - rtk_port_linkStatus_t - Down 0 - Up 1
 * Return:
 *      RT_ERR_OK                   - OK
 *      RT_ERR_FAILED               - Failed
 *      RT_ERR_SMI                  - SMI access error
 *      RT_ERR_PORT_ID              - Invalid port ID.
 * Note:
 *      The API can reset Serdes
 */
int32_t rtl8367::rtk_port_sgmiiLinkStatus_get(rtk_port_t port, uint8_t *pSignalDetect, uint8_t *pSync, uint8_t *pLink)
{
    uint32_t ext_id;

    /* Check Port Valid */
    if (rtk_switch_isSgmiiPort(port) != RT_ERR_OK)
        return RT_ERR_PORT_ID;

    if (NULL == pSignalDetect)
        return RT_ERR_NULL_POINTER;

    if (NULL == pSync)
        return RT_ERR_NULL_POINTER;

    if (NULL == pLink)
        return RT_ERR_NULL_POINTER;

    ext_id = port - 15;
    return rtl8367c_getSdsLinkStatus(ext_id, pSignalDetect, pSync, pLink);
}