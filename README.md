# The library its on development, so its a huge mess, I will clear later.

To use this library, you will need to connect the sda and sck pins to the arduino, !important! you must mantain the rtl8367 eeprom for him to start, and after that you can use the library to configure the switch. !important! the rtl8367 series is 3.3v! so you will need to use a logic level converter to use it with the arduino if you are using a 5v arduino.

This library dont use the arduino wire library, its using its own software i2c, so you can use almost any pin for sda and sck, the pins just need to have digital input and output capabilities.

# Important:

You need to wait 1 second after the switch is powered on to use the library, because the switch needs time to start and read the eeprom, if your creating a board and dont have the eeprom data, you can use the .bin file on the eeprom folder to program the eeprom, the file is a dump of the eeprom of a rtl8367 switch, so you can use it to program your eeprom.
This library is based on the realteck original library, theres a programming guide from realteck on this library.

# Functions Added:

- [x] rtk_switch_probe - tested
- [x] rtk_port_phyStatus_get - tested
- [x] rtk_vlan_init - tested
- [x] rtk_vlan_set
- [x] rtk_vlan_get
- [x] rtk_vlan_portPvid_set
- [x] rtk_vlan_portPvid_get
- [x] rtk_vlan_portIgrFilterEnable_set
- [x] rtk_vlan_portAcceptFrameType_set
- [x] rtk_vlan_tagMode_set
- [x] rtk_vlan_transparent_set
- [x] rtk_svlan_init
- [x] rtk_svlan_servicePort_add
- [x] rtk_svlan_tpidEntry_set
- [x] rtk_svlan_memberPortEntry_set
- [x] rtk_svlan_defaultSvlan_set
- [x] rtk_svlan_c2s_add
- [x] rtk_svlan_sp2c_add
- [x] rtk_svlan_untag_action_set
- [x] rtk_svlan_unmatch_action_set
- [x] rtk_svlan_dmac_vidsel_set
- [x] rtk_l2_addr_add
- [x] rtk_l2_addr_del
- [x] rtk_l2_addr_get
- [x] rtk_l2_addr_next_get
- [x] rtk_l2_mcastAddr_add
- [x] rtk_l2_mcastAddr_del
- [x] rtk_l2_mcastAddr_get
- [x] rtk_l2_mcastAddr_next_get
- [x] rtk_l2_ipMcastAddr_add
- [x] rtk_l2_ipMcastAddr_del
- [x] rtk_l2_ipMcastAddr_get
- [x] rtk_l2_ipMcastAddr_next_get
- [x] rtk_l2_ipVidMcastAddr_add
- [x] rtk_l2_ipVidMcastAddr_del
- [x] rtk_l2_ipVidMcastAddr_get
- [x] rtk_l2_ipVidMcastAddr_next_get
- [ ] rtk_l2_flushtype_set - Dont contain in api 1.3.11 or 1.3.12
- [x] rtk_qos_init
- [x] rtk_qos_portPri_set
- [x] rtk_qos_1pPriRemap_set
- [x] rtk_qos_priSel_set
- [x] rtk_qos_portPriSelIndex_set   
- [x] rtk_qos_priMap_set
- [x] rtk_qos_schedulingQueue_set
- [x] rtk_cpu_enable_set
- [x] rtk_cpu_tagPort_set
- [x] rtk_cpu_tagPort_get
- [x] rtk_int_polarity_set
- [x] rtk_int_polarity_get
- [x] rtk_int_control_set
- [x] rtk_int_control_get
- [x] rtk_int_status_get
- [x] rtk_int_status_set
- [x] rtk_int_advanceInfo_get
- [x] rtk_stat_port_get
- [x] rtk_stat_port_reset
- [x] rtk_port_phyEnableAll_set
- [x] rtk_port_phyAutoNegoAbility_set
- [x] rtk_port_phyAutoNegoAbility_get

# tested on:

- [x] RTL8367S

# used the library on:

- [x] Esp32
- [x] Esp32-s2