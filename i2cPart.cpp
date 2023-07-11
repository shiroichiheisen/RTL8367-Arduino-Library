#include "Arduino.h"
#include "rtl8367.h"

#define CLK_DURATION(x) delayMicroseconds(x)

#define ack_timer 10

void rtl8367::_smi_start()
{
    /* change GPIO pin to Output only */
    pinMode(sckPin, OUTPUT);
    pinMode(sdaPin, OUTPUT);

    /* Initial state: SCK: 0, SDA: 1 */
    digitalWrite(sckPin, 0);
    digitalWrite(sdaPin, 1);
    CLK_DURATION(usTransmissionDelay);

    /* CLK 1: 0 -> 1, 1 -> 0 */
    digitalWrite(sckPin, 1);
    CLK_DURATION(usTransmissionDelay);
    digitalWrite(sckPin, 0);
    CLK_DURATION(usTransmissionDelay);

    /* CLK 2: */
    digitalWrite(sckPin, 1);
    CLK_DURATION(usTransmissionDelay);
    digitalWrite(sdaPin, 0);
    CLK_DURATION(usTransmissionDelay);
    digitalWrite(sckPin, 0);
    CLK_DURATION(usTransmissionDelay);
    digitalWrite(sdaPin, 1);
}

void rtl8367::_smi_writeBit(uint16_t signal, uint32_t bitLen)
{

    /* change GPIO pin to Output only */
    pinMode(sdaPin, OUTPUT);

    for (; bitLen > 0; bitLen--)
    {
        CLK_DURATION(usTransmissionDelay);

        /* prepare data */
        if (signal & (1 << (bitLen - 1)))
        {
            digitalWrite(sdaPin, 1);
        }
        else
        {
            digitalWrite(sdaPin, 0);
        }
        CLK_DURATION(usTransmissionDelay);

        /* clocking */
        digitalWrite(sckPin, 1);
        CLK_DURATION(usTransmissionDelay);
        digitalWrite(sckPin, 0);
    }
}

void rtl8367::_smi_readBit(uint32_t bitLen, uint32_t *rData)
{
    uint32_t u = 0;

    /* change GPIO pin to Input only */
    pinMode(sdaPin, INPUT);

    for (*rData = 0; bitLen > 0; bitLen--)
    {
        CLK_DURATION(usTransmissionDelay);

        /* clocking */
        digitalWrite(sckPin, 1);
        CLK_DURATION(usTransmissionDelay);
        u = digitalRead(sdaPin);
        digitalWrite(sckPin, 0);

        *rData |= (u << (bitLen - 1));
    }
}

void rtl8367::_smi_stop()
{

    /* change GPIO pin to Output only */
    pinMode(sdaPin, OUTPUT);
    CLK_DURATION(usTransmissionDelay);
    digitalWrite(sdaPin, 0);
    digitalWrite(sckPin, 1);
    CLK_DURATION(usTransmissionDelay);
    digitalWrite(sdaPin, 1);
    CLK_DURATION(usTransmissionDelay);
    digitalWrite(sckPin, 1);
    CLK_DURATION(usTransmissionDelay);
    digitalWrite(sckPin, 0);
    CLK_DURATION(usTransmissionDelay);
    digitalWrite(sckPin, 1);

    /* add a click */
    CLK_DURATION(usTransmissionDelay);
    digitalWrite(sckPin, 0);
    CLK_DURATION(usTransmissionDelay);
    digitalWrite(sckPin, 1);
}

int32_t rtl8367::smi_read(uint32_t mAddrs, uint32_t *rData)
{
    uint32_t rawData = 0, ACK;
    uint8_t con;
    uint32_t ret = RT_ERR_OK;

    if (mAddrs > 0xFFFF)
        return RT_ERR_INPUT;

    if (rData == NULL)
        return RT_ERR_NULL_POINTER;

    _smi_start(); /* Start SMI */

    _smi_writeBit(0x0b, 4); /* CTRL code: 4'b1011 for RTL8370 */

    _smi_writeBit(0x4, 3); /* CTRL code: 3'b100 */

    _smi_writeBit(0x1, 1); /* 1: issue READ command */

    con = 0;
    do
    {
        con++;
        _smi_readBit(1, &ACK); /* ACK for issuing READ command*/
    } while ((ACK != 0) && (con < ack_timer));

    if (ACK != 0)
        ret = RT_ERR_FAILED;

    _smi_writeBit((mAddrs & 0xff), 8); /* Set reg_addr[7:0] */

    con = 0;
    do
    {
        con++;
        _smi_readBit(1, &ACK); /* ACK for setting reg_addr[7:0] */
    } while ((ACK != 0) && (con < ack_timer));

    if (ACK != 0)
        ret = RT_ERR_FAILED;

    _smi_writeBit((mAddrs >> 8), 8); /* Set reg_addr[15:8] */

    con = 0;
    do
    {
        con++;
        _smi_readBit(1, &ACK); /* ACK by RTL8369 */
    } while ((ACK != 0) && (con < ack_timer));
    if (ACK != 0)
        ret = RT_ERR_FAILED;

    _smi_readBit(8, &rawData); /* Read DATA [7:0] */
    *rData = rawData & 0xff;

    _smi_writeBit(0x00, 1); /* ACK by CPU */

    _smi_readBit(8, &rawData); /* Read DATA [15: 8] */

    _smi_writeBit(0x01, 1); /* ACK by CPU */
    *rData |= (rawData << 8);

    _smi_stop();

    return ret;
}

int32_t rtl8367::smi_write(uint32_t mAddrs, uint32_t rData)
{
    int8_t con;
    uint32_t ACK;
    uint32_t ret = RT_ERR_OK;

    if (mAddrs > 0xFFFF)
        return RT_ERR_INPUT;

    if (rData > 0xFFFF)
        return RT_ERR_INPUT;

    _smi_start(); /* Start SMI */

    _smi_writeBit(0x0b, 4); /* CTRL code: 4'b1011 for RTL8370*/

    _smi_writeBit(0x4, 3); /* CTRL code: 3'b100 */

    _smi_writeBit(0x0, 1); /* 0: issue WRITE command */

    con = 0;
    do
    {
        con++;
        _smi_readBit(1, &ACK); /* ACK for issuing WRITE command*/
    } while ((ACK != 0) && (con < ack_timer));
    if (ACK != 0)
        ret = RT_ERR_FAILED;

    _smi_writeBit((mAddrs & 0xff), 8); /* Set reg_addr[7:0] */

    con = 0;
    do
    {
        con++;
        _smi_readBit(1, &ACK); /* ACK for setting reg_addr[7:0] */
    } while ((ACK != 0) && (con < ack_timer));
    if (ACK != 0)
        ret = RT_ERR_FAILED;

    _smi_writeBit((mAddrs >> 8), 8); /* Set reg_addr[15:8] */

    con = 0;
    do
    {
        con++;
        _smi_readBit(1, &ACK); /* ACK for setting reg_addr[15:8] */
    } while ((ACK != 0) && (con < ack_timer));
    if (ACK != 0)
        ret = RT_ERR_FAILED;

    _smi_writeBit(rData & 0xff, 8); /* Write Data [7:0] out */

    con = 0;
    do
    {
        con++;
        _smi_readBit(1, &ACK); /* ACK for writting data [7:0] */
    } while ((ACK != 0) && (con < ack_timer));
    if (ACK != 0)
        ret = RT_ERR_FAILED;

    _smi_writeBit(rData >> 8, 8); /* Write Data [15:8] out */

    con = 0;
    do
    {
        con++;
        _smi_readBit(1, &ACK); /* ACK for writting data [15:8] */
    } while ((ACK != 0) && (con < ack_timer));
    if (ACK != 0)
        ret = RT_ERR_FAILED;

    _smi_stop();

    return ret;
}

int32_t rtl8367::rtl8367c_setAsicReg(uint32_t reg, uint32_t value)
{
    int32_t retVal;

    retVal = smi_write(reg, value);

    if (retVal != RT_ERR_OK)
        return RT_ERR_SMI;

    return RT_ERR_OK;
}

int32_t rtl8367::rtl8367c_getAsicReg(uint32_t reg, uint32_t *pValue)
{
    uint32_t regData;
    int32_t retVal;

    retVal = smi_read(reg, &regData);
    if (retVal != RT_ERR_OK)
        return RT_ERR_SMI;

    *pValue = regData;

    return RT_ERR_OK;
}