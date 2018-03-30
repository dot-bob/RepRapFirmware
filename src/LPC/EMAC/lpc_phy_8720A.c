/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "lpc_phy.h"
#include "lpc17xx_emac.h"


#define EMAC_SMSC_8720A 0x0007C0F0



//--------------------------- write_PHY -------------------------------
//********************************************************************
//@brief       Write value to PHY device
//@param[in]   PhyReg: PHY Register address
//@param[in]   Value:  Value to write
//@return      0 - if success
//             1 - if fail
//********************************************************************
/*static*/ int32_t write_PHY (uint32_t PhyReg, uint16_t Value)
{
    /* Write a data 'Value' to PHY register 'PhyReg'. */
    uint32_t tout;
    
    LPC_EMAC->MADR = EMAC_DEF_ADR | PhyReg;
    LPC_EMAC->MWTD = Value;
    
    /* Wait until operation completed */
    tout = 0;
    for (tout = 0; tout < EMAC_MII_WR_TOUT; tout++) {
        if ((LPC_EMAC->MIND & EMAC_MIND_BUSY) == 0) {
            return (0);
        }
    }
    LWIP_DEBUGF(UDP_LPC_EMAC | LWIP_DBG_TRACE,("write PHY %lu %04X failed!\n", PhyReg, Value));
    // Time out!
    return (-1);
}


//--------------------------- read_PHY -------------------------------
//********************************************************************
//*
//@brief       Read value from PHY device
//@param[in]   PhyReg: PHY Register address
//@return      0 - if success
//             1 - if fail
//*********************************************************************

/*static*/ int32_t read_PHY (uint32_t PhyReg)
{
    /* Read a PHY register 'PhyReg'. */
    uint32_t tout;
    
    LPC_EMAC->MADR = EMAC_DEF_ADR | PhyReg;
    LPC_EMAC->MCMD = EMAC_MCMD_READ;
    
    /* Wait until operation completed */
    tout = 0;
    for (tout = 0; tout < EMAC_MII_RD_TOUT; tout++) {
        if ((LPC_EMAC->MIND & EMAC_MIND_BUSY) == 0) {
            LPC_EMAC->MCMD = 0;
            return (LPC_EMAC->MRDD);
        }
    }
    LWIP_DEBUGF(UDP_LPC_EMAC | LWIP_DBG_TRACE,("read PHY %lu failed!\n", PhyReg));;
    // Time out!
    return (-1);
}


/*********************************************************************//**
* @brief       Set specified PHY mode in EMAC peripheral
* @param[in]   ulPHYMode   Specified PHY mode, should be:
*                          - EMAC_MODE_AUTO
*                          - EMAC_MODE_10M_FULL
*                          - EMAC_MODE_10M_HALF
*                          - EMAC_MODE_100M_FULL
*                          - EMAC_MODE_100M_HALF
* @return      Return (0) if no error, otherwise return (-1)
**********************************************************************/

int32_t emac_SetPHYMode(uint32_t ulPHYMode)
{
    int32_t id1, id2, tout, regv;
    
    id1 = read_PHY (EMAC_PHY_REG_IDR1);
    id2 = read_PHY (EMAC_PHY_REG_IDR2);
    
    if (((id1 << 16) | (id2 & 0xFFF0)) == EMAC_SMSC_8720A) {
        switch(ulPHYMode){
            case EMAC_MODE_AUTO:
                write_PHY (EMAC_PHY_REG_BMCR, EMAC_PHY_AUTO_NEG);
                /* Wait to complete Auto_Negotiation */
                for (tout = EMAC_PHY_RESP_TOUT; tout; tout--) {
                    regv = read_PHY (EMAC_PHY_REG_BMSR);
                    if (regv & EMAC_PHY_BMSR_AUTO_DONE) {
                        /* Auto-negotiation Complete. */
                        break;
                    }
                    if (tout == 0){
                        // Time out, return error
                        return (-1);
                    }
                }
                break;
            case EMAC_MODE_10M_FULL:
                /* Connect at 10MBit full-duplex */
                write_PHY (EMAC_PHY_REG_BMCR, EMAC_PHY_FULLD_10M);
                break;
            case EMAC_MODE_10M_HALF:
                /* Connect at 10MBit half-duplex */
                write_PHY (EMAC_PHY_REG_BMCR, EMAC_PHY_HALFD_10M);
                break;
            case EMAC_MODE_100M_FULL:
                /* Connect at 100MBit full-duplex */
                write_PHY (EMAC_PHY_REG_BMCR, EMAC_PHY_FULLD_100M);
                break;
            case EMAC_MODE_100M_HALF:
                /* Connect at 100MBit half-duplex */
                write_PHY (EMAC_PHY_REG_BMCR, EMAC_PHY_HALFD_100M);
                break;
            default:
                // un-supported
                return (-1);
        }
    }
    // It's not correct module ID
    else {
        LWIP_DEBUGF(UDP_LPC_EMAC | LWIP_DBG_TRACE,("PHY reports id %04lX %04lX - not an SMSC 8720A\n", id1, id2));
        return (-1);
    }
    
    // Update EMAC configuration with current PHY status
    if (EMAC_UpdatePHYStatus() < 0){
        return (-1);
    }
    
    // Complete
    return (0);
}

/*********************************************************************//**
                                                                        * @brief        Auto-Configures value for the EMAC configuration register to
                                                                        *                 match with current PHY mode
                                                                        * @param[in]    None
                                                                        * @return        Return (0) if no error, otherwise return (-1)
                                                                        *
                                                                        * Note: The EMAC configuration will be auto-configured:
                                                                        *         - Speed mode.
                                                                        *         - Half/Full duplex mode
                                                                        **********************************************************************/
int32_t EMAC_UpdatePHYStatus(void)
{
    int32_t regv, tout;
    
    /* Check the link status. */
#ifdef MCB_LPC_1768
    for (tout = EMAC_PHY_RESP_TOUT; tout; tout--) {
        regv = read_PHY (EMAC_PHY_REG_STS);
        if (regv & EMAC_PHY_SR_LINK) {
            /* Link is on. */
            break;
        }
        if (tout == 0){
            // time out
            return (-1);
        }
    }
    /* Configure Full/Half Duplex mode. */
    if (regv & EMAC_PHY_SR_DUP) {
        /* Full duplex is enabled. */
        LPC_EMAC->MAC2    |= EMAC_MAC2_FULL_DUP;
        LPC_EMAC->Command |= EMAC_CR_FULL_DUP;
        LPC_EMAC->IPGT     = EMAC_IPGT_FULL_DUP;
    } else {
        /* Half duplex mode. */
        LPC_EMAC->IPGT = EMAC_IPGT_HALF_DUP;
    }
    if (regv & EMAC_PHY_SR_SPEED) {
        /* 10MBit mode. */
        LPC_EMAC->SUPP = 0;
    } else {
        /* 100MBit mode. */
        LPC_EMAC->SUPP = EMAC_SUPP_SPEED;
    }
#elif defined(IAR_LPC_1768)
    for (tout = EMAC_PHY_RESP_TOUT; tout; tout--) {
        regv = read_PHY (EMAC_PHY_REG_BMSR);
        if (regv & EMAC_PHY_BMSR_LINK_STATUS) {
            /* Link is on. */
            break;
        }
        if (tout == 0){
            // time out
            return (-1);
        }
    }
    
    /* Configure Full/Half Duplex mode. */
    if (regv & EMAC_PHY_SR_FULL_DUP) {
        /* Full duplex is enabled. */
        LPC_EMAC->MAC2    |= EMAC_MAC2_FULL_DUP;
        LPC_EMAC->Command |= EMAC_CR_FULL_DUP;
        LPC_EMAC->IPGT     = EMAC_IPGT_FULL_DUP;
    } else {
        /* Half duplex mode. */
        LPC_EMAC->IPGT = EMAC_IPGT_HALF_DUP;
    }
    
    /* Configure 100MBit/10MBit mode. */
    if (!(regv & EMAC_PHY_SR_100_SPEED)) {
        /* 10MBit mode. */
        LPC_EMAC->SUPP = 0;
    } else {
        /* 100MBit mode. */
        LPC_EMAC->SUPP = EMAC_SUPP_SPEED;
    }
#endif
    // Complete
    return (0);
}

