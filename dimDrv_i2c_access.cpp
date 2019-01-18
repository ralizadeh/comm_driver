 
/*******************************************************************************
 *
 * File        : dimDrv_i2c_access.cpp
 *
 * Created Date: Aug 20, 2018
 *
 * Description : This file provides I2C driver to access DIM IDPROM
 * 
 *******************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include "os/task.h"
//#include "dimDrv_public.h"
#include "dimDrv_main.h"
#include "ehsi_dim_mgr.h"

struct timespec delay_ms; 
boolean  dimDrv_debugMde = FALSE;
boolean  CRC_error = FALSE;        /* TRUE when there is error */

/* CCITT CRC16 calculation lookup table */
const unsigned short RPD_crc16_ccitt_Table[256] = {
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
        0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
        0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
        0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
        0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
        0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
        0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
        0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
        0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
        0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
        0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
        0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
        0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
        0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
        0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
        0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
        0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
        0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
        0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
        0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
        0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
        0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
        0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
        0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
        0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
        0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
        0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
        0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
        0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
        0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
        0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
        0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

#define CCITT_CRC16_SEED         0xFFFF

uint32 mutex_lock_count   = 0;
uint32 mutex_unlock_count = 0;


/********************************************************************************
 *  
 *   registerDimI2C:  calculates I2C address 
 *                       
 *   Parameters: offset
 *                 
 *   output:  I2C_master_address
 *   return: void    
 *  
 *******************************************************************************/
void registerDimI2C(uint32 offset, uint32 *I2C_master_address)  
{
   *I2C_master_address = RPD_I2C_BASE_ADDRESS + offset; 
}

/**********************************************************
 *  RPD_i2c_access_trigger
 * 
 *  Parameters:  devIndex, PRD_I2C_base_addr, ctrl1_reg, slave_addr, sleep_ms
 *    
 *  Return:      void 
 *
 **********************************************************/
dimDrv_rc_t RPD_i2c_access_trigger(uint8 devIndex, uint32 *ctrl1_reg_p, uint32  slave_addr, uint32 sleep_ms)
{
    dimDrv_rc_t   rc = DIM_DRV_OK;
    uint32        i2c_address;
    uint32        pre_trigger_ctrl1 = RPD_REG_VALUE_RESET;
    uint32        i2c_init_fields = 0x1F;    /* for clear DELT bits*/
    uint32        count   = 0;
    delay_ms.tv_sec = 0;
    delay_ms.tv_nsec = sleep_ms * 1000 * 1000;  // sleep_ms delay


    /* clean DELT first before trigger */  
    registerDimI2C(I2C_MASTER_DELT, &i2c_address); 
    //i2c_address = RPD_I2C_BASE_ADDRESS + I2C_MASTER_DELT;
    printf("Register-map-address for clean DELT RPD_i2c_access_trigger= 0x%08x \n", i2c_address);
    
    if ( (rc= dim_RPD_write (devIndex, i2c_address, &i2c_init_fields, sizeof(i2c_init_fields))) != DIM_DRV_OK) 
    {
       DIM_DRV_DEBUG(dimDrv_debugMde,"%s, I2C write failed rc=%d\n",__FUNCTION__, rc);
       printf(" I2C write failed for clean DELT RPD_i2c_access_trigger rc=%d\n", rc);
       return rc;
    }

    /* clean up the trigger in  I2C CTRL1 */ 
    registerDimI2C(I2C_MASTER_CTL1, &i2c_address); 
    //i2c_address = RPD_I2C_BASE_ADDRESS + I2C_MASTER_CTL1;
    printf("Register-map-address for clean up RPD_i2c_access_trigger= 0x%08x \n", i2c_address);
     
    if ( (rc= dim_RPD_write (devIndex, i2c_address, &pre_trigger_ctrl1, sizeof(pre_trigger_ctrl1))) != DIM_DRV_OK) 
    {
       DIM_DRV_DEBUG(dimDrv_debugMde,"%s, I2C write failed rc=%d\n",__FUNCTION__, rc);
       printf(" I2C write failed for clean up RPD_i2c_access_trigger rc=%d\n", rc);
       return rc;
    }

    /* trigger */
    if ( (rc= dim_RPD_write (devIndex, i2c_address, ctrl1_reg_p, sizeof(ctrl1_reg_p))) != DIM_DRV_OK) 
    {
       DIM_DRV_DEBUG(dimDrv_debugMde,"%s, I2C write failed rc=%d\n",__FUNCTION__, rc);
       printf(" I2C write failed for trigger RPD_i2c_access_trigger rc=%d\n", rc);
       return rc;
    }
    
    task_sleep(&delay_ms);

    /* check for interrupt if it is completed */
    count = RPD_I2C_LOOP_LIMIT;
    registerDimI2C(I2C_MASTER_DELT, &i2c_address); 
    //i2c_address = RPD_I2C_BASE_ADDRESS + I2C_MASTER_DELT;
    printf("Register-map-address for check for interrupt if it is completed RPD_i2c_access_trigger= 0x%08x \n", i2c_address);
    do
    {  /* read & check rightaway for performance */
       if((rc = dim_RPD_read (devIndex, i2c_address, &i2c_init_fields, sizeof(i2c_init_fields))) !=  DIM_DRV_OK)
       {
          DIM_DRV_DEBUG(dimDrv_debugMde,"%s, I2C read failed rc=%d\n",__FUNCTION__, rc);
          printf(" I2C read failed for RPD_i2c_access_trigger (check for interrupt) rc=%d\n", rc);
          return rc;
       }        
   
       if (i2c_init_fields & (RPD_I2C_OP_COMPLETE | RPD_I2C_WDOG_TIMEOUT))
       {
          break; 
       }
       VOLUNTARY_PREEMPTION;    /* schedule(0), other: RPD_msleep(0) */
    } while ( count-- > 0);
   
    if ((i2c_init_fields & RPD_I2C_WDOG_TIMEOUT) == RPD_I2C_WDOG_TIMEOUT)
    {
       DIM_DRV_DEBUG(dimDrv_debugMde,"%s: I2C watchdog timeout(hw err or app err when dev pwr down),base_addr=0x%x,ctrl1_reg=0x%x,slave_addr=0x%x,int_delt=0x%x,count=%d\n",
                                     __FUNCTION__, RPD_I2C_BASE_ADDRESS, *ctrl1_reg_p, slave_addr, i2c_init_fields, count);
       printf(" I2C watchdog timeout(hw err or app err when dev pwr down),base_addr=0x%x,ctrl1_reg=0x%x,slave_addr=0x%x,int_delt=0x%x,count=%d\n",
                                      RPD_I2C_BASE_ADDRESS, *ctrl1_reg_p, slave_addr, i2c_init_fields, count);
       return DIM_DRV_I2C_WDOG_TIMEOUT; 
    }

    /* complete bit was set due to error abort */
    if  (((i2c_init_fields & RPD_I2C_OP_COMPLETE) == RPD_I2C_OP_COMPLETE)  && (i2c_init_fields & (~RPD_I2C_OP_COMPLETE)) ) 
    {
       DIM_DRV_DEBUG(dimDrv_debugMde,"%s: error encountered and operation abort, base_addr=0x%x,ctrl1_reg=0x%x,slave_addr=0x%x,int_delt=0x%x,count=%d\n",
                                     __FUNCTION__, RPD_I2C_BASE_ADDRESS, *ctrl1_reg_p, slave_addr, i2c_init_fields, count);
       printf("error encountered and operation abort - complete bit, base_addr=0x%x,ctrl1_reg=0x%x,slave_addr=0x%x,int_delt=0x%x,count=%d\n",
                                      RPD_I2C_BASE_ADDRESS, *ctrl1_reg_p, slave_addr, i2c_init_fields, count);
       return  DIM_DRV_DEV_ACCESS_ERROR;
    }

     /* count exhausted, should not happen  */
    if  ((i2c_init_fields & RPD_I2C_OP_COMPLETE) != RPD_I2C_OP_COMPLETE)
    {
        DIM_DRV_DEBUG(dimDrv_debugMde,"%s: HW ERROR?  count limit (%d) is reached, base_addr=0x%x,ctrl1_reg=0x%x,slave_addr=0x%x,int_delt=0x%x\n",
                                     __FUNCTION__, count, RPD_I2C_BASE_ADDRESS, *ctrl1_reg_p, slave_addr, i2c_init_fields);
        printf(" HW ERROR?  count limit (%d) is reached, base_addr=0x%x,ctrl1_reg=0x%x,slave_addr=0x%x,int_delt=0x%x\n",
                                      count, RPD_I2C_BASE_ADDRESS, *ctrl1_reg_p, slave_addr, i2c_init_fields);
        return DIM_DRV_I2C_UNCLASSIFIED_ERROR; 
    }

    return rc;
}

/**********************************************************
 *   utility funtion convert uint8 Array to uint32 Array    
 *
***********************************************************/
void uint8_to_uint32_conversion(uint8 *buf1_p, uint32 *buf2_p, uint32 size, boolean swap_bytes)
{
   uint32 i, j;

   DIM_DRV_DEBUG (dimDrv_debugMde, "%s, size = %d, swap =%d", __FUNCTION__, size, swap_bytes);
      
   for(i = 0; i < ALIGN4BYPE(size); i++)
   {
      j = i/4;

      if(swap_bytes)
      {
         /*  order change */
         buf2_p[j] = (uint32) ( ( buf1_p[i+3]<<24) | ( buf1_p[i+2]<<16) | (buf1_p[i+1]<<8) | buf1_p[i]);
         i +=3;         
      }
      else
      {
         /* no order change */
         buf2_p[j] = (uint32) ( ( buf1_p[i]<<24) | ( buf1_p[i+1]<<16) | (buf1_p[i+2]<<8) | buf1_p[i+3]);   
         i +=3;         
      }
   }
}

/**********************************************************
 *   RPD_i2c_access
 *
 *  ctrl1_Reg:   31     trigger_operation
 *               30     watchdog_enable
 *               29     slave_reset
 *               28-22  unused
 *               21-20  if_sel
 *               19-18  unused
 *               17-16  speed
 *               15     unused
 *               14-12  operation_type   
 *               11-8   unused
 *               7-1    slave_address
 *               0      unused 
 *
 **********************************************************/
dimDrv_rc_t RPD_i2c_access (uint8 devIndex, uint32 slave_addr, uint8 *buf, uint8 write_size, uint8 read_size, i2c_operation_type_e operation_type)
{
    dimDrv_rc_t   rc = DIM_DRV_OK;
    uint32   bigBuf[RPD_I2C_BUF_SIZE];
    uint32   i2c_address;
    uint32   i2c_speed;
    uint8    wsize;
    uint8    rsize;
    uint32   ctrl1_Reg_reset = 0;
    uint32   ctrl1_Reg = 0;
    uint32   ctrl2_Reg = 0;
    uint32   sleep_ms = 0;
    
    
    // set I2C write data
    registerDimI2C(I2C_TX_BUF, &i2c_address); 
    //i2c_address = RPD_I2C_BASE_ADDRESS + I2C_TX_BUF;   // PRD_I2C_base_addr = 0x04000
    printf("Register-map-address = 0x%08x , operation_type = %d\n", i2c_address, operation_type);
    i2c_speed = PRD_I2C_400K_SPEED;
    printf("i2c_speed = 0x%08x , operation_type = %d\n", i2c_speed, operation_type);

    sleep_ms = RPD_I2C_TIMER_SLEEP_VALUE_400K(read_size, write_size);

    switch (operation_type)
    {
       case I2C_WRITE:
           
            uint8_to_uint32_conversion(buf, bigBuf, write_size, TRUE);

            wsize = write_size/4 + ((write_size%4)?1:0);
            if((rc = dim_RPD_write (devIndex, i2c_address, (uint32*)bigBuf, wsize)) !=  DIM_DRV_OK) 
            {
                DIM_DRV_DEBUG(dimDrv_debugMde,"%s, I2C write failed rc=%d\n",__FUNCTION__, rc);
                printf(" I2C write failed for RPD_i2c_access rc=%d\n", rc);
                return rc;
            }

            ctrl2_Reg = I2C_CTRL2_REG_4_WRITE | (write_size << PRD_I2C_BIT_SHIFT) | write_size;
            printf("ctrl2_Reg0 = 0x%08x , operation_type = %d\n", ctrl2_Reg, operation_type);   
            break;

        case I2C_WRITE_FOLLOWBY_READ: 

            printf(" read i2c_address  =  0x%x \n ", i2c_address); 
            uint8_to_uint32_conversion(buf, bigBuf, write_size, TRUE);
            
            wsize = write_size/4 + ((write_size%4)?1:0);
            if((rc = dim_RPD_write (devIndex, i2c_address, (uint32*)bigBuf, wsize)) !=  DIM_DRV_OK) 
            {
                DIM_DRV_DEBUG(dimDrv_debugMde,"%s, I2C write followed by read failed rc=%d\n",__FUNCTION__, rc);
                printf(" I2C write followed by read failed for RPD_i2c_access rc=%d\n", rc);
                return rc;
            }

            ctrl2_Reg = I2C_CTRL2_REG_4_WRITE_READ | (read_size << PRD_I2C_BIT_SHIFT);
            printf("ctrl2_Reg0 = 0x%08x , operation_type = %d\n", ctrl2_Reg, operation_type);    
            break;

        case I2C_READ: 

            ctrl2_Reg = I2C_CTRL2_REG_4_WRITE_READ | (read_size << PRD_I2C_BIT_SHIFT);
            printf("ctrl2_Reg0 = 0x%08x , operation_type = %d\n", ctrl2_Reg, operation_type);     
            break;

        case I2C_RESET: 
            break;

        default:
           DIM_DRV_DEBUG(dimDrv_debugMde,"%s, operation type =0x%x",__FUNCTION__, operation_type);
           printf(" operation type for RPD_i2c_access=0x%x", operation_type);
           return DIM_DRV_ERR;
    }

    /* the number ms to sleep, I2C EEPROM write need to wait 5 ms while other based on Tx and Rx bytes length */
    if(operation_type == I2C_WRITE) 
    {
       sleep_ms = PRD_I2C_EEPROM_WRITE_SLEEP_VALUE;
    }

    /* set I2C CTRL2 */
    printf("ctrl2_Reg = 0x%08x , operation_type = %d\n", ctrl2_Reg, operation_type);  
    registerDimI2C(I2C_MASTER_CTL2, &i2c_address); 
    //i2c_address = RPD_I2C_BASE_ADDRESS + I2C_MASTER_CTL2;
    printf("Register-map-address = 0x%08x , operation_type = %d\n", i2c_address, operation_type);

    if((rc = dim_RPD_write (devIndex, i2c_address, &ctrl2_Reg, sizeof(ctrl2_Reg))) !=  DIM_DRV_OK) 
    {
          DIM_DRV_DEBUG(dimDrv_debugMde,"%s, I2C write failed rc=%d\n",__FUNCTION__, rc);
          return rc;
    }
                                         
    /* set I2C CTRL1 */
    ctrl1_Reg = (uint32) (RPD_I2C_TRIGGER | RPD_I2C_WDOG_EN | i2c_speed |
                         (operation_type << PRD_I2C_OP_TYPE_SHIFT) | 
                         ((slave_addr & PRD_I2C_SLAVE_ADDR_MASK) << PRD_I2C_SLAVE_ADDR_SHIFT));   
    printf("ctrl1_Reg = 0x%08x , operation_type = %d\n", ctrl1_Reg, operation_type);    

    rc = RPD_i2c_access_trigger(devIndex, &ctrl1_Reg, slave_addr, sleep_ms);

    ctrl1_Reg = 0;

    sleep_ms = PRD_I2C_EEPROM_WRITE_SLEEP_VALUE;

    ctrl1_Reg = (uint32) (RPD_I2C_TRIGGER | RPD_I2C_WDOG_EN | i2c_speed |
                         (operation_type << PRD_I2C_OP_TYPE_SHIFT) | 
                         ((slave_addr & PRD_I2C_SLAVE_ADDR_MASK) << PRD_I2C_SLAVE_ADDR_SHIFT));  
    
    if(rc != DIM_DRV_OK)
    {
        // retry one more time 

        // reset I2C first 
        ctrl1_Reg_reset = (uint32) (RPD_I2C_TRIGGER | RPD_I2C_WDOG_EN | i2c_speed |                              
                                   (operation_type << PRD_I2C_OP_TYPE_SHIFT) | 
                                   ((slave_addr & PRD_I2C_SLAVE_ADDR_MASK) << PRD_I2C_SLAVE_ADDR_SHIFT)); 
        printf("ctrl1_Reg_reset = 0x%08x , operation_type = %d\n", ctrl1_Reg_reset, operation_type);

        DIM_DRV_DEBUG(dimDrv_debugMde, "%s, first write failed, try to recover,Register-map-address= 0x%x, ctrl1_Reg_reset = 0x%x rc=%d\n", 
                __FUNCTION__, i2c_address, ctrl1_Reg_reset, rc);

        if( (rc = RPD_i2c_access_trigger(devIndex, &ctrl1_Reg_reset, slave_addr, sleep_ms)) != DIM_DRV_OK)
        {
            DIM_DRV_DEBUG(dimDrv_debugMde, "%s, I2C interface reset failed, rc=%d\n",__FUNCTION__, rc);
            printf("I2C interface reset failed, rc=%d\n", rc);
            return rc;
        }

        // re-trigger again 
        if( (rc = RPD_i2c_access_trigger(devIndex, &ctrl1_Reg, slave_addr, sleep_ms)) != DIM_DRV_OK)
        {
            DIM_DRV_DEBUG(dimDrv_debugMde, "%s, I2C operation failed, rc=%d\n",__FUNCTION__, rc);
            printf( " I2C operation failed for RPD_i2c_access, rc=%d\n", rc);
            return rc;
        } 
    }

    // read data
    if ( operation_type == I2C_WRITE_FOLLOWBY_READ || operation_type == I2C_READ) 
    { 
        //i2c_address = RPD_I2C_BASE_ADDRESS + I2C_RX_BUF;
        registerDimI2C(I2C_RX_BUF, &i2c_address); 
        printf("This is read after R/W+R operations");
        printf("Register-map-address = 0x%08x , operation_type = %d\n", i2c_address, operation_type);

        uint8_to_uint32_conversion(buf, bigBuf, read_size, TRUE);

        rsize = read_size/4 + ((read_size%4)?1:0);
        if((rc = dim_RPD_read (devIndex, i2c_address, (uint32*)bigBuf, rsize)) !=  DIM_DRV_OK)  
        {
                 DIM_DRV_DEBUG(dimDrv_debugMde,"%s, I2C read failed rc=%d\n",__FUNCTION__, rc);
                 printf(" I2C read failed rc=%d\n", rc);
                 return rc;
        }
    }
    return rc;
}

/**************************************************
 *   RPD_cat34t_set_eeprom_page
 * 
 *   Description:    set Idprom page0 or page 1
 *    
 *   Parameters:     pageNum             - idprom page number
 *  
 *************************************************/
dimDrv_rc_t RPD_cat34t_set_eeprom_page(uint8 devIndex, uint32 pageNum)
{
    dimDrv_rc_t   rc = DIM_DRV_OK;

    uint32  operation_type =  I2C_WRITE;
    uint32  slave_addr  = 0;
    uint8   data[] = {0xBB, 0xAA};
    uint8   write_size = 3;
    uint8   read_size = 0;
    uint8   sleep_ms = 5;
    delay_ms.tv_sec = 0;
    delay_ms.tv_nsec = sleep_ms * 1000 * 1000;  // sleep_ms delay
    
    if(pageNum == 0 ) //CAT34T_EE_MEM_PAGE0)
    {
       slave_addr = CAT34T_SET_SPD_PAGE0;      
    }
    else
    {  
       slave_addr = CAT34T_SET_SPD_PAGE1;   
    }

    rc = RPD_i2c_access (devIndex, slave_addr, data, write_size, read_size, operation_type);

    if(rc != DIM_DRV_OK)
    {
        DIM_DRV_DEBUG(dimDrv_debugMde, "%s, RPD_base_addr=0x%x, data= 0x%p, size=%x, operation type=%x, rc=%d\n", 
                    __FUNCTION__, RPD_I2C_BASE_ADDRESS, data, write_size, rc);
    }

    /* delay 5 ms to make sure the page access is changed */
    task_sleep(&delay_ms);

    return rc;
}

/**********************************************************
 *   RPD_calc_crc16
 *   
 *
 **********************************************************/
static unsigned short
RPD_calc_crc16(unsigned short crc, unsigned char* pData, unsigned len)
{
    for (; len != 0; len--)
    {
        crc = ((crc << 8) ^ RPD_crc16_ccitt_Table[(crc >> 8) ^ *pData]);   
        pData++;
    }

    printf("======================"); 
    printf("Dim crc (func) =  0x%x ", crc);
    printf("======================\n");  

    return crc;
}

/**********************************************************
 *  RPD_cat34t_access
 * 
 *
 **********************************************************/
dimDrv_rc_t RPD_cat34t_access(uint8 devIndex, uint32 slave_dev, uint8 I2C_offset, uint8 *buf, uint8 size, i2c_operation_type_e action_tye, uint32 eeprom_page)
{
    dimDrv_rc_t   rc = DIM_DRV_OK;
    uint32        slave_addr = 0;
    uint8         d_buf[IDPROM_RPD_MAX_SIZE * 2] = {0};
    uint8         page_position = 0;
    
    switch (slave_dev)
    {
      case 0:   
           slave_addr = RPD_I2C_EEPROM0_BASE_ADDRESS;
           break;
      case 1:  
           slave_addr = RPD_I2C_EEPROM1_BASE_ADDRESS;
           break;
      default:
            // Wrong slave device selection 
            DIM_DRV_DEBUG(dimDrv_debugMde, "%s, Incorrect I2C BUS device, 0x%x", __FUNCTION__, slave_dev); 
            printf(" Incorrect I2C BUS device, 0x%x", slave_dev); 
            return DIM_DRV_INVALID_DEV_IDX; 
    }

    d_buf[0] = page_position;
    DIM_DRV_DEBUG(dimDrv_debugMde, "%s, Port[%d] d_buf[0]=[0x%x], d_buf[1]=[0x%x], d_buf[2]=[0x%x], d_buf[3]=[0x%x], d_buf[4]=[0x%x], d_buf[5]=[0x%x]\n", 
                        __FUNCTION__, devIndex, d_buf[0], d_buf[1], d_buf[2], d_buf[3], d_buf[4], d_buf[5]);
    printf("Port[%d] d_buf[0]=[0x%x], d_buf[1]=[0x%x], d_buf[2]=[0x%x], d_buf[3]=[0x%x], d_buf[4]=[0x%x], d_buf[5]=[0x%x]\n", 
                        devIndex, d_buf[0], d_buf[1], d_buf[2], d_buf[3], d_buf[4], d_buf[5]);

    
    /* Lock mutex */
    //mutex_offset = RPD_base_addr & RPD_I2C_MUTEX_MASK_ADDRESS;
    //rc = dim_i2c_mutex_lock(devIndex, mutex_offset);   //???????????????????????????
    rc = dim_i2c_mutex_lock(devIndex);
    if(rc != DIM_DRV_OK)
    {
        DIM_DRV_DEBUG(dimDrv_debugMde, "%s, Port[%d] Can't lock RPD mutex before programming one page %d, rc=%d\n", 
                    __FUNCTION__, devIndex, eeprom_page, rc);
        printf( " Port[%d] Can't lock RPD mutex before programming one page, rc=%d\n", 
                     devIndex, rc);
        return DIM_DRV_I2C_MUTEX_ERROR;
    }
    mutex_lock_count ++;

    printf("\n========= Start of Page set=============\n");
    rc = RPD_cat34t_set_eeprom_page(devIndex, eeprom_page);
    printf("\n========= End of Page set=============\n");
    
    if(rc != DIM_DRV_OK)
    {
        DIM_DRV_DEBUG(dimDrv_debugMde, "%s, Port[%d] set EEPROM page was failed. rc=%d\n", 
                    __FUNCTION__, devIndex, rc);
        printf( " Port[%d] set EEPROM page was failed. rc=%d\n", 
                     devIndex, rc);
        return rc;
    }
    else
    {
        switch (action_tye) 
        {
            case I2C_WRITE:
                
                if (size <= CAT34T_MAX_WRITE_SIZE  /* bytes */) 
                {
                    /* less than 16 bytes, can do a single access */
                    memcpy(d_buf+1, buf,size);
                    rc = RPD_i2c_access (devIndex, slave_addr, d_buf, size+1, 0, action_tye);
                    
                    if(rc !=  DIM_DRV_OK) 
                    {
                        DIM_DRV_DEBUG(dimDrv_debugMde,"%s, RPD_i2c_access was failed rc=%d\n",__FUNCTION__, rc);
                        printf(" RPD_i2c_access was failed rc=%d\n", rc);
                    }
                }
                break;
        
            case I2C_WRITE_FOLLOWBY_READ:
            case I2C_READ:
               
                d_buf[0] = page_position;
                printf("read_buf[0]=page_position = 0x%x \n", d_buf[0]);
                
                if ( size <= CAT34T_MAX_WRITE_SIZE  /* bytes */) 
                {
                    /* less than 16 bytes, can do a single access */


                    printf(" read slave address  =  0x%x \n ", slave_addr);
                    rc = RPD_i2c_access (devIndex, slave_addr, d_buf, 1, size, I2C_WRITE_FOLLOWBY_READ);  
                   
                    memcpy(buf, d_buf+1, size);

                    if(rc !=  DIM_DRV_OK) 
                    {
                        DIM_DRV_DEBUG(dimDrv_debugMde,"%s, RPD_i2c_access was failed rc=%d\n",__FUNCTION__, rc);
                        printf(" RPD_i2c_access was failed rc=%d\n", rc);
                    }
                }
        
                break;
            case I2C_RESET:
                rc = RPD_i2c_access (devIndex, slave_addr, d_buf, size, size, action_tye);
                    
                    if(rc !=  DIM_DRV_OK) 
                    {
                        DIM_DRV_DEBUG(dimDrv_debugMde,"%s, RPD_i2c_access was failed rc=%d\n",__FUNCTION__, rc);
                        printf(" RPD_i2c_access was failed rc=%d\n", rc);
                    }
                    break;
        
            default:
                DIM_DRV_DEBUG (dimDrv_debugMde, "%s, Not a valid operation. RPD_base_addr (0x%x), slave_addr (0x%x) \n", 
                                                __FUNCTION__, RPD_I2C_BASE_ADDRESS, slave_addr); 
                printf (" Not a valid operation. RPD_base_addr (0x%x), slave_addr (0x%x) \n", 
                                                 RPD_I2C_BASE_ADDRESS, slave_addr); 
        }
    }

    /* Unlock mutex */ 
    if(dim_i2c_mutex_unlock(devIndex) != DIM_DRV_OK)
    {
        DIM_DRV_DEBUG(dimDrv_debugMde, "%s, Port[%d] Can't unlock RPD mutex, rc=%d\n", 
                    __FUNCTION__, devIndex, rc);
        printf( "Port[%d] Can't unlock RPD mutex, rc=%d\n", 
                     devIndex, rc);
        return DIM_DRV_I2C_MUTEX_ERROR;
    }
    mutex_unlock_count ++;

    return rc;
}

/**********************************************************
 *   I2C_IDPROM_access
 *
 **********************************************************/
dimDrv_rc_t I2C_IDPROM_access (uint8 devIndex, uint32 slave_dev, uint8 *buf, I2C_age_fields_t *dim_data, uint32 dataSize, uint32 eeprom_page, i2c_operation_type_e  access_type)
{
    dimDrv_rc_t        rc = DIM_DRV_OK;
    uint32             RPD_offset; 
    uint16             crc = CCITT_CRC16_SEED;
    uint32             size = 0;
   
   
    // Handle toggling of DIM connection 
    if (Dim_connectionToggle[devIndex] == TRUE) 
    {  
        DIM_DRV_DEBUG(dimDrv_debugMde, "%s There is a toggling in Dim connection for devIndex %d rc=%d \n", __FUNCTION__, devIndex);
        return DIM_DRV_TOGGLE;
    }
    else
    {
        switch (access_type) 
        {
        
            case I2C_WRITE_FOLLOWBY_READ:
            case I2C_READ: 
               
                RPD_offset = 0; 
                //rc = RPD_cat34t_access(devIndex, I2C_base_addr, slave_dev, RPD_offset,    
                //                       buf, dataSize, access_type, eeprom_page, mutex_id); 

                rc = RPD_cat34t_access(devIndex, slave_dev, RPD_offset,    
                                       buf, dataSize, access_type, eeprom_page); 
                
                dim_data = (I2C_age_fields_t *)buf;

                crc = RPD_calc_crc16(crc, buf, dataSize - CCITT_CRC16_SIZE); 

                dim_data->RPD_checksum = crc;

                printf("======================"); 
                printf(" Read Dim CRC =  0x%x ", dim_data->RPD_checksum);
                printf("===================\n"); 
              
                break;
        
             case I2C_WRITE: 
                   
                crc = RPD_calc_crc16(crc, buf, dataSize - CCITT_CRC16_SIZE);
                dim_data->RPD_checksum = crc;  
                
                printf("======================"); 
                printf(" Write Dim crc =  0x%x ", crc);
                printf("======================\n");   
               
                // write to PDMA
                printf("\n ============ write to PDMA section==========\n");   
                RPD_offset = 0; 
                //rc = RPD_cat34t_access(devIndex, I2C_base_addr, slave_dev, RPD_offset,    
                //                   buf, dataSize, access_type, eeprom_page, mutex_id); 

                rc = RPD_cat34t_access(devIndex, slave_dev, RPD_offset,    
                                   buf, dataSize, access_type, eeprom_page); 

                printf("======================"); 
                printf(" datasize =  0x%x ", dataSize);
                printf("======================\n");   

                for (size=0; size<dataSize; size++)
                {
                    dim_data[size].RPD_age = buf[size]; 
                    printf(" datasize =  0x%x dim_data[size].RPD_age = 0x%x , \n", size, dim_data[size].RPD_age);
                }
                
                // write last 2 bytes of CRC 
                /*printf("\n ============ write CRC section ==========\n");   
                RPD_offset += (dataSize - CCITT_CRC16_SIZE);
                
               // rc = RPD_cat34t_access(devIndex, I2C_base_addr, slave_dev, RPD_offset,    
               //                    &buf[dataSize-CCITT_CRC16_SIZE], CCITT_CRC16_SIZE, access_type, eeprom_page);
                 
                rc = RPD_cat34t_access(devIndex, slave_dev, RPD_offset,    
                &buf[dataSize-CCITT_CRC16_SIZE], CCITT_CRC16_SIZE, access_type, eeprom_page, mutex_id); 

                for (size=0; size<CCITT_CRC16_SIZE; size++)  
                {
                    dim_data[size].RPD_checksum = buf[size]; 
                    printf(" Write Dim crc =  0x%x ", dim_data[size].RPD_checksum);

                }
               
                if(rc != DIM_DRV_OK)
                {
                    DIM_DRV_DEBUG(dimDrv_debugMde, "%s, RPD_base_addr=0x%x, slave_addr=0x%x, offset=%d, num_of_bytes=0x%x, access_type=%d, eeprom_page=%d, rc=%d\n", 
                                __FUNCTION__, RPD_I2C_BASE_ADDRESS, CAT34T_SET_SPD_PAGE1, RPD_offset, dataSize, access_type, eeprom_page, rc);
                    return rc;
                } */
                
                break;
        
                default:
                   DIM_DRV_DEBUG(dimDrv_debugMde,"%s, operation type =0x%x\n",__FUNCTION__, access_type);
                   return DIM_DRV_ERR;
        }
    }
    return rc;
}


