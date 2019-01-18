/*******************************************************************************
 *
 * File        : dim_i2cDrv.cpp
 *
 * Created Date: Aug 14, 2018
 *
 * Description : This file provides I2C driver to access DIM IDPROM
 * 
 ******************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <os/task.h>
#include "dimDrv_dashboard.h" 
#include "dimDrv_access.h"
#include "dimDrv_public.h"
#include "dimDrv_main.h"

boolean  dimDrv_debugMode = FALSE;
extern struct timespec msleep_100ms;

/********************************************************************************
 *  
 *   registerDimMutex: calculate DIM mutex address
 *                       
 *   Parameters: offset
 *               STAT_offset  is one for MUTEX_STAT and 0 for MUTEX_CTRL*  
 *   output  DIM_mutex_offset    
 *  
 *******************************************************************************/
void registerDimMutex(uint32 offset, uint8 STAT_offset, uint32 *DIM_mutex_offset)  
{
   *DIM_mutex_offset =  DIM_MUTEX_BASE_ADDR + DIM_MUTEX_ID_OFFSET * (offset-1) + STAT_offset * (DIM_MUTEX_ID_OFFSET/2);
}

/********************************************************************************
 *   dim_RPD_write
 *  
 *   Description:    DIM PDMA write  
 *                       
 *   Parameters:     devIndex
 *                   offset              -  DIM EEPROM register offset 
 *                   data                -  a pointer of caller's memory
 *                   size                -  the number of words to be transfered
 *  
 *   Return Code:    dimDrv_rc_t
 *  
 *******************************************************************************/
dimDrv_rc_t dim_RPD_write (uint8 devIndex, uint32 offset, uint32 *data, uint32 wsize)
{
    dimDrv_rc_t   dim_rc = DIM_DRV_OK;
    uint8  i; 
    uint32 num_transaction = 0;
    uint32 write_size = 0;

    if (data == NULL)
    {
        DIM_DRV_DEBUG(dimDrv_debugMode,"%s Port[%d] offset=0x%08x data is NULL pointer\n", __FUNCTION__, devIndex, offset);
        return DIM_DRV_NULL_POINTER;
    }

    num_transaction = wsize/DIM_I2C_PDMA_ACCESS_MAX_WORD + ((wsize%DIM_I2C_PDMA_ACCESS_MAX_WORD)?1:0);  

    for (int j=0; j<num_transaction; j++)
    {
       if (wsize < DIM_I2C_PDMA_ACCESS_MAX_WORD)
       {
           write_size = wsize;
       } else {
           write_size = DIM_I2C_PDMA_ACCESS_MAX_WORD;
       }

       /*dim_rc = dimWriteData(devIndex,
                             DIM_FPGA_BASE_ADDR+offset + DIM_I2C_PDMA_ACCESS_MAX_BYTE*j, 
                             &data[DIM_I2C_PDMA_ACCESS_MAX_WORD*j], 
                             write_size);*/
       dim_rc = dimWriteData(devIndex,
                             offset + DIM_I2C_PDMA_ACCESS_MAX_BYTE*j, 
                             &data[DIM_I2C_PDMA_ACCESS_MAX_WORD*j], 
                             write_size);
       if (dim_rc != DIM_DRV_OK)
       {
           DIM_DRV_DEBUG(dimDrv_debugMode, "\t %s : DIM subPort[%d] PDMA write failed. reg=0x%08x write-size=%d dim_rc=%d \n",
                                   __FUNCTION__, devIndex, offset + DIM_I2C_PDMA_ACCESS_MAX_BYTE*j, write_size, dim_rc);  // -----------
           //printf( "DIM subPort[%d] PDMA write failed. reg=0x%x write-size=%d dim_rc=%d \n",
           //                         devIndex, DIM_FPGA_BASE_ADDR+offset+DIM_I2C_PDMA_ACCESS_MAX_BYTE*j, write_size, dim_rc); 
           printf( "DIM subPort[%d] PDMA write failed. reg=0x%x write-size=%d dim_rc=%d \n",
                                    devIndex, offset+DIM_I2C_PDMA_ACCESS_MAX_BYTE*j, write_size, dim_rc); 
           break;
       }

       if (wsize > DIM_I2C_PDMA_ACCESS_MAX_WORD)
       {
             wsize -=  DIM_I2C_PDMA_ACCESS_MAX_WORD;
       }

       for (i=0; i<wsize; i++)
       {
           printf( "DIM subPort[%d] reg=0x%x write-size=%d dimWrite data = 0x%x dim_rc=%d \n",
                                    devIndex, offset+DIM_I2C_PDMA_ACCESS_MAX_BYTE*j, wsize, *(data+i), dim_rc);   // ----------------------
       }

    }


    // -----------------------------------------------------------------------------
    /*dim_rc = dimWriteData(devIndex, offset, data, wsize);
    if (dim_rc != DIM_DRV_OK)
    {
       DIM_DRV_DEBUG(dimDrv_debugMode, "\t %s : DIM subPort[%d] PDMA write failed. reg=0x%08x write-size=%d dim_rc=%d \n",
                               __FUNCTION__, devIndex, offset, wsize, dim_rc); 
       printf( "DIM subPort[%d] PDMA write failed. reg=0x%x write-size=%d dim_rc=%d \n",
                                devIndex, offset, wsize, dim_rc); 
    }
    
    for (i=0; i<wsize; i++)
    {
       printf( "DIM subPort[%d] reg=0x%x write-size=%d dimWrite data = 0x%x dim_rc=%d \n",
                                devIndex, offset, wsize, *(data+i), dim_rc); 
    }*/
    // ---------------------------------------------------------------------------

    return dim_rc;
}

/********************************************************************************
 *   dim_RPD_read
 *  
 *   Description:    DIM PDMA read  
 *                       
 *   Parameters:     devIndex
 *                   offset              -  DIM EEPROM register offset 
 *                   data                -  a pointer of caller's memory
 *                   size                -  the number of words to be transfered
 *  
 *   Return Code:    dimDrv_rc_t
 *  
 *******************************************************************************/
dimDrv_rc_t dim_RPD_read (uint8 devIndex, uint32 offset, uint32 *data, uint32 wsize)
{
    dimDrv_rc_t   dim_rc = DIM_DRV_OK;
    uint8  i;   
    uint32 num_transaction = 0;
    uint32 read_size = 0;

    if (data == NULL)
    {
        DIM_DRV_DEBUG(dimDrv_debugMode,"%s Port[%d] offset=0x%08x data is NULL pointer\n", __FUNCTION__, devIndex, offset);
        return DIM_DRV_NULL_POINTER;
    }

    num_transaction = wsize/DIM_I2C_PDMA_ACCESS_MAX_WORD + ((wsize%DIM_I2C_PDMA_ACCESS_MAX_WORD)?1:0);  

    for (int j=0; j<num_transaction; j++)
    {
       if (wsize < DIM_I2C_PDMA_ACCESS_MAX_WORD)
       {
           read_size = wsize;
       } else {
           read_size = DIM_I2C_PDMA_ACCESS_MAX_WORD;
       }
       dim_rc = dimReadData(devIndex,
                            offset + DIM_I2C_PDMA_ACCESS_MAX_BYTE*j, 
                            &data[DIM_I2C_PDMA_ACCESS_MAX_WORD*j], 
                            read_size);
       if (dim_rc != DIM_DRV_OK)
       {
           DIM_DRV_DEBUG(dimDrv_debugMode, "\t %s : DIM subPort[%d] PDMA read failed. reg=0x%08x size=%d dim_rc=%d\n",
                                   __FUNCTION__, devIndex, offset + DIM_I2C_PDMA_ACCESS_MAX_BYTE*j, wsize, dim_rc); 
           printf( " DIM subPort[%d] PDMA read failed. reg=0x%x size=%d dim_rc=%d\n",
                                    devIndex, offset + DIM_I2C_PDMA_ACCESS_MAX_BYTE*j, wsize, dim_rc); 
           break;
       }

       if (wsize > DIM_I2C_PDMA_ACCESS_MAX_WORD)
       {
             wsize -=  DIM_I2C_PDMA_ACCESS_MAX_WORD;
       }

       for (i=0; i<wsize; i++)
       {
           printf( "DIM subPort[%d] reg=0x%x read-size=%d dimRead data = 0x%x dim_rc=%d \n",
                                    devIndex, offset+DIM_I2C_PDMA_ACCESS_MAX_BYTE*j, wsize, *(data+i), dim_rc); 
       }

    }

    // ------------------------------------------------------------------------
    /*dim_rc = dimReadData(devIndex, offset, data, wsize);
    if (dim_rc != DIM_DRV_OK)
    {
       DIM_DRV_DEBUG(dimDrv_debugMode, "\t %s : DIM subPort[%d] PDMA read failed. reg=0x%08x size=%d dim_rc=%d\n",
                               __FUNCTION__, devIndex, offset, wsize, dim_rc); 
       printf( " DIM subPort[%d] PDMA read failed. reg=0x%x size=%d dim_rc=%d\n",
                                devIndex, offset, wsize, dim_rc); 
    }

    for (i=0; i<wsize; i++)
    {
       printf( "DIM subPort[%d] reg=0x%x read-size=%d dimRead data = 0x%x dim_rc=%d \n",
                                devIndex, offset, wsize, *(data+i), dim_rc); 
    }*/
    //------------------------------------------------------------------------

    return dim_rc;
}

/*********************************************************************************/
/*                                  Request a mutex                              */ 
/*********************************************************************************/
dimDrv_rc_t dim_i2c_mutex_request(uint8 devIndex, mutex_id_e mutex_id)
{
   dimDrv_rc_t   dim_rc = DIM_DRV_OK;
   uint32        data = 0;
   uint32        Mutex_CTRL_offset;

   data = DIM_MUTEX_CTRL_REQ_MASK | mutex_id;    // mutex_id = 2    
   //Mutex_CTRL_offset =  DIM_MUTEX_BASE_ADDR + DIM_MUTEX_ID_OFFSET*(mutex_id-1);
   //Mutex_CTRL_offset = registerDimMutex(mutex_id, 0);
   registerDimMutex(mutex_id, 0, &Mutex_CTRL_offset);  
   dim_rc = dim_RPD_write(devIndex, Mutex_CTRL_offset, &data, 1);   

    if (dim_rc != DIM_DRV_OK)
    {
       DIM_DRV_DEBUG(dimDrv_debugMode, "\t %s : Write I2C_MUTEX_CTRL_OFFSET 0x%x failed. write_data=0x%x dim_rc=%d\n",
                               __FUNCTION__, devIndex, Mutex_CTRL_offset, data, dim_rc );  
       printf( " Write I2C_MUTEX_CTRL_OFFSET 0x%x failed. write_data=0x%x dim_rc=%d\n",
                                devIndex, Mutex_CTRL_offset, data, dim_rc );  
    }
    else
    {
       DIM_DRV_DEBUG(dimDrv_debugMode, "\t %s : Write I2C_MUTEX_CTRL_OFFSET 0x%x success. write_data=0x%x dim_rc=%d\n",
                               __FUNCTION__, devIndex, Mutex_CTRL_offset, data, dim_rc );  
       printf( " Write I2C_MUTEX_CTRL_OFFSET 0x%x success. write_data=0x%x dim_rc=%d\n",
                                devIndex, Mutex_CTRL_offset, data, dim_rc );  
    }
                                          
   return dim_rc;
}

/*********************************************************************************/
/*                                  Release a mutex                              */ 
/*********************************************************************************/
dimDrv_rc_t dim_i2c_mutex_release(uint8 devIndex, mutex_id_e mutex_id)
{
   dimDrv_rc_t   dim_rc = DIM_DRV_OK;
   uint32        data = 0;
   uint32        Mutex_CTRL_offset;

   data = !DIM_MUTEX_CTRL_REQ_MASK | mutex_id;
   //Mutex_CTRL_offset = DIM_MUTEX_BASE_ADDR + DIM_MUTEX_ID_OFFSET*(mutex_id-1);
   //Mutex_CTRL_offset = registerDimMutex(mutex_id, 0);
   registerDimMutex(mutex_id, 0, &Mutex_CTRL_offset);
   dim_rc = dim_RPD_write(devIndex, Mutex_CTRL_offset, &data, 1);  

   if (dim_rc != DIM_DRV_OK)
   {
      DIM_DRV_DEBUG(dimDrv_debugMode, "\t %s : Write I2C_MUTEX_CTRL_OFFSET 0x%x failed. write_data=0x%x dim_rc=%d\n",
                              __FUNCTION__, devIndex, Mutex_CTRL_offset, data, dim_rc ); 
      printf( " Write I2C_MUTEX_CTRL_OFFSET 0x%x failed. write_data=0x%x dim_rc=%d\n",
                               devIndex, Mutex_CTRL_offset, data, dim_rc );      
   }
   else
   {
      DIM_DRV_DEBUG(dimDrv_debugMode, "\t %s : Write I2C_MUTEX_CTRL_OFFSET 0x%x success. write_data=0x%x dim_rc=%d\n",
                              __FUNCTION__, devIndex, Mutex_CTRL_offset, data, dim_rc );   
      printf(" Write I2C_MUTEX_CTRL_OFFSET 0x%x success. write_data=0x%x dim_rc=%d\n",
                               devIndex, Mutex_CTRL_offset, data, dim_rc );   
   }
                                          
   return dim_rc;
   
}

/*********************************************************************************
*                                 DIM_I2C_mutex_lock
*  
* Description:
*  
*      1. Pre-lock checking -- needs to set timeout=10 seconds
*         If saturated, just release it by its grand id
*         If not saturated,
*            1)It is not granted, lock the mutex
*            2)It is granted, grand_id=2(flash grand_id), release it
*                             grand_id=1(DMC), wait and check it again.
*      2. Lock the mutex
*      3. Post-lock checking --verifiy if grand_id=2 is granted
*  
* Parameters:  devIndex 
* Return: dimDrv_rc_t
*  
*********************************************************************************/
dimDrv_rc_t dim_i2c_mutex_lock(uint8 devIndex, mutex_id_e mutexId)
{
   dimDrv_rc_t   dim_rc = DIM_DRV_OK;
   uint32        data =  mutexId; //  DIM_I2C_MUTEX_ID;
   uint32        Mutex_STAT_offset;
   uint32        count = 0;
  
   /* Pre-lock checking */ 
   do
   {
      /* Read I2C MUTEX STAT register offset 0x144 */ 
      //Mutex_STAT_offset = DIM_MUTEX_BASE_ADDR + DIM_MUTEX_ID_OFFSET*(mutexId-1)+(DIM_MUTEX_ID_OFFSET/2);
      //Mutex_STAT_offset = registerDimMutex(mutexId, 1);
      registerDimMutex(mutexId, 1, &Mutex_STAT_offset);
      if ((dim_rc = dim_RPD_read (devIndex, Mutex_STAT_offset, &data, 1)) != DIM_DRV_OK) 
      {
          DIM_DRV_DEBUG(dimDrv_debugMode, "%s Port[%d] Read0 DIM_I2C_MUTEX_STAT_OFFSET (0x%x) failed %d times rc=%d\n", 
                                         __FUNCTION__, devIndex, Mutex_STAT_offset, count, dim_rc);  
          printf(" Port[%d] Read0 DIM_I2C_MUTEX_STAT_OFFSET (0x%x) failed %d times rc=%d\n", 
                                          devIndex, Mutex_STAT_offset, count, dim_rc);   
      }

       /* If the mutex grant count is saturated, release the mutex who granded */
      if ((data &  DIM_MUTEX_STAT_GRANT_CNT_SAT_MASK) == DIM_MUTEX_STAT_GRANT_CNT_SAT_MASK)
      {
          DIM_DRV_DEBUG(dimDrv_debugMode, "%s Port[%d] I2C_MUTEX is saturated [offset|value]=[0x%x|0x%x]\n", 
                                          __FUNCTION__, devIndex, Mutex_STAT_offset, data);   
          printf(" Port[%d] I2C_MUTEX is saturated [offset|value]=[0x%x|0x%x]\n", 
                                           devIndex, Mutex_STAT_offset, data);   
  
          data = data & DIM_MUTEX_STAT_GRANT_ID_MASK;
          dim_rc = dim_i2c_mutex_release(devIndex, data);
          if (dim_rc == DIM_DRV_OK)
          { 
              DIM_DRV_DEBUG(dimDrv_debugMode, "%s Port[%d] I2C_MUTEX is released due to saturation\n", 
                                              __FUNCTION__, devIndex);
              printf(" Port[%d] I2C_MUTEX is released due to saturation\n", 
                                               devIndex);
              break;
          }
      }
      else // grant_cnt is not saturated.
      {
          // If not saturated,  mutex is granted(not available) 
          if ((data & DIM_MUTEX_STAT_GRANT_MASK) == DIM_MUTEX_STAT_GRANT_MASK)
          {
              // If grand_id == 2 (I2C grand_id), release it 
              if ((data & DIM_MUTEX_STAT_GRANT_ID_MASK) == mutexId)
              {                 
                 dim_rc = dim_i2c_mutex_release(devIndex, mutexId);
                 if (dim_rc == DIM_DRV_OK)
                 { 
                     DIM_DRV_DEBUG(dimDrv_debugMode, "%s Port[%d] I2C_MUTEX is hold by myself(grand_id == 2). Release it.\n", 
                                                      __FUNCTION__, devIndex);
                     printf(" Port[%d] I2C_MUTEX is hold by myself(grand_id == 2). Release it.\n", 
                                                       devIndex);
                     break;
                 }
              }
              else  // if grand_id=1(DMC), wait and check it again 
              {
                    dim_rc = DIM_DRV_I2C_MUTEX_ERROR;
                    DIM_DRV_DEBUG(dimDrv_debugMode, "%s Port[%d] DIM_I2C_MUTEX is not available. [offset|value]=[0x%x|0x%x] %d times rc=%d\n", 
                                                   __FUNCTION__, devIndex, Mutex_STAT_offset, data, count, dim_rc);  
                    printf(" Port[%d] DIM_I2C_MUTEX is not available. [offset|value]=[0x%x|0x%x] %d times rc=%d\n", 
                                                    devIndex, Mutex_STAT_offset, data, count, dim_rc);   
              } 
                                                        
          }
          else // grant_cnt is not saturated and mutex is available 
          {
              dim_rc = DIM_DRV_OK;
              DIM_DRV_DEBUG(dimDrv_debugMode, "%s Port[%d] GRANT_CNT is not saturated and mutex is available. [offset|value]=[0x%x|0x%x] %d times\n", 
                                             __FUNCTION__, devIndex, Mutex_STAT_offset, data, count);   
              printf(" Port[%d] GRANT_CNT is not saturated and mutex is available. [offset|value]=[0x%x|0x%x] %d times\n", 
                                              devIndex, Mutex_STAT_offset, data, count);     
              break;
          }
      }

      task_sleep(&msleep_100ms);
      count ++;


   }while(count <= I2C_MUTEX_ACCESS_RETRY);

   if (dim_rc == DIM_DRV_I2C_MUTEX_ERROR) 
   {
       DIM_DRV_DEBUG(dimDrv_debugMode, "%s Port[%d] DIM_I2C_MUTEX is not available. [offset|value]=[0x%x|0x%x] %d times rc=%d\n", 
                                                   __FUNCTION__, devIndex, Mutex_STAT_offset, data, count, dim_rc);   
       printf( " Port[%d] DIM_I2C_MUTEX is not available. [offset|value]=[0x%x|0x%x] %d times rc=%d\n", 
                                                    devIndex, Mutex_STAT_offset, data, count, dim_rc);   
       goto out;
   }

   // 2. Request mutex 
   dim_rc = dim_i2c_mutex_request(devIndex, mutexId);

   if (dim_rc != DIM_DRV_OK) 
   {
       DIM_DRV_DEBUG(dimDrv_debugMode, "%s Port[%d] Cannot get DIM_I2C_MUTEX at all !!(HW issue?) rc=%d\n",  
                                                   __FUNCTION__, devIndex, dim_rc);
       printf(" Port[%d] Cannot get DIM_I2C_MUTEX at all !!(HW issue?) rc=%d\n",  
                                                    devIndex, dim_rc);
       goto out;
   }

   // 3. Verify if the mutex is granted with my grand_id 
   dim_rc = dim_RPD_read (devIndex, Mutex_STAT_offset, &data, 1);

   if (dim_rc != DIM_DRV_OK) 
   {
       DIM_DRV_DEBUG(dimDrv_debugMode, "%s Port[%d] Read1 I2C_MUTEX_STAT_OFFSET(0x%x) failed. rc=%d\n",  
                                                   __FUNCTION__, devIndex, Mutex_STAT_offset, dim_rc);  
       printf(" Port[%d] Read1 I2C_MUTEX_STAT_OFFSET(0x%x) failed. rc=%d\n",  
                                                    devIndex, Mutex_STAT_offset, dim_rc);   
       goto out;
   }

    if (((data & DIM_MUTEX_STAT_GRANT_MASK) == DIM_MUTEX_STAT_GRANT_MASK) && 
        ((data & DIM_MUTEX_STAT_GRANT_ID_MASK) == mutexId))
    {
        DIM_DRV_DEBUG(dimDrv_debugMode, "%s Port[%d] DIM_I2C_MUTEX is granted successfully [offset|value]=[0x%x|0x%x]\n",  
                                                   __FUNCTION__, devIndex, Mutex_STAT_offset, data);   
        printf( " Port[%d] DIM_I2C_MUTEX is granted successfully [offset|value]=[0x%x|0x%x]\n",  
                                                    devIndex, Mutex_STAT_offset, data);   
    }
    else
    {
       dim_rc = DIM_DRV_I2C_MUTEX_ERROR;
       DIM_DRV_DEBUG(dimDrv_debugMode, "%s Port[%d] DIM_I2C_MUTEX req_id is invalid. [offset|value]=[0x%x|0x%x] rc=%d\n", 
                                                   __FUNCTION__, devIndex, Mutex_STAT_offset, data, dim_rc);  
       printf( "Port[%d] DIM_I2C_MUTEX req_id is invalid. [offset|value]=[0x%x|0x%x] rc=%d\n", 
                                                    devIndex, Mutex_STAT_offset, data, dim_rc);   
    }

out:
   return dim_rc;
}

/*********************************************************************************
*                                 DIM_I2C_mutex_unlock
*  
* Parameters:  devIndex 
* Return: dimDrv_rc_t
*  
*********************************************************************************/
dimDrv_rc_t dim_i2c_mutex_unlock(uint8 devIndex, mutex_id_e mutexId)
{
   dimDrv_rc_t   dim_rc = DIM_DRV_OK;
   uint32        data = mutexId;   //  DIM_I2C_MUTEX_ID;
   uint32        Mutex_STAT_offset;
   uint32        Mutex_CTRL_offset;
   
   //Mutex_STAT_offset = DIM_MUTEX_BASE_ADDR + DIM_MUTEX_ID_OFFSET*(mutexId-1)+(DIM_MUTEX_ID_OFFSET/2);
   //Mutex_STAT_offset = registerDimMutex(mutexId, 1);
   registerDimMutex(mutexId, 1, &Mutex_STAT_offset);
   dim_rc = dim_RPD_read (devIndex, Mutex_STAT_offset, &data, 1);   
   if (dim_rc != DIM_DRV_OK) 
   {
       DIM_DRV_DEBUG(dimDrv_debugMode, "%s Port[%d] Read0 I2C_MUTEX_STAT_OFFSET(0x%x) failed. rc=%d\n",  
                                                   __FUNCTION__, devIndex, Mutex_STAT_offset, dim_rc);  
       printf( " Port[%d] Read0 I2C_MUTEX_STAT_OFFSET(0x%x) failed. rc=%d\n",  
                                                    devIndex, Mutex_STAT_offset, dim_rc);   
       goto out;
   }

   // Check if mutex is granted 
   if (((data & DIM_MUTEX_STAT_GRANT_MASK) == DIM_MUTEX_STAT_GRANT_MASK) && 
      ((data & DIM_MUTEX_STAT_GRANT_ID_MASK) == mutexId))
   {
      DIM_DRV_DEBUG(dimDrv_debugMode, "%s Port[%d] DIM_I2C_MUTEX_STAT [offset|value]=[0x%x|0x%x]\n", 
                   __FUNCTION__, devIndex, Mutex_STAT_offset, data);   
      printf(" Port[%d] DIM_I2C_MUTEX_STAT [offset|value]=[0x%x|0x%x]\n", 
                    devIndex, Mutex_STAT_offset, data);   
   }
   else
   {
       dim_rc = DIM_DRV_I2C_MUTEX_ERROR;
       DIM_DRV_DEBUG(dimDrv_debugMode, "%s Port[%d] DIM_I2C_MUTEX req_id invalid. [offset|value]=[0x%x|0x%x] rc=%d\n",  
                                                   __FUNCTION__, devIndex, Mutex_STAT_offset, data, dim_rc);  
       printf( " Port[%d] DIM_I2C_MUTEX req_id invalid. [offset|value]=[0x%x|0x%x] rc=%d\n",  
                                                    devIndex, Mutex_STAT_offset, data, dim_rc);  
       goto out;
   }

   // Release mutex 
   //Mutex_CTRL_offset = DIM_MUTEX_BASE_ADDR + DIM_MUTEX_ID_OFFSET*(mutexId-1);
   registerDimMutex(mutexId, 0, &Mutex_CTRL_offset);
   dim_rc = dim_i2c_mutex_release(devIndex, mutexId);
   
   if (dim_rc != DIM_DRV_OK)
   {
    
       DIM_DRV_DEBUG(dimDrv_debugMode, "%s Port[%d] Write_I2C_MUTEX_CTRL_OFFSET 0x%x failed. write_data=0x%x rc=%d\n",  
                                        __FUNCTION__, devIndex, Mutex_CTRL_offset, data, dim_rc);  
       printf(" Port[%d] Write_I2C_MUTEX_CTRL_OFFSET 0x%x failed. write_data=0x%x rc=%d\n",  
                                         devIndex, Mutex_CTRL_offset, data, dim_rc);   
       
   }
   else
   {
       DIM_DRV_DEBUG(dimDrv_debugMode, "%s Port[%d] DIM_FLASH_SPI_MUTEX is released successfully rc=%d\n", 
                    __FUNCTION__, devIndex, dim_rc);
       printf( " Port[%d] DIM_FLASH_SPI_MUTEX is released successfully rc=%d\n", 
                     devIndex, dim_rc);
   }

out:
   return dim_rc;
}




































