/*==================================================================*/
/*                                                                  */
/*  dimDrv.h                                                        */
/*  ========================                                        */
/*                                                                  */
/*  Private definitions for the DIM driver                          */
/*                                                                  */
/*==================================================================*/

#ifndef __DIMDRV_H__
#define __DIMDRV_H__

#include <os/time_beacon.h>
#include "dimDrv_public.h"
#include "dimefw_dashboard.h"
#include "dimefw_swabs.h"


#define DIM_DRV_DEBUG(dimDrvDebugMode, ...)  {if(dimDrvDebugMode) {printf(__VA_ARGS__);} dimDebugLog(__VA_ARGS__);}
#define DIM_DRV_ERROR(dimDrvErrorMode, ...)  {if(dimDrvErrorMode) {printf(__VA_ARGS__);} dimErrorLog(__VA_ARGS__);}

#define DIM_DRV_CHECK_PTR(val) {if (val == NULL) {return DIM_DRV_NULL_POINTER;}}

#define DIM_FPGA_ADDR                         0xC00000
#define DIM_I2C_MUTEX_ID                      2
#define DIM_I2C_MUTEX_CTRL_REQ_MASK           0x00008000
#define DIM_I2C_MUTEX_STAT_GRANT_CNT_SAT_MASK 0x80000000
#define DIM_I2C_MUTEX_STAT_GRANT_ID_MASK      0x0000000F
#define DIM_I2C_MUTEX_STAT_GRANT_MASK         0x00008000
#define DIM_I2C_MUTEX_CTRL_OFFSET             0x140       // TOP_MUTEX_CTRL_2   0x00140
#define DIM_I2C_MUTEX_STAT_OFFSET             0x144
#define I2C_MUTEX_ACCESS_RETRY                100

dimDrv_rc_t dim_RPD_write (uint8 port, uint32 offset, uint32 *data, uint32 write_buf_size);
dimDrv_rc_t dim_RPD_read (uint8 port, uint32 offset, uint32 *data, uint32 buf_size);
dimDrv_rc_t dim_i2c_mutex_request(uint8 port, uint8 mutex_id);
dimDrv_rc_t dim_i2c_mutex_release(uint8 port, uint8 mutex_id);
dimDrv_rc_t dim_i2c_mutex_lock(uint8 port);
dimDrv_rc_t dim_i2c_mutex_unlock(uint8 port);
void dimDrv_dimAgeDiag();

#endif  

