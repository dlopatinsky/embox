
#ifndef __1888BC048_MILSTD_H
#define __1888BC048_MILSTD_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "1888bc048.h"

/** @addtogroup __1888BC048_StdPeriph_Driver
  * @{
  */
  
/******************************************************************************/
/*                         MILSTD registers                                   */
/******************************************************************************/

/*--  IR: BSI interrupt register ------------------------------------------------------------------------------*/
typedef struct {
  uint32_t BCEV                   :1;                                /*!< BC transfer event interrupt */
  uint32_t BCD                    :1;                                /*!< BC DMA error */
  uint32_t BCWK                   :1;                                /*!< BC timer interrupt */
  uint32_t                        :5;                                /*!< RESERVED */
  uint32_t RTEV                   :1;                                /*!< RT transfer event interrupt */
  uint32_t RTD                    :1;                                /*!< RT DMA error */
  uint32_t RTTE                   :1;                                /*!< RT table access error */
  uint32_t                        :5;                                /*!< RESERVED */
  uint32_t BMD                    :1;                                /*!< BM DMA error */
  uint32_t BMTOF                  :1;                                /*!< BM timer overflow */
} _MILSTD_IR_bits;

/* Bit field positions: */
#define MILSTD_IR_BCEV_Pos                    0                      /*!< BC transfer event interrupt */
#define MILSTD_IR_BCD_Pos                     1                      /*!< BC DMA error */
#define MILSTD_IR_BCWK_Pos                    2                      /*!< BC timer interrupt */
#define MILSTD_IR_RTEV_Pos                    8                      /*!< RT transfer event interrupt */
#define MILSTD_IR_RTD_Pos                     9                      /*!< RT DMA error */
#define MILSTD_IR_RTTE_Pos                    10                     /*!< RT table access error */
#define MILSTD_IR_BMD_Pos                     16                     /*!< BM DMA error */
#define MILSTD_IR_BMTOF_Pos                   17                     /*!< BM timer overflow */

/* Bit field masks: */
#define MILSTD_IR_BCEV_Msk                    0x00000001UL           /*!< BC transfer event interrupt */
#define MILSTD_IR_BCD_Msk                     0x00000002UL           /*!< BC DMA error */
#define MILSTD_IR_BCWK_Msk                    0x00000004UL           /*!< BC timer interrupt */
#define MILSTD_IR_RTEV_Msk                    0x00000100UL           /*!< RT transfer event interrupt */
#define MILSTD_IR_RTD_Msk                     0x00000200UL           /*!< RT DMA error */
#define MILSTD_IR_RTTE_Msk                    0x00000400UL           /*!< RT table access error */
#define MILSTD_IR_BMD_Msk                     0x00010000UL           /*!< BM DMA error */
#define MILSTD_IR_BMTOF_Msk                   0x00020000UL           /*!< BM timer overflow */

/*--  IENR: BSI interrupt enable register ---------------------------------------------------------------------*/
typedef struct {
  uint32_t BCEVE                  :1;                                /*!< BCEVE interrupt enable */
  uint32_t BCDE                   :1;                                /*!< BCD interrupt enable */
  uint32_t BCWKE                  :1;                                /*!< BCWK interrupt enable */
  uint32_t                        :5;                                /*!< RESERVED */
  uint32_t RTEVE                  :1;                                /*!< RTEV interrupt enable */
  uint32_t RTDE                   :1;                                /*!< RTD interrupt enable */
  uint32_t RTTEE                  :1;                                /*!< RTTE interrupt enable */
  uint32_t                        :5;                                /*!< RESERVED */
  uint32_t BMDE                   :1;                                /*!< BMD interrupt enable */
  uint32_t BMTOE                  :1;                                /*!< BMTOF interrupt enable */
} _MILSTD_IENR_bits;

/* Bit field positions: */
#define MILSTD_IENR_BCEVE_Pos                 0                      /*!< BCEVE interrupt enable */
#define MILSTD_IENR_BCDE_Pos                  1                      /*!< BCD interrupt enable */
#define MILSTD_IENR_BCWKE_Pos                 2                      /*!< BCWK interrupt enable */
#define MILSTD_IENR_RTEVE_Pos                 8                      /*!< RTEV interrupt enable */
#define MILSTD_IENR_RTDE_Pos                  9                      /*!< RTD interrupt enable */
#define MILSTD_IENR_RTTEE_Pos                 10                     /*!< RTTE interrupt enable */
#define MILSTD_IENR_BMDE_Pos                  16                     /*!< BMD interrupt enable */
#define MILSTD_IENR_BMTOE_Pos                 17                     /*!< BMTOF interrupt enable */

/* Bit field masks: */
#define MILSTD_IENR_BCEVE_Msk                 0x00000001UL           /*!< BCEVE interrupt enable */
#define MILSTD_IENR_BCDE_Msk                  0x00000002UL           /*!< BCD interrupt enable */
#define MILSTD_IENR_BCWKE_Msk                 0x00000004UL           /*!< BCWK interrupt enable */
#define MILSTD_IENR_RTEVE_Msk                 0x00000100UL           /*!< RTEV interrupt enable */
#define MILSTD_IENR_RTDE_Msk                  0x00000200UL           /*!< RTD interrupt enable */
#define MILSTD_IENR_RTTEE_Msk                 0x00000400UL           /*!< RTTE interrupt enable */
#define MILSTD_IENR_BMDE_Msk                  0x00010000UL           /*!< BMD interrupt enable */
#define MILSTD_IENR_BMTOE_Msk                 0x00020000UL           /*!< BMTOF interrupt enable */

/*--  HCON: BSI hardware configuration register ---------------------------------------------------------------*/
typedef struct {
  uint32_t CCFREQ                 :8;                                /*!< Codec clock frequency */
  uint32_t SCLK                   :1;                                /*!< Same clock */
  uint32_t ENDIAN                 :2;                                /*!< AHB Endianness */
  uint32_t XKEYS                  :1;                                /*!< Enable safety keys */
  uint32_t                        :19;                               /*!< RESERVED */
  uint32_t MOD                    :1;                                /*!< Modified */
} _MILSTD_HCON_bits;

/* Bit field positions: */
#define MILSTD_HCON_CCFREQ_Pos                0                      /*!< Codec clock frequency */
#define MILSTD_HCON_SCLK_Pos                  8                      /*!< Same clock */
#define MILSTD_HCON_ENDIAN_Pos                9                      /*!< AHB Endianness */
#define MILSTD_HCON_XKEYS_Pos                 11                     /*!< Enable safety keys */
#define MILSTD_HCON_MOD_Pos                   31                     /*!< Modified */

/* Bit field masks: */
#define MILSTD_HCON_CCFREQ_Msk                0x000000FFUL           /*!< Codec clock frequency */
#define MILSTD_HCON_SCLK_Msk                  0x00000100UL           /*!< Same clock */
#define MILSTD_HCON_ENDIAN_Msk                0x00000600UL           /*!< AHB Endianness */
#define MILSTD_HCON_XKEYS_Msk                 0x00000800UL           /*!< Enable safety keys */
#define MILSTD_HCON_MOD_Msk                   0x80000000UL           /*!< Modified */

/*--  BCSTCON: BC status and config register ------------------------------------------------------------------*/
typedef struct {
  uint32_t SCST                   :3;                                /*!< Schedule state */
  uint32_t SCADL                  :5;                                /*!< Schedule address low bits */
  uint32_t ASST                   :2;                                /*!< Asynchronous list state */
  uint32_t                        :1;                                /*!< RESERVED */
  uint32_t ASADL                  :5;                                /*!< Asynchronous list address low bits */
  uint32_t BCCHK                  :1;                                /*!< Check broadcast */
  uint32_t                        :11;                               /*!< RESERVED */
  uint32_t BCFEAT                 :3;                                /*!< BC features */
  uint32_t BCSUP                  :1;                                /*!< BC supported */
} _MILSTD_BCSTCON_bits;

/* Bit field positions: */
#define MILSTD_BCSTCON_SCST_Pos               0                      /*!< Schedule state */
#define MILSTD_BCSTCON_SCADL_Pos              3                      /*!< Schedule address low bits */
#define MILSTD_BCSTCON_ASST_Pos               8                      /*!< Asynchronous list state */
#define MILSTD_BCSTCON_ASADL_Pos              11                     /*!< Asynchronous list address low bits */
#define MILSTD_BCSTCON_BCCHK_Pos              16                     /*!< Check broadcast */
#define MILSTD_BCSTCON_BCFEAT_Pos             28                     /*!< BC features */
#define MILSTD_BCSTCON_BCSUP_Pos              31                     /*!< BC supported */

/* Bit field masks: */
#define MILSTD_BCSTCON_SCST_Msk               0x00000007UL           /*!< Schedule state */
#define MILSTD_BCSTCON_SCADL_Msk              0x000000F8UL           /*!< Schedule address low bits */
#define MILSTD_BCSTCON_ASST_Msk               0x00000300UL           /*!< Asynchronous list state */
#define MILSTD_BCSTCON_ASADL_Msk              0x0000F800UL           /*!< Asynchronous list address low bits */
#define MILSTD_BCSTCON_BCCHK_Msk              0x00010000UL           /*!< Check broadcast */
#define MILSTD_BCSTCON_BCFEAT_Msk             0x70000000UL           /*!< BC features */
#define MILSTD_BCSTCON_BCSUP_Msk              0x80000000UL           /*!< BC supported */

/*--  BCACT: BC action register -------------------------------------------------------------------------------*/
typedef struct {
  uint32_t SCSRT                  :1;                                /*!< Schedule start */
  uint32_t SCSUS                  :1;                                /*!< Schedule suspend */
  uint32_t SCSTP                  :1;                                /*!< Schedule stop */
  uint32_t SETT                   :1;                                /*!< Set external trigger */
  uint32_t CLRT                   :1;                                /*!< Clear external trigger */
  uint32_t                        :3;                                /*!< RESERVED */
  uint32_t ASSRT                  :1;                                /*!< Asynchronous list start */
  uint32_t ASSTP                  :1;                                /*!< Asynchronous list stop */
  uint32_t                        :6;                                /*!< RESERVED */
  uint32_t BCKEY                  :16;                               /*!< BC safety code */
} _MILSTD_BCACT_bits;

/* Bit field positions: */
#define MILSTD_BCACT_SCSRT_Pos                0                      /*!< Schedule start */
#define MILSTD_BCACT_SCSUS_Pos                1                      /*!< Schedule suspend */
#define MILSTD_BCACT_SCSTP_Pos                2                      /*!< Schedule stop */
#define MILSTD_BCACT_SETT_Pos                 3                      /*!< Set external trigger */
#define MILSTD_BCACT_CLRT_Pos                 4                      /*!< Clear external trigger */
#define MILSTD_BCACT_ASSRT_Pos                8                      /*!< Asynchronous list start */
#define MILSTD_BCACT_ASSTP_Pos                9                      /*!< Asynchronous list stop */
#define MILSTD_BCACT_BCKEY_Pos                16                     /*!< BC safety code */

/* Bit field masks: */
#define MILSTD_BCACT_SCSRT_Msk                0x00000001UL           /*!< Schedule start */
#define MILSTD_BCACT_SCSUS_Msk                0x00000002UL           /*!< Schedule suspend */
#define MILSTD_BCACT_SCSTP_Msk                0x00000004UL           /*!< Schedule stop */
#define MILSTD_BCACT_SETT_Msk                 0x00000008UL           /*!< Set external trigger */
#define MILSTD_BCACT_CLRT_Msk                 0x00000010UL           /*!< Clear external trigger */
#define MILSTD_BCACT_ASSRT_Msk                0x00000100UL           /*!< Asynchronous list start */
#define MILSTD_BCACT_ASSTP_Msk                0x00000200UL           /*!< Asynchronous list stop */
#define MILSTD_BCACT_BCKEY_Msk                0xFFFF0000UL           /*!< BC safety code */

/* Bit field enums: */
typedef enum {
  MILSTD_BCACT_BCKEY_Code                    = 0x1552UL,             /*!< BC safety code */
} MILSTD_BCACT_BCKEY_Enum;

/*--  BCLNP: BC transfer list next pointer --------------------------------------------------------------------*/
typedef struct {
  uint32_t SCTLP                  :32;                               /*!< Schedule transfer list pointer */
} _MILSTD_BCLNP_bits;

/* Bit field positions: */
#define MILSTD_BCLNP_SCTLP_Pos                0                      /*!< Schedule transfer list pointer */

/* Bit field masks: */
#define MILSTD_BCLNP_SCTLP_Msk                0xFFFFFFFFUL           /*!< Schedule transfer list pointer */

/*--  BCALNP: BC asynchronous list next pointer register ------------------------------------------------------*/
typedef struct {
  uint32_t ASLP                   :32;                               /*!< Asynchronous list pointer */
} _MILSTD_BCALNP_bits;

/* Bit field positions: */
#define MILSTD_BCALNP_ASLP_Pos                0                      /*!< Asynchronous list pointer */

/* Bit field masks: */
#define MILSTD_BCALNP_ASLP_Msk                0xFFFFFFFFUL           /*!< Asynchronous list pointer */

/*--  BCTIM: BC timer register --------------------------------------------------------------------------------*/
typedef struct {
  uint32_t SCTM                   :24;                               /*!< Schedule time */
} _MILSTD_BCTIM_bits;

/* Bit field positions: */
#define MILSTD_BCTIM_SCTM_Pos                 0                      /*!< Schedule time */

/* Bit field masks: */
#define MILSTD_BCTIM_SCTM_Msk                 0x00FFFFFFUL           /*!< Schedule time */

/*--  BCTIMWK: BC timer wake-up register ----------------------------------------------------------------------*/
typedef struct {
  uint32_t WKTM                   :24;                               /*!< Wake-up time */
  uint32_t                        :7;                                /*!< RESERVED */
  uint32_t WKEN                   :1;                                /*!< Wake-up timer enable */
} _MILSTD_BCTIMWK_bits;

/* Bit field positions: */
#define MILSTD_BCTIMWK_WKTM_Pos               0                      /*!< Wake-up time */
#define MILSTD_BCTIMWK_WKEN_Pos               31                     /*!< Wake-up timer enable */

/* Bit field masks: */
#define MILSTD_BCTIMWK_WKTM_Msk               0x00FFFFFFUL           /*!< Wake-up time */
#define MILSTD_BCTIMWK_WKEN_Msk               0x80000000UL           /*!< Wake-up timer enable */

/*--  BCTRP: BC transfer-triggered IRQ ring position register -------------------------------------------------*/
typedef struct {
  uint32_t SPRP                   :32;                               /*!< BC IRQ source pointer ring position */
} _MILSTD_BCTRP_bits;

/* Bit field positions: */
#define MILSTD_BCTRP_SPRP_Pos                 0                      /*!< BC IRQ source pointer ring position */

/* Bit field masks: */
#define MILSTD_BCTRP_SPRP_Msk                 0xFFFFFFFFUL           /*!< BC IRQ source pointer ring position */

/*--  BCBSW: BC per-RT bus swap register ----------------------------------------------------------------------*/
typedef struct {
  uint32_t BPRBS                  :32;                               /*!< BC per-RT bus swap */
} _MILSTD_BCBSW_bits;

/* Bit field positions: */
#define MILSTD_BCBSW_BPRBS_Pos                0                      /*!< BC per-RT bus swap */

/* Bit field masks: */
#define MILSTD_BCBSW_BPRBS_Msk                0xFFFFFFFFUL           /*!< BC per-RT bus swap */

/*--  BCTSP: BC transfer list current slot pointer ------------------------------------------------------------*/
typedef struct {
  uint32_t TSP                    :32;                               /*!< BC transfer slot pointer */
} _MILSTD_BCTSP_bits;

/* Bit field positions: */
#define MILSTD_BCTSP_TSP_Pos                  0                      /*!< BC transfer slot pointer */

/* Bit field masks: */
#define MILSTD_BCTSP_TSP_Msk                  0xFFFFFFFFUL           /*!< BC transfer slot pointer */

/*--  BCATSP: BC asynchronous list current slot pointer -------------------------------------------------------*/
typedef struct {
  uint32_t ATSP                   :32;                               /*!< BC asynchronous transfer slot pointer */
} _MILSTD_BCATSP_bits;

/* Bit field positions: */
#define MILSTD_BCATSP_ATSP_Pos                0                      /*!< BC asynchronous transfer slot pointer */

/* Bit field masks: */
#define MILSTD_BCATSP_ATSP_Msk                0xFFFFFFFFUL           /*!< BC asynchronous transfer slot pointer */

/*--  RTSTAT: RT status register ------------------------------------------------------------------------------*/
typedef struct {
  uint32_t RUN                    :1;                                /*!< RT running */
  uint32_t SHDB                   :1;                                /*!< Bus B shutdown */
  uint32_t SHDA                   :1;                                /*!< Bus A shutdown */
  uint32_t ACT                    :1;                                /*!< RT active */
  uint32_t                        :27;                               /*!< RESERVED */
  uint32_t RTSUP                  :1;                                /*!< RT supported */
} _MILSTD_RTSTAT_bits;

/* Bit field positions: */
#define MILSTD_RTSTAT_RUN_Pos                 0                      /*!< RT running */
#define MILSTD_RTSTAT_SHDB_Pos                1                      /*!< Bus B shutdown */
#define MILSTD_RTSTAT_SHDA_Pos                2                      /*!< Bus A shutdown */
#define MILSTD_RTSTAT_ACT_Pos                 3                      /*!< RT active */
#define MILSTD_RTSTAT_RTSUP_Pos               31                     /*!< RT supported */

/* Bit field masks: */
#define MILSTD_RTSTAT_RUN_Msk                 0x00000001UL           /*!< RT running */
#define MILSTD_RTSTAT_SHDB_Msk                0x00000002UL           /*!< Bus B shutdown */
#define MILSTD_RTSTAT_SHDA_Msk                0x00000004UL           /*!< Bus A shutdown */
#define MILSTD_RTSTAT_ACT_Msk                 0x00000008UL           /*!< RT active */
#define MILSTD_RTSTAT_RTSUP_Msk               0x80000000UL           /*!< RT supported */

/*--  RTCON: RT config register -------------------------------------------------------------------------------*/
typedef struct {
  uint32_t RTEN                   :1;                                /*!< RT enable */
  uint32_t RTADDR                 :5;                                /*!< RT address */
  uint32_t RTEIS                  :1;                                /*!< external RT address set */
  uint32_t                        :6;                                /*!< RESERVED */
  uint32_t BRS                    :1;                                /*!< Bus reset enable */
  uint32_t SYDS                   :1;                                /*!< Sync with data enable */
  uint32_t SYS                    :1;                                /*!< Sync enable */
  uint32_t RTKEY                  :16;                               /*!< RT safety code */
} _MILSTD_RTCON_bits;

/* Bit field positions: */
#define MILSTD_RTCON_RTEN_Pos                 0                      /*!< RT enable */
#define MILSTD_RTCON_RTADDR_Pos               1                      /*!< RT address */
#define MILSTD_RTCON_RTEIS_Pos                6                      /*!< external RT address set */
#define MILSTD_RTCON_BRS_Pos                  13                     /*!< Bus reset enable */
#define MILSTD_RTCON_SYDS_Pos                 14                     /*!< Sync with data enable */
#define MILSTD_RTCON_SYS_Pos                  15                     /*!< Sync enable */
#define MILSTD_RTCON_RTKEY_Pos                16                     /*!< RT safety code */

/* Bit field masks: */
#define MILSTD_RTCON_RTEN_Msk                 0x00000001UL           /*!< RT enable */
#define MILSTD_RTCON_RTADDR_Msk               0x0000003EUL           /*!< RT address */
#define MILSTD_RTCON_RTEIS_Msk                0x00000040UL           /*!< external RT address set */
#define MILSTD_RTCON_BRS_Msk                  0x00002000UL           /*!< Bus reset enable */
#define MILSTD_RTCON_SYDS_Msk                 0x00004000UL           /*!< Sync with data enable */
#define MILSTD_RTCON_SYS_Msk                  0x00008000UL           /*!< Sync enable */
#define MILSTD_RTCON_RTKEY_Msk                0xFFFF0000UL           /*!< RT safety code */

/* Bit field enums: */
typedef enum {
  MILSTD_RTCON_RTKEY_Code                    = 0x1553UL,             /*!< RT safety code */
} MILSTD_RTCON_RTKEY_Enum;

/*--  RTBST: RT bus status register ---------------------------------------------------------------------------*/
typedef struct {
  uint32_t TFLG                   :1;                                /*!< Terminal flag */
  uint32_t DBCA                   :1;                                /*!< Dynamic BC acceptance */
  uint32_t SSF                    :1;                                /*!< Subsystem flag */
  uint32_t BUSY                   :1;                                /*!< Busy bit */
  uint32_t SREQ                   :1;                                /*!< Service request */
  uint32_t                        :3;                                /*!< RESERVED */
  uint32_t TFDE                   :1;                                /*!< Set terminal flag */
} _MILSTD_RTBST_bits;

/* Bit field positions: */
#define MILSTD_RTBST_TFLG_Pos                 0                      /*!< Terminal flag */
#define MILSTD_RTBST_DBCA_Pos                 1                      /*!< Dynamic BC acceptance */
#define MILSTD_RTBST_SSF_Pos                  2                      /*!< Subsystem flag */
#define MILSTD_RTBST_BUSY_Pos                 3                      /*!< Busy bit */
#define MILSTD_RTBST_SREQ_Pos                 4                      /*!< Service request */
#define MILSTD_RTBST_TFDE_Pos                 8                      /*!< Set terminal flag */

/* Bit field masks: */
#define MILSTD_RTBST_TFLG_Msk                 0x00000001UL           /*!< Terminal flag */
#define MILSTD_RTBST_DBCA_Msk                 0x00000002UL           /*!< Dynamic BC acceptance */
#define MILSTD_RTBST_SSF_Msk                  0x00000004UL           /*!< Subsystem flag */
#define MILSTD_RTBST_BUSY_Msk                 0x00000008UL           /*!< Busy bit */
#define MILSTD_RTBST_SREQ_Msk                 0x00000010UL           /*!< Service request */
#define MILSTD_RTBST_TFDE_Msk                 0x00000100UL           /*!< Set terminal flag */

/*--  RTSW: RT status words register --------------------------------------------------------------------------*/
typedef struct {
  uint32_t VECW                   :16;                               /*!< Vector word */
  uint32_t BITW                   :16;                               /*!< BIT Word */
} _MILSTD_RTSW_bits;

/* Bit field positions: */
#define MILSTD_RTSW_VECW_Pos                  0                      /*!< Vector word */
#define MILSTD_RTSW_BITW_Pos                  16                     /*!< BIT Word */

/* Bit field masks: */
#define MILSTD_RTSW_VECW_Msk                  0x0000FFFFUL           /*!< Vector word */
#define MILSTD_RTSW_BITW_Msk                  0xFFFF0000UL           /*!< BIT Word */

/*--  RTSYNC: RT sync register --------------------------------------------------------------------------------*/
typedef struct {
  uint32_t SYD                    :16;                               /*!< RT sync data */
  uint32_t SYTM                   :16;                               /*!< RT sync time */
} _MILSTD_RTSYNC_bits;

/* Bit field positions: */
#define MILSTD_RTSYNC_SYD_Pos                 0                      /*!< RT sync data */
#define MILSTD_RTSYNC_SYTM_Pos                16                     /*!< RT sync time */

/* Bit field masks: */
#define MILSTD_RTSYNC_SYD_Msk                 0x0000FFFFUL           /*!< RT sync data */
#define MILSTD_RTSYNC_SYTM_Msk                0xFFFF0000UL           /*!< RT sync time */

/*--  RTSADDR: RT subaddress table base address ---------------------------------------------------------------*/
typedef struct {
  uint32_t                        :9;                                /*!< RESERVED */
  uint32_t SATB                   :23;                               /*!< RT subaddress table base */
} _MILSTD_RTSADDR_bits;

/* Bit field positions: */
#define MILSTD_RTSADDR_SATB_Pos               9                      /*!< RT subaddress table base */

/* Bit field masks: */
#define MILSTD_RTSADDR_SATB_Msk               0xFFFFFE00UL           /*!< RT subaddress table base */

/*--  RTMOD: RT mode control register -------------------------------------------------------------------------*/
typedef struct {
  uint32_t S                      :2;                                /*!< Syncronize */
  uint32_t SB                     :2;                                /*!< Synchronize broadcast */
  uint32_t SD                     :2;                                /*!< Synchronize with data word */
  uint32_t SDB                    :2;                                /*!< Synchronize with data word broadcast */
  uint32_t TS                     :2;                                /*!< Transmitter shutdown */
  uint32_t TSB                    :2;                                /*!< Transmitter shutdown broadcast */
  uint32_t TVW                    :2;                                /*!< Transmit vector word */
  uint32_t TBW                    :2;                                /*!< Transmit BIT word */
  uint32_t DBC                    :2;                                /*!< Dynamic bus control */
  uint32_t IST                    :2;                                /*!< Initiate self test */
  uint32_t ISTB                   :2;                                /*!< Initiate self test broadcast */
  uint32_t ITF                    :2;                                /*!< Inhibit terminal flag */
  uint32_t ITFB                   :2;                                /*!< Inhibit terminal flag broadcast  */
  uint32_t RRT                    :2;                                /*!< Reset RT */
  uint32_t RRTB                   :2;                                /*!< reset RT broadcast */
} _MILSTD_RTMOD_bits;

/* Bit field positions: */
#define MILSTD_RTMOD_S_Pos                    0                      /*!< Syncronize */
#define MILSTD_RTMOD_SB_Pos                   2                      /*!< Synchronize broadcast */
#define MILSTD_RTMOD_SD_Pos                   4                      /*!< Synchronize with data word */
#define MILSTD_RTMOD_SDB_Pos                  6                      /*!< Synchronize with data word broadcast */
#define MILSTD_RTMOD_TS_Pos                   8                      /*!< Transmitter shutdown */
#define MILSTD_RTMOD_TSB_Pos                  10                     /*!< Transmitter shutdown broadcast */
#define MILSTD_RTMOD_TVW_Pos                  12                     /*!< Transmit vector word */
#define MILSTD_RTMOD_TBW_Pos                  14                     /*!< Transmit BIT word */
#define MILSTD_RTMOD_DBC_Pos                  16                     /*!< Dynamic bus control */
#define MILSTD_RTMOD_IST_Pos                  18                     /*!< Initiate self test */
#define MILSTD_RTMOD_ISTB_Pos                 20                     /*!< Initiate self test broadcast */
#define MILSTD_RTMOD_ITF_Pos                  22                     /*!< Inhibit terminal flag */
#define MILSTD_RTMOD_ITFB_Pos                 24                     /*!< Inhibit terminal flag broadcast  */
#define MILSTD_RTMOD_RRT_Pos                  26                     /*!< Reset RT */
#define MILSTD_RTMOD_RRTB_Pos                 28                     /*!< reset RT broadcast */

/* Bit field masks: */
#define MILSTD_RTMOD_S_Msk                    0x00000003UL           /*!< Syncronize */
#define MILSTD_RTMOD_SB_Msk                   0x0000000CUL           /*!< Synchronize broadcast */
#define MILSTD_RTMOD_SD_Msk                   0x00000030UL           /*!< Synchronize with data word */
#define MILSTD_RTMOD_SDB_Msk                  0x000000C0UL           /*!< Synchronize with data word broadcast */
#define MILSTD_RTMOD_TS_Msk                   0x00000300UL           /*!< Transmitter shutdown */
#define MILSTD_RTMOD_TSB_Msk                  0x00000C00UL           /*!< Transmitter shutdown broadcast */
#define MILSTD_RTMOD_TVW_Msk                  0x00003000UL           /*!< Transmit vector word */
#define MILSTD_RTMOD_TBW_Msk                  0x0000C000UL           /*!< Transmit BIT word */
#define MILSTD_RTMOD_DBC_Msk                  0x00030000UL           /*!< Dynamic bus control */
#define MILSTD_RTMOD_IST_Msk                  0x000C0000UL           /*!< Initiate self test */
#define MILSTD_RTMOD_ISTB_Msk                 0x00300000UL           /*!< Initiate self test broadcast */
#define MILSTD_RTMOD_ITF_Msk                  0x00C00000UL           /*!< Inhibit terminal flag */
#define MILSTD_RTMOD_ITFB_Msk                 0x03000000UL           /*!< Inhibit terminal flag broadcast  */
#define MILSTD_RTMOD_RRT_Msk                  0x0C000000UL           /*!< Reset RT */
#define MILSTD_RTMOD_RRTB_Msk                 0x30000000UL           /*!< reset RT broadcast */

/*--  RTTIM: RT time tag control register ---------------------------------------------------------------------*/
typedef struct {
  uint32_t TVAL                   :16;                               /*!< Time tag value */
  uint32_t TRES                   :16;                               /*!< Time tag resolution */
} _MILSTD_RTTIM_bits;

/* Bit field positions: */
#define MILSTD_RTTIM_TVAL_Pos                 0                      /*!< Time tag value */
#define MILSTD_RTTIM_TRES_Pos                 16                     /*!< Time tag resolution */

/* Bit field masks: */
#define MILSTD_RTTIM_TVAL_Msk                 0x0000FFFFUL           /*!< Time tag value */
#define MILSTD_RTTIM_TRES_Msk                 0xFFFF0000UL           /*!< Time tag resolution */

/*--  RTELMSK: RT event log mask register ---------------------------------------------------------------------*/
typedef struct {
  uint32_t                        :2;                                /*!< RESERVED */
  uint32_t ELSM                   :19;                               /*!< Event log size mask */
} _MILSTD_RTELMSK_bits;

/* Bit field positions: */
#define MILSTD_RTELMSK_ELSM_Pos               2                      /*!< Event log size mask */

/* Bit field masks: */
#define MILSTD_RTELMSK_ELSM_Msk               0x001FFFFCUL           /*!< Event log size mask */

/*--  RTELP: RT event log position pointer --------------------------------------------------------------------*/
typedef struct {
  uint32_t ELWP                   :32;                               /*!< Event log write pointer */
} _MILSTD_RTELP_bits;

/* Bit field positions: */
#define MILSTD_RTELP_ELWP_Pos                 0                      /*!< Event log write pointer */

/* Bit field masks: */
#define MILSTD_RTELP_ELWP_Msk                 0xFFFFFFFFUL           /*!< Event log write pointer */

/*--  RTELIP: RT event log interrupt position register --------------------------------------------------------*/
typedef struct {
  uint32_t ELIP                   :32;                               /*!< Event log irq pointer */
} _MILSTD_RTELIP_bits;

/* Bit field positions: */
#define MILSTD_RTELIP_ELIP_Pos                0                      /*!< Event log irq pointer */

/* Bit field masks: */
#define MILSTD_RTELIP_ELIP_Msk                0xFFFFFFFFUL           /*!< Event log irq pointer */

/*--  BMSTAT: BM status register ------------------------------------------------------------------------------*/
typedef struct {
  uint32_t                        :30;                               /*!< RESERVED */
  uint32_t KEYEN                  :1;                                /*!< Key enabled */
  uint32_t BMSUP                  :1;                                /*!< BM supported */
} _MILSTD_BMSTAT_bits;

/* Bit field positions: */
#define MILSTD_BMSTAT_KEYEN_Pos               30                     /*!< Key enabled */
#define MILSTD_BMSTAT_BMSUP_Pos               31                     /*!< BM supported */

/* Bit field masks: */
#define MILSTD_BMSTAT_KEYEN_Msk               0x40000000UL           /*!< Key enabled */
#define MILSTD_BMSTAT_BMSUP_Msk               0x80000000UL           /*!< BM supported */

/*--  BMCON: BM control register ------------------------------------------------------------------------------*/
typedef struct {
  uint32_t BMEN                   :1;                                /*!< BM enable */
  uint32_t MANL                   :1;                                /*!< Manchester parity error log */
  uint32_t UDWL                   :1;                                /*!< Unexpected data word log */
  uint32_t IMCL                   :1;                                /*!< Invalid mode code log */
  uint32_t EXST                   :1;                                /*!< External sync start */
  uint32_t WRSTP                  :1;                                /*!< Wrap stop */
  uint32_t                        :10;                               /*!< RESERVED */
  uint32_t BMKEY                  :16;                               /*!< Safety key */
} _MILSTD_BMCON_bits;

/* Bit field positions: */
#define MILSTD_BMCON_BMEN_Pos                 0                      /*!< BM enable */
#define MILSTD_BMCON_MANL_Pos                 1                      /*!< Manchester parity error log */
#define MILSTD_BMCON_UDWL_Pos                 2                      /*!< Unexpected data word log */
#define MILSTD_BMCON_IMCL_Pos                 3                      /*!< Invalid mode code log */
#define MILSTD_BMCON_EXST_Pos                 4                      /*!< External sync start */
#define MILSTD_BMCON_WRSTP_Pos                5                      /*!< Wrap stop */
#define MILSTD_BMCON_BMKEY_Pos                16                     /*!< Safety key */

/* Bit field masks: */
#define MILSTD_BMCON_BMEN_Msk                 0x00000001UL           /*!< BM enable */
#define MILSTD_BMCON_MANL_Msk                 0x00000002UL           /*!< Manchester parity error log */
#define MILSTD_BMCON_UDWL_Msk                 0x00000004UL           /*!< Unexpected data word log */
#define MILSTD_BMCON_IMCL_Msk                 0x00000008UL           /*!< Invalid mode code log */
#define MILSTD_BMCON_EXST_Msk                 0x00000010UL           /*!< External sync start */
#define MILSTD_BMCON_WRSTP_Msk                0x00000020UL           /*!< Wrap stop */
#define MILSTD_BMCON_BMKEY_Msk                0xFFFF0000UL           /*!< Safety key */

/* Bit field enums: */
typedef enum {
  MILSTD_BMCON_BMKEY_Code                    = 0x1554UL,             /*!< BM safety code */
} MILSTD_BMCON_BMKEY_Enum;

/*--  BMADF: BM RT address filter register --------------------------------------------------------------------*/
typedef struct {
  uint32_t AFM                    :32;                               /*!< Address filter mask */
} _MILSTD_BMADF_bits;

/* Bit field positions: */
#define MILSTD_BMADF_AFM_Pos                  0                      /*!< Address filter mask */

/* Bit field masks: */
#define MILSTD_BMADF_AFM_Msk                  0xFFFFFFFFUL           /*!< Address filter mask */

/*--  BMSADF: BM RT subaddress filter register ----------------------------------------------------------------*/
typedef struct {
  uint32_t SAFM                   :32;                               /*!< Subaddress filter mask */
} _MILSTD_BMSADF_bits;

/* Bit field positions: */
#define MILSTD_BMSADF_SAFM_Pos                0                      /*!< Subaddress filter mask */

/* Bit field masks: */
#define MILSTD_BMSADF_SAFM_Msk                0xFFFFFFFFUL           /*!< Subaddress filter mask */

/*--  BMMODF: BM RT mode code filter register -----------------------------------------------------------------*/
typedef struct {
  uint32_t S                      :1;                                /*!< Syncronize */
  uint32_t SB                     :1;                                /*!< Syncronize broadcast */
  uint32_t SD                     :1;                                /*!< Syncronize with data word */
  uint32_t SDB                    :1;                                /*!< Syncronize with data word broadcast */
  uint32_t TS                     :1;                                /*!< Transmitter shutdown */
  uint32_t TSB                    :1;                                /*!< Transmitter shutdown broadcast */
  uint32_t TVW                    :1;                                /*!< Transmit vector word */
  uint32_t TBW                    :1;                                /*!< Transmit BIT word */
  uint32_t DBC                    :1;                                /*!< Dynamic bus control */
  uint32_t IST                    :1;                                /*!< Initiate self test */
  uint32_t ISTB                   :1;                                /*!< Initiate self test broadcast */
  uint32_t ITF                    :1;                                /*!< Inhibit terminal flag */
  uint32_t ITFB                   :1;                                /*!< Inhibit termital flag broadcast */
  uint32_t RRT                    :1;                                /*!< Reset RT */
  uint32_t RRTB                   :1;                                /*!< Reset RT broadcast */
  uint32_t TSW                    :1;                                /*!< Transmit status word */
  uint32_t TLC                    :1;                                /*!< Transmit last command */
  uint32_t STS                    :1;                                /*!< Selected transmitter shutdown */
  uint32_t STSB                   :1;                                /*!< Selected transmitter shutdown broadcast */
} _MILSTD_BMMODF_bits;

/* Bit field positions: */
#define MILSTD_BMMODF_S_Pos                   0                      /*!< Syncronize */
#define MILSTD_BMMODF_SB_Pos                  1                      /*!< Syncronize broadcast */
#define MILSTD_BMMODF_SD_Pos                  2                      /*!< Syncronize with data word */
#define MILSTD_BMMODF_SDB_Pos                 3                      /*!< Syncronize with data word broadcast */
#define MILSTD_BMMODF_TS_Pos                  4                      /*!< Transmitter shutdown */
#define MILSTD_BMMODF_TSB_Pos                 5                      /*!< Transmitter shutdown broadcast */
#define MILSTD_BMMODF_TVW_Pos                 6                      /*!< Transmit vector word */
#define MILSTD_BMMODF_TBW_Pos                 7                      /*!< Transmit BIT word */
#define MILSTD_BMMODF_DBC_Pos                 8                      /*!< Dynamic bus control */
#define MILSTD_BMMODF_IST_Pos                 9                      /*!< Initiate self test */
#define MILSTD_BMMODF_ISTB_Pos                10                     /*!< Initiate self test broadcast */
#define MILSTD_BMMODF_ITF_Pos                 11                     /*!< Inhibit terminal flag */
#define MILSTD_BMMODF_ITFB_Pos                12                     /*!< Inhibit termital flag broadcast */
#define MILSTD_BMMODF_RRT_Pos                 13                     /*!< Reset RT */
#define MILSTD_BMMODF_RRTB_Pos                14                     /*!< Reset RT broadcast */
#define MILSTD_BMMODF_TSW_Pos                 15                     /*!< Transmit status word */
#define MILSTD_BMMODF_TLC_Pos                 16                     /*!< Transmit last command */
#define MILSTD_BMMODF_STS_Pos                 17                     /*!< Selected transmitter shutdown */
#define MILSTD_BMMODF_STSB_Pos                18                     /*!< Selected transmitter shutdown broadcast */

/* Bit field masks: */
#define MILSTD_BMMODF_S_Msk                   0x00000001UL           /*!< Syncronize */
#define MILSTD_BMMODF_SB_Msk                  0x00000002UL           /*!< Syncronize broadcast */
#define MILSTD_BMMODF_SD_Msk                  0x00000004UL           /*!< Syncronize with data word */
#define MILSTD_BMMODF_SDB_Msk                 0x00000008UL           /*!< Syncronize with data word broadcast */
#define MILSTD_BMMODF_TS_Msk                  0x00000010UL           /*!< Transmitter shutdown */
#define MILSTD_BMMODF_TSB_Msk                 0x00000020UL           /*!< Transmitter shutdown broadcast */
#define MILSTD_BMMODF_TVW_Msk                 0x00000040UL           /*!< Transmit vector word */
#define MILSTD_BMMODF_TBW_Msk                 0x00000080UL           /*!< Transmit BIT word */
#define MILSTD_BMMODF_DBC_Msk                 0x00000100UL           /*!< Dynamic bus control */
#define MILSTD_BMMODF_IST_Msk                 0x00000200UL           /*!< Initiate self test */
#define MILSTD_BMMODF_ISTB_Msk                0x00000400UL           /*!< Initiate self test broadcast */
#define MILSTD_BMMODF_ITF_Msk                 0x00000800UL           /*!< Inhibit terminal flag */
#define MILSTD_BMMODF_ITFB_Msk                0x00001000UL           /*!< Inhibit termital flag broadcast */
#define MILSTD_BMMODF_RRT_Msk                 0x00002000UL           /*!< Reset RT */
#define MILSTD_BMMODF_RRTB_Msk                0x00004000UL           /*!< Reset RT broadcast */
#define MILSTD_BMMODF_TSW_Msk                 0x00008000UL           /*!< Transmit status word */
#define MILSTD_BMMODF_TLC_Msk                 0x00010000UL           /*!< Transmit last command */
#define MILSTD_BMMODF_STS_Msk                 0x00020000UL           /*!< Selected transmitter shutdown */
#define MILSTD_BMMODF_STSB_Msk                0x00040000UL           /*!< Selected transmitter shutdown broadcast */

/*--  BMLBS: BM log buffer start ------------------------------------------------------------------------------*/
typedef struct {
  uint32_t BMBS                   :32;                               /*!< BM log buffer start */
} _MILSTD_BMLBS_bits;

/* Bit field positions: */
#define MILSTD_BMLBS_BMBS_Pos                 0                      /*!< BM log buffer start */

/* Bit field masks: */
#define MILSTD_BMLBS_BMBS_Msk                 0xFFFFFFFFUL           /*!< BM log buffer start */

/*--  BMLBE: BM log buffer end --------------------------------------------------------------------------------*/
typedef struct {
  uint32_t BMBE                   :22;                               /*!< BM log buffer end */
} _MILSTD_BMLBE_bits;

/* Bit field positions: */
#define MILSTD_BMLBE_BMBE_Pos                 0                      /*!< BM log buffer end */

/* Bit field masks: */
#define MILSTD_BMLBE_BMBE_Msk                 0x003FFFFFUL           /*!< BM log buffer end */

/*--  BMLBP: BM log buffer position ---------------------------------------------------------------------------*/
typedef struct {
  uint32_t BMBP                   :22;                               /*!< BM log buffer position */
} _MILSTD_BMLBP_bits;

/* Bit field positions: */
#define MILSTD_BMLBP_BMBP_Pos                 0                      /*!< BM log buffer position */

/* Bit field masks: */
#define MILSTD_BMLBP_BMBP_Msk                 0x003FFFFFUL           /*!< BM log buffer position */

/*--  BMTIM: BM time tag control register ---------------------------------------------------------------------*/
typedef struct {
  uint32_t BMTVAL                 :24;                               /*!< Time tag value */
  uint32_t BMTRES                 :8;                                /*!< Time tag resolution */
} _MILSTD_BMTIM_bits;

/* Bit field positions: */
#define MILSTD_BMTIM_BMTVAL_Pos               0                      /*!< Time tag value */
#define MILSTD_BMTIM_BMTRES_Pos               24                     /*!< Time tag resolution */

/* Bit field masks: */
#define MILSTD_BMTIM_BMTVAL_Msk               0x00FFFFFFUL           /*!< Time tag value */
#define MILSTD_BMTIM_BMTRES_Msk               0xFF000000UL           /*!< Time tag resolution */

typedef struct {
  union {                                                               /*!< BSI interrupt register */
    __IO uint32_t IR;                                                /*!< IR    : type used for word access */
    __IO _MILSTD_IR_bits  IR_bit;                                    /*!< IR_bit: structure used for bit access */
  };
  union {                                                               /*!< BSI interrupt enable register */
    __IO uint32_t IENR;                                              /*!< IENR    : type used for word access */
    __IO _MILSTD_IENR_bits  IENR_bit;                                /*!< IENR_bit: structure used for bit access */
  };
    __IO uint32_t Reserved0[2];
  union {                                                               /*!< BSI hardware configuration register */
    __I uint32_t HCON;                                               /*!< HCON    : type used for word access */
    __I _MILSTD_HCON_bits  HCON_bit;                                 /*!< HCON_bit: structure used for bit access */
  };
    __IO uint32_t Reserved1[11];
  union {                                                               /*!< BC status and config register */
    __IO uint32_t BCSTCON;                                           /*!< BCSTCON    : type used for word access */
    __IO _MILSTD_BCSTCON_bits  BCSTCON_bit;                          /*!< BCSTCON_bit: structure used for bit access */
  };
  union {                                                               /*!< BC action register */
    __O uint32_t BCACT;                                              /*!< BCACT    : type used for word access */
    __O _MILSTD_BCACT_bits  BCACT_bit;                               /*!< BCACT_bit: structure used for bit access */
  };
  union {                                                               /*!< BC transfer list next pointer */
    __IO uint32_t BCLNP;                                             /*!< BCLNP    : type used for word access */
    __IO _MILSTD_BCLNP_bits  BCLNP_bit;                              /*!< BCLNP_bit: structure used for bit access */
  };
  union {                                                               /*!< BC asynchronous list next pointer register */
    __IO uint32_t BCALNP;                                            /*!< BCALNP    : type used for word access */
    __IO _MILSTD_BCALNP_bits  BCALNP_bit;                            /*!< BCALNP_bit: structure used for bit access */
  };
  union {                                                               /*!< BC timer register */
    __I uint32_t BCTIM;                                              /*!< BCTIM    : type used for word access */
    __I _MILSTD_BCTIM_bits  BCTIM_bit;                               /*!< BCTIM_bit: structure used for bit access */
  };
  union {                                                               /*!< BC timer wake-up register */
    __IO uint32_t BCTIMWK;                                           /*!< BCTIMWK    : type used for word access */
    __IO _MILSTD_BCTIMWK_bits  BCTIMWK_bit;                          /*!< BCTIMWK_bit: structure used for bit access */
  };
  union {                                                               /*!< BC transfer-triggered IRQ ring position register */
    __IO uint32_t BCTRP;                                             /*!< BCTRP    : type used for word access */
    __IO _MILSTD_BCTRP_bits  BCTRP_bit;                              /*!< BCTRP_bit: structure used for bit access */
  };
  union {                                                               /*!< BC per-RT bus swap register */
    __IO uint32_t BCBSW;                                             /*!< BCBSW    : type used for word access */
    __IO _MILSTD_BCBSW_bits  BCBSW_bit;                              /*!< BCBSW_bit: structure used for bit access */
  };
    __IO uint32_t Reserved2[2];
  union {                                                               /*!< BC transfer list current slot pointer */
    __I uint32_t BCTSP;                                              /*!< BCTSP    : type used for word access */
    __I _MILSTD_BCTSP_bits  BCTSP_bit;                               /*!< BCTSP_bit: structure used for bit access */
  };
  union {                                                               /*!< BC asynchronous list current slot pointer */
    __I uint32_t BCATSP;                                             /*!< BCATSP    : type used for word access */
    __I _MILSTD_BCATSP_bits  BCATSP_bit;                             /*!< BCATSP_bit: structure used for bit access */
  };
    __IO uint32_t Reserved3[4];
  union {                                                               /*!< RT status register */
    __I uint32_t RTSTAT;                                             /*!< RTSTAT    : type used for word access */
    __I _MILSTD_RTSTAT_bits  RTSTAT_bit;                             /*!< RTSTAT_bit: structure used for bit access */
  };
  union {                                                               /*!< RT config register */
    __IO uint32_t RTCON;                                             /*!< RTCON    : type used for word access */
    __IO _MILSTD_RTCON_bits  RTCON_bit;                              /*!< RTCON_bit: structure used for bit access */
  };
  union {                                                               /*!< RT bus status register */
    __IO uint32_t RTBST;                                             /*!< RTBST    : type used for word access */
    __IO _MILSTD_RTBST_bits  RTBST_bit;                              /*!< RTBST_bit: structure used for bit access */
  };
  union {                                                               /*!< RT status words register */
    __IO uint32_t RTSW;                                              /*!< RTSW    : type used for word access */
    __IO _MILSTD_RTSW_bits  RTSW_bit;                                /*!< RTSW_bit: structure used for bit access */
  };
  union {                                                               /*!< RT sync register */
    __I uint32_t RTSYNC;                                             /*!< RTSYNC    : type used for word access */
    __I _MILSTD_RTSYNC_bits  RTSYNC_bit;                             /*!< RTSYNC_bit: structure used for bit access */
  };
  union {                                                               /*!< RT subaddress table base address */
    __IO uint32_t RTSADDR;                                           /*!< RTSADDR    : type used for word access */
    __IO _MILSTD_RTSADDR_bits  RTSADDR_bit;                          /*!< RTSADDR_bit: structure used for bit access */
  };
  union {                                                               /*!< RT mode control register */
    __IO uint32_t RTMOD;                                             /*!< RTMOD    : type used for word access */
    __IO _MILSTD_RTMOD_bits  RTMOD_bit;                              /*!< RTMOD_bit: structure used for bit access */
  };
    __IO uint32_t Reserved4[2];
  union {                                                               /*!< RT time tag control register */
    __IO uint32_t RTTIM;                                             /*!< RTTIM    : type used for word access */
    __IO _MILSTD_RTTIM_bits  RTTIM_bit;                              /*!< RTTIM_bit: structure used for bit access */
  };
    __IO uint32_t Reserved5;
  union {                                                               /*!< RT event log mask register */
    __IO uint32_t RTELMSK;                                           /*!< RTELMSK    : type used for word access */
    __IO _MILSTD_RTELMSK_bits  RTELMSK_bit;                          /*!< RTELMSK_bit: structure used for bit access */
  };
  union {                                                               /*!< RT event log position pointer */
    __IO uint32_t RTELP;                                             /*!< RTELP    : type used for word access */
    __IO _MILSTD_RTELP_bits  RTELP_bit;                              /*!< RTELP_bit: structure used for bit access */
  };
  union {                                                               /*!< RT event log interrupt position register */
    __I uint32_t RTELIP;                                             /*!< RTELIP    : type used for word access */
    __I _MILSTD_RTELIP_bits  RTELIP_bit;                             /*!< RTELIP_bit: structure used for bit access */
  };
    __IO uint32_t Reserved6[2];
  union {                                                               /*!< BM status register */
    __I uint32_t BMSTAT;                                             /*!< BMSTAT    : type used for word access */
    __I _MILSTD_BMSTAT_bits  BMSTAT_bit;                             /*!< BMSTAT_bit: structure used for bit access */
  };
  union {                                                               /*!< BM control register */
    __IO uint32_t BMCON;                                             /*!< BMCON    : type used for word access */
    __IO _MILSTD_BMCON_bits  BMCON_bit;                              /*!< BMCON_bit: structure used for bit access */
  };
  union {                                                               /*!< BM RT address filter register */
    __IO uint32_t BMADF;                                             /*!< BMADF    : type used for word access */
    __IO _MILSTD_BMADF_bits  BMADF_bit;                              /*!< BMADF_bit: structure used for bit access */
  };
  union {                                                               /*!< BM RT subaddress filter register */
    __IO uint32_t BMSADF;                                            /*!< BMSADF    : type used for word access */
    __IO _MILSTD_BMSADF_bits  BMSADF_bit;                            /*!< BMSADF_bit: structure used for bit access */
  };
  union {                                                               /*!< BM RT mode code filter register */
    __IO uint32_t BMMODF;                                            /*!< BMMODF    : type used for word access */
    __IO _MILSTD_BMMODF_bits  BMMODF_bit;                            /*!< BMMODF_bit: structure used for bit access */
  };
  union {                                                               /*!< BM log buffer start */
    __IO uint32_t BMLBS;                                             /*!< BMLBS    : type used for word access */
    __IO _MILSTD_BMLBS_bits  BMLBS_bit;                              /*!< BMLBS_bit: structure used for bit access */
  };
  union {                                                               /*!< BM log buffer end */
    __IO uint32_t BMLBE;                                             /*!< BMLBE    : type used for word access */
    __IO _MILSTD_BMLBE_bits  BMLBE_bit;                              /*!< BMLBE_bit: structure used for bit access */
  };
  union {                                                               /*!< BM log buffer position */
    __IO uint32_t BMLBP;                                             /*!< BMLBP    : type used for word access */
    __IO _MILSTD_BMLBP_bits  BMLBP_bit;                              /*!< BMLBP_bit: structure used for bit access */
  };
  union {                                                               /*!< BM time tag control register */
    __IO uint32_t BMTIM;                                             /*!< BMTIM    : type used for word access */
    __IO _MILSTD_BMTIM_bits  BMTIM_bit;                              /*!< BMTIM_bit: structure used for bit access */
  };
} MILSTD_TypeDef;


/** @addtogroup Peripheral_declaration
  * @{
  */

#define MILSTD0              ((MILSTD_TypeDef              *) MKIO0_BASE)
#define MILSTD1              ((MILSTD_TypeDef              *) MKIO1_BASE)

/** @defgroup MILSTD_Exported_Macros MILSTD Exported Macros
  * @{
  */

#define IS_MILSTD_ALL_PERIPH(PERIPH)			(((PERIPH) == MILSTD0) ||\
												 ((PERIPH) == MILSTD1))
												 
												 
/** @defgroup MILSTD_Mode MILSTD Mode
  * @{
  */

#define MILSTD_ModeBusController				((uint32_t)0x01)
#define MILSTD_ModeTerminal						((uint32_t)0x02)
#define MILSTD_ModeMonitor						((uint32_t)0x03)

#define IS_MILSTD_MODE(MODE)					(((MODE) == MILSTD_ModeBusController) ||\
												 ((MODE) == MILSTD_ModeMonitor)       ||\
												 ((MODE) == MILSTD_ModeTerminal))												 

/*--  Milstd Transfer descriptor Word 0 configure word --------------------------------*/
typedef struct {
    uint32_t STIME      :16;                                          /*!< Slot time (STIME) - Allocated time in 4 microsecond units, remaining time after transfer will insert delay */
    uint32_t RESERVED   :2;                                           /*!< Reserved */
    uint32_t GAP        :1;                                           /*!< Extended intermessage gap (GAP) - If set, adds an additional amount of gap time, corresponding to the RTTO
                                                                     field, after the transfer */
    uint32_t STBUS      :1;                                           /*!< Store bus (STBUS) - If the transfer succeeds and this bit is set, store the bus on which the transfer succeeded (0
                                                                     for bus A, 1 for bus B) into the per-RT bus swap register. If the transfer fails and this bit is set, store the opposite
                                                                     bus instead. (only if the per-RT bus mask is supported in the core) */
    uint32_t NRET       :3;                                           /*!< Number of retries (NRET) - Number of automatic retries per bus
                                                                     The total number of tries (including the first attempt) is NRET+1 for RETMD=00, 2 x (NRET+1) for RETMD=01/10 */
    uint32_t RETMD      :2;                                           /*!< Retry mode (RETMD).  */
    uint32_t SUSN       :1;                                           /*!< Suspend normally (SUSN) - Always suspends after transfer */
    uint32_t SUSE       :1;                                           /*!< Suspend on Error (SUSE) - Suspends the schedule (or stops the async transfer list) on error */
    uint32_t IRQN       :1;                                           /*!< IRQ normally (IRQN) - Always interrupts after transfer */
    uint32_t IRQE       :1;                                           /*!< IRQ after transfer on Error (IRQE) */
    uint32_t EXCL       :1;                                           /*!< Exclusive time slot (EXCL) - Do not schedule asynchronous messages */
    uint32_t WTRIG      :1;                                           /*!< Wait for external trigger (WTRIG) */
    uint32_t ZERO       :1;                                           /*!< Must be 0 to identify as descriptor */
} _MILSTD_BC_TDESC_WORD0_bits;

/* Bit field positions: */
#define MILSTD_BC_TDESC_WORD0_STIME_Pos             0                    /*!< Slot time (STIME) - Allocated time in 4 microsecond units, remaining time after transfer will insert delay */
#define MILSTD_BC_TDESC_WORD0_GAP_Pos               18                   /*!< Extended intermessage gap (GAP) - If set, adds an additional amount of gap time, corresponding to the RTTO field, after the transfer */
#define MILSTD_BC_TDESC_WORD0_STBUS_Pos             19                   /*!< Store bus (STBUS) - If the transfer succeeds and this bit is set, store the bus on which the transfer succeeded */
#define MILSTD_BC_TDESC_WORD0_NRET_Pos              20                   /*!< Number of retries (NRET) - Number of automatic retries per bus */
#define MILSTD_BC_TDESC_WORD0_RETMD_Pos             23                   /*!< Retry mode (RETMD) */
#define MILSTD_BC_TDESC_WORD0_SUSN_Pos              25                   /*!< Suspend normally (SUSN) - Always suspends after transfer */
#define MILSTD_BC_TDESC_WORD0_SUSE_Pos              26                   /*!< Suspend on Error (SUSE) - Suspends the schedule (or stops the async transfer list) on error */
#define MILSTD_BC_TDESC_WORD0_IRQN_Pos              27                   /*!< IRQ normally (IRQN) - Always interrupts after transfer */
#define MILSTD_BC_TDESC_WORD0_IRQE_Pos              28                   /*!< IRQ after transfer on Error (IRQE) */
#define MILSTD_BC_TDESC_WORD0_EXCL_Pos              29                   /*!< Exclusive time slot (EXCL) - Do not schedule asynchronous messages */
#define MILSTD_BC_TDESC_WORD0_WTRIG_Pos             30                   /*!< Wait for external trigger (WTRIG) */

/* Bit field masks: */
#define MILSTD_BC_TDESC_WORD0_STIME_Msk             0x0000FFFFUL            /*!< Slot time (STIME) - Allocated time in 4 microsecond units, remaining time after transfer will insert delay */
#define MILSTD_BC_TDESC_WORD0_GAP_Msk               0x00040000UL            /*!< Extended intermessage gap (GAP) - If set, adds an additional amount of gap time, corresponding to the RTTO field, after the transfer */
#define MILSTD_BC_TDESC_WORD0_STBUS_Msk             0x00080000UL            /*!< Store bus (STBUS) - If the transfer succeeds and this bit is set, store the bus on which the transfer succeeded */
#define MILSTD_BC_TDESC_WORD0_NRET_Msk              0x00700000UL            /*!< Number of retries (NRET) - Number of automatic retries per bus */
#define MILSTD_BC_TDESC_WORD0_RETMD_Msk             0x01800000UL            /*!< Retry mode (RETMD) */
#define MILSTD_BC_TDESC_WORD0_SUSN_Msk              0x02000000UL            /*!< Suspend normally (SUSN) - Always suspends after transfer */
#define MILSTD_BC_TDESC_WORD0_SUSE_Msk              0x04000000UL            /*!< Suspend on Error (SUSE) - Suspends the schedule (or stops the async transfer list) on error */
#define MILSTD_BC_TDESC_WORD0_IRQN_Msk              0x08000000UL            /*!< IRQ normally (IRQN) - Always interrupts after transfer */
#define MILSTD_BC_TDESC_WORD0_IRQE_Msk              0x10000000UL            /*!< IRQ after transfer on Error (IRQE) */
#define MILSTD_BC_TDESC_WORD0_EXCL_Msk              0x20000000UL            /*!< Exclusive time slot (EXCL) - Do not schedule asynchronous messages */
#define MILSTD_BC_TDESC_WORD0_WTRIG_Msk             0x40000000UL            /*!< Wait for external trigger (WTRIG) */
#define MILSTD_BC_TDESC_WORD0_ZERO_Msk              0x80000000UL            /*!< Wait for external trigger (WTRIG) */

/* Bit field enums: */
typedef enum {
  MILSTD_BC_TDESC_WORD0_RETMD_SameBus                      = 0x0UL,      /*!< Retry on same bus only */
  MILSTD_BC_TDESC_WORD0_RETMD_BothBus                      = 0x1UL,      /*!< Retry alternating on both buses */
  MILSTD_BC_TDESC_WORD0_RETMD_FirstSameBus                 = 0x2UL,      /*!< Retry first on same bus, then on alternating bus */
} MILSTD_BC_TDESC_WORD0_RETMD_Enum;

/*--  Milstd Transfer descriptor Word 1 configure word --------------------------------*/
typedef struct {
    uint32_t WCMC       :5;                                          /*!< Word count/Mode code (WCMC) */
    uint32_t RTSA1      :5;                                          /*!< RT Subaddress (RTSA1) */
    uint32_t TR         :1;                                          /*!< Transmit/receive (TR) */
    uint32_t RTAD1      :5;                                          /*!< RT Address (RTAD1) */
    uint32_t RTSA2      :5;                                          /*!< Second RT Subaddress for RT-to-RT transfer (RTSA2) */
    uint32_t RTAD2      :5;                                          /*!< Second RT Address for RT-to-RT transfer (RTAD2) */
    uint32_t RTTO       :4;                                          /*!< RT Timeout (RTTO) - Extra RT status word timeout above nominal in units of 4 us
                                                                        Note: This extra time is also used as extra intermessage gap time if the GAP bit is set. */
    uint32_t BUS        :1;                                          /*!< Bus selection (BUS) - Bus to use for transfer, 0 - Bus A, 1 - Bus B */
    uint32_t DUM        :1;                                          /*!< Dummy transfer (DUM) - If set to �1� no bus traffic is generated and transfer �succeeds� immediately
                                                                        For dummy transfers, the EXCL,IRQN,SUSN,STBUS,GAP,STIME settings are still in effect, other bits and
                                                                        the data buffer pointer are ignored. */
} _MILSTD_BC_TDESC_WORD1_bits;

/* Bit field positions: */
#define MILSTD_BC_TDESC_WORD1_WCMC_Pos             0                       /*!< Word count/Mode code (WCMC) */
#define MILSTD_BC_TDESC_WORD1_RTSA1_Pos            5                       /*!< RT Subaddress (RTSA1) */
#define MILSTD_BC_TDESC_WORD1_TR_Pos               10                      /*!< Transmit/receive (TR) */
#define MILSTD_BC_TDESC_WORD1_RTAD1_Pos            11                      /*!< RT Address (RTAD1) */
#define MILSTD_BC_TDESC_WORD1_RTSA2_Pos            16                      /*!< Second RT Subaddress for RT-to-RT transfer (RTSA2) */
#define MILSTD_BC_TDESC_WORD1_RTAD2_Pos            21                      /*!< Second RT Address for RT-to-RT transfer (RTAD2) */
#define MILSTD_BC_TDESC_WORD1_RTTO_Pos             26                      /*!< RT Timeout (RTTO) - Extra RT status word timeout above nominal in units of 4 us */
#define MILSTD_BC_TDESC_WORD1_BUS_Pos              30                      /*!< Bus selection (BUS) - Bus to use for transfer, 0 - Bus A, 1 - Bus B */
#define MILSTD_BC_TDESC_WORD1_DUM_Pos              31                      /*!< Dummy transfer (DUM) */

/* Bit field masks: */
#define MILSTD_BC_TDESC_WORD1_WCMC_Msk             0x0000001FUL            /*!< Word count/Mode code (WCMC) */
#define MILSTD_BC_TDESC_WORD1_RTSA1_Msk            0x000003E0UL            /*!< RT Subaddress (RTSA1) */
#define MILSTD_BC_TDESC_WORD1_TR_Msk               0x00000400UL            /*!< Transmit/receive (TR) */
#define MILSTD_BC_TDESC_WORD1_RTAD1_Msk            0x0000F800UL            /*!< RT Address (RTAD1) */
#define MILSTD_BC_TDESC_WORD1_RTSA2_Msk            0x001F0000UL            /*!< Second RT Subaddress for RT-to-RT transfer (RTSA2) */
#define MILSTD_BC_TDESC_WORD1_RTAD2_Msk            0x03E00000UL            /*!< Second RT Address for RT-to-RT transfer (RTAD2) */
#define MILSTD_BC_TDESC_WORD1_RTTO_Msk             0x3C000000UL            /*!< RT Timeout (RTTO) - Extra RT status word timeout above nominal in units of 4 us */
#define MILSTD_BC_TDESC_WORD1_BUS_Msk              0x40000000UL            /*!< Bus selection (BUS) - Bus to use for transfer, 0 - Bus A, 1 - Bus B */
#define MILSTD_BC_TDESC_WORD1_DUM_Msk              0x80000000UL            /*!< Dummy transfer (DUM) */

/* Bit field enums: */
typedef enum {
  MILSTD_BC_TDESC_WORD1_RTTO_14us                          = 0x0UL,        /*!< RT Timeout 14us */
  MILSTD_BC_TDESC_WORD1_RTTO_18us                          = 0x1UL,        /*!< RT Timeout 18us */
  MILSTD_BC_TDESC_WORD1_RTTO_22us                          = 0x2UL,        /*!< RT Timeout 22us */
  MILSTD_BC_TDESC_WORD1_RTTO_26us                          = 0x3UL,        /*!< RT Timeout 26us */
  MILSTD_BC_TDESC_WORD1_RTTO_30us                          = 0x4UL,        /*!< RT Timeout 30us */
  MILSTD_BC_TDESC_WORD1_RTTO_34us                          = 0x5UL,        /*!< RT Timeout 34us */
  MILSTD_BC_TDESC_WORD1_RTTO_38us                          = 0x6UL,        /*!< RT Timeout 38us */
  MILSTD_BC_TDESC_WORD1_RTTO_42us                          = 0x7UL,        /*!< RT Timeout 42us */
  MILSTD_BC_TDESC_WORD1_RTTO_46us                          = 0x8UL,        /*!< RT Timeout 46us */
  MILSTD_BC_TDESC_WORD1_RTTO_50us                          = 0x9UL,        /*!< RT Timeout 50us */
  MILSTD_BC_TDESC_WORD1_RTTO_54us                          = 0xAUL,        /*!< RT Timeout 54us */
  MILSTD_BC_TDESC_WORD1_RTTO_58us                          = 0xBUL,        /*!< RT Timeout 58us */
  MILSTD_BC_TDESC_WORD1_RTTO_62us                          = 0xCUL,        /*!< RT Timeout 62us */
  MILSTD_BC_TDESC_WORD1_RTTO_66us                          = 0xDUL,        /*!< RT Timeout 66us */
  MILSTD_BC_TDESC_WORD1_RTTO_70us                          = 0xEUL,        /*!< RT Timeout 70us */
  MILSTD_BC_TDESC_WORD1_RTTO_74us                          = 0xFUL,        /*!< RT Timeout 74us */
} MILSTD_BC_TDESC_WORD1_RTTO_Enum;

typedef enum {
  MILSTD_BC_TDESC_WORD1_BUS_A                              = 0x0UL,        /*!< Bus to use for transfer - Bus A */
  MILSTD_BC_TDESC_WORD1_BUS_B                              = 0x1UL,        /*!< Bus to use for transfer - Bus B */
} MILSTD_BC_TDESC_WORD1_BUS_Enum;

/*--  Milstd Transfer descriptor Result Word configure word --------------------------------*/
typedef struct {
  uint32_t TFRST      :3;           /*!< Transfer status (TFRST) - Outcome of last try */
  uint32_t RES        :1;           /*!< Reserved - Mask away on read for forward compatibility */
  uint32_t RETCNT     :4;           /*!< Retry count (RETCNT) - Number of retries performed */
  uint32_t RTST       :8;           /*!< RT Status Bits (RTST) - Status bits from RT (transmitting RT in RT-to-RT transfer)*/
  uint32_t RT2ST      :8;           /*!< RT 2 Status Bits (RT2ST) - Status bits from receiving RT in RT-to-RT transfer, otherwise 0
                                      Same bit pattern as for RTST below */
  uint32_t RESERVED   :7;           /*!< Reserved - Mask away on read for forward compatibility */
  uint32_t ZERO       :1;           /*!< Always written as 0 */
} _MILSTD_BC_TDESC_RESULT_bits;

/* Bit field positions: */
#define MILSTD_BC_TDESC_RESULT_TFRST_Pos             0                      /*!< Transfer status (TFRST) - Outcome of last try */
#define MILSTD_BC_TDESC_RESULT_RETCNT_Pos            4                      /*!< Retry count (RETCNT) - Number of retries performed */
#define MILSTD_BC_TDESC_RESULT_RTST_Pos              8                      /*!< RT Status Bits (RTST) - Status bits from RT (transmitting RT in RT-to-RT transfer) */
#define MILSTD_BC_TDESC_RESULT_RT2ST_Pos             16                     /*!< RT 2 Status Bits (RT2ST) - Status bits from receiving RT in RT-to-RT transfer */
#define MILSTD_BC_TDESC_RESULT_ZERO_Pos              31                     /*!< Retry count (RETCNT) - Number of retries performed */

/* Bit field masks: */
#define MILSTD_BC_TDESC_RESULT_TFRST_Msk             0x00000007UL           /*!< Transfer status (TFRST) - Outcome of last try */
#define MILSTD_BC_TDESC_RESULT_RETCNT_Msk            0x000000F0UL           /*!< Retry count (RETCNT) - Number of retries performed */
#define MILSTD_BC_TDESC_RESULT_RTST_Msk              0x0000FF00UL           /*!< RT Status Bits (RTST) - Status bits from RT (transmitting RT in RT-to-RT transfer) */
#define MILSTD_BC_TDESC_RESULT_RT2ST_Msk             0x00FF0000UL           /*!< RT 2 Status Bits (RT2ST) - Status bits from receiving RT in RT-to-RT transfer */
#define MILSTD_BC_TDESC_RESULT_ZERO_Msk              0x80000000UL           /*!< Retry count (RETCNT) - Number of retries performed */

/* Bit field enums: */
typedef enum {
  MILSTD_BC_TDESC_RESULT_TFRST_Success                      = 0x0UL,        /*!< Success (or dummy bit was set) */
  MILSTD_BC_TDESC_RESULT_TFRST_Trans_RT_NotRespond          = 0x1UL,        /*!< RT did not respond (transmitting RT in RT-to-RT transfer) */
  MILSTD_BC_TDESC_RESULT_TFRST_Receiv_RT_NotRespond         = 0x2UL,        /*!< Receiving RT of RT-to-RT transfer did not respond */
  MILSTD_BC_TDESC_RESULT_TFRST_StatusWordError              = 0x3UL,        /*!< A responding RT:s status word had message error, busy, instrumentation or reserved bit set (*) */
  MILSTD_BC_TDESC_RESULT_TFRST_ProtocolError                = 0x4UL,        /*!< Protocol error (improperly timed data words, decoder error, wrong number of data words) */
  MILSTD_BC_TDESC_RESULT_TFRST_TD_Invalid                   = 0x5UL,        /*!< The transfer descriptor was invalid */
  MILSTD_BC_TDESC_RESULT_TFRST_Error                        = 0x6UL,        /*!< Data buffer DMA timeout or error response */
  MILSTD_BC_TDESC_RESULT_TFRST_Aborted                      = 0x7UL,        /*!< Transfer aborted due to loop back check failure */
} MILSTD_BC_TDESC_RESULT_TFRST_Enum;

typedef enum {
  MILSTD_BC_TDESC_RESULT_RTST_TerminalFlag                  = 0x8UL,        /*!< Terminal flag */
  MILSTD_BC_TDESC_RESULT_RTST_DynamicBusCtrl                = 0x9UL,        /*!< Dynamic bus control acceptance */
  MILSTD_BC_TDESC_RESULT_RTST_SubSystem                     = 0xAUL,        /*!< Subsystem flag */
  MILSTD_BC_TDESC_RESULT_RTST_Busy                          = 0xBUL,        /*!< Busy bit */
  MILSTD_BC_TDESC_RESULT_RTST_BroadcastCmd                  = 0xCUL,        /*!< Broadcast command received */
  MILSTD_BC_TDESC_RESULT_RTST_SrvReq                        = 0xDUL,        /*!< Service request */
  MILSTD_BC_TDESC_RESULT_RTST_InstrBit                      = 0xEUL,        /*!< Instrumentation bit or reserved bit set */
  MILSTD_BC_TDESC_RESULT_RTST_MsgError                      = 0xFUL,        /*!< Message error */
} MILSTD_BC_TDESC_RESULT_RTST_Enum;

/*--  Milstd BC Transfer descriptor structure --------------------------------------------------*/
typedef struct
{
    union {
        __IO uint32_t WORD0;                            /*!< Milstd Transfer descriptor Word 0 configure word */
        __IO _MILSTD_BC_TDESC_WORD0_bits WORD0_bit;     /*!< Milstd Transfer descriptor Word 0 configure word: bit access */
    };
    union {
        __IO uint32_t WORD1;                            /*!< Milstd Transfer descriptor Word 1 configure word */
        __IO _MILSTD_BC_TDESC_WORD1_bits WORD1_bit;     /*!< Milstd Transfer descriptor Word 1 configure word: bit access */
    };
    __IO uint32_t DATA_BUF_PTR;                         /*!< Milstd Transfer descriptor Data buffer pointer, 16-bit aligned.*/
    union {
        __IO uint32_t RESULT;                           /*!< Milstd Transfer descriptor Result Word configure word */
        __IO _MILSTD_BC_TDESC_RESULT_bits RESULT_bit;   /*!< Milstd Transfer descriptor Result Word configure word: bit access */
    };
} MILSTD_BC_TDESC_TypeDef;

/*--  Milstd branch condition Word Structure definition ----------------------------------*/
typedef struct {
  uint32_t STCC       :8;           /*!< Status Condition Code (STCC) - Mask with bits corresponding to status value of last transfer */
  uint32_t RTCC       :8;           /*!< RT Condition Code (RTCC) - Mask with bits corresponding to RTST in result word of last transfer */
  uint32_t RT2CC      :8;           /*!< RT 2 Condition Code (RT2CC) - Mask with bits corresponding to RT2ST in result word of last transfer */
  uint32_t MODE       :1;           /*!< Logic mode (MODE) */
  uint32_t ACT        :1;           /*!< Action (ACT) - What to do if condition is met, 0 - Suspend schedule, 1 - Jump */
  uint32_t IRQC       :1;           /*!< Interrupt if condition met (IRQC) */
  uint32_t RESERVED   :4;           /*!< Reserved - Set to 0 */
  uint32_t ONE        :1;           /*!< Must be 1 to identify as branch */
} _MILSTD_BC_BR_CONDWORD_bits;

/* Bit field positions: */
#define MILSTD_BC_BR_CONDWORD_STCC_Pos                   0                      /*!< Status Condition Code (STCC) - Mask with bits corresponding to status value of last transfer */
#define MILSTD_BC_BR_CONDWORD_RTCC_Pos                   8                      /*!< RT Condition Code (RTCC) - Mask with bits corresponding to RTST in result word of last transfer */
#define MILSTD_BC_BR_CONDWORD_RT2CC_Pos                  16                     /*!< RT 2 Condition Code (RT2CC) - Mask with bits corresponding to RT2ST in result word of last transfer */
#define MILSTD_BC_BR_CONDWORD_MODE_Pos                   24                     /*!< Logic mode (MODE) */
#define MILSTD_BC_BR_CONDWORD_ACT_Pos                    25                     /*!< Action (ACT) - What to do if condition is met, 0 - Suspend schedule, 1 - Jump  */
#define MILSTD_BC_BR_CONDWORD_IRQC_Pos                   26                     /*!< Interrupt if condition met (IRQC) */
#define MILSTD_BC_BR_CONDWORD_ONE_Pos                    31                     /*!< Must be 1 to identify as branch */

/* Bit field masks: */
#define MILSTD_BC_BR_CONDWORD_STCC_Msk                   0x00000007UL           /*!< Status Condition Code (STCC) - Mask with bits corresponding to status value of last transfer */
#define MILSTD_BC_BR_CONDWORD_RTCC_Msk                   0x000000F0UL           /*!< RT Condition Code (RTCC) - Mask with bits corresponding to RTST in result word of last transfer */
#define MILSTD_BC_BR_CONDWORD_RT2CC_Msk                  0x0000FF00UL           /*!< RT 2 Condition Code (RT2CC) - Mask with bits corresponding to RT2ST in result word of last transfer */
#define MILSTD_BC_BR_CONDWORD_MODE_Msk                   0x00FF0000UL           /*!< Logic mode (MODE)  */
#define MILSTD_BC_BR_CONDWORD_ACT_Msk                    0x80000000UL           /*!< Action (ACT) - What to do if condition is met, 0 - Suspend schedule, 1 - Jump */
#define MILSTD_BC_BR_CONDWORD_IRQC_Msk                   0x00FF0000UL           /*!< Interrupt if condition met (IRQC) */
#define MILSTD_BC_BR_CONDWORD_ONE_Msk                    0x80000000UL           /*!< Must be 1 to identify as branch */

/* Bit field enums: */
typedef enum {
  MILSTD_BC_BR_CONDWORD_MODE_Or      = 0x0UL,        /*!< Or mode (any bit set in RT2CC, RTCC is set in RT2ST,RTST, or result is in STCC mask) */
  MILSTD_BC_BR_CONDWORD_MODE_And     = 0x1UL,        /*!< And mode (all bits set in RT2CC,RTCC are set in RT2ST,RTST and result is in STCC mask) */
} MILSTD_BC_BR_CONDWORD_MODE_Enum;

/*--  Milstd BC Branch structure ----------------------------------*/
typedef struct {
    union {
        __IO uint32_t CONDWORD;                             /*!< Condition word */
        __IO _MILSTD_BC_BR_CONDWORD_bits CONDWORD_bit;      /*!< Condition word: bit access */
    };
    __IO uint32_t BRADDR;                                   /*!< Branch address, 128-bit aligned */
    __IO uint32_t RESERVED0;                                /*!< Reserved */
    __IO uint32_t RESERVED1;                                /*!< Reserved */
} MILSTD_RTD_TypeDef;

/*--  Milstd RT Subaddress table control word Structure definition ----------------------------------*/
typedef struct {
  uint32_t TXSZ       :5;           /*!< Maximum legal transmit size (TXSZ) from this subaddress - in 16-bit words, 0 means 32 */
  uint32_t TXIRQ      :1;           /*!< Interrupt on transmit transfers (TXIRQ) - Each transmit transfer will cause an interrupt (only if TXEN,TXLOG=1) */
  uint32_t TXLOG      :1;           /*!< Log transmit transfers (TXLOG) - Log all transmit transfers in event log ring (only if also TXEN=1) */
  uint32_t TXEN       :1;           /*!< Transmit enable (TXEN) - Allow transmit transfers from this subaddress */

  uint32_t RXSZ       :5;           /*!< Maximum legal receive size (RXSZ) from this subaddress - in 16-bit words, 0 means 32 */
  uint32_t RXIRQ      :1;           /*!< Interrupt on receive transfers (RXIRQ) - Each receive transfer will cause an interrupt (only if RXEN,RXLOG=1) */
  uint32_t RXLOG      :1;           /*!< Log receive transfers (RXLOG) - Log all receive transfers in event log ring (only if also RXEN=1) */
  uint32_t RXEN       :1;           /*!< Receive enable (RXEN) - Allow receive transfers to this subaddress */

  uint32_t BCRXE      :1;           /*!< Broadcast receive enable (BCRXEN) - Allow broadcast receive transfers to this subaddress */
  uint32_t IGNDV      :1;           /*!< Ignore data valid bit (IGNDV) - If this is �1� then receive transfers will proceed (and overwrite the buffer) if the receive
                                      descriptor has the data valid bit set, instead of not responding to the request.
                                      This can be used for descriptor rings where you don�t care if the oldest data is overwritten. */
  uint32_t WRAP       :1;           /*!< Auto-wraparound enable (WRAP) - Enables a test mode for this subaddress, where transmit transfers send back the
                                      last received data. This is done by copying the finished transfer�s descriptor pointer to the transmit descriptor pointer
                                      address after each successful transfer.
                                      Note: If WRAP=1, you should not set TXSZ > RXSZ as this might cause reading beyond buffer end */
  uint32_t ZERO       :13;          /*!< Reserved - set to 0 for forward compatibility */
} _MILSTD_RT_SAT_CTRL_TypeDef;

/* Bit field positions: */
#define MILSTD_RT_SAT_CTRL_TXSZ_Pos         0                   /*!< Maximum legal transmit size (TXSZ) from this subaddress - in 16-bit words, 0 means 32 */
#define MILSTD_RT_SAT_CTRL_TXIRQ_Pos        5                   /*!< Interrupt on transmit transfers (TXIRQ) - Each transmit transfer will cause an interrupt (only if TXEN,TXLOG=1) */
#define MILSTD_RT_SAT_CTRL_TXLOG_Pos        6                   /*!< Log transmit transfers (TXLOG) - Log all transmit transfers in event log ring (only if also TXEN=1) */
#define MILSTD_RT_SAT_CTRL_TXEN_Pos         7                   /*!< Transmit enable (TXEN) - Allow transmit transfers from this subaddress */
#define MILSTD_RT_SAT_CTRL_RXSZ_Pos         8                   /*!< Maximum legal receive size (RXSZ) from this subaddress - in 16-bit words, 0 means 32 */
#define MILSTD_RT_SAT_CTRL_RXIRQ_Pos        13                  /*!< Interrupt on receive transfers (RXIRQ) - Each receive transfer will cause an interrupt (only if RXEN,RXLOG=1) */
#define MILSTD_RT_SAT_CTRL_RXLOG_Pos        14                  /*!< Log receive transfers (RXLOG) - Log all receive transfers in event log ring (only if also RXEN=1) */
#define MILSTD_RT_SAT_CTRL_RXEN_Pos         15                  /*!< Receive enable (RXEN) - Allow receive transfers to this subaddress */
#define MILSTD_RT_SAT_CTRL_BCRXE_Pos        16                  /*!< Broadcast receive enable (BCRXEN) - Allow broadcast receive transfers to this subaddress */
#define MILSTD_RT_SAT_CTRL_IGNDV_Pos        17                  /*!< Ignore data valid bit (IGNDV) */
#define MILSTD_RT_SAT_CTRL_WRAP_Pos         18                  /*!< Auto-wraparound enable (WRAP)  */
#define MILSTD_RT_SAT_CTRL_ZERO_Pos         19                  /*!< Reserved - set to 0 for forward compatibility */

/* Bit field masks: */
#define MILSTD_RT_SAT_CTRL_TXSZ_Msk         0x0000001FUL        /*!< Maximum legal transmit size (TXSZ) from this subaddress - in 16-bit words, 0 means 32 */
#define MILSTD_RT_SAT_CTRL_TXIRQ_Msk        0x00000020UL        /*!< Interrupt on transmit transfers (TXIRQ) - Each transmit transfer will cause an interrupt (only if TXEN,TXLOG=1) */
#define MILSTD_RT_SAT_CTRL_TXLOG_Msk        0x00000040UL        /*!< Log transmit transfers (TXLOG) - Log all transmit transfers in event log ring (only if also TXEN=1) */
#define MILSTD_RT_SAT_CTRL_TXEN_Msk         0x00000080UL        /*!< Transmit enable (TXEN) - Allow transmit transfers from this subaddress */
#define MILSTD_RT_SAT_CTRL_RXSZ_Msk         0x00001F00UL        /*!< Maximum legal receive size (RXSZ) from this subaddress - in 16-bit words, 0 means 32 */
#define MILSTD_RT_SAT_CTRL_RXIRQ_Msk        0x00002000UL        /*!< Interrupt on receive transfers (RXIRQ) - Each receive transfer will cause an interrupt (only if RXEN,RXLOG=1) */
#define MILSTD_RT_SAT_CTRL_RXLOG_Msk        0x00004000UL        /*!< Log receive transfers (RXLOG) - Log all receive transfers in event log ring (only if also RXEN=1) */
#define MILSTD_RT_SAT_CTRL_RXEN_Msk         0x00008000UL        /*!< Receive enable (RXEN) - Allow receive transfers to this subaddress */
#define MILSTD_RT_SAT_CTRL_BCRXE_Msk        0x00010000UL        /*!< Broadcast receive enable (BCRXEN) - Allow broadcast receive transfers to this subaddress */
#define MILSTD_RT_SAT_CTRL_IGNDV_Msk        0x00020000UL        /*!< Ignore data valid bit (IGNDV) */
#define MILSTD_RT_SAT_CTRL_WRAP_Msk         0x00040000UL        /*!< Auto-wraparound enable (WRAP) */
#define MILSTD_RT_SAT_CTRL_ZERO_Msk         0xFFF80000UL        /*`!< Reserved - set to 0 for forward compatibility */

/*--  Milstd RT SubAddress Table Structure definition ----------------------------------*/
typedef struct {
    union {
        __IO uint32_t CTRL;                             /*!< RT Subaddress table control word */
        __IO _MILSTD_RT_SAT_CTRL_TypeDef CTRL_bit;      /*!< RT Subaddress table control word: bit access */
    };
    __IO uint32_t TDSCR;                                /*!< Transmit descriptor pointer, 16-byte aligned (0x3 to indicate invalid pointer) */
    __IO uint32_t RDSCR;                                /*!< Receive descriptor pointer, 16-byte aligned (0x3 to indicate invalid pointer) */
    __IO uint32_t RESERVED;                             /*!< Reserved */
} MILSTD_RT_SAT_TypeDef;

typedef struct
{
    MILSTD_RT_SAT_TypeDef SAT[32];  /*!< RT Subaddress table array */
} MILSTD_RT_SAT_Arr_TypeDef;

/*--  Milstd RT Event Log entry format Structure definition ----------------------------------*/
typedef struct {
  uint32_t TRES     :3;         /*!< Transfer result (TRES) */
  uint32_t SZ       :6;         /*!< Transfer size (SZ) - Count in 16-bit words (0-32) */
  uint32_t BC       :1;         /*!< Broadcast (BC) - Set to 1 if request was to the broadcast address */
  uint32_t TIMEL    :14;        /*!< Low 14 bits of time tag counter */
  uint32_t SAMC     :5;         /*!< Subaddress / Mode code (SAMC) -
                                    If TYPE=00/01 this is the transfer subaddress,
                                    If TYPE=10, this is the mode code */
  uint32_t TYPE     :2;         /*!< Transfer type (TYPE) */
  uint32_t IRQSR    :1;         /*!< IRQ Source (IRQSRC) - Set to �1� if this transfer caused an interrup */
} MILSTD_RT_LOG_TypeDef;

/* Bit field positions: */
#define MILSTD_RT_LOG_TRES_Pos      0                   /*!< Transfer result (TRES) */
#define MILSTD_RT_LOG_SZ_Pos        3                   /*!< Transfer size (SZ) - Count in 16-bit words (0-32) */
#define MILSTD_RT_LOG_BC_Pos        9                   /*!< Broadcast (BC) - Set to 1 if request was to the broadcast address */
#define MILSTD_RT_LOG_TIMEL_Pos     10                  /*!< Low 14 bits of time tag counter */
#define MILSTD_RT_LOG_SAMC_Pos      24                  /*!< Subaddress / Mode code (SAMC) */
#define MILSTD_RT_LOG_TYPE_Pos      29                  /*!< Transfer type (TYPE) */
#define MILSTD_RT_LOG_IRQSR_Pos     31                  /*!< IRQ Source (IRQSRC) - Set to �1� if this transfer caused an interrup */

/* Bit field masks: */
#define MILSTD_RT_LOG_TRES_Msk      0x00000007UL        /*!< Transfer result (TRES) */
#define MILSTD_RT_LOG_SZ_Msk        0x000001F8UL        /*!< Transfer size (SZ) - Count in 16-bit words (0-32) */
#define MILSTD_RT_LOG_BC_Msk        0x00000200UL        /*!< Broadcast (BC) - Set to 1 if request was to the broadcast address */
#define MILSTD_RT_LOG_TIMEL_Msk     0x00FFFC00UL        /*!< Low 14 bits of time tag counter */
#define MILSTD_RT_LOG_SAMC_Msk      0x1F000000UL        /*!< Subaddress / Mode code (SAMC) */
#define MILSTD_RT_LOG_TYPE_Msk      0x60000000UL        /*!< Transfer type (TYPE) */
#define MILSTD_RT_LOG_IRQSR_Msk     0x80000000UL        /*!< IRQ Source (IRQSRC) - Set to �1� if this transfer caused an interrup */

/* Bit field enums: */
typedef enum {
  MILSTD_RT_LOG_TRES_Success        = 0x0UL,      /*!< Success */
  MILSTD_RT_LOG_TRES_Superseded     = 0x1UL,      /*!< Superseded (canceled because a new command was given on the other bus) */
  MILSTD_RT_LOG_TRES_DMAErr         = 0x2UL,      /*!< DMA error or memory timeout occurred */
  MILSTD_RT_LOG_TRES_ProtocolErr    = 0x3UL,      /*!< Protocol error (improperly timed data words or decoder error) */
  MILSTD_RT_LOG_TRES_MsgErr         = 0x4UL,      /*!< The busy bit or message error bit was set in the transmitted status word and no data was sent */
  MILSTD_RT_LOG_TRES_Aborted        = 0x5UL,      /*!< Transfer aborted due to loop back checker error */
} MILSTD_RT_LOG_TRES_Enum;

typedef enum {
  MILSTD_RT_LOG_TYPE_TxData         = 0x0UL,      /*!< Transmit data */
  MILSTD_RT_LOG_TYPE_RxData         = 0x1UL,      /*!< Receive data */
  MILSTD_RT_LOG_TYPE_ModeCode       = 0x2UL,      /*!< Mode code */
} MILSTD_RT_LOG_TYPE_Enum;

/*--  Milstd RT Descriptor control/status word Structure definition ----------------------------------*/
typedef struct {
  uint32_t TRES       :3;           /*!< Transfer result (TRES)
                                      000 = Success
                                      001 = Superseded (canceled because a new command was given on the other bus)
                                      010 = DMA error or memory timeout occurred
                                      011 = Protocol error (improperly timed data words or decoder error)
                                      100 = The busy bit or message error bit was set in the transmitted status word and no data was sent
                                      101 = Transfer aborted due to loop back checker error */
  uint32_t SZ         :6;           /*!< Transfer size (SZ) - Count in 16-bit words (0-32) */
  uint32_t BC         :1;           /*!< Broadcast (BC) - Set by the core if the transfer was a broadcast transfer */
  uint32_t TIME       :16;          /*!< Transmission time tag (TTIME) - Set by the core to the value of the RT timer when the transfer finished. */
  uint32_t RESERVED   :4;           /*!< Reserved - Write 0 and mask out on read for forward compatibility */
  uint32_t IRQEN      :1;           /*!< IRQ Enable override (IRQEN) - Log and IRQ after transfer regardless of SA control word settings
                                      Can be used for getting an interrupt when nearing the end of a descriptor list. */
  uint32_t DV         :1;           /*!< Data valid (DV) - Should be set to 0 by software before and set to 1 by hardware after transfer.
                                      If DV=1 in the current receive descriptor before the receive transfer begins then a descriptor table error will
                                      be triggered. You can override this by setting the IGNDV bit in the subaddress table. */
} _MILSTD_RT_DESC_CTRLSTAT_TypeDef;

/* Bit field positions: */
#define MILSTD_RT_DESC_CTRLSTAT_TRES_Pos        0               /*!< Transfer result (TRES) */
#define MILSTD_RT_DESC_CTRLSTAT_SZ_Pos          3               /*!< Transfer size (SZ) - Count in 16-bit words (0-32) */
#define MILSTD_RT_DESC_CTRLSTAT_BC_Pos          9               /*!< Broadcast (BC) - Set by the core if the transfer was a broadcast transfer */
#define MILSTD_RT_DESC_CTRLSTAT_TIME_Pos        10              /*!< Transmission time tag (TTIME) - Set by the core to the value of the RT timer when the transfer finished. */
#define MILSTD_RT_DESC_CTRLSTAT_IRQEN_Pos       30              /*!< IRQ Enable override (IRQEN) */
#define MILSTD_RT_DESC_CTRLSTAT_DV_Pos          31              /*!< Data valid (DV) */

/* Bit field masks: */
#define MILSTD_RT_DESC_CTRLSTAT_TRES_Msk        0x00000003UL    /*!< Transfer result (TRES) */
#define MILSTD_RT_DESC_CTRLSTAT_SZ_Msk          0x000001FCUL    /*!< Transfer size (SZ) - Count in 16-bit words (0-32) */
#define MILSTD_RT_DESC_CTRLSTAT_BC_Msk          0x00000200UL    /*!< Broadcast (BC) - Set by the core if the transfer was a broadcast transfer */
#define MILSTD_RT_DESC_CTRLSTAT_TIME_Msk        0x03FFFC00UL    /*!< Transmission time tag (TTIME) - Set by the core to the value of the RT timer when the transfer finished. */
#define MILSTD_RT_DESC_CTRLSTAT_IRQEN_Msk       0x40000000UL    /*!< IRQ Enable override (IRQEN) */
#define MILSTD_RT_DESC_CTRLSTAT_DV_Msk          0x80000000UL    /*!< Data valid (DV) */

/* Bit field enums: */
typedef enum {
  MILSTD_RT_DESC_CTRLSTAT_TRES_Success        = 0x0UL,      /*!< Success */
  MILSTD_RT_DESC_CTRLSTAT_TRES_Superseded     = 0x1UL,      /*!< Superseded (canceled because a new command was given on the other bus) */
  MILSTD_RT_DESC_CTRLSTAT_TRES_DMAErr         = 0x2UL,      /*!< DMA error or memory timeout occurred */
  MILSTD_RT_DESC_CTRLSTAT_TRES_ProtocolErr    = 0x3UL,      /*!< Protocol error (improperly timed data words or decoder error) */
  MILSTD_RT_DESC_CTRLSTAT_TRES_MsgErr         = 0x4UL,      /*!< The busy bit or message error bit was set in the transmitted status word and no data was sent */
  MILSTD_RT_DESC_CTRLSTAT_TRES_Aborted        = 0x5UL,      /*!< Transfer aborted due to loop back checker error */
} MILSTD_RT_DESC_CTRLSTAT_TRES_Enum;

/*--  Milstd RT Descriptor format Structure definition ----------------------------------*/
typedef struct {
    union {
        __IO uint32_t CTRLSTAT;                                  /*!< Control and status word */
        __IO _MILSTD_RT_DESC_CTRLSTAT_TypeDef CTRLSTAT_bit;      /*!< Control and status word: bit access */
    };
    __IO uint32_t DBUFF_PTR;                                     /*!< Data buffer pointer, 16-bit aligned */
    __IO uint32_t NEXT_DESCR_PTR;                                /*!< Pointer to next descriptor, 16-byte aligned or 0x0000003 to indicate end of list */
	__IO uint32_t RESERVED;
} MILSTD_RT_DESC_TypeDef;

/*--  Milstd BM Log entry word 0 Structure definition ----------------------------------*/
typedef struct {
  uint32_t TIME         :24;        /*!< Time tag (TIME) */
  uint32_t RESERVED     :7;         /*!< Reserved - Mask out on read for forward compatibility */
  uint32_t ONE          :1;         /*!< Always written as 1 */
} _BM_LOG_WORD0_TypeDef;

/* Bit field positions: */
#define MILSTD_BM_LOG_WORD0_TIME_Pos        0               /*!< Time tag (TIME) */
#define MILSTD_BM_LOG_WORD0_ONE_Pos         31              /*!< Always written as 1 */

/* Bit field masks: */
#define MILSTD_BM_LOG_WORD0_TIME_Msk        0x00FFFFFUL     /*!< Time tag (TIME) */
#define MILSTD_BM_LOG_WORD0_ONE_Msk         0x8000000UL     /*!< Always written as 1 */

/*--  Milstd BM Log entry word 1 Structure definition ----------------------------------*/
typedef struct {
  uint32_t WD           :16;    /*!< Word data (WD) */
  uint32_t WTP          :1;     /*!< Word type (WTP) */
  uint32_t WST          :2;     /*!< Word status (WST) - 00=word OK, 01=Manchester error, 10=Parity error */
  uint32_t BUS          :1;     /*!< Receive data bus (BUS) - 0:A, 1:B */
  uint32_t RESERVED     :11;    /*!< Reserved - Mask out on read for forward compatibility */
  uint32_t ZERO         :1;     /*!< Always written as 0 */
} _BM_LOG_WORD1_TypeDef;

/* Bit field positions: */
#define MILSTD_BM_LOG_WORD1_WD_Pos      0               /*!< Word data (WD)*/
#define MILSTD_BM_LOG_WORD1_WTP_Pos     16              /*!< Word type (WTP) */
#define MILSTD_BM_LOG_WORD1_WST_Pos     17              /*!< Word status (WST) */
#define MILSTD_BM_LOG_WORD1_BUS_Pos     19              /*!< Receive data bus (BUS) */
#define MILSTD_BM_LOG_WORD1_ZERO_Pos    31              /*!< Always written as 0 */

/* Bit field masks: */
#define MILSTD_BM_LOG_WORD1_WD_Msk      0x0000FFFFUL    /*!< Word data (WD)*/
#define MILSTD_BM_LOG_WORD1_WTP_Msk     0x00010000UL    /*!< Word type (WTP) */
#define MILSTD_BM_LOG_WORD1_WST_Msk     0x00060000UL    /*!< Word status (WST) */
#define MILSTD_BM_LOG_WORD1_BUS_Msk     0x00080000UL    /*!< Receive data bus (BUS) */
#define MILSTD_BM_LOG_WORD1_ZERO_Msk    0x80000000UL    /*!< Always written as 0 */

/* Bit field enums: */
typedef enum {
  MILSTD_BM_LOG_WORD1_WTP_Data        = 0x0UL,    /*!< Data */
  MILSTD_BM_LOG_WORD1_WTP_CmdStatus   = 0x1UL     /*!< Command/status */
} MILSTD_BM_LOG_WORD1_WTP_Enum;

typedef enum {
  MILSTD_BM_LOG_WORD1_WST_OK          = 0x0UL,    /*!< Word OK */
  MILSTD_BM_LOG_WORD1_WST_ManchErr    = 0x1UL,    /*!< Manchester error */
  MILSTD_BM_LOG_WORD1_WST_ParityErr   = 0x2UL     /*!< Parity error */
} MILSTD_BM_LOG_WORD1_WST_Enum;

typedef enum {
  MILSTD_BM_LOG_WORD1_BUS_A           = 0x0UL,    /*!< Bus to use for transfer - Bus A */
  MILSTD_BM_LOG_WORD1_BUS_B           = 0x1UL,    /*!< Bus to use for transfer - Bus B */
} MILSTD_BM_LOG_WORD1_BUS_Enum;

typedef struct {
    union {
        __IO uint32_t WORD0;                       /*!< BM Log entry word 0 */
        __IO _BM_LOG_WORD0_TypeDef WORD0_bit;      /*!< BM Log entry word 0: bit access */
    };
    union {
        __IO uint32_t WORD1;                       /*!< BM Log entry word 1 */
        __IO _BM_LOG_WORD1_TypeDef WORD1_bit;      /*!< BM Log entry word 1: bit access */
    };
} MILSTD_BM_LOG_TypeDef;

//
#ifdef __cplusplus
}
#endif

#endif /* __1888BC048_MILSTD_H */
/**
  * @}
  */
  
