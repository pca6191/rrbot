/*
 * util_crc.h
 *
 *  Created on: 2020年12月25日
 *      Author: kc.chang
 */

#ifndef  MY_REACHTRUCK_INCLUDE_UTIL_CRC_H_
#define  MY_REACHTRUCK_INCLUDE_UTIL_CRC_H_

#include <stdint.h>
// ---------------------------------------------------------------------------
// Initialization values for CRC high and low byte accumulators
// ---------------------------------------------------------------------------
#define CRC_HI      0xFF
#define CRC_LO      0xFF

// pilot2_Bingsyun_0029 start 改使用耗時更少的硬體CRC，原運算法持續於軟體模擬使用

#define CRCLEN      2
namespace amr {
uint16_t util_crc_calc(uint8_t *pdata, uint16_t ncount);
}
#endif  /* MY_REACHTRUCK_INCLUDE_UTIL_CRC_H_ */
