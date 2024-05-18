#ifndef _CRC16_H_
#define _CRC16_H_
#include <stdio.h>
#include <iostream>

unsigned short CalculateChecksumCRC16CCITT(const unsigned char* buf,
                                           int idx_start, int idx_end);

#endif /* _CRC16_H_ */