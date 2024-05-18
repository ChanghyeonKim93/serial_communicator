#ifndef CRC16_H_
#define CRC16_H_

#include <stdio.h>
#include <iostream>

unsigned short CalculateChecksumCRC16CCITT(const unsigned char* buf,
                                           const int length);

#endif /* _CRC16_H_ */