#ifndef _SPI_DRV_H_
#define _SPI_DRV_H_

#define BitSet(p,m) ((p) |= (1<<(m)))
#define BitReset(p,m) ((p) &= ~(1<<(m)))
#define BitFlip(p,m) ((p) ^= (m))
#define BitWrite(c,p,m) ((c) ? BitSet(p,m) : BitReset(p,m))
#define BitIsSet(reg, bit) (((reg) & (1<<(bit))) != 0)
#define BitIsReset(reg, bit) (((reg) & (1<<(bit))) == 0)

void SPI_Init();
void SPI_DeInit();
uint16_t SPI_Send(uint16_t data);

#endif //_SPI_DRV_H_
