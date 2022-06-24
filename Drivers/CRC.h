#ifndef CRC16_H
#define CRC16_H

unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
unsigned int Verify_CRC16_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC16_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
unsigned int UNVerify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
unsigned int UNVerify_CRC16_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);


#endif

