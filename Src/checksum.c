#include "checksum.h"

/* CRC16-CCITT algorithm */
bool IsChecksumm16bCorrect(uint8_t *msg, uint16_t length)
{
	int i;
	uint16_t crc = 0, crc_got = (uint16_t)((msg[length - 2] << 8) + msg[length - 1]);

	for(i = 0; i < length - 2; ++i) {
		crc = (uint8_t)(crc >> 8) | (crc << 8);
		crc ^= msg[i];
		crc ^= (uint8_t)(crc & 0xff) >> 4;
		crc ^= (crc << 8) << 4;
		crc ^= ((crc & 0xff) << 4) << 1;
	}

	if(crc == crc_got ) {
		return 1;
	}
	else {
		return 0;
	}
}

/* CRC16-CCITT algorithm */
void AddChecksumm16b(uint8_t *msg, uint16_t length)
{
    uint16_t crc = 0;
    int i = 0;

    for(i = 0; i < length - 2; ++i){
            crc = (uint8_t)(crc >> 8) | (crc << 8);
            crc ^= msg[i];
            crc ^= (uint8_t)(crc & 0xff) >> 4;
            crc ^= (crc << 8) << 4;
            crc ^= ((crc & 0xff) << 4) << 1;
        }

    msg[length - 2] = (uint8_t) (crc >> 8);
    msg[length - 1] = (uint8_t) crc;
}

bool IsChecksumm8bCorrect(uint8_t *msg, uint16_t length)
{
    uint8_t crcGot, crc = 0;
    int i;

    crcGot = msg[length-1] ;

        for(i=0; i < length - 1; i++){
            crc ^= msg[i];
        }

    if(crc == crcGot)
        return 1;
    else return 0;
}

void AddChecksumm8b(uint8_t *msg, uint16_t length)
{
	uint8_t crc = 0;
	int i = 0;

	for(i=0; i < length - 1; i++) {
		crc ^= msg[i];
	}

	msg[length-1] = crc;
}

bool IsChecksumm8bCorrectVma(uint8_t *msg, uint16_t length)
{
	uint8_t crcGot, crc = 0;
	int i;

	crcGot = msg[length-1] ;

	for (i = 1; i < length - 1; ++i) {
		crc ^= msg[i];
	}

	if (crc == crcGot) {
		return 1;
	}
	else {
		return 0;
	}
}

void AddChecksumm8bVma(uint8_t *msg, uint16_t length)
{
	uint8_t crc = 0;

	for(int i = 1; i < length - 1; i++) {
		crc ^= msg[i];
	}

	msg[length-1] = crc;
}

void AddChecksum16bS(uint8_t *msg, uint16_t length)
{
    uint16_t checksum = 0;
  for(uint8_t i = 0; i < length; ++i) {
    checksum += msg[i];
  }
    msg[length - 2] = (uint8_t) (checksum >> 8);
    msg[length - 1] = (uint8_t) checksum;
}

bool IsChecksum16bSCorrect(uint8_t *msg, uint16_t length)
{
    uint16_t checksum = 0, checksum_in = (uint16_t) ((msg[length - 2] << 8) + (msg[length - 1]));
  for(uint8_t i = 0; i < length-2; ++i) {
    checksum += msg[i];
  }
    if(checksum == checksum_in) {
        return true;
    }
    else {
        return false;
    }
}

void CompChecksum(uint8_t *upbyte, uint8_t *lowbyte, uint8_t *msg, uint8_t size)
{
    uint16_t checksum = 0;
    for(uint8_t i=0; i<size; i++)
        checksum += (uint16_t) msg[i];

    *lowbyte = (uint8_t) ((checksum & 0xFF00) >> 8);
    *upbyte = (uint8_t) (checksum & 0x00FF);
}

int16_t MergeBytes(uint8_t most, uint8_t least)
{
	return ((int16_t) most << 8) | least;
}

uint16_t MergeUBytes(uint8_t most, uint8_t least)
{
	return ((uint16_t) most << 8) | least;
}

float FloatFromUint8(uint8_t *buff, uint8_t high_byte_pos)
{
	return (float) ((buff[high_byte_pos] << 24) | (buff[high_byte_pos + 1] << 16) | (buff[high_byte_pos + 2] << 8) | buff[high_byte_pos + 3]);
}

void Uint8FromFloat(float input, uint8_t *outArray)
{
	uint8_t *d = (uint8_t *) &input;

	outArray[0] = *d;
	outArray[1] = *(d + 1);
	outArray[2] = *(d + 2);
	outArray[3] = *(d + 2);
}

void nullIntArray(uint8_t *array, uint8_t size)
{
    for (uint8_t i = 0; i < size; ++i) {
        array[i] = 0x00;
    }
}

bool PickBit(uint8_t input, uint8_t bit)
{
	return (bool) ((input << (8 - bit)) >> 8);
}

void SetBit(uint8_t *byte, uint8_t bit, bool state)
{
	*byte = (*byte & ~(1 << bit)) | ((uint8_t) state << bit);
}

void writeBit(uint8_t *byte, uint8_t value, uint8_t biteNumb)
{
    if (value == 0){
        *byte &= ~(1 << biteNumb);
    }
    else{
        *byte |= (1 << biteNumb);
    }
}

uint8_t ComputeChecksum8b(uint8_t *msg, uint16_t length)
{
    uint8_t crc = 0;
    int i = 0;

    for(i=0; i < length - 1; i++) {
            crc ^= msg[i];
        }

    return crc;
}
