//
// Created by zep on 21/05/18.
//

#include "Utils.h"


void Utils::set_uuid(uint8_t* uuid, jlong uuid_msb, jlong uuid_lsb) const {
    for (int i = 0; i != 8; ++i) {
        uuid[i] = (uuid_lsb >> (8 * i)) & 0xFF;
        uuid[i + 8] = (uuid_msb >> (8 * i)) & 0xFF;
    }
}
