//
// Created by zep on 21/05/18.
//

#ifndef EXNATIVEAPP_UTILS_H
#define EXNATIVEAPP_UTILS_H

#include <cstdint>
#include <jni.h>

typedef struct {
    uint8_t uu[16];
} sp_uuid_t;

class Utils {
public:
    void set_uuid(uint8_t* uuid, jlong uuid_msb, jlong uuid_lsb) const;
};


#endif //EXNATIVEAPP_UTILS_H
