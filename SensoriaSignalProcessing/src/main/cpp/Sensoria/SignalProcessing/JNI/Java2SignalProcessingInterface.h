//
// Created by zep on 23/05/18.
//

#ifndef SIGNALPROCESSING_JAVA2SIGNALPROCESSINGINTERFACE_H
#define SIGNALPROCESSING_JAVA2SIGNALPROCESSINGINTERFACE_H

#define SIGNALPROCESSING_PROFILE_GAIT_ID "gait"

#include "../../../../../../../app/src/main/cpp/Utils.h"

namespace Sensoria {
    namespace SignalProcessing {
        namespace JNI {
            class Java2SignalProcessingInterface {
            public:
                void register_client(sp_uuid_t *uuid,const std::string &)const;
            };
        }
    }
}


#endif //SIGNALPROCESSING_JAVA2SIGNALPROCESSINGINTERFACE_H
