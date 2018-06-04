#include <jni.h>
#include <string>
#include "native-lib.h"
#include "Utils.h"
#include "Java2SignalProcessingInterface.h"

static const Utils *utils=NULL;

using namespace Sensoria::SignalProcessing::JNI;

static const Java2SignalProcessingInterface *spi=NULL;

JNIEXPORT void JNICALL
Java_com_sensoriainc_exnativeapp_SignalProcessingService_classInitCallbackNative(
        JNIEnv *env,jobject instance) {


}
JNIEXPORT jboolean JNICALL
Java_com_sensoriainc_exnativeapp_SignalProcessingService_cleanupNative(
        JNIEnv *env, jobject instance) {


}
JNIEXPORT void JNICALL
Java_com_sensoriainc_exnativeapp_SignalProcessingService_gaitRegisterAppNative(
        JNIEnv *env, jobject instance, jlong app_uuid_lsb, jlong app_uuid_msb) {

    if(utils==NULL)
        utils=new Utils();

    sp_uuid_t uuid;

    utils->set_uuid(uuid.uu,app_uuid_lsb,app_uuid_msb);
    if(spi==NULL) {
        spi=new Java2SignalProcessingInterface();
    }
    spi->register_client(&uuid, SIGNALPROCESSING_PROFILE_GAIT_ID);

}


EventHandler::EventHandler() {
//    m_objHello=new Hello(this);
}

EventHandler::~EventHandler() {
//    delete m_objHello;
}

EventHandler::EventHandler(JNIEnv *env, jobject obj) : EventHandler() {
    m_env=env;
    m_obj=obj;
    m_cls = env->GetObjectClass(obj);
    m_setHelloMid = env->GetMethodID(m_cls, "callbackHello", "(Ljava/lang/String;)V");
}

