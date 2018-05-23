#include <jni.h>
#include <string>
#include "native-lib.h"
#include "hello.h"
#include "Utils.h"

static const Utils *utils=NULL;
static const SignalProcessingInterface spIf=NULL;

JNIEXPORT void JNICALL
Java_com_sensoriainc_exnativeapp_SignalProcessingService_classInitCallbackNative(
        JNIEnv *env,jobject instance) {


}
JNIEXPORT jboolean JNICALL
Java_com_sensoriainc_exnativeapp_SignalProcessingService_cleanupNative(
        JNIEnv *env, jobject instance) {


}
JNIEXPORT void JNICALL
Java_com_sensoriainc_exnativeapp_SignalProcessingService_signalProcessingRegisterAppNative(
        JNIEnv *env, jobject instance, jlong app_uuid_lsb, jlong app_uuid_msb) {

    if(utils==NULL)
        utils=new Utils();

    sp_uuid_t uuid;

    utils->set_uuid(uuid.uu,app_uuid_lsb,app_uuid_msb);
    sSignalProcessing->client->register_client(&uuid);

}

JNIEXPORT jstring JNICALL
Java_com_sensoriainc_exnativeapp_MainActivity_stringFromJNI(
        JNIEnv *env,
        jobject /* this */) {
    Hello hello;
    return env->NewStringUTF(hello.get_hello().c_str());
}
JNIEXPORT void JNICALL
Java_com_sensoriainc_exnativeapp_MainActivity_setEvent(JNIEnv *env, jobject obj)
{
    EventHandler evh(env,obj);
    evh.m_objHello->startEvent();
}

void EventHandler::setHello(std::string &msg) {
    if(m_setHelloMid == 0)
        return;
    m_env->CallVoidMethod(m_obj,m_setHelloMid,m_env->NewStringUTF(msg.c_str()));
}

EventHandler::EventHandler() {
    m_objHello=new Hello(this);
}

EventHandler::~EventHandler() {
    delete m_objHello;
}

EventHandler::EventHandler(JNIEnv *env, jobject obj) : EventHandler() {
    m_env=env;
    m_obj=obj;
    m_cls = env->GetObjectClass(obj);
    m_setHelloMid = env->GetMethodID(m_cls, "callbackHello", "(Ljava/lang/String;)V");
}

