#include <jni.h>
#include <string>
#include "native-lib.h"
#include "hello.h"

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
