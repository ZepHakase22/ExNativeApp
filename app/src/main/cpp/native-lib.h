//
// Created by zep on 12/05/18.
//

#include <jni.h>
#include "hello.h"

#ifndef EXNATIVEAPP_NATIVE_LIB_H
#define EXNATIVEAPP_NATIVE_LIB_H
#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/**
 * Class:       MainActivity
 * Method:      stringFromJNI
 * Signature:   ()Ljava/lang/String;
 */
JNIEXPORT jstring JNICALL Java_com_sensoriainc_exnativeapp_MainActivity_stringFromJNI
        (JNIEnv *, jobject);

/**
 * Class:       MainActivity
 * Method:      setEvent
 * Signature    ()V
 */

JNIEXPORT void JNICALL
Java_com_sensoriainc_exnativeapp_MainActivity_setEvent
        (JNIEnv *, jobject);

class EventHandler: public HelloEvents {
private:
    void setHello(std::string &);
    JNIEnv *m_env;
    jobject m_obj;
    jclass m_cls;
    jmethodID m_setHelloMid;

public:
    Hello *m_objHello;

    EventHandler();
    EventHandler(JNIEnv *env, jobject obj);
    ~EventHandler();

};

#ifdef __cplusplus
}
#endif //__cplusplus
#endif //EXNATIVEAPP_NATIVE_LIB_H
