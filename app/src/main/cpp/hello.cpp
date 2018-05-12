//
// Created by zep on 12/05/18.
//

#include "hello.h"
using namespace std;

Hello::Hello() {
    this->m_objHelloEvents=0L;
    this->cMessage="Hello world from C code";
    this->cEventMessage="Hello world from C Event Code";
}

Hello::Hello(HelloEvents *objHelloEvents):Hello() {
    this->m_objHelloEvents=objHelloEvents;
}

std::string Hello::get_hello() {
    return this->cMessage;
}

void Hello::startEvent() {
    if(m_objHelloEvents) {
        m_objHelloEvents->setHello(this->cEventMessage);
    }
}

