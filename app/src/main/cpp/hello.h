//
// Created by zep on 12/05/18.
//

#include <string>

#ifndef EXNATIVEAPP_HELLO_H
#define EXNATIVEAPP_HELLO_H


class HelloEvents;

class Hello {


private:
    HelloEvents *m_objHelloEvents;
    std::string cMessage;
    std::string cEventMessage;

public:
    /**
     * .ctor
     */
    Hello();
    Hello(HelloEvents *);

    std::string get_hello();
    void startEvent();

};

class HelloEvents {
public:
    virtual void setHello(std::string &)= 0;
};

#endif //EXNATIVEAPP_HELLO_H