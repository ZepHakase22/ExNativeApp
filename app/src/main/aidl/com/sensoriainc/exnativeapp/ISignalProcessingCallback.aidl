// ISignalProcessingCallback.aidl
package com.sensoriainc.exnativeapp;

// Declare any non-default types here with import statements

interface ISignalProcessingCallback {
    void onClientRegistered(in int status, in int clientIf);
}
