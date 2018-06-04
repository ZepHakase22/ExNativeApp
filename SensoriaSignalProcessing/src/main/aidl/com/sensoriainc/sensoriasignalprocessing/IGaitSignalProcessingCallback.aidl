// ISignalProcessingCallback.aidl
package com.sensoriainc.sensoriasignalprocessing;

// Declare any non-default types here with import statements

interface IGaitSignalProcessingCallback {
    void onClientRegistered(in int status, in int clientIf);
}
