// ISignalProcessing.aidl
package com.sensoriainc.exnativeapp;

// Declare any non-default types here with import statements

import android.os.ParcelUuid;
import com.sensoriainc.exnativeapp.ISignalProcessingCallback;
import com.sensoriainc.exnativeapp.ISignalProcessingServerCallback;

interface ISignalProcessing {

    void registerClient(in ParcelUuid uuid,in ISignalProcessingCallback callback);
    void unregisterClient(in int clientIf);
    void clientConnect(in int clientIf);
    void clientDisconnect(in int clientIf);
    void registerForNotification(in int clientIf,in int handle,in boolean enable);
    void registerServer(in ParcelUuid uuid,in ISignalProcessingServerCallback callback);
    void unregisterServer(in int serverIf);
    void serverConnect(in int serverIf);
    void serverDisconnect(in int serverIf);
}
