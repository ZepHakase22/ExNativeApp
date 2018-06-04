// ISignalProcessing.aidl
package com.sensoriainc.sensoriasignalprocessing;

// Declare any non-default types here with import statements

import android.os.ParcelUuid;
import com.sensoriainc.sensoriasignalprocessing.IGaitSignalProcessingCallback;

interface IGaitSignalProcessing {

    void registerClient(in ParcelUuid uuid,in IGaitSignalProcessingCallback callback);
    void unregisterClient(in int clientIf);
    void clientConnect(in int clientIf);
    void clientDisconnect(in int clientIf);
    void registerForNotification(in int clientIf,in int handle,in boolean enable);
}
