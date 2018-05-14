package com.sensoriainc.exnativeapp;

import android.os.Handler;
import android.util.Log;

public class SignalProcessing {
    private static final String TAG="SignalProcessing";
    private int mConnState;
    private final Object mStateLock = new Object();

    private static final int CONN_STATE_IDLE = 0;
    private static final int CONN_STATE_CONNECTING = 1;
    private static final int CONN_STATE_CONNECTED = 2;
    private static final int CONN_STATE_DISCONNECTING = 3;
    private static final int CONN_STATE_CLOSED = 4;

    SignalProcessing() {
        mConnState = CONN_STATE_IDLE;
    }
    public boolean connect(SignalProcessingCallback callback, Handler handler) {
        Log.d(TAG,"connect() to SignalProcessing");

        synchronized(mStateLock) {
            if (mConnState != CONN_STATE_IDLE) {
                throw new IllegalStateException("Not idle");
            }
            mConnState = CONN_STATE_CONNECTING;
        }

/*        if (!registerApp(callback, handler)) {
            synchronized(mStateLock) {
                mConnState = CONN_STATE_IDLE;
            }
            Log.e(TAG, "Failed to register callback");
            return false;
        }
*/
        // The connection will continue in the onClientRegistered callback
        return true;
    }
}
