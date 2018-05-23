package com.sensoriainc.exnativeapp;

import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.os.Handler;
import android.os.IBinder;
import android.os.ParcelUuid;
import android.os.RemoteException;
import android.util.Log;

import java.util.UUID;

public class SignalProcessing {
    private static final String TAG="SignalProcessing";
    private int mConnState;
    private final Object mStateLock = new Object();
    private SignalProcessingCallback mCallback;
    private Handler mHandler;
    private int mClientIf;

    private ISignalProcessing mService;

    private static final int CONN_STATE_IDLE = 0;
    private static final int CONN_STATE_CONNECTING = 1;
    private static final int CONN_STATE_CONNECTED = 2;
    private static final int CONN_STATE_DISCONNECTING = 3;
    private static final int CONN_STATE_CLOSED = 4;

    /** A SIGNAL PROCESSING operation completed successfully */
    public static final int SIGNAL_PROCESSING_SUCCESS = 0;

    /** A SIGNAL PROCESSING operation failed, errors other than the above */
    public static final int SIGNAL_PROCESSING_FAILURE = 0x101;

    private void getSignalProcessing(Context context) {

        ServiceConnection mServiceConnection=new ServiceConnection() {
            @Override
            public void onServiceConnected(ComponentName name, IBinder service) {
                mService=ISignalProcessing.Stub.asInterface(service);
            }

            @Override
            public void onServiceDisconnected(ComponentName name) {

            }
        };
        Intent callService=new Intent(context,SignalProcessingService.class);
        context.bindService(callService,mServiceConnection,Context.BIND_AUTO_CREATE);
    }
    SignalProcessing(Context context) {
        mConnState = CONN_STATE_IDLE;
        getSignalProcessing(context);
    }
    public boolean connect(SignalProcessingCallback callback, Handler handler) {
        Log.d(TAG,"connect() to SignalProcessing");

        synchronized(mStateLock) {
            if (mConnState != CONN_STATE_IDLE) {
                throw new IllegalStateException("Not idle");
            }
            mConnState = CONN_STATE_CONNECTING;
        }

        if (!registerApp(callback, handler)) {
            synchronized(mStateLock) {
                mConnState = CONN_STATE_IDLE;
            }
            Log.e(TAG, "Failed to register callback");
            return false;
        }

        // The connection will continue in the onClientRegistered callback
        return true;
    }
    private boolean registerApp(SignalProcessingCallback callback,Handler handler) {
        Log.d(TAG,"registerApp to SignalProcessing");

        if(mService==null) return false;

        mCallback=callback;
        mHandler=handler;
        UUID uuid=UUID.randomUUID();

        Log.d(TAG, "registerApp() - UUID=" + uuid);

        try {
            mService.registerClient(new ParcelUuid(uuid), mSignalProcessingCallback);
        } catch(RemoteException e) {
            Log.e(TAG, "", e);
            return false;
        }
        return true;
    }

    private void runOrQueueCallback(final Runnable cb) {
        if (mHandler == null) {
            try {
                cb.run();
            } catch (Exception ex) {
                Log.w(TAG, "Unhandled exception in callback", ex);
            }
        } else {
            mHandler.post(cb);
        }
    }

    private final ISignalProcessingCallback mSignalProcessingCallback=
        new ISignalProcessingCallback.Stub() {
            @Override
            public void onClientRegistered(int status, int clientIf) {
                Log.d(TAG,"onClientRegistered() - status=" + status+ " clientIf=" + clientIf);
                mClientIf=clientIf;

                if(status!=SIGNAL_PROCESSING_SUCCESS) {
                    runOrQueueCallback(new Runnable() {
                        @Override
                        public void run() {
                            if(mCallback!=null) {
                                mCallback.onConnectionStateChange(SignalProcessing.this,
                                        SIGNAL_PROCESSING_FAILURE,
                                        SignalProcessingProfile.STATE_DISCONNECTED);
                            }
                        }
                    });

                    synchronized (mStateLock) {
                        mConnState=CONN_STATE_IDLE;
                    }
                }
                try {
                    mService.clientConnect(mClientIf);
                } catch(RemoteException e) {
                    Log.e(TAG,e.getMessage());
                }
            }
        };
}
