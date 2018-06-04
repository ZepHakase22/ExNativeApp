package com.sensoriainc.sensoriasignalprocessing;

import android.app.Service;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.IBinder;
import android.util.Log;

public abstract class SignalProcessingService extends Service {
    private static final String TAG = "GenericService";
    protected boolean mStartError=false;
    private boolean mCleaningUp = false;
    protected String mName;
    private static final int PROFILE_SERVICE_MODE=Service.START_NOT_STICKY;
    public static final String BLUETOOTH_ADMIN_PERM =
            android.Manifest.permission.BLUETOOTH_ADMIN;

    protected IGenericServiceBinder mBinder;
    protected abstract IGenericServiceBinder initBinder();

    public static interface IGenericServiceBinder extends IBinder {
        public boolean cleanup();
    }

    protected abstract boolean cleanup();
    protected abstract boolean start();
    protected abstract boolean stop();

    protected String getName() {
        return getClass().getSimpleName();
    }

    protected boolean isAvailable() {
        return !mStartError && !mCleaningUp;
    }

    protected SignalProcessingService() {
        mName=getName();
    }

    protected void finalize() {

    }

    @Override
    public void onCreate() {
        Log.d(TAG,"onCreate");

        super.onCreate();
        mBinder=initBinder();
        create();
    }

    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {
        Log.d(mName, "onStartCommand");

        if(mStartError) {
            Log.w(mName,"Stopping profile service: device does not have BT");
            doStop(intent);
            return PROFILE_SERVICE_MODE;
        }
        if(checkCallingOrSelfPermission(BLUETOOTH_ADMIN_PERM)!= PackageManager.PERMISSION_GRANTED) {
            Log.e(mName,"Permission denied");
            return PROFILE_SERVICE_MODE;
        }
        if(intent==null) {
            Log.d(mName, "Restart generic service ...");
            return PROFILE_SERVICE_MODE;
        } else {
            String action = intent.getAction();
            if(action.equals("SIGNAL_PROCESSING_START")) {
                Log.d(mName, "Received start request. Starting profile...");
                doStart(intent);
            } else if(action.equals("SIGNAL_PROCESSING_STOP")) {
                Log.d(mName, "Received stop request...Stopping profile...");
                doStop(intent);
            }
        }
        return PROFILE_SERVICE_MODE;
    }

    protected  void create() {}

    @Override
    public IBinder onBind(Intent intent) {
        Log.d(mName,"onBind");
        return mBinder;
    }

    @Override
    public boolean onUnbind(Intent intent) {
        Log.d(mName, "onUnbind");
        return super.onUnbind(intent);
    }

    @Override
    public void onDestroy() {
        Log.d(mName, "Destroying service.");
        if(mCleaningUp)
            Log.d(mName,"Cleanup already started... Skipping cleanup()...");
        else {
            Log.d(mName,"cleanup()");
            mCleaningUp=true;
            cleanup();
            if(mBinder!=null) {
                mBinder.cleanup();
                mBinder=null;
            }
        }
        super.onDestroy();
    }

    private void doStart(Intent intent) {
        Log.d(mName,"start()");
        mStartError=!start();
        if(mStartError)
            Log.e(mName, "Error starting SignalProcessing. SignalProcessing is null");
    }

    private void doStop(Intent intent) {
        if(stop()) {
            Log.d(mName,"stop()");
            stopSelf();
        } else {
            Log.e(mName, "unable to stop SignalProcessing");
        }
    }

}
