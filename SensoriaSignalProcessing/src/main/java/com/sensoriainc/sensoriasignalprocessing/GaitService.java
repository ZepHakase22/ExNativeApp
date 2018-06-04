package com.sensoriainc.sensoriasignalprocessing;

import android.app.AppOpsManager;
import android.content.Intent;
import android.os.Build;
import android.os.IBinder;
import android.os.IInterface;
import android.os.Parcel;
import android.os.ParcelUuid;
import android.os.RemoteException;
import android.support.annotation.NonNull;
import android.support.annotation.Nullable;
import android.util.Log;

import com.sensoriainc.sensoriasignalprocessing.SignalProcessingService;

import java.io.FileDescriptor;
import java.util.HashMap;
import java.util.UUID;

public class GaitService extends SignalProcessingService {

    private native void signalProcessingRegisterAppNative(long app_uuid_lsb, long app_uuid_msb);
    private native boolean cleanupNative();
    private static native void classInitCallbackNative();
    private static final String TAG="SignalProcessingService";

    HashMap<UUID,ISignalProcessingCallback> mClientMap=new HashMap<>();


    static {
        classInitCallbackNative();
    }

    @Override
    protected IGenericServiceBinder initBinder() {
        return new SignalProcessingBinder(this);
    }

    @Override
    protected boolean cleanup() {
        Log.d(TAG, "cleanup()");
        cleanupNative();
        return  true;
    }

    @Override
    protected boolean start() {
        Log.d(TAG, "start()");
        initializeNative();
        if(Build.VERSION.SDK_INT >= 23)
            mAppOps=getSystemService(AppOpsManager.class);
        return true;
    }

    @Override
    protected boolean stop() {
        Log.d(TAG,"stop()");
        mClientMap.clear();
        mServerMap.clear();
        mHandleMap.clear();
        mReleableQueue.clear();
        return true;
    }

    @Override
    protected String getName() {
        return TAG;
    }

    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {
        return super.onStartCommand(intent, flags, startId);
    }

    class ServerDeathRecipient implements IBinder.DeathRecipient {
        int mAppIf;

        public ServerDeathRecipient(int appIf) {
            mAppIf=appIf;
        }

        @Override
        public void binderDied() {

            Log.d(TAG, "Binder is dead - unregistering server (" + mAppIf + ")!");
            unregisterServer(mAppIf);
        }
    }

    class ClientDeathRecipient implements IBinder.DeathRecipient {
        int mAppIf;

        public ClientDeathRecipient(int appIf) {
            mAppIf=appIf;
        }

        @Override
        public void binderDied() {
            Log.d(TAG,"Binder is dead - unregistering client (\" + mAppIf + \")!");
            unregisterClient(mAppIf);
        }
    }

    private static class SignalProcessingBinder extends ISignalProcessing.Stub implements IGenericServiceBinder {
        private SignalProcessingService mService;

        public SignalProcessingBinder(SignalProcessingService svc) {
            mService=svc;
        }

        // IGenericServiceBinder
        @Override
        public boolean cleanup() {
            mService=null;
            return true;
        }

        private SignalProcessingService getService() {
            if (mService != null && mService.isAvailable()) return mService;
            Log.e(TAG, "getService() - Service requested but not available");
            return null;
        }

        @Override
        public void registerClient(ParcelUuid uuid, ISignalProcessingCallback callback) {
            SignalProcessingService service=getService();
            if (service == null) return;
            service.registerClient(uuid.getUuid(), callback);
        }

        @Override
        public void unregisterClient(int clientIf) {
            SignalProcessingService service=getService();
            if (service == null) return;
            service.unregisterClient(clientIf);
        }

        @Override
        public void clientConnect(int clientIf) {
            SignalProcessingService service=getService();
            if (service == null) return;
            service.clientConnect(clientIf);
        }

        @Override
        public void clientDisconnect(int clientIf) {
            SignalProcessingService service=getService();
            if (service == null) return;
            service.clientDisconnect(clientIf);
        }

        @Override
        public void registerForNotification(int clientIf, int handle, boolean enable) {
            SignalProcessingService service = getService();
            if (service == null) return;
            service.registerForNotification(clientIf, handle, enable);
        }

        @Override
        public void registerServer(ParcelUuid uuid, ISignalProcessingServerCallback callback) {
            SignalProcessingService service=getService();
            if (service == null) return;
            service.registerServer(uuid.getUuid(), callback);
        }

        @Override
        public void unregisterServer(int serverIf) {
            SignalProcessingService service=getService();
            if (service == null) return;
            service.unregisterServer(serverIf);
        }

        @Override
        public void serverConnect(int serverIf) {
            SignalProcessingService service=getService();
            if (service == null) return;
            service.serverConnect(serverIf);
        }

        @Override
        public void serverDisconnect(int serverIf) {
            SignalProcessingService service=getService();
            if (service == null) return;
            service.serverDisconnect(serverIf);
        }
    };


    void registerClient(UUID uuid, ISignalProcessingCallback callback) {
        Log.d(TAG, "registerClient() - UUID=" + uuid);

        mClientMap.put(uuid,callback);
        gaitRegisterAppNative(uuid.getLeastSignificantBits(),uuid.getMostSignificantBits());
    }

    void unregisterClient(int clientIf) {
        enforceCallingOrSelfPermission(BLUETOOTH_PERM, "Need BLUETOOTH permission");
        Log.d(TAG, "unregisterClient() - clientIf=" + clientIf);
        mClientMap.remove(clientIf);
        signaProcessingtUnregisterAppNative(clientIf);
    }

    void clientConnect(int clientIf) {
        enforceCallingOrSelfPermission(BLUETOOTH_PERM, "Need BLUETOOTH permission");
        Log.d(TAG, "clientConnect()");
        signalProcessingConnectNative(clientIf);
    }

    void clientDisconnect(int clientIf) {
        enforceCallingOrSelfPermission(BLUETOOTH_PERM, "Need BLUETOOTH permission");
        Integer connId = mClientMap.connIdByAddress(clientIf);
        Log.d(TAG, "clientDisconnect() - connId=" + connId);
        signalProcessingDisconnectNative(clientIf, connId != null ? connId : 0);
    }

    void registerForNotification(int clientIf, int handle, boolean enable) {
        enforceCallingOrSelfPermission(BLUETOOTH_PERM, "Need BLUETOOTH permission");
        Log.d(TAG, "registerForNotification() - enable: " + enable);
        Integer connId = mClientMap.connIdByAddress(clientIf, address);
        if (connId == null) {
            Log.e(TAG, "registerForNotification() - No connection ...");
        }
        if (!permissionCheck(connId, handle)) {
            Log.w(TAG, "registerForNotification() - permission check failed!");
            return;
        }
        signalProcessingRegisterForNotificationsNative(clientIf, address, handle, enable);
    }

    void registerServer(UUID uuid, ISignalProcessingServerCallback callback) {
        enforceCallingOrSelfPermission(BLUETOOTH_PERM, "Need BLUETOOTH permission");
        Log.d(TAG, "registerServer() - UUID=" + uuid);
        mServerMap.add(uuid, null, callback, null, this);
        signalProcessingServerRegisterAppNative(uuid.getLeastSignificantBits(),uuid.getMostSignificantBits());
    }

    void unregisterServer(int serverIf) {
        enforceCallingOrSelfPermission(BLUETOOTH_PERM, "Need BLUETOOTH permission");
        Log.d(TAG, "unregisterServer() - serverIf=" + serverIf);
        deleteServices(serverIf);
        mServerMap.remove(serverIf);
        signalProcessingServerUnregisterAppNative(serverIf);
    }

    void serverConnect(int serverIf) {
        enforceCallingOrSelfPermission(BLUETOOTH_PERM, "Need BLUETOOTH permission");
        Log.d(TAG, "serverConnect()")   ;
        signalProcessinServerConnectNative(serverIf);
    }

    void serverDisconnect(int serverIf) {
        enforceCallingOrSelfPermission(BLUETOOTH_PERM, "Need BLUETOOTH permission");
        Integer connId = mServerMap.connIdByAddress(serverIf);
        Log.d(TAG, "serverDisconnect() - connId=" + connId);
        signalProcessingServerDisconnectNative(serverIf, address, connId != null ? connId : 0);
    }

}
