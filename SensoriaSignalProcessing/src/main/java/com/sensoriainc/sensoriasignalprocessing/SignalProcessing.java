package com.sensoriainc.sensoriasignalprocessing;

import android.content.Context;
import android.os.Handler;

public class SignalProcessing {
    // Used to load the 'native-lib' library on application startup.
    static {
        System.loadLibrary("SignalProcessingJNI");
    }

    private SignalProcessing() {

    }
    public static SignalProcessing getSignalProcessing() {
        return new SignalProcessing();
    }
    public GaitSignalProcessing connectGait(Context context, GaitCallback callback) {
        return connectGait(context,callback,null);
    }

    public GaitSignalProcessing connectGait(Context context, GaitCallback callback, Handler handler) {
        if(callback==null) {
            throw new NullPointerException("callback is null");
        }
        GaitSignalProcessing gaitSignalProcessing=new GaitSignalProcessing(context);
        gaitSignalProcessing.connect(callback,handler);
        return gaitSignalProcessing;
    }
}
