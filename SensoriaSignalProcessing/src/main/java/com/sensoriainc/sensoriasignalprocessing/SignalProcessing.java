package com.sensoriainc.sensoriasignalprocessing;

import android.content.Context;
import android.os.Handler;
import android.os.Parcel;
import android.os.Parcelable;

public class SignalProcessing implements Parcelable{
    // Used to load the 'native-lib' library on application startup.
    static {
        System.loadLibrary("SignalProcessingJNI");
    }

    private SignalProcessing() {

    }

    protected SignalProcessing(Parcel in) {
    }

    public static final Creator<SignalProcessing> CREATOR = new Creator<SignalProcessing>() {
        @Override
        public SignalProcessing createFromParcel(Parcel in) {
            return new SignalProcessing(in);
        }

        @Override
        public SignalProcessing[] newArray(int size) {
            return new SignalProcessing[size];
        }
    };

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

    @Override
    public int describeContents() {
        return 0;
    }

    @Override
    public void writeToParcel(Parcel dest, int flags) {
    }
}
