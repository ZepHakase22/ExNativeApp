package com.sensoriainc.sensoriasignalprocessing;

public abstract class GaitCallback {
    void onConnectionStateChange(GaitSignalProcessing gaitSignalProcessing, int status, int newState) {}
}
