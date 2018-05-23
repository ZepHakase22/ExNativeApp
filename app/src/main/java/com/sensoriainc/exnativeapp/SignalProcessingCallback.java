package com.sensoriainc.exnativeapp;

public abstract class SignalProcessingCallback {
    void onConnectionStateChange(SignalProcessing signalProcessing, int status, int newState) {}
}
