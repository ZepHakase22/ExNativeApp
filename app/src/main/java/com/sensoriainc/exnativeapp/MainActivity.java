package com.sensoriainc.exnativeapp;

import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.view.View;
import android.widget.TextView;

public class MainActivity extends AppCompatActivity {
    private TextView etv;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // Example of a call to a native method
        TextView tv= findViewById(R.id.sample_text);
        etv  = findViewById(R.id.event_view);
        tv.setText(stringFromJNI());
    }

    /**
     * A native method that is implemented by the 'native-lib' native library,
     * which is packaged with this application.
     * The Java class MainActivity contains a main method which is used to invoke the program.
     * private native String getStringFrom(String prompt);
     * Notice that the declarations for native methods are almost identical to the declarations for
     * regular, non-native Java methods. There are two differences. Native methods must have
     * the native keyword. The native keyword informs the Java compiler that the implementation
     * for this method is provided in another language. Also, the native method declaration is
     * terminated with a semicolon, the statement terminator symbol, because there are no
     * implementations for native methods in the Java class file.
     */
    public native String stringFromJNI();
    public native void setEvent();

    public void callbackHello(String hello) {
        etv.setText(hello);
    }

    public void sayHello(View view) {
        setEvent();

    }


}
