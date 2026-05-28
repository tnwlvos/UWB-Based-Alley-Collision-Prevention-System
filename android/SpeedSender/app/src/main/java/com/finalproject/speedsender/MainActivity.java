package com.finalproject.speedsender;

import android.Manifest;
import android.app.Activity;
import android.app.PendingIntent;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.pm.PackageManager;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.provider.Settings;
import android.view.Gravity;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.Spinner;
import android.widget.TextView;

import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialProber;

import java.io.IOException;
import java.io.OutputStream;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Set;
import java.util.UUID;

public class MainActivity extends Activity {
    private static final int REQ_PERMISSIONS = 100;
    private static final int USB_BAUD_RATE = 115200;
    private static final String ACTION_USB_PERMISSION = "com.finalproject.speedsender.USB_PERMISSION";
    private static final UUID SPP_UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");

    private enum SendMode {
        USB,
        BLUETOOTH
    }

    private final Handler handler = new Handler(Looper.getMainLooper());
    private final List<BluetoothDevice> pairedDevices = new ArrayList<>();

    private TextView modeText;
    private TextView speedText;
    private TextView statusText;
    private TextView logText;
    private EditText tagIdInput;
    private Spinner bluetoothSpinner;
    private Button startButton;
    private Button stopButton;

    private SendMode currentMode = SendMode.USB;
    private LocationManager locationManager;
    private float currentSpeedMps = 0.0f;
    private boolean sending = false;

    private UsbSerialPort usbPort;
    private UsbDeviceConnection usbConnection;
    private BluetoothSocket bluetoothSocket;
    private OutputStream bluetoothOutput;

    private final LocationListener locationListener = new LocationListener() {
        @Override
        public void onLocationChanged(Location location) {
            if (location.hasSpeed()) {
                currentSpeedMps = location.getSpeed();
                updateSpeedText();
            }
        }

        @Override
        public void onProviderEnabled(String provider) {
            appendLog(provider + " enabled");
        }

        @Override
        public void onProviderDisabled(String provider) {
            appendLog(provider + " disabled");
        }
    };

    private final Runnable sendLoop = new Runnable() {
        @Override
        public void run() {
            if (!sending) {
                return;
            }

            String payload = buildPayload();
            sendPayload(payload);
            handler.postDelayed(this, 1000);
        }
    };

    private final BroadcastReceiver usbPermissionReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            if (!ACTION_USB_PERMISSION.equals(intent.getAction())) {
                return;
            }

            UsbDevice device = intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
            boolean granted = intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false);
            if (granted && device != null) {
                openUsbDevice(device);
            } else {
                setStatus("USB permission denied");
            }
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            registerReceiver(usbPermissionReceiver, new IntentFilter(ACTION_USB_PERMISSION), RECEIVER_NOT_EXPORTED);
        } else {
            registerReceiver(usbPermissionReceiver, new IntentFilter(ACTION_USB_PERMISSION));
        }

        buildUi();
        requestRequiredPermissions();
        refreshBluetoothDevices();
    }

    @Override
    protected void onDestroy() {
        stopSending();
        unregisterReceiver(usbPermissionReceiver);
        super.onDestroy();
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == REQ_PERMISSIONS) {
            refreshBluetoothDevices();
            appendLog("permissions updated");
        }
    }

    private void buildUi() {
        ScrollView scrollView = new ScrollView(this);
        LinearLayout root = new LinearLayout(this);
        root.setOrientation(LinearLayout.VERTICAL);
        root.setPadding(32, 32, 32, 32);
        scrollView.addView(root);

        TextView title = new TextView(this);
        title.setText("Speed Sender");
        title.setTextSize(28);
        title.setGravity(Gravity.START);
        root.addView(title);

        modeText = label("Mode: USB C-to-C");
        speedText = label("Speed: 0.00 m/s");
        statusText = label("Status: idle");
        root.addView(modeText);
        root.addView(speedText);
        root.addView(statusText);

        tagIdInput = new EditText(this);
        tagIdInput.setHint("Tag ID");
        tagIdInput.setSingleLine(true);
        tagIdInput.setText("2");
        root.addView(tagIdInput);

        LinearLayout modeRow = new LinearLayout(this);
        modeRow.setOrientation(LinearLayout.HORIZONTAL);
        root.addView(modeRow);

        Button usbModeButton = new Button(this);
        usbModeButton.setText("USB C-to-C");
        usbModeButton.setOnClickListener(v -> setMode(SendMode.USB));
        modeRow.addView(usbModeButton, new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1));

        Button bluetoothModeButton = new Button(this);
        bluetoothModeButton.setText("Bluetooth");
        bluetoothModeButton.setOnClickListener(v -> setMode(SendMode.BLUETOOTH));
        modeRow.addView(bluetoothModeButton, new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1));

        bluetoothSpinner = new Spinner(this);
        bluetoothSpinner.setVisibility(View.GONE);
        bluetoothSpinner.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {
            }
        });
        root.addView(bluetoothSpinner);

        startButton = new Button(this);
        startButton.setText("Start Sending");
        startButton.setOnClickListener(v -> startSending());
        root.addView(startButton);

        stopButton = new Button(this);
        stopButton.setText("Stop");
        stopButton.setEnabled(false);
        stopButton.setOnClickListener(v -> stopSending());
        root.addView(stopButton);

        Button locationSettingsButton = new Button(this);
        locationSettingsButton.setText("Open Location Settings");
        locationSettingsButton.setOnClickListener(v -> startActivity(new Intent(Settings.ACTION_LOCATION_SOURCE_SETTINGS)));
        root.addView(locationSettingsButton);

        logText = label("");
        logText.setTextSize(13);
        root.addView(logText);

        setContentView(scrollView);
    }

    private TextView label(String text) {
        TextView view = new TextView(this);
        view.setText(text);
        view.setTextSize(16);
        view.setPadding(0, 16, 0, 8);
        return view;
    }

    private void setMode(SendMode mode) {
        currentMode = mode;
        bluetoothSpinner.setVisibility(mode == SendMode.BLUETOOTH ? View.VISIBLE : View.GONE);
        modeText.setText(mode == SendMode.USB ? "Mode: USB C-to-C" : "Mode: Bluetooth");
        setStatus("idle");
    }

    private void requestRequiredPermissions() {
        List<String> permissions = new ArrayList<>();
        permissions.add(Manifest.permission.ACCESS_FINE_LOCATION);
        permissions.add(Manifest.permission.ACCESS_COARSE_LOCATION);

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            permissions.add(Manifest.permission.BLUETOOTH_CONNECT);
            permissions.add(Manifest.permission.BLUETOOTH_SCAN);
        }

        requestPermissions(permissions.toArray(new String[0]), REQ_PERMISSIONS);
    }

    private void refreshBluetoothDevices() {
        pairedDevices.clear();
        List<String> names = new ArrayList<>();

        BluetoothAdapter adapter = BluetoothAdapter.getDefaultAdapter();
        if (adapter == null) {
            names.add("Bluetooth not supported");
        } else if (hasBluetoothPermission()) {
            Set<BluetoothDevice> bondedDevices = adapter.getBondedDevices();
            for (BluetoothDevice device : bondedDevices) {
                pairedDevices.add(device);
                names.add(device.getName() + " / " + device.getAddress());
            }
            if (names.isEmpty()) {
                names.add("No paired devices");
            }
        } else {
            names.add("Bluetooth permission required");
        }

        ArrayAdapter<String> adapterView = new ArrayAdapter<>(this, android.R.layout.simple_spinner_item, names);
        adapterView.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        bluetoothSpinner.setAdapter(adapterView);
    }

    private void startSending() {
        if (!hasLocationPermission()) {
            requestRequiredPermissions();
            return;
        }

        startLocationUpdates();

        if (currentMode == SendMode.USB) {
            connectUsb();
        } else {
            connectBluetooth();
        }
    }

    private void startSendingLoop() {
        sending = true;
        startButton.setEnabled(false);
        stopButton.setEnabled(true);
        handler.removeCallbacks(sendLoop);
        handler.post(sendLoop);
    }

    private void stopSending() {
        sending = false;
        handler.removeCallbacks(sendLoop);
        startButton.setEnabled(true);
        stopButton.setEnabled(false);
        stopLocationUpdates();
        closeUsb();
        closeBluetooth();
        setStatus("idle");
    }

    private void startLocationUpdates() {
        locationManager = (LocationManager) getSystemService(Context.LOCATION_SERVICE);
        if (locationManager == null || !hasLocationPermission()) {
            return;
        }

        try {
            locationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 500, 0, locationListener);
            locationManager.requestLocationUpdates(LocationManager.NETWORK_PROVIDER, 500, 0, locationListener);
            setStatus("location active");
        } catch (IllegalArgumentException ignored) {
            setStatus("location provider unavailable");
        }
    }

    private void stopLocationUpdates() {
        if (locationManager != null) {
            locationManager.removeUpdates(locationListener);
        }
    }

    private void connectUsb() {
        UsbManager usbManager = (UsbManager) getSystemService(Context.USB_SERVICE);
        if (usbManager == null) {
            setStatus("USB manager unavailable");
            return;
        }

        List<UsbSerialDriver> drivers = UsbSerialProber.getDefaultProber().findAllDrivers(usbManager);
        if (drivers.isEmpty()) {
            setStatus("No USB serial device");
            return;
        }

        UsbSerialDriver driver = drivers.get(0);
        UsbDevice device = driver.getDevice();
        if (!usbManager.hasPermission(device)) {
            PendingIntent permissionIntent = PendingIntent.getBroadcast(
                    this,
                    0,
                    new Intent(ACTION_USB_PERMISSION),
                    Build.VERSION.SDK_INT >= Build.VERSION_CODES.S ? PendingIntent.FLAG_MUTABLE : 0
            );
            usbManager.requestPermission(device, permissionIntent);
            setStatus("Requesting USB permission");
            return;
        }

        openUsbDevice(device);
    }

    private void openUsbDevice(UsbDevice device) {
        UsbManager usbManager = (UsbManager) getSystemService(Context.USB_SERVICE);
        if (usbManager == null) {
            setStatus("USB manager unavailable");
            return;
        }

        List<UsbSerialDriver> drivers = UsbSerialProber.getDefaultProber().findAllDrivers(usbManager);
        UsbSerialDriver targetDriver = null;
        for (UsbSerialDriver driver : drivers) {
            if (driver.getDevice().equals(device)) {
                targetDriver = driver;
                break;
            }
        }

        if (targetDriver == null || targetDriver.getPorts().isEmpty()) {
            setStatus("USB serial port unavailable");
            return;
        }

        usbConnection = usbManager.openDevice(device);
        if (usbConnection == null) {
            setStatus("Failed to open USB device");
            return;
        }

        usbPort = targetDriver.getPorts().get(0);
        try {
            usbPort.open(usbConnection);
            usbPort.setParameters(USB_BAUD_RATE, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE);
            setStatus("USB connected");
            startSendingLoop();
        } catch (IOException e) {
            setStatus("USB error: " + e.getMessage());
            closeUsb();
        }
    }

    private void closeUsb() {
        try {
            if (usbPort != null) {
                usbPort.close();
            }
        } catch (IOException ignored) {
        }
        usbPort = null;
        usbConnection = null;
    }

    private void connectBluetooth() {
        if (!hasBluetoothPermission()) {
            requestRequiredPermissions();
            return;
        }
        if (pairedDevices.isEmpty() || bluetoothSpinner.getSelectedItemPosition() >= pairedDevices.size()) {
            setStatus("Pair ESP32 first");
            return;
        }

        BluetoothDevice device = pairedDevices.get(bluetoothSpinner.getSelectedItemPosition());
        setStatus("Connecting Bluetooth");
        new Thread(() -> {
            try {
                bluetoothSocket = device.createRfcommSocketToServiceRecord(SPP_UUID);
                bluetoothSocket.connect();
                bluetoothOutput = bluetoothSocket.getOutputStream();
                handler.post(() -> {
                    setStatus("Bluetooth connected");
                    startSendingLoop();
                });
            } catch (IOException e) {
                handler.post(() -> {
                    closeBluetooth();
                    setStatus("Bluetooth error: " + e.getMessage());
                });
            }
        }).start();
    }

    private void closeBluetooth() {
        try {
            if (bluetoothOutput != null) {
                bluetoothOutput.close();
            }
            if (bluetoothSocket != null) {
                bluetoothSocket.close();
            }
        } catch (IOException ignored) {
        }
        bluetoothOutput = null;
        bluetoothSocket = null;
    }

    private String buildPayload() {
        String tagId = tagIdInput.getText().toString().trim();
        if (tagId.isEmpty()) {
            tagId = "0";
        }
        long timestamp = System.currentTimeMillis();
        return String.format(Locale.US, "TAG,%s,SPD,%.2f,TS,%d\n", tagId, currentSpeedMps, timestamp);
    }

    private void sendPayload(String payload) {
        byte[] bytes = payload.getBytes(StandardCharsets.UTF_8);
        try {
            if (currentMode == SendMode.USB && usbPort != null) {
                usbPort.write(bytes, 1000);
            } else if (currentMode == SendMode.BLUETOOTH && bluetoothOutput != null) {
                bluetoothOutput.write(bytes);
                bluetoothOutput.flush();
            } else {
                setStatus("not connected");
                return;
            }
            appendLog("sent " + payload.trim());
        } catch (IOException e) {
            setStatus("send error: " + e.getMessage());
            stopSending();
        }
    }

    private boolean hasLocationPermission() {
        return checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED
                || checkSelfPermission(Manifest.permission.ACCESS_COARSE_LOCATION) == PackageManager.PERMISSION_GRANTED;
    }

    private boolean hasBluetoothPermission() {
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.S) {
            return true;
        }
        return checkSelfPermission(Manifest.permission.BLUETOOTH_CONNECT) == PackageManager.PERMISSION_GRANTED;
    }

    private void updateSpeedText() {
        speedText.setText(String.format(Locale.US, "Speed: %.2f m/s  %.1f km/h", currentSpeedMps, currentSpeedMps * 3.6f));
    }

    private void setStatus(String status) {
        statusText.setText("Status: " + status);
        appendLog(status);
    }

    private void appendLog(String message) {
        String old = logText == null ? "" : logText.getText().toString();
        String next = message + "\n" + old;
        if (next.length() > 3000) {
            next = next.substring(0, 3000);
        }
        if (logText != null) {
            logText.setText(next);
        }
    }
}
