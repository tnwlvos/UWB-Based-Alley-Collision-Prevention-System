package com.example.myapplication

import android.Manifest
import android.app.PendingIntent
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.content.pm.PackageManager
import android.hardware.usb.UsbManager
import android.os.Build
import android.os.Bundle
import android.os.Looper
import android.widget.Toast
import androidx.activity.ComponentActivity
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.Button
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableFloatStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.core.content.ContextCompat
import com.example.myapplication.ui.theme.MyApplicationTheme
import com.google.android.gms.location.FusedLocationProviderClient
import com.google.android.gms.location.LocationCallback
import com.google.android.gms.location.LocationRequest
import com.google.android.gms.location.LocationResult
import com.google.android.gms.location.LocationServices
import com.google.android.gms.location.Priority
import com.hoho.android.usbserial.driver.UsbSerialDriver
import com.hoho.android.usbserial.driver.UsbSerialPort
import com.hoho.android.usbserial.driver.UsbSerialProber
import java.io.IOException
import java.util.Locale
import kotlinx.coroutines.delay

private const val SPEED_RESEND_INTERVAL_MS = 100L

class MainActivity : ComponentActivity() {
    private lateinit var fusedLocationClient: FusedLocationProviderClient
    private var usbSerialPort: UsbSerialPort? = null
    private val ACTION_USB_PERMISSION = "com.example.myapplication.USB_PERMISSION"

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        fusedLocationClient = LocationServices.getFusedLocationProviderClient(this)
        enableEdgeToEdge()
        setContent {
            MyApplicationTheme {
                MainScreen(fusedLocationClient, ::sendDataToUsb, ::connectUsb)
            }
        }
    }

    private fun sendDataToUsb(data: String) {
        val port = usbSerialPort
        if (port != null && port.isOpen) {
            try {
                port.write(data.toByteArray(), 1000)
            } catch (e: IOException) {
                e.printStackTrace()
            }
        }
    }

    private fun connectUsb(onStatusUpdate: (String) -> Unit) {
        val manager = getSystemService(USB_SERVICE) as UsbManager
        val availableDrivers = UsbSerialProber.getDefaultProber().findAllDrivers(manager)
        if (availableDrivers.isEmpty()) {
            onStatusUpdate("No USB Devices Found")
            return
        }

        val driver = availableDrivers[0]
        val device = driver.device

        if (!manager.hasPermission(device)) {
            val flags = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) PendingIntent.FLAG_MUTABLE else 0
            val permissionIntent = PendingIntent.getBroadcast(
                this,
                0,
                Intent(ACTION_USB_PERMISSION).setPackage(packageName),
                flags
            )
            
            val filter = IntentFilter(ACTION_USB_PERMISSION)
            val receiver = object : BroadcastReceiver() {
                override fun onReceive(context: Context, intent: Intent) {
                    try {
                        if (ACTION_USB_PERMISSION == intent.action) {
                            synchronized(this) {
                                if (intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)) {
                                    openPort(manager, driver, onStatusUpdate)
                                } else {
                                    onStatusUpdate("USB Permission Denied")
                                }
                            }
                        }
                    } finally {
                        try {
                            context.unregisterReceiver(this)
                        } catch (_: IllegalArgumentException) {
                        }
                    }
                }
            }

            ContextCompat.registerReceiver(this, receiver, filter, ContextCompat.RECEIVER_NOT_EXPORTED)
            
            try {
                manager.requestPermission(device, permissionIntent)
            } catch (e: RuntimeException) {
                onStatusUpdate("USB permission error: ${e.message}")
            }
        } else {
            openPort(manager, driver, onStatusUpdate)
        }
    }

    private fun openPort(manager: UsbManager, driver: UsbSerialDriver, onStatusUpdate: (String) -> Unit) {
        val port = driver.ports.firstOrNull()
        if (port == null) {
            onStatusUpdate("USB serial port not found")
            return
        }

        val connection = try {
            manager.openDevice(driver.device)
        } catch (e: SecurityException) {
            onStatusUpdate("USB permission missing: ${e.message}")
            return
        }

        if (connection == null) {
            onStatusUpdate("USB Connection Failed")
            return
        }

        try {
            usbSerialPort?.takeIf { it.isOpen }?.close()
            port.open(connection)
            port.setParameters(115200, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE)
            this.usbSerialPort = port
            onStatusUpdate("USB Connected: ${driver.device.deviceName}")
        } catch (e: IOException) {
            try {
                port.close()
            } catch (_: IOException) {
            }
            connection.close()
            onStatusUpdate("Error: ${e.message}")
        } catch (e: RuntimeException) {
            try {
                port.close()
            } catch (_: IOException) {
            }
            connection.close()
            onStatusUpdate("USB error: ${e.message}")
        }
    }
}

@Composable
fun MainScreen(
    fusedLocationClient: FusedLocationProviderClient,
    onSpeedUpdate: (String) -> Unit,
    onConnectUsb: ((String) -> Unit) -> Unit
) {
    val context = LocalContext.current
    var isGpsEnabled by remember { mutableStateOf(false) }
    var currentSpeed by remember { mutableFloatStateOf(0f) }
    var lastSpeedUpdateMillis by remember { mutableStateOf(0L) }
    var usbStatus by remember { mutableStateOf("USB Not Connected") }

    val permissionLauncher = rememberLauncherForActivityResult(
        contract = ActivityResultContracts.RequestMultiplePermissions()
    ) { permissions ->
        val granted = permissions.entries.all { it.value }
        if (granted) {
            isGpsEnabled = true
        } else {
            Toast.makeText(context, "Location permission required", Toast.LENGTH_SHORT).show()
        }
    }

    // Automatically check for permissions on launch
    LaunchedEffect(Unit) {
        val hasPermission = ContextCompat.checkSelfPermission(context, Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED
        if (hasPermission) {
            isGpsEnabled = true
        } else {
            permissionLauncher.launch(arrayOf(Manifest.permission.ACCESS_FINE_LOCATION, Manifest.permission.ACCESS_COARSE_LOCATION))
        }
    }

    val locationCallback = remember {
        object : LocationCallback() {
            override fun onLocationResult(locationResult: LocationResult) {
                for (location in locationResult.locations) {
                    val speedKmH = location.speed * 3.6f
                    currentSpeed = speedKmH
                    lastSpeedUpdateMillis = System.currentTimeMillis()
                }
            }
        }
    }

    LaunchedEffect(isGpsEnabled) {
        while (isGpsEnabled) {
            if (lastSpeedUpdateMillis > 0L) {
                onSpeedUpdate(String.format(Locale.US, "%.2f\n", currentSpeed))
            }
            delay(SPEED_RESEND_INTERVAL_MS)
        }
    }

    DisposableEffect(isGpsEnabled) {
        if (isGpsEnabled) {
            val locationRequest = LocationRequest.Builder(Priority.PRIORITY_HIGH_ACCURACY, 100)
                .setMinUpdateIntervalMillis(50)
                .build()

            if (ContextCompat.checkSelfPermission(context, Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED) {
                fusedLocationClient.requestLocationUpdates(locationRequest, locationCallback, Looper.getMainLooper())
            }
        } else {
            fusedLocationClient.removeLocationUpdates(locationCallback)
        }
        onDispose {
            fusedLocationClient.removeLocationUpdates(locationCallback)
        }
    }

    Scaffold(modifier = Modifier.fillMaxSize()) { innerPadding ->
        Column(
            modifier = Modifier
                .padding(innerPadding)
                .fillMaxSize(),
            horizontalAlignment = Alignment.CenterHorizontally,
            verticalArrangement = Arrangement.Center
        ) {
            Text(text = "Current Speed", fontSize = 20.sp)
            Text(text = "%.2f km/h".format(currentSpeed), fontSize = 48.sp, color = MaterialTheme.colorScheme.primary)
            
            Spacer(modifier = Modifier.height(32.dp))
            
            Button(onClick = {
                if (!isGpsEnabled) {
                    permissionLauncher.launch(
                        arrayOf(
                            Manifest.permission.ACCESS_FINE_LOCATION,
                            Manifest.permission.ACCESS_COARSE_LOCATION
                        )
                    )
                } else {
                    isGpsEnabled = false
                }
            }) {
                Text(if (isGpsEnabled) "Turn GPS OFF" else "Turn GPS ON")
            }

            Spacer(modifier = Modifier.height(16.dp))

            Button(onClick = {
                onConnectUsb { status -> usbStatus = status }
            }) {
                Text("Connect USB (DWM1000)")
            }

            Text(text = usbStatus, modifier = Modifier.padding(8.dp), fontSize = 12.sp)
        }
    }
}
