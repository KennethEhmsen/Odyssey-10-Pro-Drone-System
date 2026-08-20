package dk.odyssey.ridtest.scan;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothManager;
import android.bluetooth.le.BluetoothLeScanner;
import android.bluetooth.le.ScanCallback;
import android.bluetooth.le.ScanFilter;
import android.bluetooth.le.ScanResult;
import android.bluetooth.le.ScanSettings;
import android.content.Context;
import android.os.Build;
import android.os.ParcelUuid;
import android.util.Log;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import dk.odyssey.ridtest.AircraftStore;
import dk.odyssey.ridtest.odid.OdidParser;

/**
 * Bluetooth LE scanner for ASTM Remote ID advertisements.
 *
 * <p>Two details matter and are easy to get wrong:
 *
 * <ol>
 *   <li><b>Legacy scanning will not see the aircraft.</b> The Odyssey Remote ID module
 *       advertises on the Coded PHY using extended advertising, because that is what
 *       "Bluetooth 5 Long Range" means in the standard. A scanner left on the default
 *       1M PHY simply never sees it, and the failure looks identical to the aircraft
 *       being switched off. {@link #isExtendedScanSupported()} reports whether this
 *       handset can do it at all.</li>
 *   <li><b>Service data, not manufacturer data.</b> ASTM uses Service Data under the
 *       16-bit UUID 0xFFFA. Some other Remote ID implementations use manufacturer-
 *       specific data instead, so both are examined before giving up.</li>
 * </ol>
 */
public final class BleScanner {

    private static final String TAG = "OdyRID.Ble";

    /** ASTM assigned 16-bit UUID, expressed in the full Bluetooth base UUID form. */
    public static final ParcelUuid ASTM_UUID =
            ParcelUuid.fromString("0000fffa-0000-1000-8000-00805f9b34fb");

    public interface Listener {
        void onScanEvent(String message);
    }

    private final Context context;
    private final AircraftStore store;
    private final Listener listener;

    private BluetoothLeScanner scanner;
    private ScanCallback callback;
    private boolean scanning;

    public BleScanner(Context context, AircraftStore store, Listener listener) {
        this.context = context.getApplicationContext();
        this.store = store;
        this.listener = listener;
    }

    /** True when the handset can scan extended advertisements, i.e. see Coded PHY. */
    public boolean isExtendedScanSupported() {
        BluetoothAdapter a = adapter();
        return a != null && Build.VERSION.SDK_INT >= Build.VERSION_CODES.O
                         && a.isLeExtendedAdvertisingSupported();
    }

    public boolean isCodedPhySupported() {
        BluetoothAdapter a = adapter();
        return a != null && Build.VERSION.SDK_INT >= Build.VERSION_CODES.O
                         && a.isLeCodedPhySupported();
    }

    private BluetoothAdapter adapter() {
        BluetoothManager m = (BluetoothManager)
                context.getSystemService(Context.BLUETOOTH_SERVICE);
        return m == null ? null : m.getAdapter();
    }

    public boolean isScanning() { return scanning; }

    /** @return null on success, or a human-readable reason the scan could not start. */
    public String start() {
        if (scanning) return null;

        BluetoothAdapter a = adapter();
        if (a == null)          return "this device has no Bluetooth adapter";
        if (!a.isEnabled())     return "Bluetooth is turned off";

        scanner = a.getBluetoothLeScanner();
        if (scanner == null)    return "no BLE scanner available";

        // Filter on the ASTM service UUID. An empty filter list would work too but wakes
        // the app for every advertisement in range, which flattens the phone.
        List<ScanFilter> filters = new ArrayList<>();
        filters.add(new ScanFilter.Builder().setServiceUuid(ASTM_UUID).build());
        // Some stacks report the UUID only in the service DATA, not the UUID list, so
        // also accept anything carrying service data for that UUID.
        filters.add(new ScanFilter.Builder()
                .setServiceData(ASTM_UUID, new byte[0], new byte[0]).build());

        ScanSettings.Builder sb = new ScanSettings.Builder()
                .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
                .setReportDelay(0);

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            sb.setLegacy(false);                       // see extended advertisements
            if (isCodedPhySupported()) {
                sb.setPhy(ScanSettings.PHY_LE_ALL_SUPPORTED);
            }
        }

        callback = new ScanCallback() {
            @Override public void onScanResult(int callbackType, ScanResult result) {
                handle(result);
            }
            @Override public void onBatchScanResults(List<ScanResult> results) {
                for (ScanResult r : results) handle(r);
            }
            @Override public void onScanFailed(int errorCode) {
                scanning = false;
                emit("BLE scan failed, error " + errorCode + " (" + scanFailureText(errorCode) + ")");
            }
        };

        try {
            scanner.startScan(filters, sb.build(), callback);
        } catch (SecurityException e) {
            return "Bluetooth scan permission not granted";
        }
        scanning = true;

        if (!isExtendedScanSupported()) {
            emit("WARNING: this handset cannot scan extended advertisements. "
               + "The aircraft transmits on the Coded PHY, so it will NOT be seen. "
               + "This looks identical to the aircraft being silent -- try another phone.");
        }
        return null;
    }

    public void stop() {
        if (!scanning || scanner == null || callback == null) return;
        try {
            scanner.stopScan(callback);
        } catch (SecurityException ignored) {
            // Permission revoked while scanning; nothing useful to do.
        }
        scanning = false;
    }

    private void handle(ScanResult result) {
        if (result.getScanRecord() == null) return;

        byte[] payload = null;

        Map<ParcelUuid, byte[]> sd = result.getScanRecord().getServiceData();
        if (sd != null) {
            byte[] astm = sd.get(ASTM_UUID);
            if (astm != null) payload = astm;
        }

        // Android strips the UUID from the service data map, so the app code that the
        // parser expects at offset 0 is already the first byte here. Nothing to trim.
        if (payload == null) return;

        OdidParser.Decoded d = OdidParser.parseBluetoothServiceData(payload);
        store.onAdvertisement(result.getDevice().getAddress(),
                              AircraftStore.Medium.BLUETOOTH,
                              result.getRssi(), d, System.currentTimeMillis());

        if (d.error != null) {
            Log.w(TAG, "decode failed: " + d.error + "  raw=" + OdidParser.hex(payload));
        }
    }

    private void emit(String s) {
        Log.i(TAG, s);
        if (listener != null) listener.onScanEvent(s);
    }

    private static String scanFailureText(int code) {
        switch (code) {
            case ScanCallback.SCAN_FAILED_ALREADY_STARTED:            return "already started";
            case ScanCallback.SCAN_FAILED_APPLICATION_REGISTRATION_FAILED: return "registration failed";
            case ScanCallback.SCAN_FAILED_INTERNAL_ERROR:             return "internal error";
            case ScanCallback.SCAN_FAILED_FEATURE_UNSUPPORTED:        return "feature unsupported";
            default:                                                  return "unknown";
        }
    }
}
