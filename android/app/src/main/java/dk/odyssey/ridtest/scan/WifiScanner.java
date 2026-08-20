package dk.odyssey.ridtest.scan;

import android.content.Context;
import android.net.wifi.ScanResult;
import android.net.wifi.WifiManager;
import android.os.Build;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;

import java.util.List;

import dk.odyssey.ridtest.AircraftStore;
import dk.odyssey.ridtest.odid.OdidParser;

/**
 * Wi-Fi beacon scanner for ASTM Remote ID vendor information elements.
 *
 * <p>This path is markedly weaker than Bluetooth on Android and the limitations are worth
 * stating plainly rather than discovering in a field:
 *
 * <ul>
 *   <li>Reading raw information elements needs {@code ScanResult.getInformationElements()},
 *       which is <b>API 30 (Android 11) and later</b>. Below that the vendor IE is simply
 *       not exposed and this scanner reports as unavailable.</li>
 *   <li>Android throttles Wi-Fi scans hard -- typically four foreground scans per
 *       two minutes. A 1 Hz Remote ID broadcast therefore cannot be sampled at anything
 *       like its true rate, so the <b>rate conformance checks are meaningless over
 *       Wi-Fi</b> and should be judged over Bluetooth.</li>
 *   <li>Scan results require location permission and location services switched on.</li>
 * </ul>
 *
 * <p>It is included because some regulator-supplied receivers use the Wi-Fi path, so
 * confirming the aircraft is present on it has value even at a poor sample rate.
 */
public final class WifiScanner {

    private static final String TAG = "OdyRID.Wifi";

    /** Vendor-specific information element ID. */
    private static final int VENDOR_IE_ID = 221;

    public interface Listener { void onScanEvent(String message); }

    private final Context context;
    private final AircraftStore store;
    private final Listener listener;
    private final Handler handler = new Handler(Looper.getMainLooper());

    private boolean running;
    private Runnable pump;

    public WifiScanner(Context context, AircraftStore store, Listener listener) {
        this.context = context.getApplicationContext();
        this.store = store;
        this.listener = listener;
    }

    public static boolean isSupported() {
        return Build.VERSION.SDK_INT >= Build.VERSION_CODES.R;
    }

    public boolean isRunning() { return running; }

    /** @return null on success, or the reason scanning is unavailable. */
    public String start() {
        if (!isSupported()) {
            return "Wi-Fi Remote ID needs Android 11 or later to read information elements";
        }
        WifiManager wm = (WifiManager) context.getSystemService(Context.WIFI_SERVICE);
        if (wm == null)          return "no Wi-Fi service";
        if (!wm.isWifiEnabled()) return "Wi-Fi is turned off";

        running = true;
        // Android rate-limits startScan() severely, so poll gently and read whatever
        // results the system already has rather than demanding fresh scans.
        pump = new Runnable() {
            @Override public void run() {
                if (!running) return;
                collect(wm);
                handler.postDelayed(this, 15000);
            }
        };
        handler.post(pump);

        emit("Wi-Fi scanning started. Note: Android throttles Wi-Fi scans to a few per "
           + "minute, so rate checks are not meaningful on this path -- judge the "
           + "1 Hz requirement over Bluetooth.");
        return null;
    }

    public void stop() {
        running = false;
        if (pump != null) handler.removeCallbacks(pump);
    }

    private void collect(WifiManager wm) {
        List<ScanResult> results;
        try {
            results = wm.getScanResults();
            wm.startScan();          // best effort; may be silently throttled
        } catch (SecurityException e) {
            emit("Wi-Fi scan permission not granted");
            return;
        }
        if (results == null) return;

        for (ScanResult r : results) {
            if (Build.VERSION.SDK_INT < Build.VERSION_CODES.R) continue;
            List<ScanResult.InformationElement> ies = r.getInformationElements();
            if (ies == null) continue;

            for (ScanResult.InformationElement ie : ies) {
                if (ie.getId() != VENDOR_IE_ID) continue;

                java.nio.ByteBuffer buf = ie.getBytes();
                byte[] body = new byte[buf.remaining()];
                buf.get(body);

                OdidParser.Decoded d = OdidParser.parseWifiVendorIe(body);
                // Most vendor IEs on the air belong to somebody else; only report the
                // ones that actually decoded as ASTM.
                if (d.error != null && d.isEmpty()) continue;

                store.onAdvertisement(r.BSSID, AircraftStore.Medium.WIFI,
                                      r.level, d, System.currentTimeMillis());
                Log.i(TAG, "ASTM vendor IE from " + r.BSSID);
            }
        }
    }

    private void emit(String s) {
        Log.i(TAG, s);
        if (listener != null) listener.onScanEvent(s);
    }
}
