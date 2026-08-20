package dk.odyssey.ridtest;

import android.Manifest;
import android.app.Activity;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ScrollView;
import android.widget.TextView;
import android.widget.Toast;

import java.util.ArrayList;
import java.util.List;

import dk.odyssey.ridtest.odid.OdidSelfTest;
import dk.odyssey.ridtest.scan.BleScanner;
import dk.odyssey.ridtest.scan.WifiScanner;

/**
 * Odyssey Remote ID test receiver.
 *
 * <p>Deliberately one screen and a text report rather than a map and a card list. The job
 * is answering "is the aircraft broadcasting what I configured, at the required rate" on a
 * bench, and a report you can read, screenshot and paste into a build log serves that
 * better than a UI that looks impressive.
 */
public final class MainActivity extends Activity implements BleScanner.Listener,
                                                            WifiScanner.Listener {

    private static final int REQ_PERMISSIONS = 1;
    private static final long REFRESH_MS = 1000;

    private final AircraftStore store = new AircraftStore();
    private final Handler handler = new Handler(Looper.getMainLooper());

    private BleScanner ble;
    private WifiScanner wifi;

    private TextView report;
    private TextView events;
    private ScrollView scroll;
    private Button scanButton;
    private EditText expectedUasId;
    private EditText expectedOperatorId;

    private final StringBuilder eventLog = new StringBuilder();
    private Runnable refresher;

    @Override protected void onCreate(Bundle saved) {
        super.onCreate(saved);
        setContentView(R.layout.activity_main);

        report = findViewById(R.id.report);
        events = findViewById(R.id.events);
        scroll = findViewById(R.id.scroll);
        scanButton = findViewById(R.id.scanButton);
        expectedUasId = findViewById(R.id.expectedUasId);
        expectedOperatorId = findViewById(R.id.expectedOperatorId);

        ble  = new BleScanner(this, store, this);
        wifi = new WifiScanner(this, store, this);

        scanButton.setOnClickListener(v -> toggleScan());
        findViewById(R.id.selfTestButton).setOnClickListener(v -> runSelfTest());
        findViewById(R.id.clearButton).setOnClickListener(v -> {
            store.clear();
            eventLog.setLength(0);
            events.setText("");
            refresh();
        });
        findViewById(R.id.shareButton).setOnClickListener(v -> shareReport());

        appendEvent("Odyssey Remote ID test receiver");
        appendEvent("Extended advertising (Coded PHY) supported: "
                  + ble.isExtendedScanSupported());
        appendEvent("Wi-Fi IE reading supported: " + WifiScanner.isSupported()
                  + (WifiScanner.isSupported() ? "" : " (needs Android 11+)"));
        appendEvent("Run the self-test first -- it rules out the decoder.");

        refresher = new Runnable() {
            @Override public void run() {
                refresh();
                handler.postDelayed(this, REFRESH_MS);
            }
        };
        handler.post(refresher);
        refresh();
    }

    @Override protected void onDestroy() {
        super.onDestroy();
        handler.removeCallbacks(refresher);
        if (ble != null) ble.stop();
        if (wifi != null) wifi.stop();
    }

    // ---------------------------------------------------------------------------------
    private void toggleScan() {
        if (ble.isScanning() || wifi.isRunning()) {
            ble.stop();
            wifi.stop();
            scanButton.setText(R.string.start_scan);
            appendEvent("Scanning stopped.");
            return;
        }
        if (!hasPermissions()) { requestPermissions(); return; }
        startScanning();
    }

    private void startScanning() {
        store.expectedUasId = expectedUasId.getText().toString().trim();
        store.expectedOperatorId = expectedOperatorId.getText().toString().trim();

        String bleErr = ble.start();
        if (bleErr != null) appendEvent("Bluetooth: " + bleErr);
        else                appendEvent("Bluetooth scanning started.");

        String wifiErr = wifi.start();
        if (wifiErr != null) appendEvent("Wi-Fi: " + wifiErr);

        if (bleErr != null && wifiErr != null) {
            Toast.makeText(this, "Neither radio could start", Toast.LENGTH_LONG).show();
            return;
        }
        scanButton.setText(R.string.stop_scan);
    }

    private void runSelfTest() {
        List<OdidSelfTest.Line> lines = OdidSelfTest.run();
        StringBuilder sb = new StringBuilder("Decoder self-test\n");
        sb.append("=================\n");
        int fail = 0;
        for (OdidSelfTest.Line l : lines) {
            sb.append("  ").append(l).append('\n');
            if (!l.ok) fail++;
        }
        sb.append('\n');
        if (fail == 0) {
            sb.append("Decoder is working. If you still see no aircraft, the problem is\n")
              .append("the radio, the permissions, or the aircraft -- not this app.\n\n")
              .append("Remember the Odyssey firmware broadcasts NOTHING until its UAS ID\n")
              .append("and operator registration are both valid. Check its serial console.\n");
        } else {
            sb.append(fail).append(" self-test failure(s). The decoder itself is faulty;\n")
              .append("do not trust anything else this app reports.\n");
        }
        report.setText(sb.toString());
        appendEvent("Self-test: " + (lines.size() - fail) + " passed, " + fail + " failed");
    }

    private void refresh() {
        if (store.size() > 0 || !report.getText().toString().startsWith("Decoder self-test")) {
            report.setText(store.report(System.currentTimeMillis()));
        }
    }

    private void shareReport() {
        String text = report.getText().toString() + "\n\n--- events ---\n" + eventLog;
        Intent i = new Intent(Intent.ACTION_SEND);
        i.setType("text/plain");
        i.putExtra(Intent.EXTRA_SUBJECT, "Odyssey Remote ID test report");
        i.putExtra(Intent.EXTRA_TEXT, text);
        startActivity(Intent.createChooser(i, "Share report"));
    }

    private void appendEvent(String s) {
        eventLog.append(s).append('\n');
        events.setText(eventLog.toString());
        scroll.post(() -> scroll.fullScroll(View.FOCUS_DOWN));
    }

    @Override public void onScanEvent(String message) {
        runOnUiThread(() -> appendEvent(message));
    }

    // ---------------------------------------------------------------------------------
    //  Permissions
    // ---------------------------------------------------------------------------------
    private String[] neededPermissions() {
        List<String> p = new ArrayList<>();
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            p.add(Manifest.permission.BLUETOOTH_SCAN);
            p.add(Manifest.permission.BLUETOOTH_CONNECT);
        }
        // Location is required for BLE scanning below API 31, and for Wi-Fi scan
        // results on every version.
        p.add(Manifest.permission.ACCESS_FINE_LOCATION);
        return p.toArray(new String[0]);
    }

    private boolean hasPermissions() {
        for (String s : neededPermissions()) {
            if (checkSelfPermission(s) != PackageManager.PERMISSION_GRANTED) return false;
        }
        return true;
    }

    private void requestPermissions() {
        requestPermissions(neededPermissions(), REQ_PERMISSIONS);
    }

    @Override public void onRequestPermissionsResult(int requestCode, String[] permissions,
                                                     int[] grantResults) {
        if (requestCode != REQ_PERMISSIONS) return;
        for (int r : grantResults) {
            if (r != PackageManager.PERMISSION_GRANTED) {
                appendEvent("Permission denied. Bluetooth scanning cannot run without it, "
                          + "and the result looks the same as a silent aircraft.");
                return;
            }
        }
        startScanning();
    }
}
