package dk.odyssey.ridtest.odid;

import java.util.ArrayList;
import java.util.Locale;
import java.util.List;

/**
 * On-device decoder self-test.
 *
 * <p>This exists because of a specific behaviour in the Odyssey firmware: when its
 * identifiers are not valid it <b>broadcasts nothing at all</b>, deliberately, because an
 * untraceable identifier in a receiver's log is worse than silence.
 *
 * <p>That creates an ambiguity for whoever is holding the phone. "I see nothing" could
 * mean the aircraft is silent, or the Bluetooth permission was denied, or this decoder is
 * broken. Running a known vector through the decoder collapses one of those possibilities
 * before you start chasing the others.
 */
public final class OdidSelfTest {

    private OdidSelfTest() {}

    public static final class Line {
        public final boolean ok;
        public final String  text;
        Line(boolean ok, String text) { this.ok = ok; this.text = text; }
        @Override public String toString() { return (ok ? "PASS  " : "FAIL  ") + text; }
    }

    /** Builds a representative broadcast and checks it decodes back to the same values. */
    public static List<Line> run() {
        List<Line> out = new ArrayList<>();

        final String uasId = "DNK87astrdge12k8";
        final String opId  = "DNK87astrdge12k8";
        final double lat = 55.676098, lon = 12.568337;   // Copenhagen
        final double heightM = 42.5;

        byte[] pack = OdidEncoder.messagePack(
                OdidEncoder.basicId(OdidMessage.ID_TYPE_CAA_REGISTRATION, 2, uasId),
                OdidEncoder.location(2, 90.0, 12.0, -1.5, lat, lon, 90.0, heightM, 1234.5),
                OdidEncoder.system(1, 0, lat, lon, 47.0, 1000000L),
                OdidEncoder.operatorId(opId));

        byte[] sd = OdidEncoder.bluetoothServiceData(0x42, pack);
        OdidParser.Decoded d = OdidParser.parseBluetoothServiceData(sd);

        check(out, d.error == null, "advertisement decodes"
                + (d.error != null ? " (" + d.error + ")" : ""));
        check(out, d.messageCounter == 0x42, "message counter read (0x42)");
        check(out, d.totalMessages() == 4, "four messages in the pack, got " + d.totalMessages());

        check(out, d.basicIds.size() == 1 && d.basicIds.get(0).uasId.equals(uasId),
              "UAS ID round-trips: " + (d.basicIds.isEmpty() ? "<none>" : d.basicIds.get(0).uasId));
        check(out, !d.basicIds.isEmpty()
                && d.basicIds.get(0).idType == OdidMessage.ID_TYPE_CAA_REGISTRATION,
              "ID type is CAA registration");

        check(out, d.operators.size() == 1 && d.operators.get(0).operatorId.equals(opId),
              "operator ID round-trips");
        check(out, d.operators.isEmpty() || !d.operators.get(0).operatorId.contains("-"),
              "no secret suffix present");

        if (d.locations.size() == 1) {
            OdidMessage.Location l = d.locations.get(0);
            check(out, l.positionValid && near(l.latitude, lat, 1e-6)
                    && near(l.longitude, lon, 1e-6),
                  String.format(Locale.ROOT, "position round-trips: %.6f, %.6f", l.latitude, l.longitude));
            check(out, l.heightValid && near(l.heightM, heightM, 0.5),
                  String.format(Locale.ROOT, "height round-trips: %.1f m", l.heightM));
            check(out, l.status == 2, "status is airborne");
            check(out, l.speedHorizontalValid && near(l.speedHorizontalMps, 12.0, 0.3),
                  String.format(Locale.ROOT, "ground speed round-trips: %.2f m/s", l.speedHorizontalMps));
            check(out, l.speedVerticalValid && near(l.speedVerticalMps, -1.5, 0.3),
                  String.format(Locale.ROOT, "vertical speed round-trips: %.2f m/s", l.speedVerticalMps));
        } else {
            check(out, false, "exactly one Location message (got " + d.locations.size() + ")");
        }

        check(out, d.systems.size() == 1 && d.systems.get(0).classEu == 0,
              "EU class is undeclared, correct for a home build");

        // The Wi-Fi path must reach the same result from the same payload.
        OdidParser.Decoded w = OdidParser.parseWifiVendorIe(
                OdidEncoder.wifiVendorIe(0x42, pack));
        check(out, w.error == null && w.totalMessages() == d.totalMessages(),
              "Wi-Fi vendor IE decodes identically");

        // Malformed input must be rejected, not guessed at.
        check(out, OdidParser.parseBluetoothServiceData(new byte[] { 0x0D }).error != null,
              "truncated advertisement is rejected");
        check(out, OdidParser.parseBluetoothServiceData(
                new byte[] { 0x01, 0x02, 0x03 }).error != null,
              "non-ASTM application code is rejected");

        return out;
    }

    public static boolean allPassed(List<Line> lines) {
        for (Line l : lines) if (!l.ok) return false;
        return true;
    }

    private static void check(List<Line> out, boolean ok, String text) {
        out.add(new Line(ok, text));
    }

    private static boolean near(double a, double b, double tol) {
        return Math.abs(a - b) <= tol;
    }
}
