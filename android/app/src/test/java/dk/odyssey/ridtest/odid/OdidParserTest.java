package dk.odyssey.ridtest.odid;

import java.util.Locale;

/**
 * Decoder tests.
 *
 * <p>Plain {@code main()} rather than JUnit, deliberately. The whole app has no external
 * dependencies, and that includes its tests: this compiles and runs with nothing but
 * {@code javac} and {@code java} in about a second, which is what lets the pre-push hook
 * run it on every push without dragging in Gradle. See
 * {@code tools/run_android_parser_tests.sh}.
 *
 * <p>These verify the decoder against the encoder in this repository. They do NOT verify
 * conformance to ASTM F3411 -- for that, decode a real broadcast from the aircraft, which
 * encodes with the reference library. See the note on {@link OdidParser}.
 */
public final class OdidParserTest {

    private static int passed = 0;
    private static int failed = 0;

    public static void main(String[] args) {
        System.out.println("=====================================================================");
        System.out.println(" Odyssey Remote ID -- decoder tests");
        System.out.println("=====================================================================");

        testSelfTestVector();
        testBasicIdTypes();
        testLocationEncoding();
        testUnknownSentinels();
        testMessagePack();
        testMalformed();
        testStringHandling();
        testOperatorSecretNeverDecodesFromPublic();

        System.out.println("\n=====================================================================");
        System.out.printf(" %d passed, %d failed%n", passed, failed);
        System.out.println("=====================================================================");
        System.exit(failed == 0 ? 0 : 1);
    }

    // ---------------------------------------------------------------------------------
    private static void testSelfTestVector() {
        section("The on-device self-test vector");
        for (OdidSelfTest.Line l : OdidSelfTest.run()) {
            check(l.ok, l.text);
        }
    }

    private static void testBasicIdTypes() {
        section("Basic ID");

        // The CAA registration route: the default, needing no ICAO manufacturer code.
        OdidParser.Decoded d = OdidParser.parseMessages(
                OdidEncoder.basicId(OdidMessage.ID_TYPE_CAA_REGISTRATION, 2, "DNK87astrdge12k8"));
        check(d.basicIds.size() == 1, "one Basic ID decoded");
        check(d.basicIds.get(0).idType == OdidMessage.ID_TYPE_CAA_REGISTRATION,
              "ID type CAA registration survives");
        check(d.basicIds.get(0).uasId.equals("DNK87astrdge12k8"), "CAA registration text");
        check(d.basicIds.get(0).uaType == 2, "UA type multirotor");

        // The CTA serial route, for a manufactured airframe or a bought module.
        d = OdidParser.parseMessages(
                OdidEncoder.basicId(OdidMessage.ID_TYPE_SERIAL_NUMBER, 2, "K7E3F000000000000001"));
        check(d.basicIds.get(0).idType == OdidMessage.ID_TYPE_SERIAL_NUMBER,
              "ID type serial number survives");
        check(d.basicIds.get(0).uasId.equals("K7E3F000000000000001"),
              "a full 20-character CTA serial survives without truncation");
        check(d.basicIds.get(0).uasId.length() == 20,
              "20 characters exactly fills the ODID field");

        check(OdidMessage.idTypeName(OdidMessage.ID_TYPE_CAA_REGISTRATION)
                .equals("CAA registration"), "ID type name");
    }

    private static void testLocationEncoding() {
        section("Location encoding round-trips");

        // Southern and western hemispheres, to catch sign handling in the 32-bit fields.
        double[][] places = {
            { 55.676098,  12.568337 },   // Copenhagen
            { -33.868820, 151.209290 },  // Sydney
            { -34.603722, -58.381592 },  // Buenos Aires
            {  64.146582, -21.942635 },  // Reykjavik
        };
        for (double[] p : places) {
            OdidParser.Decoded d = OdidParser.parseMessages(
                    OdidEncoder.location(2, 45, 10, 0, p[0], p[1], 100, 50, 0));
            OdidMessage.Location l = d.locations.get(0);
            check(l.positionValid && near(l.latitude, p[0], 1e-6) && near(l.longitude, p[1], 1e-6),
                  String.format(Locale.ROOT, "position %.6f, %.6f round-trips", p[0], p[1]));
        }

        // Direction spans two encodings, split at 180 by the E/W segment bit.
        for (int dir : new int[] { 0, 45, 179, 180, 270, 359 }) {
            OdidParser.Decoded d = OdidParser.parseMessages(
                    OdidEncoder.location(2, dir, 5, 0, 55, 12, 100, 50, 0));
            OdidMessage.Location l = d.locations.get(0);
            check(l.directionValid && Math.abs(l.directionDeg - dir) < 1.5,
                  "direction " + dir + " deg round-trips (got " + l.directionDeg + ")");
        }

        // Horizontal speed uses two ranges selected by the multiplier bit; check both
        // sides of the switch-over at 63.75 m/s.
        for (double sp : new double[] { 0, 5.0, 12.0, 50.0, 63.0, 70.0, 120.0 }) {
            OdidParser.Decoded d = OdidParser.parseMessages(
                    OdidEncoder.location(2, 0, sp, 0, 55, 12, 100, 50, 0));
            OdidMessage.Location l = d.locations.get(0);
            check(l.speedHorizontalValid && Math.abs(l.speedHorizontalMps - sp) <= 0.8,
                  String.format(Locale.ROOT, "speed %.1f m/s round-trips (got %.2f)",
                                sp, l.speedHorizontalMps));
        }

        // Vertical speed is signed.
        for (double vs : new double[] { -10.0, -1.5, 0.0, 2.5, 15.0 }) {
            OdidParser.Decoded d = OdidParser.parseMessages(
                    OdidEncoder.location(2, 0, 0, vs, 55, 12, 100, 50, 0));
            OdidMessage.Location l = d.locations.get(0);
            check(l.speedVerticalValid && Math.abs(l.speedVerticalMps - vs) <= 0.5,
                  String.format(Locale.ROOT, "vertical speed %.1f m/s round-trips", vs));
        }

        // Altitude uses a +1000 m offset, so negative altitudes must survive.
        for (double alt : new double[] { -400.0, 0.0, 120.0, 3000.0 }) {
            OdidParser.Decoded d = OdidParser.parseMessages(
                    OdidEncoder.location(2, 0, 0, 0, 55, 12, alt, alt, 0));
            OdidMessage.Location l = d.locations.get(0);
            check(l.altitudeGeoValid && Math.abs(l.altitudeGeoM - alt) <= 0.5,
                  String.format(Locale.ROOT, "altitude %.0f m round-trips", alt));
        }

        // Status values map to the right names.
        OdidParser.Decoded d = OdidParser.parseMessages(
                OdidEncoder.location(3, 0, 0, 0, 55, 12, 100, 50, 0));
        check(d.locations.get(0).status == 3, "emergency status survives");
        check(OdidMessage.statusName(3).equals("emergency"), "status name");
    }

    private static void testUnknownSentinels() {
        section("Unknown values are reported as unknown, not as zero");

        // A receiver that renders an unknown altitude as 0.0 is worse than one that says
        // nothing, because 0.0 looks like a measurement.
        OdidParser.Decoded d = OdidParser.parseMessages(
                OdidEncoder.location(1, -1, -1, 0, 55, 12, 100, 50, 0));
        OdidMessage.Location l = d.locations.get(0);
        check(!l.directionValid, "unknown direction flagged invalid");
        check(!l.speedHorizontalValid, "unknown horizontal speed flagged invalid");

        // The encoder writes 0 for an unknown barometric altitude.
        check(!l.altitudeBaroValid, "unset barometric altitude flagged invalid");

        // 0,0 position is the standard's invalid marker -- and was literally what the
        // original beacon design broadcast for its entire endurance (finding 7).
        d = OdidParser.parseMessages(OdidEncoder.location(1, 0, 0, 0, 0, 0, 100, 50, 0));
        check(!d.locations.get(0).positionValid,
              "0,0 position is rejected -- the Null Island case from finding 7");
    }

    private static void testMessagePack() {
        section("Message pack");

        byte[] pack = OdidEncoder.messagePack(
                OdidEncoder.basicId(OdidMessage.ID_TYPE_CAA_REGISTRATION, 2, "A"),
                OdidEncoder.location(2, 0, 0, 0, 55, 12, 100, 50, 0),
                OdidEncoder.system(1, 0, 55, 12, 40, 1),
                OdidEncoder.operatorId("DNK87astrdge12k8"),
                OdidEncoder.selfId("bench test"));

        OdidParser.Decoded d = OdidParser.parseMessages(pack);
        check(d.error == null, "five-message pack decodes");
        check(d.totalMessages() == 5, "all five recovered, got " + d.totalMessages());
        check(d.selfIds.size() == 1 && d.selfIds.get(0).description.equals("bench test"),
              "self ID text survives");
        check(d.systems.size() == 1 && d.systems.get(0).categoryEu == 1,
              "EU category open survives");

        // A pack claiming more messages than it carries must be rejected, not
        // partially decoded from whatever memory happens to follow.
        byte[] truncated = new byte[pack.length - 10];
        System.arraycopy(pack, 0, truncated, 0, truncated.length);
        OdidParser.Decoded bad = OdidParser.parseMessages(truncated);
        check(bad.error != null, "a truncated pack is rejected, not partially decoded");

        // Bare back-to-back messages, without a pack wrapper.
        byte[] two = new byte[50];
        System.arraycopy(OdidEncoder.basicId(2, 2, "X"), 0, two, 0, 25);
        System.arraycopy(OdidEncoder.operatorId("Y"), 0, two, 25, 25);
        OdidParser.Decoded bare = OdidParser.parseMessages(two);
        check(bare.totalMessages() == 2, "two bare messages decode without a pack");
    }

    private static void testMalformed() {
        section("Malformed input is rejected");

        check(OdidParser.parseBluetoothServiceData(null).error != null, "null service data");
        check(OdidParser.parseBluetoothServiceData(new byte[0]).error != null, "empty");
        check(OdidParser.parseBluetoothServiceData(new byte[] { 0x0D }).error != null,
              "app code with no counter");
        check(OdidParser.parseBluetoothServiceData(new byte[] { 0x42, 0x00 }).error != null,
              "wrong application code");
        check(OdidParser.parseMessages(new byte[] { 0x02, 0x03 }).error != null,
              "message shorter than 25 bytes");

        check(OdidParser.parseWifiVendorIe(null).error != null, "null vendor IE");
        check(OdidParser.parseWifiVendorIe(new byte[] { 0x00, 0x01, 0x02, 0x03, 0x04 }).error != null,
              "wrong OUI is rejected");

        // A pack declaring a non-standard message size must be refused rather than
        // walked with the wrong stride.
        byte[] weird = new byte[] { (byte) 0xF2, 30, 2, 0, 0, 0 };
        check(OdidParser.parseMessages(weird).error != null,
              "a pack declaring 30-byte messages is refused");

        // An unknown message type must be counted, not silently dropped or crashed on.
        byte[] unknown = new byte[25];
        unknown[0] = (byte) 0xA2;                 // type 0xA is not defined
        OdidParser.Decoded d = OdidParser.parseMessages(unknown);
        check(d.unknownTypes == 1, "an unknown message type is counted");
        check(d.error == null, "an unknown type is not treated as an error");
    }

    private static void testStringHandling() {
        section("Identifier text handling");

        check(OdidParser.fixedString(new byte[] { 'A', 'B', 0, 'X' }, 0, 4).equals("AB"),
              "null terminates the field");
        check(OdidParser.fixedString(new byte[] { 'A', ' ', ' ', ' ' }, 0, 4).equals("A"),
              "space padding is trimmed");
        check(OdidParser.fixedString(new byte[] { 'A', (byte) 0xFF, 'B', 0 }, 0, 4).equals("AB"),
              "non-printable bytes are dropped, not rendered as mojibake");
        check(OdidParser.fixedString(new byte[] { 'A' }, 0, 20).equals("A"),
              "a field longer than the buffer does not overrun");

        check(OdidParser.hex(new byte[] { 0x00, (byte) 0xFF, 0x0D }).equals("00FF0D"),
              "hex helper");
        check(OdidParser.unhex("00FF0D")[1] == (byte) 0xFF, "unhex helper");
    }

    private static void testOperatorSecretNeverDecodesFromPublic() {
        section("Operator secret must not appear on air");

        // The firmware transmits only the public 16 characters. If a hyphen ever shows
        // up in a decoded operator ID, the 3-character secret is going out over the air,
        // which the conformance evaluation must flag as a failure.
        OdidConformance.Tracker t = new OdidConformance.Tracker();
        long now = 1_000_000L;

        OdidParser.Decoded leaky = OdidParser.parseMessages(
                OdidEncoder.operatorId("DNK87astrdge12k8-xyz"));
        t.onDecoded(leaky, now);

        boolean flagged = false;
        for (OdidConformance.Observation o :
                OdidConformance.evaluate(t, null, null)) {
            if (o.name.equals("Operator secret withheld")
                    && o.result == OdidConformance.Result.FAIL) {
                flagged = true;
            }
        }
        check(flagged, "a broadcast secret suffix is flagged as a FAILURE");

        // The correct case must pass.
        OdidConformance.Tracker clean = new OdidConformance.Tracker();
        clean.onDecoded(OdidParser.parseMessages(
                OdidEncoder.operatorId("DNK87astrdge12k8")), now);
        boolean ok = false;
        for (OdidConformance.Observation o : OdidConformance.evaluate(clean, null, null)) {
            if (o.name.equals("Operator secret withheld")
                    && o.result == OdidConformance.Result.PASS) {
                ok = true;
            }
        }
        check(ok, "a public-only operator ID passes");
    }

    // ---------------------------------------------------------------------------------
    private static void section(String s) {
        System.out.println("\n" + s);
        StringBuilder u = new StringBuilder();
        for (int i = 0; i < s.length(); ++i) u.append('-');
        System.out.println(u);
    }

    private static void check(boolean ok, String what) {
        if (ok) { passed++; System.out.println("  PASS  " + what); }
        else    { failed++; System.out.println("  FAIL  " + what); }
    }

    private static boolean near(double a, double b, double tol) {
        return Math.abs(a - b) <= tol;
    }
}
