package dk.odyssey.ridtest.odid;

/**
 * Minimal ASTM F3411 encoder.
 *
 * <p>This exists for two reasons, neither of which is transmitting:
 *
 * <ol>
 *   <li><b>Round-trip testing.</b> Encoding a known structure and decoding it back
 *       exercises every field of {@link OdidParser} without needing an aircraft.</li>
 *   <li><b>On-device self-test.</b> {@link OdidSelfTest} uses it so a user can tell
 *       "this app is broken" from "the aircraft is not broadcasting" -- a distinction
 *       that matters, because the Odyssey firmware deliberately goes silent when its
 *       identifiers are invalid.</li>
 * </ol>
 *
 * <p>A round trip proves the decoder is self-consistent with THIS encoder. It does not
 * prove either conforms to the standard -- for that, decode a real broadcast from the
 * aircraft, which encodes with the reference library. See the note on {@link OdidParser}.
 */
public final class OdidEncoder {

    private OdidEncoder() {}

    public static final int PROTOCOL_VERSION = 2;

    private static byte[] blank(int type) {
        byte[] m = new byte[OdidMessage.MESSAGE_SIZE];
        m[0] = (byte) ((type << 4) | PROTOCOL_VERSION);
        return m;
    }

    public static byte[] basicId(int idType, int uaType, String uasId) {
        byte[] m = blank(OdidMessage.TYPE_BASIC_ID);
        m[1] = (byte) (((idType & 0x0F) << 4) | (uaType & 0x0F));
        putFixed(m, 2, 20, uasId);
        return m;
    }

    public static byte[] operatorId(String operatorId) {
        byte[] m = blank(OdidMessage.TYPE_OPERATOR_ID);
        m[1] = 0;                                     // operator ID type
        putFixed(m, 2, 20, operatorId);
        return m;
    }

    public static byte[] selfId(String description) {
        byte[] m = blank(OdidMessage.TYPE_SELF_ID);
        m[1] = 0;
        putFixed(m, 2, 23, description);
        return m;
    }

    /**
     * @param directionDeg     0..359, or negative for unknown
     * @param speedHorizontal  m/s, or negative for unknown
     * @param heightM          metres above the take-off point
     */
    public static byte[] location(int status, double directionDeg, double speedHorizontal,
                                  double speedVertical, double lat, double lon,
                                  double altitudeGeoM, double heightM,
                                  double timestampSec) {
        byte[] m = blank(OdidMessage.TYPE_LOCATION);

        boolean ewSegment = directionDeg >= 180;
        boolean speedMult = speedHorizontal > (255 * 0.25);

        m[1] = (byte) (((status & 0x0F) << 4)
                     | (1 << 2)                       // height is above take-off
                     | (ewSegment ? (1 << 1) : 0)
                     | (speedMult ? 1 : 0));

        if (directionDeg < 0) {
            m[2] = (byte) 181;                        // outside 0..179 => unknown
        } else {
            m[2] = (byte) ((int) directionDeg % 180);
        }

        if (speedHorizontal < 0) {
            m[3] = (byte) 255;
        } else if (speedMult) {
            m[3] = (byte) clamp((int) Math.round((speedHorizontal - (255 * 0.25)) / 0.75), 0, 254);
        } else {
            m[3] = (byte) clamp((int) Math.round(speedHorizontal / 0.25), 0, 254);
        }

        m[4] = (byte) clamp((int) Math.round(speedVertical / 0.5), -62, 62);

        putLe32(m, 5,  (int) Math.round(lat * 1e7));
        putLe32(m, 9,  (int) Math.round(lon * 1e7));
        putLe16(m, 13, 0);                            // barometric altitude unknown
        putLe16(m, 15, encodeAltitude(altitudeGeoM));
        putLe16(m, 17, encodeAltitude(heightM));

        m[19] = (byte) ((0 << 4) | 10);               // vert unknown, horiz <10 m
        m[20] = 0;
        putLe16(m, 21, (int) Math.round(timestampSec * 10.0));
        m[23] = 0;
        return m;
    }

    public static byte[] system(int categoryEu, int classEu, double operatorLat,
                                double operatorLon, double operatorAltM, long timestamp) {
        byte[] m = blank(OdidMessage.TYPE_SYSTEM);
        m[1] = (byte) ((1 << 2) | 0);                 // EU classification, takeoff location
        putLe32(m, 2,  (int) Math.round(operatorLat * 1e7));
        putLe32(m, 6,  (int) Math.round(operatorLon * 1e7));
        putLe16(m, 10, 1);                            // area count
        m[12] = 0;                                    // area radius
        putLe16(m, 13, 0);                            // ceiling unknown
        putLe16(m, 15, 0);                            // floor unknown
        m[17] = (byte) (((categoryEu & 0x0F) << 4) | (classEu & 0x0F));
        putLe16(m, 18, encodeAltitude(operatorAltM));
        putLe32(m, 20, (int) timestamp);
        return m;
    }

    /** Wraps messages into a Message Pack, which is how several are sent in one advert. */
    public static byte[] messagePack(byte[]... messages) {
        byte[] out = new byte[3 + messages.length * OdidMessage.MESSAGE_SIZE];
        out[0] = (byte) ((OdidMessage.TYPE_MESSAGE_PACK << 4) | PROTOCOL_VERSION);
        out[1] = (byte) OdidMessage.MESSAGE_SIZE;
        out[2] = (byte) messages.length;
        for (int i = 0; i < messages.length; ++i) {
            System.arraycopy(messages[i], 0, out,
                             3 + i * OdidMessage.MESSAGE_SIZE, OdidMessage.MESSAGE_SIZE);
        }
        return out;
    }

    /** Wraps a payload as Bluetooth service data: app code, counter, then messages. */
    public static byte[] bluetoothServiceData(int counter, byte[] payload) {
        byte[] out = new byte[2 + payload.length];
        out[0] = OdidParser.ASTM_APP_CODE;
        out[1] = (byte) counter;
        System.arraycopy(payload, 0, out, 2, payload.length);
        return out;
    }

    /** Wraps a payload as a Wi-Fi vendor IE body: OUI, type, counter, then messages. */
    public static byte[] wifiVendorIe(int counter, byte[] payload) {
        byte[] out = new byte[5 + payload.length];
        System.arraycopy(OdidParser.WIFI_OUI, 0, out, 0, 3);
        out[3] = OdidParser.WIFI_OUI_TYPE;
        out[4] = (byte) counter;
        System.arraycopy(payload, 0, out, 5, payload.length);
        return out;
    }

    // ---------------------------------------------------------------------------------
    private static int encodeAltitude(double metres) {
        return clamp((int) Math.round((metres + 1000.0) / 0.5), 1, 0xFFFF);
    }

    private static int clamp(int v, int lo, int hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    private static void putFixed(byte[] d, int off, int len, String s) {
        for (int i = 0; i < len; ++i) {
            d[off + i] = (i < s.length()) ? (byte) s.charAt(i) : 0;
        }
    }

    private static void putLe16(byte[] d, int off, int v) {
        d[off]     = (byte) (v & 0xFF);
        d[off + 1] = (byte) ((v >> 8) & 0xFF);
    }

    private static void putLe32(byte[] d, int off, int v) {
        d[off]     = (byte) (v & 0xFF);
        d[off + 1] = (byte) ((v >> 8) & 0xFF);
        d[off + 2] = (byte) ((v >> 16) & 0xFF);
        d[off + 3] = (byte) ((v >> 24) & 0xFF);
    }
}
