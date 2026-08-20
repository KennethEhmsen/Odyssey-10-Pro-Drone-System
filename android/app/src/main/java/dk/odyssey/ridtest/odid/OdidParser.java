package dk.odyssey.ridtest.odid;

import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;

/**
 * ASTM F3411 / ASD-STAN prEN 4709-002 message decoder.
 *
 * <h2>Confidence</h2>
 *
 * The aircraft firmware encodes with {@code opendroneid/opendroneid-core-c}, the
 * reference implementation, precisely because the field packing is bit-exact and easy to
 * get subtly wrong. This decoder is hand-written from the documented structure, so it
 * carries the opposite risk.
 *
 * <p>That is deliberate and it is the point of the tool. The encoder is trusted; the
 * decoder is not. When this app decodes a real broadcast from the aircraft and the
 * identifiers and position come out matching what was configured, <b>both sides are
 * cross-validated</b> -- an independent implementation agreeing with the reference one
 * is far stronger evidence than either alone.
 *
 * <p>Until that has happened on real hardware, treat a decode failure as "the decoder
 * might be wrong" rather than "the aircraft is wrong". {@link OdidSelfTest} decodes a
 * built-in vector so you can tell a broken app from a silent aircraft.
 *
 * <h2>Layout</h2>
 *
 * Every message is 25 bytes: header byte {@code (type << 4) | version}, then 24 bytes of
 * body. Multi-byte integers are little-endian.
 */
public final class OdidParser {

    private OdidParser() {}

    /** Bluetooth: ASTM service data sits under 16-bit UUID 0xFFFA. */
    public static final int  ASTM_SERVICE_UUID = 0xFFFA;
    /** First byte of the service data payload identifies the ASTM application. */
    public static final byte ASTM_APP_CODE     = 0x0D;

    /** Wi-Fi: vendor-specific IE under OUI FA-0B-BC, type 0x0D. */
    public static final byte[] WIFI_OUI      = { (byte) 0xFA, 0x0B, (byte) 0xBC };
    public static final byte   WIFI_OUI_TYPE = 0x0D;

    /** Result of decoding one advertisement, which may carry several messages. */
    public static final class Decoded {
        public final List<OdidMessage.BasicId>    basicIds   = new ArrayList<>();
        public final List<OdidMessage.Location>   locations  = new ArrayList<>();
        public final List<OdidMessage.SystemMsg>  systems    = new ArrayList<>();
        public final List<OdidMessage.OperatorId> operators  = new ArrayList<>();
        public final List<OdidMessage.SelfId>     selfIds    = new ArrayList<>();
        public int     messageCounter = -1;
        public int     protocolVersion = -1;
        public int     unknownTypes = 0;
        public String  error;

        public boolean isEmpty() {
            return basicIds.isEmpty() && locations.isEmpty() && systems.isEmpty()
                && operators.isEmpty() && selfIds.isEmpty();
        }

        public int totalMessages() {
            return basicIds.size() + locations.size() + systems.size()
                 + operators.size() + selfIds.size() + unknownTypes;
        }
    }

    // ===================================================================================
    //  Entry points
    // ===================================================================================

    /**
     * Decodes the payload of a Bluetooth Service Data field for UUID 0xFFFA.
     *
     * @param serviceData bytes AFTER the 16-bit UUID, i.e. starting at the app code.
     */
    public static Decoded parseBluetoothServiceData(byte[] serviceData) {
        Decoded out = new Decoded();
        if (serviceData == null || serviceData.length < 2) {
            out.error = "service data too short (" +
                        (serviceData == null ? 0 : serviceData.length) + " bytes)";
            return out;
        }
        if (serviceData[0] != ASTM_APP_CODE) {
            out.error = String.format("not ASTM application data (app code 0x%02X)",
                                      serviceData[0]);
            return out;
        }
        out.messageCounter = serviceData[1] & 0xFF;

        byte[] body = new byte[serviceData.length - 2];
        System.arraycopy(serviceData, 2, body, 0, body.length);
        parseMessagesInto(body, out);
        return out;
    }

    /**
     * Decodes a Wi-Fi vendor-specific information element.
     *
     * @param ieBody bytes after the element ID and length, i.e. starting at the OUI.
     */
    public static Decoded parseWifiVendorIe(byte[] ieBody) {
        Decoded out = new Decoded();
        if (ieBody == null || ieBody.length < 5) {
            out.error = "vendor IE too short";
            return out;
        }
        if (ieBody[0] != WIFI_OUI[0] || ieBody[1] != WIFI_OUI[1] || ieBody[2] != WIFI_OUI[2]) {
            out.error = "not the ASTM OUI";
            return out;
        }
        if (ieBody[3] != WIFI_OUI_TYPE) {
            out.error = "wrong OUI type";
            return out;
        }
        out.messageCounter = ieBody[4] & 0xFF;

        byte[] body = new byte[ieBody.length - 5];
        System.arraycopy(ieBody, 5, body, 0, body.length);
        parseMessagesInto(body, out);
        return out;
    }

    /** Decodes a raw run of one or more 25-byte messages, or a message pack. */
    public static Decoded parseMessages(byte[] data) {
        Decoded out = new Decoded();
        parseMessagesInto(data, out);
        return out;
    }

    private static void parseMessagesInto(byte[] data, Decoded out) {
        if (data == null || data.length == 0) {
            out.error = "no message data";
            return;
        }

        final int type = (data[0] >> 4) & 0x0F;
        out.protocolVersion = data[0] & 0x0F;

        if (type == OdidMessage.TYPE_MESSAGE_PACK) {
            if (data.length < 3) { out.error = "truncated message pack header"; return; }
            final int msgSize = data[1] & 0xFF;
            final int msgCount = data[2] & 0xFF;

            if (msgSize != OdidMessage.MESSAGE_SIZE) {
                out.error = "message pack declares " + msgSize + "-byte messages, expected "
                          + OdidMessage.MESSAGE_SIZE;
                return;
            }
            final int need = 3 + msgSize * msgCount;
            if (data.length < need) {
                out.error = "message pack declares " + msgCount + " messages (" + need
                          + " bytes) but only " + data.length + " present";
                return;
            }
            for (int i = 0; i < msgCount; ++i) {
                parseSingle(data, 3 + i * msgSize, out);
            }
            return;
        }

        // Not a pack: one or more bare messages back to back.
        int offset = 0;
        while (offset + OdidMessage.MESSAGE_SIZE <= data.length) {
            parseSingle(data, offset, out);
            offset += OdidMessage.MESSAGE_SIZE;
        }
        if (offset == 0) {
            out.error = "truncated message: " + data.length + " bytes, need "
                      + OdidMessage.MESSAGE_SIZE;
        }
    }

    private static void parseSingle(byte[] d, int off, Decoded out) {
        final int type = (d[off] >> 4) & 0x0F;
        final int base = off + 1;                 // body starts after the header byte

        switch (type) {
            case OdidMessage.TYPE_BASIC_ID:    out.basicIds.add(basicId(d, base));   break;
            case OdidMessage.TYPE_LOCATION:    out.locations.add(location(d, base)); break;
            case OdidMessage.TYPE_SYSTEM:      out.systems.add(system(d, base));     break;
            case OdidMessage.TYPE_OPERATOR_ID: out.operators.add(operatorId(d, base));break;
            case OdidMessage.TYPE_SELF_ID:     out.selfIds.add(selfId(d, base));     break;
            default:                           out.unknownTypes++;                   break;
        }
    }

    // ===================================================================================
    //  Individual messages
    // ===================================================================================

    private static OdidMessage.BasicId basicId(byte[] d, int b) {
        OdidMessage.BasicId m = new OdidMessage.BasicId();
        m.idType = (d[b] >> 4) & 0x0F;
        m.uaType = d[b] & 0x0F;
        m.uasId  = fixedString(d, b + 1, 20);
        return m;
    }

    private static OdidMessage.Location location(byte[] d, int b) {
        OdidMessage.Location m = new OdidMessage.Location();

        final int flags = d[b] & 0xFF;
        m.status              = (flags >> 4) & 0x0F;
        m.heightAboveTakeoff  = ((flags >> 2) & 0x01) != 0;
        final boolean ewSegment = ((flags >> 1) & 0x01) != 0;
        final boolean speedMult = (flags & 0x01) != 0;

        // Direction: 0..179 in the byte, plus 180 when the E/W segment bit is set.
        final int dir = d[b + 1] & 0xFF;
        if (dir <= 179) {
            m.directionDeg = dir + (ewSegment ? 180 : 0);
            m.directionValid = true;
        } else {
            m.directionDeg = -1;             // 361 is the "unknown" sentinel
            m.directionValid = false;
        }

        // Horizontal speed uses a two-range encoding selected by the multiplier bit.
        final int sh = d[b + 2] & 0xFF;
        if (sh == 255) {
            m.speedHorizontalValid = false;
        } else {
            m.speedHorizontalMps = speedMult ? (sh * 0.75) + (255 * 0.25) : sh * 0.25;
            m.speedHorizontalValid = true;
        }

        final int sv = d[b + 3];             // signed
        if (sv == 63) {                      // 63 * 0.5 = 31.5 is the unknown sentinel
            m.speedVerticalValid = false;
        } else {
            m.speedVerticalMps = sv * 0.5;
            m.speedVerticalValid = true;
        }

        final int latRaw = le32(d, b + 4);
        final int lonRaw = le32(d, b + 8);
        m.latitude  = latRaw / 1e7;
        m.longitude = lonRaw / 1e7;
        // 0,0 is the standard's "invalid" marker and also a real place nobody flies.
        m.positionValid = !(latRaw == 0 && lonRaw == 0)
                       && Math.abs(m.latitude) <= 90.0
                       && Math.abs(m.longitude) <= 180.0;

        m.altitudeBaroValid = decodeAltitude(le16(d, b + 12));
        m.altitudeBaroM     = altitudeValue(le16(d, b + 12));
        m.altitudeGeoValid  = decodeAltitude(le16(d, b + 14));
        m.altitudeGeoM      = altitudeValue(le16(d, b + 14));
        m.heightValid       = decodeAltitude(le16(d, b + 16));
        m.heightM           = altitudeValue(le16(d, b + 16));

        m.horizontalAccuracy = d[b + 18] & 0x0F;
        m.verticalAccuracy   = (d[b + 18] >> 4) & 0x0F;

        final int ts = le16(d, b + 20);
        if (ts == 0xFFFF) {
            m.timestampValid = false;
        } else {
            m.timestampSec = ts / 10.0;      // tenths of a second past the hour
            m.timestampValid = true;
        }
        return m;
    }

    // Altitudes share one encoding: (metres + 1000) / 0.5, with 0 meaning unknown.
    private static boolean decodeAltitude(int raw) { return raw != 0; }
    private static double  altitudeValue(int raw)  { return (raw * 0.5) - 1000.0; }

    private static OdidMessage.SystemMsg system(byte[] d, int b) {
        OdidMessage.SystemMsg m = new OdidMessage.SystemMsg();
        final int flags = d[b] & 0xFF;
        m.operatorLocationType = flags & 0x03;
        m.classificationType   = (flags >> 2) & 0x07;

        final int latRaw = le32(d, b + 1);
        final int lonRaw = le32(d, b + 5);
        m.operatorLatitude  = latRaw / 1e7;
        m.operatorLongitude = lonRaw / 1e7;
        m.operatorPositionValid = !(latRaw == 0 && lonRaw == 0);

        m.categoryEu = (d[b + 16] >> 4) & 0x0F;
        m.classEu    = d[b + 16] & 0x0F;

        final int altRaw = le16(d, b + 17);
        m.operatorAltitudeValid = decodeAltitude(altRaw);
        m.operatorAltitudeGeoM  = altitudeValue(altRaw);

        m.timestamp = le32(d, b + 19) & 0xFFFFFFFFL;
        return m;
    }

    private static OdidMessage.OperatorId operatorId(byte[] d, int b) {
        OdidMessage.OperatorId m = new OdidMessage.OperatorId();
        m.idType = d[b] & 0xFF;
        m.operatorId = fixedString(d, b + 1, 20);
        return m;
    }

    private static OdidMessage.SelfId selfId(byte[] d, int b) {
        OdidMessage.SelfId m = new OdidMessage.SelfId();
        m.descriptionType = d[b] & 0xFF;
        m.description = fixedString(d, b + 1, 23);
        return m;
    }

    // ===================================================================================
    //  Primitives
    // ===================================================================================

    /**
     * Reads a fixed-width field as text. The standard null-pads rather than
     * null-terminates, and some encoders pad with spaces, so both are trimmed. Bytes
     * outside printable ASCII are dropped rather than rendered as mojibake -- an
     * identifier is only useful if it can be read back accurately.
     */
    static String fixedString(byte[] d, int off, int len) {
        if (off + len > d.length) len = Math.max(0, d.length - off);
        int end = off + len;
        StringBuilder sb = new StringBuilder(len);
        for (int i = off; i < end; ++i) {
            final int c = d[i] & 0xFF;
            if (c == 0) break;
            if (c >= 0x20 && c < 0x7F) sb.append((char) c);
        }
        return sb.toString().trim();
    }

    static int le16(byte[] d, int off) {
        return (d[off] & 0xFF) | ((d[off + 1] & 0xFF) << 8);
    }

    static int le32(byte[] d, int off) {
        return (d[off] & 0xFF)
             | ((d[off + 1] & 0xFF) << 8)
             | ((d[off + 2] & 0xFF) << 16)
             | ((d[off + 3] & 0xFF) << 24);
    }

    /** Convenience for logs and the self-test. */
    public static String hex(byte[] d) {
        StringBuilder sb = new StringBuilder(d.length * 2);
        for (byte b : d) sb.append(String.format("%02X", b));
        return sb.toString();
    }

    public static byte[] unhex(String s) {
        s = s.replaceAll("\\s", "");
        byte[] out = new byte[s.length() / 2];
        for (int i = 0; i < out.length; ++i) {
            out[i] = (byte) Integer.parseInt(s.substring(i * 2, i * 2 + 2), 16);
        }
        return out;
    }
}
