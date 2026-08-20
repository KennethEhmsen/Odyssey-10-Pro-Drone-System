package dk.odyssey.ridtest.odid;

/**
 * Decoded OpenDroneID messages.
 *
 * <p>Every ASTM F3411 message is exactly 25 bytes: one header byte carrying the message
 * type and protocol version, then 24 bytes of body. This file holds the decoded forms.
 *
 * <p><b>No Android imports anywhere in this package.</b> That is deliberate: the decoder
 * is the part worth testing, and keeping it free of the Android framework means it runs
 * under a plain JVM. See {@code tools/run_android_parser_tests.sh}.
 */
public final class OdidMessage {

    private OdidMessage() {}

    public static final int MESSAGE_SIZE = 25;

    // Message types, from the high nibble of the header byte.
    public static final int TYPE_BASIC_ID     = 0x0;
    public static final int TYPE_LOCATION     = 0x1;
    public static final int TYPE_AUTH         = 0x2;
    public static final int TYPE_SELF_ID      = 0x3;
    public static final int TYPE_SYSTEM       = 0x4;
    public static final int TYPE_OPERATOR_ID  = 0x5;
    public static final int TYPE_MESSAGE_PACK = 0xF;

    // UAS ID types. The Odyssey firmware defaults to CAA_REGISTRATION, which is the
    // route that needs no ICAO manufacturer code -- see docs section 12.3.
    public static final int ID_TYPE_NONE             = 0;
    public static final int ID_TYPE_SERIAL_NUMBER    = 1;
    public static final int ID_TYPE_CAA_REGISTRATION = 2;
    public static final int ID_TYPE_UTM_UUID         = 3;
    public static final int ID_TYPE_SESSION_ID       = 4;

    public static String idTypeName(int t) {
        switch (t) {
            case ID_TYPE_NONE:             return "none";
            case ID_TYPE_SERIAL_NUMBER:    return "CTA-2063-A serial";
            case ID_TYPE_CAA_REGISTRATION: return "CAA registration";
            case ID_TYPE_UTM_UUID:         return "UTM assigned UUID";
            case ID_TYPE_SESSION_ID:       return "session ID";
            default:                       return "unknown(" + t + ")";
        }
    }

    public static String statusName(int s) {
        switch (s) {
            case 0:  return "undeclared";
            case 1:  return "ground";
            case 2:  return "airborne";
            case 3:  return "emergency";
            case 4:  return "remote ID system failure";
            default: return "reserved(" + s + ")";
        }
    }

    public static String uaTypeName(int t) {
        switch (t) {
            case 0:  return "none";
            case 1:  return "aeroplane";
            case 2:  return "multirotor";
            case 3:  return "gyroplane";
            case 4:  return "hybrid lift";
            case 5:  return "ornithopter";
            case 6:  return "glider";
            case 7:  return "kite";
            case 8:  return "free balloon";
            case 9:  return "captive balloon";
            case 10: return "airship";
            case 11: return "parachute";
            case 12: return "rocket";
            case 13: return "tethered powered";
            case 14: return "ground obstacle";
            default: return "other(" + t + ")";
        }
    }

    // -------------------------------------------------------------------------------
    /** Basic ID: what the aircraft is and what identifier it carries. */
    public static final class BasicId {
        public int idType;
        public int uaType;
        public String uasId = "";

        @Override public String toString() {
            return "BasicID{" + idTypeName(idType) + "=\"" + uasId
                 + "\", ua=" + uaTypeName(uaType) + "}";
        }
    }

    // -------------------------------------------------------------------------------
    /**
     * Location/Vector: the dynamic message, required at 1 Hz minimum.
     *
     * <p>Fields decode to physical units. Where the standard defines a sentinel for
     * "unknown" the corresponding {@code *Valid} flag is false rather than the value
     * being silently passed through -- a receiver that shows 0.0 for an unknown altitude
     * is worse than one that shows nothing.
     */
    public static final class Location {
        public int     status;
        public boolean heightAboveTakeoff;   // false = above ground level
        public double  directionDeg;         // 0..359, or -1 when unknown
        public double  speedHorizontalMps;
        public double  speedVerticalMps;
        public double  latitude;
        public double  longitude;
        public double  altitudeBaroM;
        public double  altitudeGeoM;
        public double  heightM;
        public int     horizontalAccuracy;   // enum index; see accuracyMetres()
        public int     verticalAccuracy;
        public double  timestampSec;         // seconds past the hour

        public boolean directionValid;
        public boolean speedHorizontalValid;
        public boolean speedVerticalValid;
        public boolean positionValid;
        public boolean altitudeBaroValid;
        public boolean altitudeGeoValid;
        public boolean heightValid;
        public boolean timestampValid;

        @Override public String toString() {
            return "Location{" + statusName(status)
                 + (positionValid ? String.format(java.util.Locale.ROOT,
                        ", %.6f, %.6f", latitude, longitude) : ", position unknown")
                 + (heightValid ? String.format(java.util.Locale.ROOT,
                        ", height %.1f m", heightM) : "")
                 + "}";
        }
    }

    // -------------------------------------------------------------------------------
    /** System: operator/takeoff location and the EU category/class declaration. */
    public static final class SystemMsg {
        public int     operatorLocationType;
        public int     classificationType;
        public double  operatorLatitude;
        public double  operatorLongitude;
        public boolean operatorPositionValid;
        public int     categoryEu;
        public int     classEu;
        public double  operatorAltitudeGeoM;
        public boolean operatorAltitudeValid;
        public long    timestamp;            // seconds since 2019-01-01

        /** The Odyssey firmware declares UNDECLARED, because a home build is not class-marked. */
        public String classEuName() {
            switch (classEu) {
                case 0:  return "undeclared";
                case 1:  return "C0";
                case 2:  return "C1";
                case 3:  return "C2";
                case 4:  return "C3";
                case 5:  return "C4";
                case 6:  return "C5";
                case 7:  return "C6";
                default: return "reserved(" + classEu + ")";
            }
        }

        public String categoryEuName() {
            switch (categoryEu) {
                case 0:  return "undeclared";
                case 1:  return "open";
                case 2:  return "specific";
                case 3:  return "certified";
                default: return "reserved(" + categoryEu + ")";
            }
        }

        @Override public String toString() {
            return "System{" + categoryEuName() + "/" + classEuName() + "}";
        }
    }

    // -------------------------------------------------------------------------------
    /** Operator ID: the registration issued to the person, never the secret suffix. */
    public static final class OperatorId {
        public int idType;
        public String operatorId = "";

        @Override public String toString() {
            return "OperatorID{\"" + operatorId + "\"}";
        }
    }

    // -------------------------------------------------------------------------------
    /** Self ID: a free-text description the operator may set. */
    public static final class SelfId {
        public int descriptionType;
        public String description = "";

        @Override public String toString() { return "SelfID{\"" + description + "\"}"; }
    }

    /**
     * Horizontal accuracy enum to metres, for display. The standard encodes accuracy as
     * an index into a fixed ladder rather than a number.
     */
    public static String accuracyMetres(int index) {
        switch (index) {
            case 1:  return "<10 NM";
            case 2:  return "<4 NM";
            case 3:  return "<2 NM";
            case 4:  return "<1 NM";
            case 5:  return "<0.5 NM";
            case 6:  return "<0.3 NM";
            case 7:  return "<0.1 NM";
            case 8:  return "<0.05 NM";
            case 9:  return "<30 m";
            case 10: return "<10 m";
            case 11: return "<3 m";
            case 12: return "<1 m";
            default: return "unknown";
        }
    }
}
