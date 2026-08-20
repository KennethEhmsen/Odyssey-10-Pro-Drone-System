package dk.odyssey.ridtest.odid;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Turns a stream of decoded messages into pass/fail observations.
 *
 * <p>This is what makes the app a <i>test</i> receiver rather than a viewer. Seeing an
 * aircraft appear proves something is transmitting; it does not prove the broadcast meets
 * the rate requirements, carries the identifiers you configured, or reports a plausible
 * position. Those are the things that are actually worth checking on a bench.
 *
 * <p>Deliberately <b>not</b> a compliance test. EU 2019/945 Part 6 requires tamper
 * resistance, product conformity and a great deal else that no receiver can observe. See
 * specification section 12.4.
 */
public final class OdidConformance {

    /** Location/Vector must be broadcast at 1 Hz minimum. */
    public static final long LOCATION_MAX_INTERVAL_MS = 1500;   // 1 Hz + tolerance
    /** Static messages at least every 3 s. */
    public static final long STATIC_MAX_INTERVAL_MS   = 3500;

    public enum Result { PASS, FAIL, UNKNOWN }

    public static final class Observation {
        public final String name;
        public final Result result;
        public final String detail;

        Observation(String name, Result result, String detail) {
            this.name = name; this.result = result; this.detail = detail;
        }

        @Override public String toString() {
            return String.format("%-28s %-7s %s", name, result, detail);
        }
    }

    /** Rolling statistics for one aircraft. */
    public static final class Tracker {
        public String  uasId = "";
        public int     idType = -1;
        public String  operatorId = "";
        public long    firstSeenMs;
        public long    lastSeenMs;

        public long    lastLocationMs = 0;
        public long    lastBasicIdMs  = 0;
        public long    lastSystemMs   = 0;
        public long    lastOperatorMs = 0;

        public int     locationCount = 0;
        public int     basicIdCount  = 0;
        public int     systemCount   = 0;
        public int     operatorCount = 0;

        public long    worstLocationGapMs = 0;
        public long    worstStaticGapMs   = 0;

        public OdidMessage.Location  lastLocation;
        public OdidMessage.SystemMsg lastSystem;

        public int     decodeErrors = 0;
        /** Counter values seen, to detect a stuck or non-incrementing message counter. */
        public int     lastCounter = -1;
        public int     counterStalls = 0;

        public void onDecoded(OdidParser.Decoded d, long nowMs) {
            if (firstSeenMs == 0) firstSeenMs = nowMs;
            lastSeenMs = nowMs;

            if (d.error != null) { decodeErrors++; return; }

            if (d.messageCounter >= 0) {
                if (d.messageCounter == lastCounter) counterStalls++;
                lastCounter = d.messageCounter;
            }

            for (OdidMessage.BasicId b : d.basicIds) {
                if (lastBasicIdMs != 0) {
                    worstStaticGapMs = Math.max(worstStaticGapMs, nowMs - lastBasicIdMs);
                }
                lastBasicIdMs = nowMs;
                basicIdCount++;
                uasId = b.uasId;
                idType = b.idType;
            }
            for (OdidMessage.Location l : d.locations) {
                if (lastLocationMs != 0) {
                    worstLocationGapMs = Math.max(worstLocationGapMs, nowMs - lastLocationMs);
                }
                lastLocationMs = nowMs;
                locationCount++;
                lastLocation = l;
            }
            for (OdidMessage.SystemMsg s : d.systems) {
                lastSystemMs = nowMs;
                systemCount++;
                lastSystem = s;
            }
            for (OdidMessage.OperatorId o : d.operators) {
                lastOperatorMs = nowMs;
                operatorCount++;
                operatorId = o.operatorId;
            }
        }

        public long observedMs() { return lastSeenMs - firstSeenMs; }
    }

    /**
     * @param expectedUasId      what you configured, or null to skip the comparison
     * @param expectedOperatorId what you registered, or null to skip
     */
    public static List<Observation> evaluate(Tracker t, String expectedUasId,
                                             String expectedOperatorId) {
        List<Observation> out = new ArrayList<>();
        final long observed = t.observedMs();

        // ---- Rates -------------------------------------------------------------------
        if (t.locationCount < 2 || observed < 3000) {
            out.add(new Observation("Location rate >= 1 Hz", Result.UNKNOWN,
                    "need a few seconds of observation (" + t.locationCount + " so far)"));
        } else if (t.worstLocationGapMs <= LOCATION_MAX_INTERVAL_MS) {
            out.add(new Observation("Location rate >= 1 Hz", Result.PASS,
                    String.format(Locale.ROOT, "worst gap %.1f s over %.0f s",
                                  t.worstLocationGapMs / 1000.0, observed / 1000.0)));
        } else {
            out.add(new Observation("Location rate >= 1 Hz", Result.FAIL,
                    String.format(Locale.ROOT, "worst gap %.1f s exceeds %.1f s",
                                  t.worstLocationGapMs / 1000.0,
                                  LOCATION_MAX_INTERVAL_MS / 1000.0)));
        }

        if (t.basicIdCount < 2 || observed < 7000) {
            out.add(new Observation("Static msgs <= 3 s", Result.UNKNOWN,
                    "need ~10 s of observation"));
        } else if (t.worstStaticGapMs <= STATIC_MAX_INTERVAL_MS) {
            out.add(new Observation("Static msgs <= 3 s", Result.PASS,
                    String.format(Locale.ROOT, "worst gap %.1f s",
                                  t.worstStaticGapMs / 1000.0)));
        } else {
            out.add(new Observation("Static msgs <= 3 s", Result.FAIL,
                    String.format(Locale.ROOT, "worst gap %.1f s exceeds %.1f s",
                                  t.worstStaticGapMs / 1000.0,
                                  STATIC_MAX_INTERVAL_MS / 1000.0)));
        }

        // ---- Required message types --------------------------------------------------
        out.add(present("Basic ID present",    t.basicIdCount));
        out.add(present("Location present",    t.locationCount));
        out.add(present("System present",      t.systemCount));
        out.add(present("Operator ID present", t.operatorCount));

        // ---- Identifiers match what was configured -----------------------------------
        if (expectedUasId != null && !expectedUasId.isEmpty()) {
            if (t.uasId.isEmpty()) {
                out.add(new Observation("UAS ID matches", Result.UNKNOWN, "none received"));
            } else if (t.uasId.equals(expectedUasId)) {
                out.add(new Observation("UAS ID matches", Result.PASS, t.uasId));
            } else {
                out.add(new Observation("UAS ID matches", Result.FAIL,
                        "broadcast \"" + t.uasId + "\", expected \"" + expectedUasId + "\""));
            }
        }
        if (expectedOperatorId != null && !expectedOperatorId.isEmpty()) {
            if (t.operatorId.isEmpty()) {
                out.add(new Observation("Operator ID matches", Result.UNKNOWN, "none received"));
            } else if (t.operatorId.equals(expectedOperatorId)) {
                out.add(new Observation("Operator ID matches", Result.PASS, t.operatorId));
            } else {
                out.add(new Observation("Operator ID matches", Result.FAIL,
                        "broadcast \"" + t.operatorId + "\""));
            }
        }

        // ---- The operator secret must never appear on air -----------------------------
        // The registration is PUBLIC 16 characters, then a hyphen, then 3 SECRET ones.
        // If a hyphen shows up the firmware is transmitting the secret, which is a
        // serious defect -- see specification section 12.3.
        if (!t.operatorId.isEmpty() && t.operatorId.contains("-")) {
            out.add(new Observation("Operator secret withheld", Result.FAIL,
                    "broadcast operator ID contains a hyphen -- the 3-character secret "
                  + "may be going out on air"));
        } else if (!t.operatorId.isEmpty()) {
            out.add(new Observation("Operator secret withheld", Result.PASS,
                    "no secret suffix in the broadcast"));
        }

        // ---- Plausibility -------------------------------------------------------------
        if (t.lastLocation != null) {
            if (!t.lastLocation.positionValid) {
                out.add(new Observation("Position plausible", Result.FAIL,
                        "latitude/longitude is 0,0 or out of range"));
            } else {
                out.add(new Observation("Position plausible", Result.PASS,
                        String.format(Locale.ROOT, "%.5f, %.5f",
                                      t.lastLocation.latitude, t.lastLocation.longitude)));
            }
        }

        if (t.lastSystem != null) {
            // A home build is not class-marked, so UNDECLARED is the correct answer here.
            if (t.lastSystem.classEu == 0) {
                out.add(new Observation("EU class declaration", Result.PASS,
                        "undeclared -- correct for a privately built aircraft"));
            } else {
                out.add(new Observation("EU class declaration", Result.UNKNOWN,
                        "declares " + t.lastSystem.classEuName()
                      + " -- only valid if the airframe is actually class-marked"));
            }
        }

        // ---- Health -------------------------------------------------------------------
        if (t.decodeErrors > 0) {
            out.add(new Observation("Decode errors", Result.FAIL,
                    t.decodeErrors + " advertisement(s) failed to decode"));
        } else {
            out.add(new Observation("Decode errors", Result.PASS, "none"));
        }

        if (t.counterStalls > 2) {
            out.add(new Observation("Message counter advances", Result.FAIL,
                    t.counterStalls + " repeats -- a stuck counter hides lost packets"));
        } else {
            out.add(new Observation("Message counter advances", Result.PASS,
                    "counter incrementing"));
        }

        return out;
    }

    private static Observation present(String name, int count) {
        return count > 0
            ? new Observation(name, Result.PASS, count + " received")
            : new Observation(name, Result.FAIL, "never seen");
    }

    public static boolean allPassed(List<Observation> obs) {
        for (Observation o : obs) if (o.result == Result.FAIL) return false;
        return true;
    }
}
