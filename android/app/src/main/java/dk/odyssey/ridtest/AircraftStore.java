package dk.odyssey.ridtest;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

import dk.odyssey.ridtest.odid.OdidConformance;
import dk.odyssey.ridtest.odid.OdidMessage;
import dk.odyssey.ridtest.odid.OdidParser;

/**
 * Accumulates decoded broadcasts, keyed by transmitter.
 *
 * <p>Keyed on the radio address rather than the UAS ID, because an aircraft that has not
 * yet sent a Basic ID still needs somewhere to put its Location messages -- and because
 * two aircraft transmitting the same identifier is itself worth seeing rather than
 * silently merging.
 */
public final class AircraftStore {

    /** Where a broadcast arrived from. */
    public enum Medium { BLUETOOTH, WIFI }

    public static final class Entry {
        public final String  address;
        public final Medium  medium;
        public final OdidConformance.Tracker tracker = new OdidConformance.Tracker();
        public int rssi;
        public String lastError;

        Entry(String address, Medium medium) {
            this.address = address;
            this.medium = medium;
        }

        public String title() {
            String id = tracker.uasId.isEmpty() ? "(no ID yet)" : tracker.uasId;
            return id + "  [" + medium + " " + address + "]";
        }
    }

    private final Map<String, Entry> entries = new LinkedHashMap<>();

    /** Identifiers the operator expects, used by the conformance evaluation. */
    public volatile String expectedUasId = "";
    public volatile String expectedOperatorId = "";

    public synchronized void onAdvertisement(String address, Medium medium, int rssi,
                                             OdidParser.Decoded decoded, long nowMs) {
        Entry e = entries.get(address);
        if (e == null) {
            e = new Entry(address, medium);
            entries.put(address, e);
        }
        e.rssi = rssi;
        e.lastError = decoded.error;
        e.tracker.onDecoded(decoded, nowMs);
    }

    public synchronized List<Entry> snapshot() {
        return Collections.unmodifiableList(new ArrayList<>(entries.values()));
    }

    public synchronized void clear() { entries.clear(); }

    public synchronized int size() { return entries.size(); }

    /** Renders the whole store as the plain-text report the UI shows and exports. */
    public synchronized String report(long nowMs) {
        StringBuilder sb = new StringBuilder();
        if (entries.isEmpty()) {
            sb.append("No Remote ID broadcasts received.\n\n")
              .append("If you expected some:\n")
              .append("  * Run the decoder self-test to rule this app out.\n")
              .append("  * Check Bluetooth and Location permissions are granted.\n")
              .append("  * Remember the Odyssey firmware broadcasts NOTHING until its\n")
              .append("    UAS ID and operator registration are both valid. Silence is a\n")
              .append("    deliberate state, not necessarily a fault -- check the module's\n")
              .append("    serial console.\n");
            return sb.toString();
        }

        for (Entry e : entries.values()) {
            sb.append("================================================================\n");
            sb.append(e.title()).append('\n');
            sb.append(String.format(Locale.ROOT, "RSSI %d dBm   seen for %.0f s\n",
                      e.rssi, e.tracker.observedMs() / 1000.0));

            if (e.tracker.idType >= 0) {
                sb.append("ID type: ")
                  .append(OdidMessage.idTypeName(e.tracker.idType)).append('\n');
            }
            if (!e.tracker.operatorId.isEmpty()) {
                sb.append("Operator: ").append(e.tracker.operatorId).append('\n');
            }

            OdidMessage.Location l = e.tracker.lastLocation;
            if (l != null) {
                sb.append(String.format(Locale.ROOT, "Status: %s\n",
                          OdidMessage.statusName(l.status)));
                if (l.positionValid) {
                    sb.append(String.format(Locale.ROOT, "Position: %.6f, %.6f  (%s)\n",
                              l.latitude, l.longitude,
                              OdidMessage.accuracyMetres(l.horizontalAccuracy)));
                } else {
                    sb.append("Position: not valid\n");
                }
                if (l.heightValid) {
                    sb.append(String.format(Locale.ROOT, "Height: %.1f m %s\n",
                              l.heightM, l.heightAboveTakeoff ? "above take-off" : "AGL"));
                }
                if (l.speedHorizontalValid) {
                    sb.append(String.format(Locale.ROOT, "Speed: %.1f m/s", l.speedHorizontalMps));
                    if (l.directionValid) {
                        sb.append(String.format(Locale.ROOT, "  heading %.0f deg", l.directionDeg));
                    }
                    sb.append('\n');
                }
            }

            sb.append(String.format(Locale.ROOT,
                      "Counts: basic %d, location %d, system %d, operator %d\n",
                      e.tracker.basicIdCount, e.tracker.locationCount,
                      e.tracker.systemCount, e.tracker.operatorCount));

            if (e.lastError != null) {
                sb.append("Last decode error: ").append(e.lastError).append('\n');
            }

            sb.append("\n-- conformance ------------------------------------------------\n");
            List<OdidConformance.Observation> obs =
                    OdidConformance.evaluate(e.tracker, expectedUasId, expectedOperatorId);
            for (OdidConformance.Observation o : obs) {
                sb.append("  ").append(o).append('\n');
            }
            sb.append(OdidConformance.allPassed(obs)
                      ? "\n  ALL OBSERVABLE CHECKS PASSED\n"
                      : "\n  ONE OR MORE CHECKS FAILED\n");
            sb.append('\n');
        }

        sb.append("Note: these are receiver-observable checks only. They do not\n")
          .append("constitute an EU 2019/945 Part 6 conformity assessment -- see\n")
          .append("specification section 12.4.\n");
        return sb.toString();
    }
}
