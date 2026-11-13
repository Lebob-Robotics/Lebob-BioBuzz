package org.firstinspires.ftc.teamcode.utils;

import static org.junit.Assert.*;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import org.junit.Test;

/** Replays complete home-camera detection sequences, rather than selected favorable tag pairs. */
public class HomeVisionAcquisitionTest {
  private double acquire(String capture) throws Exception {
    Map<Long, List<String[]>> batches = new TreeMap<>();
    try (BufferedReader reader = new BufferedReader(new InputStreamReader(
        getClass().getResourceAsStream("/vision/" + capture), "UTF-8"))) {
      String line;
      while ((line = reader.readLine()) != null) {
        if (line.startsWith("#")) { continue; }
        String[] row = line.split(",");
        long now = Long.parseLong(row[0]);
        if (!batches.containsKey(now)) { batches.put(now, new ArrayList<>()); }
        batches.get(now).add(row);
      }
    }
    VisionCorrector corrector = new VisionCorrector();
    long first = batches.keySet().iterator().next();
    for (Map.Entry<Long, List<String[]>> batch : batches.entrySet()) {
      List<TagObservation> tags = new ArrayList<>();
      Map<Long, Pose> recorded = new TreeMap<>();
      String[] last = null;
      for (String[] row : batch.getValue()) {
        last = row;
        long frame = Long.parseLong(row[1]);
        CameraMount camera = row[2].equals("Left") ? ShootOnTheMoveConstants.LEFT_CAMERA
            : ShootOnTheMoveConstants.RIGHT_CAMERA;
        tags.add(new TagObservation(Integer.parseInt(row[3]), camera, n(row, 4), n(row, 5),
            n(row, 6), n(row, 7), frame));
        recorded.put(frame - 1, new Pose(n(row, 8), n(row, 9), n(row, 10)));
        recorded.put(frame + 1, new Pose(n(row, 8), n(row, 9), n(row, 10)));
      }
      PoseHistory history = new PoseHistory(ShootOnTheMoveConstants.POSE_HISTORY_SIZE);
      for (Map.Entry<Long, Pose> p : recorded.entrySet()) { history.add(p.getKey(), p.getValue()); }
      VisionResult result = corrector.process(tags, history, new Pose(n(last, 11), n(last, 12), n(last, 13)),
          n(last, 14), n(last, 15), n(last, 16), false, false, batch.getKey());
      if (result.gain > 0 && result.headingGain > 0) { return (batch.getKey() - first) * 1e-9; }
    }
    return Double.POSITIVE_INFINITY;
  }

  private static double n(String[] row, int column) { return Double.parseDouble(row[column]); }

  @Test public void oldHomeDetectionSequenceNeverEstablishedTheStartingPose() throws Exception {
    assertEquals(Double.POSITIVE_INFINITY, acquire("home-half-resolution.csv"), 0);
  }

  @Test public void workingHomeSettingsEstablishACompletePoseAtHalfResolution() throws Exception {
    double seconds = acquire("home-working-half-resolution.csv");
    assertTrue("No complete field pose in the half-resolution capture: " + seconds, seconds < 3);
  }
}
