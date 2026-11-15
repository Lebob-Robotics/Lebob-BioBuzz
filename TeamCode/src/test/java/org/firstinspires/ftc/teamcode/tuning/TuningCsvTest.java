package org.firstinspires.ftc.teamcode.tuning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.Arrays;
import org.junit.After;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;
import org.psilynx.psikit.core.Logger;

public class TuningCsvTest {
  @Rule public final TemporaryFolder folder = new TemporaryFolder();

  @After
  public void leaveTheHubPaths() {
    TuningCsv.directory = null;
    Logger.setReplay(false);
  }

  @Test
  public void writesTheHeaderOnceThenOneRowALine() throws IOException {
    String path = new File(folder.getRoot(), "Tuning.csv").getPath();
    assertTrue(TuningCsv.append(path, "a,b", "1,2"));
    assertTrue(TuningCsv.append(path, "a,b", "3,4"));
    assertEquals(Arrays.asList("a,b", "1,2", "3,4"),
        Files.readAllLines(new File(path).toPath(), StandardCharsets.UTF_8));
  }

  @Test
  public void keepsTheRowsAlreadyThere() throws IOException {
    File file = folder.newFile("Tuning.csv");
    Files.write(file.toPath(), Arrays.asList("a,b", "1,2"), StandardCharsets.UTF_8);
    assertTrue(TuningCsv.append(file.getPath(), "a,b", "3,4"));
    assertEquals(Arrays.asList("a,b", "1,2", "3,4"), Files.readAllLines(file.toPath(), StandardCharsets.UTF_8));
  }

  @Test
  public void replayWritesNothingToTheHubPaths() throws IOException {
    File file = new File(folder.getRoot(), "Tuning.csv");
    Logger.setReplay(true);
    assertFalse(TuningCsv.append(file.getPath(), "a,b", "1,2"));
    assertFalse(file.exists());
  }

  @Test
  public void aTestFolderTakesTheFilesUnderTheirOwnNamesEvenInReplay() throws IOException {
    TuningCsv.directory = folder.getRoot();
    Logger.setReplay(true);
    assertEquals(new File(folder.getRoot(), "ShotTuning_v2.csv"), TuningCsv.file("/sdcard/FIRST/ShotTuning_v2.csv"));
    assertTrue(TuningCsv.append("/sdcard/FIRST/ShotTuning_v2.csv", "a", "1"));
    assertEquals(Arrays.asList("a", "1"),
        Files.readAllLines(new File(folder.getRoot(), "ShotTuning_v2.csv").toPath(), StandardCharsets.UTF_8));
  }

  @Test
  public void aFileThatCannotBeWrittenThrows() {
    try {
      TuningCsv.append(new File(folder.getRoot(), "missing/Tuning.csv").getPath(), "a", "1");
      fail("wrote into a folder that isn't there");
    } catch (IOException expected) {
      // The OpModes show this to the driver
    }
  }
}
