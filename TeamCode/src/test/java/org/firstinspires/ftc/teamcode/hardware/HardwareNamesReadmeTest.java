package org.firstinspires.ftc.teamcode.hardware;

import static org.junit.Assert.assertEquals;

import java.io.File;
import java.io.IOException;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.junit.Test;

/** README.md's robot configuration table names every device in HardwareNames, and nothing else. */
public class HardwareNamesReadmeTest {
  private static final Pattern ROW = Pattern.compile("^\\| `([^`]+)` \\|");

  @Test
  public void theReadmeTableMatchesHardwareNames() throws Exception {
    Set<String> names = new TreeSet<>();
    for (Field f : HardwareNames.class.getDeclaredFields()) {
      if (Modifier.isStatic(f.getModifiers()) && f.getType() == String.class) {
        names.add((String) f.get(null));
      }
    }
    Set<String> table = new TreeSet<>();
    boolean inSection = false;
    for (String line : readme()) {
      if (line.startsWith("## ")) {
        inSection = line.equals("## Robot configuration");
      } else if (inSection) {
        Matcher m = ROW.matcher(line);
        if (m.find()) {
          table.add(m.group(1));
        }
      }
    }
    assertEquals(names, table);
  }

  private static List<String> readme() throws IOException {
    for (String path : new String[] {"../README.md", "README.md"}) {
      File file = new File(path);
      if (file.isFile()) {
        return Files.readAllLines(file.toPath(), StandardCharsets.UTF_8);
      }
    }
    throw new IllegalStateException("no README.md from " + new File(".").getAbsolutePath());
  }
}
