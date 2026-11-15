package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.io.File;
import java.io.IOException;
import java.lang.reflect.Array;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.net.MalformedURLException;
import java.net.URL;
import java.net.URLClassLoader;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import org.junit.Test;

/**
 * Every static constant in TeamCode keeps the value it had at the regionals code (main 28246ea), wherever it has
 * moved to since. The values are read in a fresh class loader, so settings other tests change don't show.
 *
 * <p>{@code src/test/resources/constants/snapshot.txt} holds the original values. {@code moves.txt} says where a
 * class or a constant went: {@code old -> new} for a class or a single constant, or {@code old -> (deleted)} for
 * one removed as unused. Set WRITE_CONSTANTS_SNAPSHOT=1 to rewrite the snapshot from the current code.
 */
public class ConstantsSnapshotTest {
  private static final String ROOT = "org.firstinspires.ftc.teamcode";
  private static final String DELETED = "(deleted)";

  @Test
  public void everyConstantKeepsItsRegionalsValue() throws Exception {
    File dir = resourceDir();
    File snapshotFile = new File(dir, "snapshot.txt");
    try (URLClassLoader loader = freshLoader()) {
      if ("1".equals(System.getenv("WRITE_CONSTANTS_SNAPSHOT"))) {
        Map<String, String> now = readAll(loader, sourceClassNames());
        List<String> lines = new ArrayList<>();
        for (Map.Entry<String, String> e : now.entrySet()) {
          lines.add(e.getKey() + " = " + e.getValue());
        }
        Files.write(snapshotFile.toPath(), lines, StandardCharsets.UTF_8);
        return;
      }
      Map<String, String> moves = readMoves(new File(dir, "moves.txt"));
      List<String> problems = new ArrayList<>();
      int checked = 0;
      for (String line : Files.readAllLines(snapshotFile.toPath(), StandardCharsets.UTF_8)) {
        int eq = line.indexOf(" = ");
        String key = line.substring(0, eq);
        String expected = line.substring(eq + 3);
        String where = resolve(key, moves);
        if (DELETED.equals(where)) {
          continue;
        }
        String actual = read(loader, where);
        checked++;
        if (!expected.equals(actual)) {
          problems.add(key + (where.equals(key) ? "" : " (now " + where + ")") + ": was " + expected + ", now "
              + actual);
        }
      }
      if (!problems.isEmpty()) {
        fail(problems.size() + " constants changed or went missing:\n" + String.join("\n", problems));
      }
      assertTrue("the snapshot is empty", checked > 0);
    }
  }

  /** Where a snapshot key lives now: a constant's own move first, else its class's, followed through renames. */
  static String resolve(String key, Map<String, String> moves) {
    String current = key;
    for (int hop = 0; hop < 20; hop++) {
      String next = moves.get(current);
      if (next == null) {
        int dot = current.lastIndexOf('.');
        String movedClass = moves.get(current.substring(0, dot));
        next = movedClass == null ? null : movedClass + current.substring(dot);
      }
      if (next == null || DELETED.equals(next)) {
        return next == null ? current : DELETED;
      }
      current = next;
    }
    throw new IllegalStateException("moves.txt loops at " + key);
  }

  private static Map<String, String> readMoves(File file) throws IOException {
    Map<String, String> moves = new LinkedHashMap<>();
    if (!file.exists()) {
      return moves;
    }
    for (String raw : Files.readAllLines(file.toPath(), StandardCharsets.UTF_8)) {
      String line = raw.trim();
      if (line.isEmpty() || line.startsWith("#")) {
        continue;
      }
      String[] parts = line.split("\\s*->\\s*");
      moves.put(parts[0], parts[1]);
    }
    return moves;
  }

  private static File resourceDir() {
    for (String base : new String[] {"src/test/resources/constants", "TeamCode/src/test/resources/constants"}) {
      File dir = new File(base);
      if (dir.isDirectory()) {
        return dir;
      }
    }
    throw new IllegalStateException("no constants resource dir from " + new File(".").getAbsolutePath());
  }

  private static URLClassLoader freshLoader() throws MalformedURLException {
    List<URL> urls = new ArrayList<>();
    for (String path : System.getProperty("java.class.path").split(File.pathSeparator)) {
      urls.add(new File(path).toURI().toURL());
    }
    return new URLClassLoader(urls.toArray(new URL[0]), null);
  }

  private static List<String> sourceClassNames() {
    File src = null;
    for (String base : new String[] {"src/main/java", "TeamCode/src/main/java"}) {
      File dir = new File(base, ROOT.replace('.', '/'));
      if (dir.isDirectory()) {
        src = dir;
      }
    }
    if (src == null) {
      throw new IllegalStateException("no TeamCode sources from " + new File(".").getAbsolutePath());
    }
    List<String> names = new ArrayList<>();
    collect(src, ROOT, names);
    return names;
  }

  private static void collect(File dir, String pkg, List<String> names) {
    File[] files = dir.listFiles();
    if (files == null) {
      return;
    }
    for (File f : files) {
      if (f.isDirectory()) {
        collect(f, pkg + "." + f.getName(), names);
      } else if (f.getName().endsWith(".java")) {
        names.add(pkg + "." + f.getName().substring(0, f.getName().length() - 5));
      }
    }
  }

  private static Map<String, String> readAll(ClassLoader loader, List<String> classNames) {
    Map<String, String> values = new TreeMap<>();
    for (String name : classNames) {
      Class<?> type;
      try {
        type = Class.forName(name, true, loader);
      } catch (Throwable t) {
        continue; // Needs the robot's hardware to load; nothing to compare either way
      }
      readClass(type, values);
    }
    return values;
  }

  private static void readClass(Class<?> type, Map<String, String> values) {
    try {
      for (Field f : type.getDeclaredFields()) {
        if (Modifier.isStatic(f.getModifiers()) && !f.isSynthetic() && !f.isEnumConstant() && comparable(f.getType())) {
          f.setAccessible(true);
          values.put(type.getName() + "." + f.getName(), format(f.get(null)));
        }
      }
      for (Class<?> inner : type.getDeclaredClasses()) {
        readClass(inner, values);
      }
    } catch (Throwable t) {
      // A class whose fields can't be read off the robot is left out
    }
  }

  private static String read(ClassLoader loader, String key) {
    int dot = key.lastIndexOf('.');
    try {
      Field f = Class.forName(key.substring(0, dot), true, loader).getDeclaredField(key.substring(dot + 1));
      f.setAccessible(true);
      return format(f.get(null));
    } catch (Throwable t) {
      return "<missing: " + t.getClass().getSimpleName() + ">";
    }
  }

  private static boolean comparable(Class<?> t) {
    while (t.isArray()) {
      t = t.getComponentType();
    }
    return t.isPrimitive() || t == String.class || t.isEnum() || Number.class.isAssignableFrom(t)
        || t == Boolean.class || t == Character.class;
  }

  private static String format(Object value) {
    if (value == null) {
      return "null";
    }
    if (value.getClass().isArray()) {
      StringBuilder s = new StringBuilder("[");
      for (int i = 0; i < Array.getLength(value); i++) {
        s.append(i == 0 ? "" : ", ").append(format(Array.get(value, i)));
      }
      return s.append(']').toString();
    }
    if (value instanceof Enum) {
      return ((Enum<?>) value).name();
    }
    return String.valueOf(value).replace("\n", "\\n");
  }
}
