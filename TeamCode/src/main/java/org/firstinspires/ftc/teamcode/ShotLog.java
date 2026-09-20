package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * One CSV per OpMode run under /sdcard/FIRST/shots, pulled off the hub with adb after practice.
 * Wi-Fi dashboards are banned at events (R704), so this is the tuning record.
 */
public final class ShotLog {
    public static final String HEADER = "t_s,x_m,y_m,heading_rad,vx_mps,vy_mps,dist_m,radial_mps,tangential_mps,"
            + "table_rpm,left_rpm,right_rpm,heading_err_rad,valid,fired";
    private static final int FLUSH_EVERY = 50;

    private final BufferedWriter out;
    private final String fileName;
    private final long startNanos = System.nanoTime();
    private int rows;

    private ShotLog(BufferedWriter out, String fileName) {
        this.out = out;
        this.fileName = fileName;
    }

    /** Creates the directory and a timestamped file. Returns null, and never throws, if that fails. */
    @SuppressLint("SimpleDateFormat")
    public static ShotLog open(String dir) {
        try {
            File folder = new File(dir);
            if (!folder.isDirectory() && !folder.mkdirs()) return null;
            String name = new SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(new Date()) + ".csv";
            BufferedWriter w = new BufferedWriter(new FileWriter(new File(folder, name)));
            w.write(HEADER);
            w.newLine();
            return new ShotLog(w, name);
        } catch (IOException e) {
            return null;
        }
    }

    /** Appends one row. The first column, elapsed seconds, is added here. */
    public void row(double... values) {
        try {
            StringBuilder sb = new StringBuilder();
            sb.append(String.format(Locale.US, "%.3f", (System.nanoTime() - startNanos) / 1e9));
            for (double v : values) sb.append(',').append(String.format(Locale.US, "%.4f", v));
            out.write(sb.toString());
            out.newLine();
            if (++rows % FLUSH_EVERY == 0) out.flush();
        } catch (IOException ignored) {
            // A failed log line must never stop the robot.
        }
    }

    public void close() {
        try {
            out.flush();
            out.close();
        } catch (IOException ignored) {
        }
    }

    public String fileName() {
        return fileName;
    }
}
