// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Simulation implementation of LEDIO for testing with WPILib Glass. Publishes LED data to
 * NetworkTables/SmartDashboard for visualization. View in Glass under SmartDashboard/LED Sim/.
 */
public class LEDIOSim implements LEDIO {
  private final AddressableLEDBuffer buffer;
  private final int length;
  private final String tableName;

  // Track previous state to minimize NT updates
  private int lastDominantR = -1;
  private int lastDominantG = -1;
  private int lastDominantB = -1;

  /** Creates a new simulation LED IO instance with default table name. */
  public LEDIOSim(int length) {
    this(length, "LED Sim");
  }

  /** Creates a new simulation LED IO instance with a custom table name. */
  public LEDIOSim(int length, String tableName) {
    this.length = length;
    this.tableName = tableName;
    this.buffer = new AddressableLEDBuffer(length);

    // Initialize SmartDashboard entries
    SmartDashboard.putNumber(tableName + "/LED Count", length);
    SmartDashboard.putString(tableName + "/State", "Initializing");
    SmartDashboard.putNumberArray(tableName + "/Dominant Color RGB", new double[] {0, 0, 0});
    SmartDashboard.putString(tableName + "/Dominant Color Hex", "#000000");
  }

  @Override
  public void update() {
    // Calculate dominant color (average of all LEDs)
    int totalR = 0, totalG = 0, totalB = 0;

    for (int i = 0; i < length; i++) {
      Color color = buffer.getLED(i);
      int r = (int) (color.red * 255);
      int g = (int) (color.green * 255);
      int b = (int) (color.blue * 255);

      totalR += r;
      totalG += g;
      totalB += b;

      // Publish individual LED data (for detailed debugging)
      // Only publish every 8th LED to reduce NT traffic
      if (i % 8 == 0) {
        SmartDashboard.putNumberArray(tableName + "/LED " + i, new double[] {r, g, b});
      }
    }

    // Calculate average (dominant) color
    int avgR = totalR / length;
    int avgG = totalG / length;
    int avgB = totalB / length;

    // Only update if changed (reduces NT traffic)
    if (avgR != lastDominantR || avgG != lastDominantG || avgB != lastDominantB) {
      lastDominantR = avgR;
      lastDominantG = avgG;
      lastDominantB = avgB;

      SmartDashboard.putNumberArray(
          tableName + "/Dominant Color RGB", new double[] {avgR, avgG, avgB});

      // Also publish as hex string for easy reading
      String hex = String.format("#%02X%02X%02X", avgR, avgG, avgB);
      SmartDashboard.putString(tableName + "/Dominant Color Hex", hex);
    }

    // Publish first few LEDs for pattern visualization
    StringBuilder patternPreview = new StringBuilder();
    for (int i = 0; i < Math.min(10, length); i++) {
      Color color = buffer.getLED(i);
      patternPreview.append(getColorChar(color));
    }
    if (length > 10) {
      patternPreview.append("...");
    }
    SmartDashboard.putString(tableName + "/Pattern Preview", patternPreview.toString());
  }

  /** Gets a character representation of a color for the pattern preview. */
  private char getColorChar(Color color) {
    int r = (int) (color.red * 255);
    int g = (int) (color.green * 255);
    int b = (int) (color.blue * 255);

    if (r < 30 && g < 30 && b < 30) return '_'; // Off/Black
    if (r > 200 && g < 100 && b < 100) return 'R'; // Red
    if (r < 100 && g > 200 && b < 100) return 'G'; // Green
    if (r < 100 && g < 100 && b > 200) return 'B'; // Blue
    if (r > 200 && g > 200 && b < 100) return 'Y'; // Yellow
    if (r > 200 && g > 100 && b < 100) return 'O'; // Orange
    if (r < 100 && g > 200 && b > 200) return 'C'; // Cyan
    if (r > 200 && g < 100 && b > 200) return 'P'; // Purple/Magenta
    if (r > 200 && g > 200 && b > 200) return 'W'; // White
    return '*'; // Other
  }

  @Override
  public AddressableLEDBuffer getBuffer() {
    return buffer;
  }

  @Override
  public int getLength() {
    return length;
  }

  @Override
  public void setSolidColor(int r, int g, int b) {
    for (int i = 0; i < length; i++) {
      buffer.setRGB(i, r, g, b);
    }
  }

  @Override
  public void setLED(int index, int r, int g, int b) {
    if (index >= 0 && index < length) {
      buffer.setRGB(index, r, g, b);
    }
  }

  @Override
  public boolean isSimulation() {
    return true;
  }

  /** Updates the state name displayed in SmartDashboard. */
  public void setStateName(String stateName) {
    SmartDashboard.putString(tableName + "/State", stateName);
  }
}
