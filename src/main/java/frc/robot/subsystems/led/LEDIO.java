// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * Hardware abstraction interface for LED control. Allows swapping between real hardware and
 * simulation.
 */
public interface LEDIO {

  /** Updates the physical LED strip with the current buffer data. */
  void update();

  /** @return The AddressableLEDBuffer for writing patterns */
  AddressableLEDBuffer getBuffer();

  /** @return Total number of LEDs in the strip */
  int getLength();

  /** Sets all LEDs to a solid color (RGB 0-255). */
  void setSolidColor(int r, int g, int b);

  /** Sets a specific LED to a color (RGB 0-255). */
  void setLED(int index, int r, int g, int b);

  /** Turns off all LEDs. */
  default void off() {
    setSolidColor(0, 0, 0);
  }

  /** @return true if running in simulation */
  boolean isSimulation();
}
