// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/** Hardware implementation of LEDIO for real AddressableLED strips (WS2812/WS2815). */
public class LEDIOHardware implements LEDIO {
  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;
  private final int length;

  /** Creates a new hardware LED IO instance on the specified PWM port. */
  public LEDIOHardware(int pwmPort, int length) {
    this.length = length;
    this.led = new AddressableLED(pwmPort);
    this.buffer = new AddressableLEDBuffer(length);

    // Configure the LED strip
    led.setLength(length);
    led.setData(buffer);
    led.start();
  }

  @Override
  public void update() {
    led.setData(buffer);
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
    return false;
  }
}
