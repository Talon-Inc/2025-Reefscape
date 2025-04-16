// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  private final PWM led = new PWM(0);
  private final PWM led2 = new PWM(1);
  private double color = 0.99;

  /** Creates a new LED. */
  public LED() {}

  public void setBlack() {
    color = 0.99;
  }

  public void setViolet() {
    color = 0.91;
  }

  public void setGreen() {
    color = 0.77;
  }

  public void setOceanRainbow() {
    color = -0.95;
  }

  public void setColorWave() {
    color = 0.35;
  }

  public void setFire() {
    color = -0.57;
  }

  public void strobeRed() {
    color = -0.11;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    led.setSpeed(color);
    led2.setSpeed(color);
  }
}
