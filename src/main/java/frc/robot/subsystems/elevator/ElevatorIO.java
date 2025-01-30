// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double motorCurrent = 0;
    public double motorVoltage = 0;
    public double motorAngle = 0;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}
  // Sets the power to the elevator motor
  public default void set(double voltage) {}

  // Gets the current position of the elevator (in enconder units)
  public default double getPosition() {
    return 0;
  }

  public default void setPosition(double position) {}

  // Gets the current velocity of the elevator
  public default double getVelocity() {
    return 0;
  }

  // Resets the encoder position to a specific value
  public default void resetPosition() {}

  public default void stop() {}
}
