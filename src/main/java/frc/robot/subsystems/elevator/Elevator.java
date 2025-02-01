// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private final ElevatorIO io;

  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  // Constructor
  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  // Method to set power for the elevator
  public void setVoltage(double voltage) {
    System.out.println("Elevator position: " + getPosition());
    io.set(voltage);
  }

  // Method to stop the elevator
  public void stop() {
    io.stop();
  }

  // Set the elevatorto a specific position
  public void setPosition(double position) {
    io.setPosition(position);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getPosition() {
    return io.getPosition();
  }

  public double getVelocity() {
    return io.getVelocity();
  }

  public void resetPosition() {
    io.resetPosition();
  }
}
