// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private final ElevatorIOSparkMAX io;

  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  // Constructor
  public Elevator(ElevatorIOSparkMAX io) {
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
    io.goToPosition(position);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Encoder", io.getPosition());
    // SmartDashboard.putNumber("Encoder", io.getPosition());
    // SmartDashboard.putNumber("Elevator Velocity", io.getVelocity());
    // SmartDashboard.putNumber("Encoder Linear Distance", io.getPosition() * 2 * Math.PI *
    // .472441);
    // SmartDashboard.putNumber("Desired Position", io.getSetpoint());
    Logger.recordOutput("Desired Motion", io.getSetpoint());
    Logger.recordOutput("Setpoint Velocity", io.getDesiredVeloicty());
    Logger.recordOutput("Real Velocity", io.getVelocity());
    Logger.recordOutput("L1 Reached Position", io.getPosition() == 3);
    Logger.recordOutput("Position Error", io.getPositionError());
    // SmartDashboard.putNumber("Desired Velocity", io.getDesiredVeloicty());
    // SmartDashboard.putBoolean("L1 Reached Position", io.getPosition() == 3);
    // SmartDashboard.putNumber("Position Error", io.getPositionError());
    Logger.recordOutput("Motor 11 Voltage", io.get11Voltage());
    Logger.recordOutput("Motor 11 Current", io.get11Current());
    Logger.recordOutput("Motor 12 Voltage", io.get12Voltage());
    Logger.recordOutput("Motor 12 Current", io.get12Current());
  }

  public double getPosition() {
    return io.getPosition();
  }

  public double getVelocity() {
    return io.getVelocity();
  }

  public void resetPosition(double newPosition, double newVelocity) {
    io.resetPosition(newPosition, newVelocity);
  }

  public boolean checkGoal() {
    return io.checkGoal();
  }

  public double getPositionError() {
    return io.getPositionError();
  }

  // public void setGoal(double goal) {
  //   io.setGoal(goal);
  // }
}
