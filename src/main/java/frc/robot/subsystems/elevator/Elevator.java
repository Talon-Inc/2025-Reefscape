// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
    SmartDashboard.putNumber("Encoder", io.getPosition());
    SmartDashboard.putNumber("Elevator Velocity", io.getVelocity());
    // SmartDashboard.putNumber("Encoder Linear Distance", io.getPosition() * 2 * Math.PI *
    // .472441);
    SmartDashboard.putNumber("Desired Position", io.getSetpoint());
    SmartDashboard.putNumber("Desired Velocity", io.getDesiredVeloicty());
    SmartDashboard.putBoolean("L1 Reached Position", io.getPosition() == 3);
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

  public boolean checkGoal() {
    return io.checkGoal();
  }

  // public void setGoal(double goal) {
  //   io.setGoal(goal);
  // }
}
