// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorToL1 extends Command {
  private final Elevator elevator;
  private final double position = .07125;

  /** Creates a new elevatorToL1. */
  public ElevatorToL1(Elevator elevator) {
    this.elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.resetPosition(elevator.getPosition(), elevator.getVelocity());
    SmartDashboard.putBoolean("L1 Finished", elevator.checkGoal());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // elevator.resetPosition(elevator.getPosition());
    elevator.setPosition(position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("L1 Finished", elevator.checkGoal());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.checkGoal();
  }
}
