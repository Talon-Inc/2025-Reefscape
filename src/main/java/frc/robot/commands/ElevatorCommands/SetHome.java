// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetHome extends Command {
  private final Elevator elevator;
  private final double position = 0;
  /** Creates a new setHome. */
  public SetHome(Elevator elevator) {
    this.elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.resetPosition(elevator.getPosition(), elevator.getVelocity());
    SmartDashboard.putBoolean("Set Home Finished", elevator.checkGoal());
    Logger.recordOutput("SetHome Started", true);
    Logger.recordOutput("SetHome Finished", false);
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
    SmartDashboard.putBoolean("Set Home Finished", elevator.checkGoal());
    elevator.stop();
    Logger.recordOutput("SetHome Started", false);
    Logger.recordOutput("SetHome Finished", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.checkGoal();
  }
}
