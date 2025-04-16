// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorToL2 extends Command {
  private final Elevator elevator;
  private final double position = .1963;
  // private LoggedTunableNumber KG = new LoggedTunableNumber("Elevator/kG", 0.0);

  /** Creates a new elevatorToL2. */
  public ElevatorToL2(Elevator elevator) {
    this.elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Changes kP Values depending on if the Elevator is moving Up or Down
    if (elevator.getPosition() > position) {
      elevator.getController().setP(11);
    } else {
      elevator.getController().setP(3.5);
    }

    // Resets Elevator Profiled PID Controller (Stops Elevator from going Crazy)
    elevator.resetPosition(elevator.getPosition(), elevator.getVelocity());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Runs Elevator ProfiledPIDController
    elevator.setPosition(position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.checkGoal();
  }
}
