// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoral extends Command {
  private final Shooter shooter;
  private boolean flag;

  /** Creates a new intakeCoral. */
  public IntakeCoral(Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flag = false;
    Logger.recordOutput("Intake Finished", false);
    Logger.recordOutput("Intake Started", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.isCoralLoaded()) flag = true;

    shooter.intakeCoral();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    Logger.recordOutput("Intake Finished", true);
    Logger.recordOutput("Intake Started", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (flag) return !shooter.isCoralLoaded();
    else return false;
  }
}
