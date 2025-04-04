// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCoral extends Command {
  private final Shooter shooter;
  private Timer timer;
  private double start;

  /** Creates a new shootCoral. */
  public ShootCoral(Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    timer = new Timer();
    start = timer.get();
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // if (shooter.isCoralLoaded() == false) {
    //   end(isScheduled());
    // }
    Logger.recordOutput("Shooter Finished", false);
    Logger.recordOutput("Shooter Started", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.shoot();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    Logger.recordOutput("Shooter Finished", true);
    Logger.recordOutput("Shooter Started", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return !shooter.isCoralLoaded();
    if (timer.get() - start > 1) {
      return true;
    }
    return false;
  }
}
