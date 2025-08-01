// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.VisionCommands;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LeftAutoAlign extends Command {

  // Creates Trapezoidal Constraints for ProfiledPIDControllers
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS =
      new TrapezoidProfile.Constraints(2.25, 4);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS =
      new TrapezoidProfile.Constraints(2.25, 4);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =
      new TrapezoidProfile.Constraints(2, 5);

  // Lists an Array of Every Tag on the Reef (From both Sides), this stops it from trying to align
  // to the deposit
  private static final int[] REEF_TAGS = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};

  // Creates a Desired Pose of the robot in relation to the April Tag
  private static final Transform3d TAG_TO_GOAL =
      new Transform3d(
          new Translation3d(Units.inchesToMeters(16.685), Units.inchesToMeters(-3), 0),
          new Rotation3d(0, 0, -Math.PI));

  // Pulls Current Robot Pose
  private static Pose2d robotPose;

  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final LED led;

  // Creates ProfiledPID Controllers for the X-Axis, Y-Axis, and Rotation
  private final ProfiledPIDController xController =
      new ProfiledPIDController(2.75, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController =
      new ProfiledPIDController(2.75, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController =
      new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);

  // Pulls Data from Photon Vision
  private PhotonTrackedTarget lastTarget;
  private int bestTargetID;

  /** Creates a new leftAutoAlign. */
  public LeftAutoAlign(Drive drive, Vision vision, LED led) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.vision = vision;
    this.led = led;

    xController.setTolerance(.01);
    yController.setTolerance(.01);
    omegaController.setTolerance(Units.degreesToRadians(2));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastTarget = null;
    robotPose = drive.getPose();
    omegaController.reset(
        robotPose.getRotation().getRadians(), drive.getChassisSpeeds().omegaRadiansPerSecond);
    xController.reset(robotPose.getX(), drive.getChassisSpeeds().vxMetersPerSecond);
    yController.reset(robotPose.getY(), drive.getChassisSpeeds().vyMetersPerSecond);

    final int cameraIndex;
    if (vision.hasTargets(1)) {
      cameraIndex = 1;
    } else if (vision.hasTargets(0)) {
      cameraIndex = 0;
    } else {
      cameraIndex = 1;
    }

    if (vision.hasTargets(0) || vision.hasTargets(1)) {
      if (Arrays.stream(REEF_TAGS).anyMatch(n -> n == vision.bestTargetID(cameraIndex))) {
        lastTarget = vision.bestTrackedTarget(cameraIndex);
        bestTargetID = lastTarget.getFiducialId();
      }
    }

    led.setOceanRainbow();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    robotPose = drive.getPose();
    var robotPose3D =
        new Pose3d(
            robotPose.getX(),
            robotPose.getY(),
            0,
            new Rotation3d(0, 0, robotPose.getRotation().getRadians()));
    final int cameraIndex;
    if (vision.hasTargets(1)) {
      cameraIndex = 1;
    } else if (vision.hasTargets(0)) {
      cameraIndex = 0;
    } else {
      cameraIndex = 1;
    }
    if (vision.hasTargets(0) || vision.hasTargets(1)) {
      if (Arrays.stream(REEF_TAGS).anyMatch(n -> n == vision.bestTargetID(cameraIndex))) {
        lastTarget = vision.bestTrackedTarget(cameraIndex);
        var targetPose = aprilTagLayout.getTagPose(bestTargetID);
        Pose3d targetPose3D =
            new Pose3d(
                targetPose.get().getX(),
                targetPose.get().getY(),
                targetPose.get().getZ(),
                targetPose.get().getRotation());
        var goalPose = targetPose3D.transformBy(TAG_TO_GOAL).toPose2d();

        xController.setGoal(goalPose.getX());
        yController.setGoal(goalPose.getY());
        omegaController.setGoal(goalPose.getRotation().getRadians());
        Logger.recordOutput("Goal Pose", goalPose);
        Logger.recordOutput("X Error", xController.getPositionError());
        Logger.recordOutput("Y Error", yController.getPositionError());
        Logger.recordOutput("Omega Error", omegaController.getPositionError());
        Logger.recordOutput("Pose X", robotPose.getX());
        Logger.recordOutput("Pose Y", robotPose.getY());
        Logger.recordOutput("Pose Omega", robotPose.getRotation().getRadians());
        Logger.recordOutput("Goal Pose X", xController.getSetpoint().position);
        Logger.recordOutput("Goal Pose Y", yController.getSetpoint().position);
        Logger.recordOutput("Goal Pose Omega", omegaController.getSetpoint().position);
      }
    }

    if (lastTarget == null) {
      drive.stop();
    } else {
      // Drive to the Target
      var xSpeed = xController.calculate(robotPose3D.getX());
      if (xController.atGoal()) {
        xSpeed = 0;
      }

      var ySpeed = yController.calculate(robotPose3D.getY());
      if (yController.atGoal()) {
        ySpeed = 0;
      }

      var omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians());
      if (omegaController.atGoal()) {
        omegaSpeed = 0;
      }

      drive.runVelocity(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              xSpeed, ySpeed, omegaSpeed, robotPose.getRotation()));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
    // led.setGreen();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (xController.atGoal() && yController.atGoal() && omegaController.atGoal()) {
      led.setGreen();
      return true;
    } else {
      return false;
    }
  }
}
