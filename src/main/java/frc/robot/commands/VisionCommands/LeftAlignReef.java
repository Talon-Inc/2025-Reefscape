// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.VisionCommands;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.vision.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LeftAlignReef extends Command {
  
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(maxVelocity:3, maxAcceleration: 2);
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(maxVelocity:3, maxAcceleration: 2);
  private static final TrapezoidProfile.Constraints OMEGA_COSTRAINTS = new TrapezoidProfile.Constraints(maxVelocity: 8, maxAcceleration: 8);

  private static final int TAG_TO_CHASE = 2;
  private static final Transform3d TAG_TO_GOAL =
    new Transform3d(
      new Translation3d(x:1.5, y:0.0, z:0.0),
      new Rotational3d(roll:0.0, pitch:0.0, Math.PI));
      
    private final PhotonCamera photonCamer;
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final Supplier<Pose2d> poseProvider;

    private final ProfilePIDController xController = new ProfilePIDController(Kp: 3, Ki:0, Kd: 0, X_CONSTRAINTS);
    private final ProfilePIDController xController = new ProfilePidController(Kp: 3, Ki:0, Kd: 0, Y_CONSTRAINTS);
    private final ProfilePIDController omegaController = new ProfilePIDController(Kp: 2, Ki: 0, Kd: 0, OMEGA_CONSTRAINTS);

    private PhotonTrackedTarget lastTarget;

    public ChaseTagCommand(
            PhotnCamera photonCamera,
            DrivetrainSubsystem drivertrainSubsystem,
            Supplier<Pose2d> poseProvider) {
      this.photonCamera = photonCamera;
      this.drivetrainSubsystem = drivetrainSubsystem;
      this.poseProvider = poseProvider;

      xController.setTolerance(positionTolerance: 0.2);
      omegaController.setTolerance(Units.degreeToRadians(degrees: 3));
      omegaController.enableContinuousInput(-Math.PI, Math.PI);

      addRequiremens(drivetrainSubsystem);
   }

Override
public void initialize() {
  lastTarget = null; 
  var robotPose = poseProvider.get();
  var robotPose = 
     new Pose3d(
      robotPose2d.getx(),
      robotPose2d.getY(),
      z: 0.0,
      new Rotation3d(roll: 0.0, pitch: 0.0, robotRotation().getRadian()

      var targetOpt = photoRes.getTargets().stream()
          .filter(t -> t.getFiducialID() == TAG_TO_CHASE)
          .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2 && t.getPose)
        .findFirst();
      if (targetOpt.isPresent()) {
          var target = targetOpt.get();
          // This is new target data, so recalculate the goal
          lastTarget = target;
  
          // Transform the robot's pose to find the camera's pose
          var cameraPose = rbotPose.transformBy(ROBOT_TO_CAMERA)
          
          // Transform the camera's to the target's pose
          var targetPose = cameraPose.transformBy(camToTarget);
          var goalPose = targetPose.targetPose.transformBy(TAG_TO_GOAL).toPose2d();
          
          //Transform the tag'spose to set our goal
          var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

          // Drive 
          xController.setGoal(goalPose.getx());
          yController.setGoal(goalPose.getY());
          omegaController.setGoal(goalPose.getRotation().getRadians());
         }
}

    if (lastTarget == null) {
    // No target has been visible
    drivetrianSubsystem.stop();
    } else {
        // Drive to the target 
        var xSpeed = xController.calculate(robotPose.getX());
        if (xCon)

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;


  }
}
