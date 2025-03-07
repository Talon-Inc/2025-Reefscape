// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
// import frc.robot.Configs.Elevator;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorCommands.ElevatorDown;
import frc.robot.commands.ElevatorCommands.ElevatorToL1;
import frc.robot.commands.ElevatorCommands.ElevatorToL2;
import frc.robot.commands.ElevatorCommands.ElevatorToL3;
import frc.robot.commands.ElevatorCommands.ElevatorToL4;
import frc.robot.commands.ElevatorCommands.setElevatorSpeed;
import frc.robot.commands.ElevatorCommands.setHome;
import frc.robot.commands.VisionCommands.leftAutoAlign;
import frc.robot.commands.VisionCommands.rightAutoAlign;
import frc.robot.commands.climb;
import frc.robot.commands.deployClimb;
import frc.robot.commands.intakeCoral;
import frc.robot.commands.reverseShooter;
import frc.robot.commands.shootCoral;
import frc.robot.commands.shootCoralSidways;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;
// import frc.robot.commands.VisionCommands.leftAutoAlign;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOSparkMAX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Elevator elevator;
  private final Shooter shooter;
  private final Climber climber;
  private final Vision vision;

  // Commands
  private final setHome setHome;
  private final ElevatorToL1 elevatorL1;
  private final ElevatorToL2 elevatorL2;
  private final ElevatorToL3 elevatorL3;
  private final ElevatorToL4 elevatorL4;
  private final intakeCoral intake;
  private final shootCoral shootCoral;
  private final reverseShooter shootReverse;
  private final climb climb;
  private final deployClimb deployClimb;
  private final shootCoralSidways shootSideways;
  private final leftAutoAlign leftAuto;
  private final rightAutoAlign rightAuto;
  private final setElevatorSpeed setElevatorSpeed;
  private final ElevatorDown elevatorDown;

  // Controller
  private final CommandPS5Controller driverController = new CommandPS5Controller(0);
  private final CommandPS5Controller operatorController = new CommandPS5Controller(1);

  // Dashboard inputs
  // private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Subsystems
    elevator = new Elevator(new ElevatorIOSparkMAX());
    shooter = new Shooter();
    climber = new Climber();

    // Commands
    elevatorL4 = new ElevatorToL4(elevator);
    elevatorL3 = new ElevatorToL3(elevator);
    elevatorL2 = new ElevatorToL2(elevator);
    elevatorL1 = new ElevatorToL1(elevator);
    setHome = new setHome(elevator);
    intake = new intakeCoral(shooter);
    shootCoral = new shootCoral(shooter);
    shootReverse = new reverseShooter(shooter);
    climb = new climb(climber);
    deployClimb = new deployClimb(climber);
    shootSideways = new shootCoralSidways(shooter);
    setElevatorSpeed = new setElevatorSpeed(elevator);
    elevatorDown = new ElevatorDown(elevator);

    drive =
        new Drive(
            new GyroIONavX(),
            new ModuleIOSpark(0),
            new ModuleIOSpark(1),
            new ModuleIOSpark(2),
            new ModuleIOSpark(3));
    vision =
        new Vision(
            drive::addVisionMeasurement,
            new VisionIOPhotonVision(camera0Name, robotToCamera0),
            new VisionIOPhotonVision(camera1Name, robotToCamera1));
    // leftAuto = new leftAutoAlign(drive, vision);
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // drive =
        //     new Drive(
        //         new GyroIONavX(),
        //         new ModuleIOSpark(0),
        //         new ModuleIOSpark(1),
        //         new ModuleIOSpark(2),
        //         new ModuleIOSpark(3));
        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement, new VisionIOPhotonVision(camera0Name,
        // robotToCamera0));
        leftAuto = new leftAutoAlign(drive, vision);
        rightAuto = new rightAutoAlign(drive, vision);
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        // drive =
        //     new Drive(
        //         new GyroIO() {},
        //         new ModuleIOSim(),
        //         new ModuleIOSim(),
        //         new ModuleIOSim(),
        //         new ModuleIOSim());
        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose));
        leftAuto = new leftAutoAlign(drive, vision);
        rightAuto = new rightAutoAlign(drive, vision);
        break;

      default:
        // Replayed robot, disable IO implementations
        // drive =
        //     new Drive(
        //         new GyroIO() {},
        //         new ModuleIO() {},
        //         new ModuleIO() {},
        //         new ModuleIO() {},
        //         new ModuleIO() {});
        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOPhotonVision(camera0Name, robotToCamera0) {},
        //         new VisionIO() {}) {};
        leftAuto = new leftAutoAlign(drive, vision);
        rightAuto = new rightAutoAlign(drive, vision);
        break;
    }

    // Set up auto routines
    // autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Set Up Commands for PathPlanner
    // NamedCommands.registerCommand("Shoot Coral", shootCoral);
    // NamedCommands.registerCommand("Intake Coral", intake);
    // NamedCommands.registerCommand("Elevator To L4", elevatorL4);
    // NamedCommands.registerCommand("Elevator To Home", setHome);

    // // Set Up Autos For PathPlanner
    // autoChooser.addOption("4 Piece Coral Bottom", AutoBuilder.buildAuto("4 Piece Coral Bottom"));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    // Lock to 0° when Square button is held
    driverController
        .square()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    driverController.cross().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when Circle button is pressed
    driverController
        .circle()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // Move Elevator to Level 1
    driverController.povLeft().whileTrue(elevatorL2);
    // driverController.povRight().onTrue(elevatorL2);
    // driverController.povUp().onTrue(elevatorL3);
    // driverController.povDown().onTrue(setHome);
    driverController.L1().whileTrue(intake);
    driverController.R1().whileTrue(shootCoral);
    driverController.R2().whileTrue(shootSideways);
    driverController.L2().whileTrue(shootReverse);
    driverController.povUp().whileTrue(setElevatorSpeed);
    driverController.povDown().whileTrue(elevatorDown);
    // driverController.create().whileTrue(deployClimb);
    // driverController.create().whileTrue(climb);

    driverController.L3().whileTrue(leftAuto);
    driverController.R3().whileTrue(rightAuto);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   return autoChooser.get();
  // }
}
