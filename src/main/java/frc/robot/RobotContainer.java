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

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.*;
import frc.robot.commands.ElevatorCommands.*;
import frc.robot.commands.VisionCommands.*;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOSparkMAX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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
  private final ElevatorToL1 elevatorL1;
  private final ElevatorToL2 elevatorL2;
  private final ElevatorToL3 elevatorL3;
  private final ElevatorToL4 elevatorL4;
  private final SetHome setHome;
  private final IntakeCoral intake;
  private final ShootCoral shootCoral;
  private final ShootCoralSideways shootSideways;
  private final ReverseShooter shootReverse;
  private final Climb climb;
  private final DeployClimb deployClimb;
  private final LeftAutoAlign leftAlign;
  private final RightAutoAlign rightAlign;
  // private final SetElevatorSpeed setElevatorSpeed;
  // private final ElevatorDown elevatorDown;

  // Controller
  private final CommandPS5Controller driverController = new CommandPS5Controller(0);
  private final CommandPS5Controller operatorController = new CommandPS5Controller(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Subsystems
    elevator = new Elevator(new ElevatorIOSparkMAX());
    shooter = new Shooter();
    climber = new Climber();

    // Commands
    // Elevator Commands
    elevatorL1 = new ElevatorToL1(elevator);
    elevatorL2 = new ElevatorToL2(elevator);
    elevatorL3 = new ElevatorToL3(elevator);
    elevatorL4 = new ElevatorToL4(elevator);
    setHome = new SetHome(elevator);
    // setElevatorSpeed = new setElevatorSpeed(elevator);
    // elevatorDown = new ElevatorDown(elevator);

    // Shooter Commands
    intake = new IntakeCoral(shooter);
    shootCoral = new ShootCoral(shooter);
    shootReverse = new ReverseShooter(shooter);
    shootSideways = new ShootCoralSideways(shooter);

    // Climber Commands
    climb = new Climb(climber);
    deployClimb = new DeployClimb(climber);

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
        leftAlign = new LeftAutoAlign(drive, vision);
        rightAlign = new RightAutoAlign(drive, vision);
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
        leftAlign = new LeftAutoAlign(drive, vision);
        rightAlign = new RightAutoAlign(drive, vision);
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
        leftAlign = new LeftAutoAlign(drive, vision);
        rightAlign = new RightAutoAlign(drive, vision);
        break;
    }

    // Set Up Commands for PathPlanner
    NamedCommands.registerCommand("elevatorToL4", elevatorL4);
    NamedCommands.registerCommand("elevatorHome", setHome);
    NamedCommands.registerCommand("intakeCoral", intake);
    NamedCommands.registerCommand("shootCoral", shootCoral);
    NamedCommands.registerCommand("alignToLeft", leftAlign);
    NamedCommands.registerCommand("alignToRight", rightAlign);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // // Set Up Autos For PathPlanner
    autoChooser.addOption("4 Piece Coral Bottom", AutoBuilder.buildAuto("4 Piece Coral Bottom"));
    autoChooser.addOption("1 Piece Coral", AutoBuilder.buildAuto("1 Piece Coral"));
    autoChooser.addOption("2 Piece Coral Bottom", AutoBuilder.buildAuto("2 Piece Coral Bottom"));
    autoChooser.addOption("2 Piece Coral Top", AutoBuilder.buildAuto("2 Piece Coral Top"));
    autoChooser.addOption(
        "Copy of 2 Piece Coral Top", AutoBuilder.buildAuto("Copy of 2 Piece Coral Top"));
    autoChooser.addOption("Anthony's Test", AutoBuilder.buildAuto("Anthony's Test"));
    autoChooser.addOption("Elevator Test", AutoBuilder.buildAuto("Elevator Test"));
    autoChooser.addOption("Anthony's Test 2", AutoBuilder.buildAuto("Anthony's Test 2"));
    autoChooser.addOption(
        "Slower Anthony's Test 2", AutoBuilder.buildAuto("Slower Anthony's Test 2"));
    autoChooser.addOption("Test Gyro", AutoBuilder.buildAuto("Test Gyro"));
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

    // Driver
    driverController.L1().onTrue(intake);
    driverController.R1().whileTrue(shootCoral);
    driverController.R2().whileTrue(shootSideways);
    driverController.L2().whileTrue(shootReverse);
    driverController.L3().whileTrue(leftAlign);
    driverController.R3().whileTrue(rightAlign);
    driverController.create().whileTrue(climb);
    driverController.options().whileTrue(deployClimb);
    // driverController.povUp().whileTrue(deployClaws);
    // driverController.povDown().whileTrue(retractClaws);

    // Operator
    operatorController.povDown().onTrue(elevatorL1);
    operatorController.povLeft().onTrue(elevatorL2);
    operatorController.povRight().onTrue(elevatorL3);
    operatorController.triangle().onTrue(elevatorL4);
    operatorController.cross().whileTrue(setHome);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
