// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.ElevatorConstants;

/** Add your docs here. */
public class ElevatorIOSparkMAX implements ElevatorIO {
  private static double kDt = 0.02;
  private static double kMaxVelocity = .5;
  private static double kMaxAccerlation = 2;
  private static double kP = 1.5;
  private static double kI = 0;
  private static double kD = 0.0;
  private static double kS = 0.2;
  private static double kG = .58;
  private static double kV = .671;
  private static double ka = 0.0301;
  private static double lastSpeed = 0;
  private static double lastTime = Timer.getFPGATimestamp();

  // Initialize the SparkMAX motors for main and follower
  private final SparkMax leadMotor =
      new SparkMax(ElevatorConstants.kLeadElevatorCanId, MotorType.kBrushless);

  private final SparkMax followerMotor =
      new SparkMax(ElevatorConstants.kFollowerElevatorCanId, MotorType.kBrushless);

  // private final SparkClosedLoopController m_controller = leadMotor.getClosedLoopController();
  // private final RelativeEncoder encoder = leadMotor.getEncoder();

  // Creates Profile Constraints
  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAccerlation);

  // PID Controller
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(kP, kI, kD, m_constraints);

  // Elevator Feedforward
  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(kS, kG, kV, ka);

  private final RelativeEncoder m_encoder = leadMotor.getEncoder();

  // Constructor
  public ElevatorIOSparkMAX() {
    /*
     * Apply the appropriate configurations to the SPARKs.
     *
     * kResetSafeParameters is used to get the SPARK to a known state. This
     * is useful in case the SPARK is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK loses power. This is useful for power cycles that may occur
     * mid-operation.
     */

    // Invert follower
    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig
        .follow(11, true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(50)
        .voltageCompensation(12);

    followerMotor.configure(
        followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig leadMotorConfig = new SparkMaxConfig();
    // Set MAX Motion parameters
    leadMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50).voltageCompensation(12);

    leadMotor.configure(
        leadMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // m_encoder.setPosition(1.0 / 360.0 * 2.0 * Math.PI * 1.5);
    org.littletonrobotics.junction.Logger.recordOutput("Encoder", m_encoder.getPosition());
  }

  @Override
  public void set(double voltage) {
    // Set the power to the main motor
    leadMotor.set(voltage);
  }

  @Override
  public double getPosition() {
    // Get the position from the encoder
    return m_encoder.getPosition();
  }

  @Override
  public double getVelocity() {
    // Get the Velocity fromt the motor
    return m_encoder.getVelocity();
  }

  @Override
  public void resetPosition() {
    // Reset the encoder to the specificed position
    m_encoder.setPosition(0);
  }

  public double getSetpoint() {
    return m_controller.getSetpoint().position;
  }

  public double getDesiredVeloicty() {
    return m_controller.getSetpoint().velocity;
  }

  // public void setGoal(double goal) {
  //   m_controller.setGoal(goal);
  // }

  @Override
  public void goToPosition(double goalPosition) {
    // leadMotor
    //     .getClosedLoopController()
    //     .setReference(position, ControlType.kMAXMotionPositionControl);
    // double linearDistance = m_encoder.getPosition() * 2 * Math.PI;
    m_controller.setGoal(goalPosition);

    double pidVal = m_controller.calculate(m_encoder.getPosition(), goalPosition);
    double acceleration =
        (m_controller.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);
    leadMotor.setVoltage(
        pidVal + m_feedforward.calculate(m_controller.getSetpoint().velocity, acceleration));
    lastSpeed = m_controller.getSetpoint().velocity;
    lastTime = Timer.getFPGATimestamp();

    // leadMotor.setVoltage(
    //     m_controller.calculate(m_encoder.getPosition())
    //         + m_feedforward.calculateWithVelocities(
    //             m_encoder.getVelocity() / 60 * 2 * Math.PI * 0.012,
    //             m_controller.getSetpoint().velocity));

    // m_controller.setReference(
    //     goalPosition,
    //     ControlType.kPosition,
    //     ClosedLoopSlot.kSlot0,
    //     m_feedforward.calculate(m_encoder.getVelocity() / 60 * 2 * Math.PI * .012));
  }

  @Override
  public void stop() {
    leadMotor.set(0);
  }
}
