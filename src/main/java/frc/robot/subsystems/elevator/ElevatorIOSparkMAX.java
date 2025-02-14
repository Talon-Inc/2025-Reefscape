// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import java.lang.ModuleLayer.Controller;

import javax.security.auth.login.AccountLockedException;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.ElevatorConstants;

/** Add your docs here. */
public class ElevatorIOSparkMAX implements ElevatorIO {
  private static double kDt =  0.02;
  private static double kMaxVelocity = 1.75;
  private static double kMaxAccerlation = 0.75;
  private static double kP = 1.3;
  private static double kI = 0;
  private static double kD = 0.7;
  private static double kS = 1.1;
  private static double kG = 1.2;
  private static double kV = 1.3;
  private static double lastSpeed = 0;
  private static double lastTime = Timer.getFPGATimestamp();

  // Initialize the SparkMAX motors for main and follower
  private final SparkMax leadMotor =
      new SparkMax(ElevatorConstants.kLeadElevatorCanId, MotorType.kBrushless);
  ;
  private final SparkMax followerMotor =
      new SparkMax(ElevatorConstants.kFollowerElevatorCanId, MotorType.kBrushless);
  ;
  // private final SparkClosedLoopController m_controller = leadMotor.getClosedLoopController();
  private final RelativeEncoder encoder = leadMotor.getEncoder();

  // Creates Profile Constraints
  private final TrapezoidProfile.Constraints m_constraints = 
    new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAccerlation);
  
  // PID Controller
  private final ProfiledPIDController m_controller = 
    new ProfiledPIDController(kP, kI, kD, m_constraints);

  // Elevator Feedforward
  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(kS, kG, kV);

  private final Encoder m_encoder = new Encoder(1,2);

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
    followerConfig.follow(11, true);

    // leadMotor.configure(
    //     followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    followerMotor.configure(
        followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_encoder.setDistancePerPulse(1.0 / 360.0 * 2.0 * Math.PI * 1.5);
  }

  @Override
  public void set(double voltage) {
    // Set the power to the main motor
    leadMotor.set(voltage);
  }

  @Override
  public double getPosition() {
    // Get the position from the encoder
    return encoder.getPosition();
  }

  @Override
  public double getVelocity() {
    // Get the Velocity fromt the motor
    return encoder.getVelocity();
  }

  @Override
  public void resetPosition() {
    // Reset the encoder to the specificed position
    encoder.setPosition(0);
  }

  @Override
  public void setPosition(double position) {
    // leadMotor
    //     .getClosedLoopController()
    //     .setReference(position, ControlType.kMAXMotionPositionControl);
    double pidVal = m_controller.calculate(m_encoder.getDistance(), position);
    double acceleration = (m_controller.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);
    leadMotor.setVoltage(
      pidVal
      + m_feedforward.calculate(m_controller.getSetpoint().velocity, acceleration)); 
      lastSpeed = m_controller.getSetpoint().velocity;
      lastTime = Timer.getFPGATimestamp();
  }

  @Override
  public void stop() {
    leadMotor.setVoltage(0);
  }
}
