// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.ElevatorConstants;

/** Add your docs here. */
public class ElevatorIOSparkMAX implements ElevatorIO {
  // Initialize the SparkMAX motors for main and follower
  private final SparkMax leadMotor =
      new SparkMax(ElevatorConstants.kLeadElevatorCanId, MotorType.kBrushless);
  ;
  private final SparkMax followerMotor =
      new SparkMax(ElevatorConstants.kFollowerElevatorCanId, MotorType.kBrushless);
  ;
  private final SparkClosedLoopController m_controller = leadMotor.getClosedLoopController();
  private final RelativeEncoder encoder = leadMotor.getEncoder();

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
    followerConfig.follow(ElevatorConstants.kLeadElevatorCanId, true);

    followerMotor.configure(
        followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig maxMotionConfig = new SparkMaxConfig();
    // Set MAX Motion parameters
    maxMotionConfig
        .closedLoop
        .p(0)
        .i(0)
        .d(0)
        .outputRange(0, 0) // set PID gains
        .maxMotion
        .maxVelocity(0.3)
        .maxAcceleration(0.3)
        .allowedClosedLoopError(0);

    leadMotor.configure(
        maxMotionConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
    leadMotor
        .getClosedLoopController()
        .setReference(position, ControlType.kMAXMotionPositionControl);
  }

  public void moveUp() {
    leadMotor.set(.15);
  }

  public void moveDown() {
    leadMotor.set(-.15);
  }

  @Override
  public void stop() {
    leadMotor.set(0);
  }
}
