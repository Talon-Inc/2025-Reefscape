// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

/** Add your docs here. */
public class ElevatorIOSparkMAX implements ElevatorIO {
  private final SparkMax leadMotor;
  private final SparkMax followerMotor;
  private final RelativeEncoder encoder;

  // Constructor
  public ElevatorIOSparkMAX() {
    // Initialize the SparkMAX motors for main and follower
    leadMotor = new SparkMax(11, MotorType.kBrushless);
    followerMotor = new SparkMax(12, MotorType.kBrushless);
    SparkClosedLoopController m_controller = leadMotor.getClosedLoopController();

    // Invert follower
    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig.follow(11, true);

    followerMotor.configure(followerConfig, null, null);

    // Initialize the encoder for main
    encoder = leadMotor.getEncoder();

    SparkMaxConfig maxMotionConfig = new SparkMaxConfig();
    // Set MAX Motion parameters
    maxMotionConfig
        .closedLoop
        .maxMotion
        .maxVelocity(0)
        .maxAcceleration(0)
        .allowedClosedLoopError(0);

    SparkMaxConfig pidConfig = new SparkMaxConfig();
    // Set PID Gains
    pidConfig.closedLoop.p(0).i(0).d(0).outputRange(0, 0);
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
    leadMotor.getClosedLoopController().setReference(position, ControlType.kPosition);
  }

  @Override
  public void stop() {
    leadMotor.setVoltage(0);
  }
}
