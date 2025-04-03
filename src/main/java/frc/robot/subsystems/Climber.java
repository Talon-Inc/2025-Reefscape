// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private final SparkMax climberMotor =
      new SparkMax(ClimberConstants.kClimberMotorCanId, MotorType.kBrushless);
  /** Creates a new Climber. */
  public Climber() {
    climberMotor.configure(
        Configs.Climber.climberConfig,
        com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void deployClimb() {
    climberMotor.set(-1);
  }

  public void climb() {
    climberMotor.set(.85);
  }

  public void stopClimber() {
    climberMotor.stopMotor();
  }

  public void holdClimber() {
    climberMotor.set(.05);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
