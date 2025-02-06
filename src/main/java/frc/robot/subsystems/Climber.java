// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private final SparkMax climberMotor =
      new SparkMax(ClimberConstants.kClimberCanId, MotorType.kBrushless);
  private final RelativeEncoder encoder = climberMotor.getEncoder();
  private final DigitalInput topLimitSwitch = new DigitalInput(0);
  private final DigitalInput bottomLimitSwitch = new DigitalInput(1);

  /** Creates a new Climber. */
  public Climber() {}

  public void climberUp() {
    climberMotor.set(.5);
  }

  public void climberDown() {
    climberMotor.set(-.5);
  }

  public void stop() {
    climberMotor.stopMotor();
  }

  public boolean getLimitSwitchTop() {
    return topLimitSwitch.get();
  }

  public boolean getLimitSwitchBottom() {
    return bottomLimitSwitch.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
