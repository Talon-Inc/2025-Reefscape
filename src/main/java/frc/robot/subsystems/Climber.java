// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.servohub.ServoHub;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private final SparkMax climberMotor =
      new SparkMax(ClimberConstants.kLeftMotorCanId, MotorType.kBrushless);
  private final ServoHub servo1 = new ServoHub(18);
  private final Servo servo2 = new Servo(2);
  private final DigitalInput limitSwitchTop = new DigitalInput(2);
  private final DigitalInput limitSwitchBottom = new DigitalInput(3);

  /** Creates a new Climber. */
  public Climber() {
    climberMotor.configure(
        Configs.Climber.climberConfig,
        com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    servo2.setSpeed(1);
  }

  public void deployClaws() {
    servo2.setPosition(1);
  }

  public void retractClaws() {
    servo2.setPosition(0);
  }

  public void deployClimb() {
    climberMotor.set(-.6);
  }

  public void climb() {
    climberMotor.set(.6);
  }

  public void stopClimber() {
    climberMotor.stopMotor();
  }

  public boolean getLimitSwitchTop() {
    return limitSwitchTop.get();
  }

  public boolean getLimitSwitchBottom() {
    return limitSwitchBottom.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
