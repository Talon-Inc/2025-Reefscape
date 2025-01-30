// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax shooterMotor;
  private final SparkMaxPIDController pidController;
  private final CANEncoder encoder;


public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  public Shooter(int motorPort) {
    Shooter = new CANSparkMax(motorPort, MotorType.kBrushless);
        pidController = shooter.getPIDController();
        encoder = shooter.getEncoder(); 
 }
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
