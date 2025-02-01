// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final SparkMax left_ShooterMotor = new SparkMax(0, MotorType.kBrushless);
  private final SparkMax right_ShooterMotor = new SparkMax(0, MotorType.kBrushless);
  private final SparkClosedLoopController left_pidController = left_ShooterMotor.getClosedLoopController();
  private final SparkClosedLoopController right_pidController = right_ShooterMotor.getClosedLoopController();
  private final RelativeEncoder left_encoder = left_ShooterMotor.getEncoder();
  private final RelativeEncoder right_encoder = right_ShooterMotor.getEncoder();

  public static final SparkMaxConfig left_config = new SparkMaxConfig();
  public static final SparkMaxConfig config = new SparkMaxConfig();
  /** Creates a new Shooter. */
  public Shooter() {
    config.idleMode(SparkBaseConfig.IdleMode.kBrake);

    left_ShooterMotor.clearFaults();
    left_ShooterMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    right_ShooterMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void shoot() {
    left_ShooterMotor.set(0);
    right_ShooterMotor.set(0);
  }

  public void stop() {
    left_ShooterMotor.stopMotor();
    right_ShooterMotor.stopMotor();
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
