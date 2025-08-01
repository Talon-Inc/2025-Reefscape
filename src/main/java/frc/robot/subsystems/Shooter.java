// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final SparkMax leftShooterMotor =
      new SparkMax(ShooterConstants.kLeftShooterCanId, MotorType.kBrushless);
  private final SparkMax rightShooterMotor =
      new SparkMax(ShooterConstants.kRightShooterCanId, MotorType.kBrushless);
  private final AnalogInput intakeSensor = new AnalogInput(1);

  /** Creates a new Shooter. */
  public Shooter() {
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
    leftShooterMotor.configure(
        Configs.Shooter.leftShooter,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    rightShooterMotor.configure(
        Configs.Shooter.rightShooter,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void intakeCoral() {
    leftShooterMotor.set(.3);
    rightShooterMotor.set(.3);
  }

  public boolean isCoralLoaded() {
    return intakeSensor.getVoltage() < .5;
  }

  public void shoot() {
    leftShooterMotor.set(.4);
    rightShooterMotor.set(.4);
  }

  public void reverseShoot() {
    leftShooterMotor.set(-.15);
    rightShooterMotor.set(-.15);
  }

  public void shootSideways() {
    leftShooterMotor.set(.075);
    rightShooterMotor.set(.55);
  }

  public void stop() {
    leftShooterMotor.stopMotor();
    rightShooterMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run4
    Logger.recordOutput("Color Sensor Log", intakeSensor.getVoltage());
  }
}
