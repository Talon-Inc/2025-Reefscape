// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;
import org.littletonrobotics.junction.Logger;

public class Algae extends SubsystemBase {
  /** Creates a new Algae. */
  private static double kDt = 0;

  private static double kMaxVelocity = 0;
  private static double kMaxAccerlation = 0;
  private static double kP = 0;
  private static double kI = 0;
  private static double kD = 0.0;
  private static double kS = 0;
  private static double kG = 0;
  private static double kV = 0;
  private static double ka = 0;
  private static double lastSpeed = 0;
  private static double lastTime = Timer.getFPGATimestamp();

  private final SparkMax armMotor =
      new SparkMax(AlgaeConstants.kLeftMotorCanId, MotorType.kBrushless);

  private final SparkMax topWheelMotor = new SparkMax(19, MotorType.kBrushless);

  private final SparkMax bottomWheelMotor = new SparkMax(20, MotorType.kBrushless);

  private final SparkMax deploymentMotor = new SparkMax(21, MotorType.kBrushless);

  // Create profile constraints
  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAccerlation);

  // Create ProfiledPIDController Using Trapezoidal Constraints
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(kP, kI, kD, m_constraints);

  // Elevator FF
  private final ArmFeedforward m_feedforward = new ArmFeedforward(kS, kG, kV, ka);

  private final RelativeEncoder m_encoder = armMotor.getEncoder();

  public Algae() {

    SparkMaxConfig armMotorConfig = new SparkMaxConfig();
    armMotorConfig.smartCurrentLimit(30).voltageCompensation(12).idleMode(IdleMode.kBrake);

    SparkMaxConfig topWheelMotorConfig = new SparkMaxConfig();
    topWheelMotorConfig.smartCurrentLimit(20).voltageCompensation(12).idleMode(IdleMode.kBrake);

    SparkMaxConfig bottomWheelMotorConfig = new SparkMaxConfig();
    bottomWheelMotorConfig.smartCurrentLimit(20).voltageCompensation(12).idleMode(IdleMode.kBrake);

    SparkMaxConfig deploymentMotorConfig = new SparkMaxConfig();
    deploymentMotorConfig.smartCurrentLimit(20).voltageCompensation(12).idleMode(IdleMode.kBrake);

    armMotor.configure(
        armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    topWheelMotor.configure(
        topWheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomWheelMotor.configure(
        bottomWheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    deploymentMotor.configure(
        deploymentMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_controller.setTolerance(0.02);
  }

  public boolean checkArmGoal() {
    return m_controller.atGoal();
  }

  public void intakeAlgae() {
    topWheelMotor.set(-.1);
    bottomWheelMotor.set(-.1);
  }

  public void holdAlgae() {
    topWheelMotor.set(0);
    bottomWheelMotor.set(0);
  }

  public void shootAlgaeProcessor() {
    topWheelMotor.set(.1);
    bottomWheelMotor.set(.1);
  }

  public void shootAlgaeBarge() {
    topWheelMotor.set(.25);
    bottomWheelMotor.set(.25);
  }

  public void deployAlgaeClaw() {
    deploymentMotor.set(.1);
  }

  public void retractAlgaeClaw() {
    deploymentMotor.set(-.1);
  }

  public void stopAlgaeShooter() {
    topWheelMotor.stopMotor();
    bottomWheelMotor.stopMotor();
  }

  public void stopAlgaeClaw() {
    deploymentMotor.stopMotor();
  }

  public void moveArmUp() {
    armMotor.set(.25);
  }

  public void moveArmDown() {
    armMotor.set(-.1);
  }

  public void stopArmMotor() {
    armMotor.stopMotor();
  }

  public void resetArmPosition(double newPosition, double newVelocity) {
    m_controller.reset(newPosition, newVelocity);
  }

  public double getArmPosition() {
    return m_encoder.getPosition();
  }

  public double getArmVelocity() {
    return m_encoder.getVelocity();
  }

  public boolean checkIntakeAlgae() {
    if (topWheelMotor.getOutputCurrent() > 20 && bottomWheelMotor.getOutputCurrent() > 20) {
      return true;
    } else {
      return false;
    }
  }

  public void goToPositionArm(double goalPosition) {
    m_controller.setGoal(goalPosition);
    double pidVal = m_controller.calculate(m_encoder.getPosition());
    double feedForward =
        m_feedforward.calculate(
            m_controller.getSetpoint().position, m_controller.getSetpoint().velocity);
    armMotor.setVoltage(pidVal + feedForward);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Arm Encoder", m_encoder.getPosition());
    Logger.recordOutput("Arm Velocity", m_encoder.getVelocity());
    Logger.recordOutput("Arm Setpoint", m_controller.getSetpoint().position);
    Logger.recordOutput("Arm Position Error", m_controller.getPositionError());
    Logger.recordOutput("Arm Motor Current", armMotor.getOutputCurrent());
    Logger.recordOutput("Arm Motor", m_controller.atGoal());

    Logger.recordOutput("Top Motor Current", topWheelMotor.getOutputCurrent());
    Logger.recordOutput("Bottom Motor Current", bottomWheelMotor.getOutputCurrent());
  }
}
