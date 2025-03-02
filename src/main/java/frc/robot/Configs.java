// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

/** Add your docs here. */
public final class Configs {
  public static final class Algae {
    public static final SparkMaxConfig algaeConfig = new SparkMaxConfig();

    static {
      // Configure basic settings of the algae motor
      algaeConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50).voltageCompensation(12);
    }
  }

  public static final class Climber {
    public static final SparkMaxConfig climberConfig = new SparkMaxConfig();

    static {
      // Configure basic settings of the climber motor
      climberConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50).voltageCompensation(12);
    }
  }

  public static final class Elevator {
    public static final SparkMaxConfig leadElevatorConfig = new SparkMaxConfig();
    public static final SparkMaxConfig followerElevatorConfig = new SparkMaxConfig();

    static {
      // Configure basic settings of the elevator motor
      leadElevatorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50).voltageCompensation(12);

      /*
       * Configure the reverse limit switch for the elevator. By enabling the limit switch, this
       * will prevent any actuation of the elevator in the reverse direction if the limit switch is
       * pressed.
       */
      leadElevatorConfig
          .limitSwitch
          .reverseLimitSwitchEnabled(true)
          .reverseLimitSwitchType(Type.kNormallyOpen);

      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      leadElevatorConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control
          .p(0.1)
          .i(0)
          .d(0)
          .outputRange(-1, 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(4200)
          .maxAcceleration(6000)
          .allowedClosedLoopError(0.5);

      // Configure basic settings of the elevator motor
      followerElevatorConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50)
          .voltageCompensation(12);

      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       * Also, invert the follower
       */
      followerElevatorConfig
          .follow(Constants.ElevatorConstants.kLeadElevatorCanId, true)
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control
          .p(0.1)
          .i(0)
          .d(0)
          .outputRange(-1, 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(4200)
          .maxAcceleration(6000)
          .allowedClosedLoopError(0.5);
    }
  }

  public static final class Intake {
    public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();

    static {
      // Configure basic settings of the intake motor
      intakeConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50).voltageCompensation(12);
    }
  }

  public static final class Shooter {
    public static final SparkMaxConfig leftShooter = new SparkMaxConfig();
    public static final SparkMaxConfig rightShooter = new SparkMaxConfig();

    static {
      // Configure basic settings of the left shooter motor
      leftShooter.idleMode(IdleMode.kBrake).smartCurrentLimit(20).voltageCompensation(12);

      // Configure basic settings of the right shooter motor
      rightShooter.idleMode(IdleMode.kBrake).smartCurrentLimit(20).voltageCompensation(12).follow(13, true);
    }
  }
}
