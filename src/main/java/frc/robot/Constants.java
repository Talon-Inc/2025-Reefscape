// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class AlgaeConstants {
    // SPARK MAX CAN IDs

  }

  public static final class ClimberConstants {
    // SPARK MAX CAN IDs
    public static final int kLeftMotorCanId = 17;
  }

  public static final class ElevatorConstants {
    // SPARK MAX CAN IDs
    public static final int kLeadElevatorCanId = 11;
    public static final int kFollowerElevatorCanId = 12;
  }

  public static final class IntakeConstants {
    // SPARK MAX CAN IDs

  }

  public static final class ShooterConstants {
    // SPARK MAX CAN IDs
    public static final int kLeftShooterCanId = 13;
    public static final int kRightShooterCanId = 14;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.1;
    public static final double kTriggerButtonThreshold = 0.2;
  }
}
