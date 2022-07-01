// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.sorutil.motor.PidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class RobotDimensions {
        public static final double WIDTH = Units.inchesToMeters(24);
        public static final double LENGTH = Units.inchesToMeters(24);

        public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4) * Math.PI;

        public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS =
                new SwerveDriveKinematics(
                        new Translation2d(-RobotDimensions.LENGTH / 2, RobotDimensions.WIDTH / 2),
                        new Translation2d(-RobotDimensions.LENGTH / 2, -RobotDimensions.WIDTH / 2),
                        new Translation2d(RobotDimensions.LENGTH / 2, RobotDimensions.WIDTH / 2),
                        new Translation2d(RobotDimensions.LENGTH / 2, -RobotDimensions.WIDTH / 2));
    }

    public static final class Drive {
        // Drive settings here
        // TODO: replace these with actual constants
        public static final double MAX_SPEED = 0; // m/s
    }

    public static final class Subsystem {
        public static final class Intake {
            public static final double SPEED = 4200; // RPM
            public static final PidProfile PID_PROFILE = new PidProfile(0.01, 0, 0);
        }

        public static final class Shooter {
            public static final PidProfile PID_PROFILE = new PidProfile(0.01, 0, 0.001);
            public static final PidProfile FEEDER_PROFILE = new PidProfile(0.1, 0.001, 0.01);

            public static final int FEEDER_ENCODER_CHANNEL_A = 1;
            public static final int FEEDER_ENCODER_CHANNEL_B = 2;

            public static final double SHOOTER_ERROR_TOLERANCE = 0.05; // 5%

            public static final double FEEDER_SPEED = 500; // RPM
        }

        public static final class Swerve {
            public static final PidProfile STEER_PROFILE = new PidProfile(3.5, 0, 1.5);
            public static final PidProfile POWER_PROFILE = new PidProfile(0.55, 0, 0.025);

            public static final double SWERVE_MAX_SPEED = 5.18; // m/s
            public static final double SWERVE_MAX_ACCELERATION = 3; // m/s^2
            public static final double SWERVE_ROTATION_MAX_SPEED = 3; // rad/s
            public static final double SWERVE_ROTATION_MAX_ACCELERATION = Math.PI / 4; // rads/s^2

            public static final double SWERVE_DEADBAND = 0.05;

            public static final class Characteristics {
                public static final double MPS_TO_RPM = 1315 / 4;
                public static final double COUNTS_PER_100MS = 4201;
            }
        }
    }

    public static final class Motor {
        // Motor indexes here
        public static final int SHOOTER = 1;
        public static final int FEEDER = 6;
        public static final int INTAKE = 7;

        // Swerve
        public static final int SWERVE_FRONT_LEFT_POWER = 4;
        public static final int SWERVE_FRONT_LEFT_STEER = 15;

        public static final int SWERVE_FRONT_RIGHT_POWER = 3;
        public static final int SWERVE_FRONT_RIGHT_STEER = 14;

        public static final int SWERVE_BACK_LEFT_POWER = 2;
        public static final int SWERVE_BACK_LEFT_STEER = 13;

        public static final int SWERVE_BACK_RIGHT_POWER = 1;
        public static final int SWERVE_BACK_RIGHT_STEER = 12;
    }

    public static final class Solenoid {
        public static final int INTAKE_EXTEND = 1;
        public static final int INTAKE_RETRACT = 2;
    }
}
