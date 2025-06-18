// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/** Add your docs here. */
public class Constants {
    public static final int gyro = 31;
    
    public class SwerveWheelConstants{
        public class FrontWheels{
            public final static int RightDrive = 42;
            public final static int LeftDrive = 30;
            public final static int RightAngle = 13;
            public final static int LeftAngle = 14;

            public final static int RightCancoder = 1;
            public static final double RightCancoderOffset = 0;
            public final static int LeftCancoder = 22;
            public static final double LeftCancoderOffset = 0;
        }
        public class BackWheels{
            public final static int RightDrive = 31;
            public final static int LeftDrive = 32;
            public final static int RightAngle = 11;
            public final static int LeftAngle = 12;

            public final static int RightCancoder = 21;
            public static final double RightCancoderOffset = 0;
            public final static int LeftCancoder = 23;
            public static final double LeftCancoderOffset = 0;
        }
        public class PIDConstants{
            public final static double kP = 0.85; // Proportion
            public final static double kI = 0; // Integral
            public final static double kD = 0.001; // Derivative
        }
        public class ChassisConstants{
            public static double length = 1;
            public static double width = 1;

            public static double maxDriveSpeed = 1;
            public static double maxRotationSpeed = Math.PI;

            public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(length / 2.0, width / 2.0),
            new Translation2d(length / 2.0, -width / 2.0),
            new Translation2d(-length / 2.0, width / 2.0),
            new Translation2d(-length / 2.0, -width / 2.0));
        }
        public enum kMotors {
            FL,
            FR,
            BR,
            BL
        }
    }
}
