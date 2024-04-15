// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {
    public class SwerveWheelConstants{
        public class FrontWheels{
            public int RightDrive = 1;
            public int LeftDrive = 2;
            public int RightAngle = 3;
            public int LeftAngle = 4;

            public int RightCancoder = 5;
            public int LeftCancoder = 6;


        }
        public class BackWheels{
            public int RightDrive = 7;
            public int LeftDrive = 8;
            public int RightAngle = 9;
            public int LeftAngle = 10;

            public int RightCancoder = 11;
            public int LeftCancoder = 12;
        }
        public class PIDConstants{
            public final static double kP = 0.085; // Proportion
            public final static double kI = 0; // Integral
            public final static double kD = 0.001; // Derivative
        }
        public class ChassisConstants{
            public double length = 1;
            public double width = 1;

            public double maxDriveSpeed = 1;
            public double maxRotationSpeed = Math.PI;
        }
    }
}
