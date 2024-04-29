package frc.robot;

public class SwerveConstants {
    public class Left{
        public static int Front = 1; 
        public static Boolean FrontReversed = false;
        public static int Back = 2; 
        public static Boolean BackReversed = false;
    }
    public class Right{
        public static int Front = 3; 
        public static Boolean FrontReversed = true;
        public static int Back = 4; 
        public static Boolean BackReversed = true;
    }
    public static int DriverPort = 0; 
    public static double StickDeadband = 0.03; 
}
