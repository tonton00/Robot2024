package ravenrobotics.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants 
{
    //Constants for controller IDs, etc.
    public static class DriverStationConstants
    {
        public static final int kDriverPort = 0;
    }
    //Constants for the drivetrain, like motor IDs and whether they should be inverted.
    public static class DrivetrainConstants
    {
        /////////////////////
        //////Motor IDs//////
        /////////////////////
        public static final int kFrontLeftMotor = 1;
        public static final int kFrontRightMotor = 2;
        public static final int kBackLeftMotor = 3;
        public static final int kBackRightMotor = 4;
        /////////////////////
        ////Invert Motors////
        ////////////////////
        public static final boolean invertLeftSide = true;
        public static final boolean invertRightSide = false;
        /////////////////////
        /////Max Voltage/////
        /////////////////////
        public static final int kDriveMaxVoltage = 10;
        /////////////////////
        ///Other Constants///
        /////////////////////
        //TODO: Find actual speed of the drivetrain, using number from online for now.
        public static final double kDriveMaxSpeedMPS = Units.feetToMeters(13.87);
    }
    //Kinematics-related constants
    public static class KinematicsConstants
    {
        //Offset from the center of the robot to a wheel.
        public static final double kOffset = Units.inchesToMeters(30) / 2;
        //Translation2d offsets for each wheel.
        public static final Translation2d kFrontLeftOffset = new Translation2d(kOffset, kOffset);
        public static final Translation2d kFrontRightOffset = new Translation2d(kOffset, -kOffset);
        public static final Translation2d kBackLeftOffset = new Translation2d(-kOffset, kOffset);
        public static final Translation2d kBackRightOffset = new Translation2d(-kOffset, -kOffset);
        //Actual kinematics object for performing calculations.
        public static final MecanumDriveKinematics kDriveKinematics = new MecanumDriveKinematics(
            kFrontLeftOffset,
            kFrontRightOffset,
            kBackLeftOffset,
            kBackRightOffset);
    }
}
