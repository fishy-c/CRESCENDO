package frc.robot.Constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class swerveConstants {
    public static final class moduleConstants {
        /* Inverts FL, FR, BL, BR */
        public static final InvertedValue[] driveMotorInverts = {};
        public static final InvertedValue[] steerMotorInverts = {};
        public static final SensorDirectionValue[] CANcoderInverts = {};
        public static final Rotation2d[] CANcoderOffsets = {}; //degrees or rotations

        /* CANcoder Offset FL, FR, BL, BR */
        public static final double[] CANcoderOffset = {};

        /* Gear Ratios */
        public static final double driveGearRatio = 6.55;
        public static final double steerGearRatio = 10.28;

        /* Max Speeds */
        public static final double maxSpeed = 0.0;
        public static final double maxAngularVelocity = 0.0;
        
        /* Current Limits */
        public static final double driveStatorCurrentLimit = 120;
        public static final double steerStatorCurrentLimit = 50;

        /* PID Values */
        public static final double drivekP = 0.0;
        public static final double drivekD = 0.0;
        public static final double drivekS = 0.0;
        public static final double drivekV = 0.0;

        public static final double anglekP = 0.0;
        public static final double anglekD = 0.0;
        public static final double anglekS = 0.0;
        public static final double anglekV = 0.0;

        /* Wheel Circumference */
        public static final double wheelCircumferenceMeters = 0.0;
    }

    public static final class kinematicsConstants{
        /* Drivetrain Constants */
        public static final double robotLength = Units.inchesToMeters(28);
        public static final double robotWidth = Units.inchesToMeters(28.5);

        /* Swerve Kinematics */
        public static final Translation2d FL = new Translation2d(robotLength / 2.0, robotWidth / 2.0);
        public static final Translation2d FR = new Translation2d(robotLength / 2.0, -robotWidth / 2.0);
        public static final Translation2d BL = new Translation2d(-robotLength / 2.0, robotWidth / 2.0);
        public static final Translation2d BR = new Translation2d(-robotLength / 2.0, -robotWidth / 2.0);

    }
}