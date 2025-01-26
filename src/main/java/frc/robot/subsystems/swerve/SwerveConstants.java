package frc.robot.subsystems.swerve;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SwerveConstants {
    public static final double maxSpeed = 5.0;
    public static final double maxAngularSpeed = 1.0;

    public static class AutoConstants {
        public static double kMaxSpeedMetersPerSecond = 1; // 3
        public static double kMaxAccelerationMetersPerSecondSquared = 1; // 3
        public static double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;


        //especially these values
        public static double kPXController = 0.6;
        public static double kPYController = 0.6;
        public static double kPThetaController = 4;

        /* Constraint for the motion profiled robot angle controller */
        public static TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

        public static TrapezoidProfile.Constraints kXControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
        
        public static TrapezoidProfile.Constraints kYControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
    }

}
