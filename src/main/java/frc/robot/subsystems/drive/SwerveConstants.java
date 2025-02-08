package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.drive.generated.TunerConstants;

public class SwerveConstants {
    public static final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double maxAngularSpeed = RotationsPerSecond.of(0.9).in(RadiansPerSecond);

    public static class AutoConstants {
        public static double kMaxSpeedMetersPerSecond = 7; // 3
        public static double kMaxAccelerationMetersPerSecondSquared = 2.5; // 3
        public static double kMaxAngularSpeedRadiansPerSecond = 4.2 * Math.PI;
        public static double kMaxAngularSpeedRadiansPerSecondSquared = 4 * Math.PI;


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
