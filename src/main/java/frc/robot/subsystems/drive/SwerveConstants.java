package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import frc.robot.subsystems.drive.generated.TunerConstants;

public class SwerveConstants {
    public static final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double maxAngularSpeed = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    public static class AutonConstants {
        public static final double kPPose = 0.0;
        public static final double kIPose = 0.0;
        public static final double kDPose = 0.0;

        public static final double kPAngle = 0.0;
        public static final double kIAngle = 0.0;
        public static final double kDAngle = 0.0;

        public static final double maxSpeed = 2.0;
        public static final double maxAngularSpeed = 0.5;
    }
}
