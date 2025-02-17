package frc.robot.subsystems.lights;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;

public class LightsConstants {
    public static final int CANdleID = 5;

    public static final int numLEDs = 59 + 8;
    public static final double brightness = 0.50;

    public static final double blinkInterval = 0.2;

    public static final double fadeDuration = 0.2;

    public static CANdleConfiguration caNdleConfiguration = new CANdleConfiguration();

    static {
        caNdleConfiguration.brightnessScalar = LightsConstants.brightness;
        caNdleConfiguration.stripType = CANdle.LEDStripType.GRB;
        caNdleConfiguration.v5Enabled = true;
        caNdleConfiguration.disableWhenLOS = false;// TODO: true -- why is this triggering?
    }
}
