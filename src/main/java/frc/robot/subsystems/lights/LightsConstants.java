package frc.robot.subsystems.lights;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;

public class LightsConstants {
    public static final int CANdleID = 5;
    private static final int CANdleOnboardLEDs = 8;

    public static final int numLEDs = 63 + CANdleOnboardLEDs;
    public static final double brightness = 1.0; // TODO: change to desired brightness

    public static final double blinkInterval = 0.2;

    public static final double fadeDuration = 1.10;

    public static final CANdleConfiguration CANdleConfiguration = new CANdleConfiguration();
    static {
        CANdleConfiguration.brightnessScalar = brightness;
        CANdleConfiguration.stripType = CANdle.LEDStripType.GRB;
        CANdleConfiguration.v5Enabled = true;
        CANdleConfiguration.vBatOutputMode = CANdle.VBatOutputMode.Off;
        CANdleConfiguration.disableWhenLOS = false;// TODO: why is this triggering?
    }
}
