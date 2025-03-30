package frc.robot.subsystems.lights;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;

public class LightsConstants {
    public static final int CANdleID = 5;

    public static final int numLEDs = 59 + 8;
    public static final double brightness = 0.50;

    public static final double blinkInterval = 0.2;

    public static final double fadeDuration = 2.0;

    public static final CANdleConfiguration CANdleConfiguration = new CANdleConfiguration();
    static {
        CANdleConfiguration.brightnessScalar = LightsConstants.brightness;
        CANdleConfiguration.stripType = CANdle.LEDStripType.GRB;
        CANdleConfiguration.v5Enabled = true;
        CANdleConfiguration.vBatOutputMode = CANdle.VBatOutputMode.Off;
        CANdleConfiguration.disableWhenLOS = false;// TODO: why is this triggering?
    }
}
