package frc.robot.subsystems.lights;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;

import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevatorwrist.ElevatorWristSubsystem;

/*
 * Portions of this code were taken from Team 9496 Lynk
 */

public class LightsSubsystem extends SubsystemBase {
    private static LightsSubsystem instance = null;
    private static final boolean doFadePercent = false;

    private final CANdle candle = LightsConstants.CANdleConfig.createDevice(CANdle::new);

    private boolean fading = false;
    private double fadeStartTime = 0.0;
    private double fadePercent = 0.0;
    private Color fadeStartColor = Colors.OFF;
    private Color fadeEndColor = Colors.OFF;

    private final Timer blinkTimer = new Timer();
    private boolean blinking = false;
    private boolean blinkOff = false;

    private boolean isRainbow = false;

    private int[] currentColor = new int[]{0, 0, 0};

    private enum LEDState { STATIC, FADING, BLINKING, FADING_BLINKING, RAINBOW }
    private LEDState currentState = LEDState.STATIC;

    public static final class Colors {
        public static final Color OFF = new Color(0, 0, 0);
        public static final Color WHITE = new Color(255, 255, 255);
        public static final Color RED = new Color(255, 0, 0);
        public static final Color YELLOW = new Color(255, 255, 0);
        public static final Color ORANGE = new Color(255, 165, 0);
        public static final Color GREEN = new Color(0, 255, 0);
        public static final Color CYAN = new Color(0, 255, 255);
        public static final Color BLUE = new Color(0, 0, 255);
        public static final Color PURPLE = new Color(132, 0, 255);
        public static final Color MAGENTA = new Color(255, 0, 255);
        public static final Color PINK = new Color(255, 192, 203);

        public static final Color TEAM_751 = new Color(48, 131, 255);
        public static final Color PICTION_BLUE = new Color(0, 171, 231);
        public static final Color PERSIAN_BLUE = new Color(37, 65, 178);
        public static final Color AQUAMARINE = new Color(166, 244, 220);
        public static final Color MAJORELLE_BLUE = new Color(114, 76, 249);
        public static final Color ULTRAVIOLET = new Color(86, 69, 146);

        public static final Color[] allColors = new Color[]{Colors.WHITE, Colors.RED, Colors.YELLOW, Colors.ORANGE, Colors.GREEN, Colors.CYAN, Colors.BLUE, Colors.PURPLE, Colors.MAGENTA, Colors.PINK, Colors.AQUAMARINE};
    }

    public static LightsSubsystem getInstance() {
        if (instance == null) instance = new LightsSubsystem();
        return instance;
    }

    private LightsSubsystem() {
        blinkTimer.start();
        requestColor(Colors.RED, true);
    }

    public void requestColor(Color color, boolean blink) {
        isRainbow = false;
        blinking = blink;
        blinkTimer.reset();
        fadeBetweenColors(new Color(currentColor[0], currentColor[1], currentColor[2]), color);

        currentState = blinking ? LEDState.FADING_BLINKING : LEDState.FADING;
    }

    public void requestColor(Color color) {
        requestColor(color, blinking);
    }

    private void fadeBetweenColors(Color start, Color end) {
        fadeStartColor = start;
        fadeEndColor = end;
        fadeStartTime = Timer.getFPGATimestamp();
        fading = true;

        candle.setControl(new SolidColor(0, LightsConstants.numLEDs).withColor(new RGBWColor(start)));
    }

    public void requestRainbow() {
        candle.setControl(new com.ctre.phoenix6.controls.RainbowAnimation(0, LightsConstants.numLEDs)
                .withBrightness(1)
                .withDirection(AnimationDirectionValue.Forward)
        );
        isRainbow = true;
        fading = false;
        currentState = LEDState.RAINBOW;
    }

    public void requestBlinking(boolean blink) {
        blinking = blink;
        if (!blinking) setBrightness(1);
        blinkTimer.reset();
        currentState = blinking ? (fading ? LEDState.FADING_BLINKING : LEDState.BLINKING) : (fading ? LEDState.FADING : LEDState.STATIC);
    }

    private void setBrightness(double brightness) {
        candle.getConfigurator().apply(
                LightsConstants.CANdleConfig.config.withLED(
                        LightsConstants.CANdleConfig.config.LED.withBrightnessScalar(brightness)
                )
        );
    }

    public void requestToggleBlinking() {
        requestBlinking(!blinking);
    }

    @Override
    public void periodic() {
        if (fading) updateFade();
        if (blinking) updateBlink();
    }

    private void updateFade() {
        double elapsed = Timer.getFPGATimestamp() - fadeStartTime;
        double fraction = doFadePercent ? MathUtil.clamp(fadePercent, 0, 1) : elapsed / LightsConstants.fadeDuration;
        fraction = Math.min(fraction, 1.0);

        currentColor[0] = (int) MathUtil.interpolate(fadeStartColor.red, fadeEndColor.red, fraction);
        currentColor[1] = (int) MathUtil.interpolate(fadeStartColor.green, fadeEndColor.green, fraction);
        currentColor[2] = (int) MathUtil.interpolate(fadeStartColor.green, fadeEndColor.green, fraction);

        candle.setControl(new SolidColor(0, LightsConstants.numLEDs).withColor(new RGBWColor(fadeStartColor)));

        if (fraction >= 1.0) fading = false;
    }

    private void updateBlink() {
        if (blinkTimer.hasElapsed(LightsConstants.blinkInterval)) {
            blinkOff = !blinkOff;
            blinkTimer.reset();

            if (blinkOff) setBrightness(0.125);
            else setBrightness(1);
        }
    }

    public void requestAllianceColor() {
        requestBlinking(false);
        requestColor(DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red ? Colors.RED : Colors.BLUE);
        if (ElevatorWristSubsystem.getInstance().isClimbing()) requestRainbow();
    }
}