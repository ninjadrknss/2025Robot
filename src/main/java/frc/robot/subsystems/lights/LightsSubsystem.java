package frc.robot.subsystems.lights;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;

/*
 * Portions of this code were taken from Team 9496 Lynk
 */

public class LightsSubsystem extends SubsystemBase {
    private static LightsSubsystem instance = null;

    private final CANdle candle = new CANdle(LightsConstants.CANdleID, Robot.riobus.getName());

    private final Timer blinkTimer = new Timer();
    private boolean blinking = false;
    private boolean blinkOff = false;

    private boolean doFadePercent = true;
    private boolean fading = false;
    private double fadeStartTime = 0.0;
    private double fadeDurationSec = 0.0;
    private double fadePercent = 0.0;
    private Color fadeStartColor = Colors.OFF;
    private Color fadeEndColor = Colors.OFF;

    private Color currentColor = Colors.OFF;

    public static class Color {
        private final int R, G, B;

        public Color(int r, int g, int b) {
            R = r;
            G = g;
            B = b;
        }
    }

    @SuppressWarnings("unused")
    public static final class Colors {
        // -----------------------------------------------------
        // Basic Colors
        // -----------------------------------------------------
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

        // -----------------------------------------------------
        // Palette Colors
        // -----------------------------------------------------
        /* 751 */ public static final Color TEAM_751 = new Color(48, 131, 255);
        /* P1 */ public static final Color PICTION_BLUE = new Color(0, 171, 231);
        /* P2 */ public static final Color PERSIAN_BLUE = new Color(37, 65, 178);
        /* P3 */ public static final Color AQUAMARINE = new Color(166, 244, 220);
        /* P4 */ public static final Color MAJORELLE_BLUE = new Color(114, 76, 249);
        /* P5 */ public static final Color ULTRAVIOLET = new Color(86, 69, 146);
    }

    public static LightsSubsystem getInstance() {
        if (instance == null) instance = new LightsSubsystem();
        return instance;
    }

    private LightsSubsystem() {
       candle.configAllSettings(LightsConstants.caNdleConfiguration);

        requestColor(Colors.RED);

        new Trigger(DriverStation::isDSAttached).onTrue(new InstantCommand(this::requestRainbow).ignoringDisable(true)); // TODO: see if this works
        new Trigger(DriverStation::isDSAttached).onFalse(new InstantCommand(() -> requestColor(Colors.RED)).ignoringDisable(true));
        blinkTimer.start();
    }

    public void requestColor(Color color, boolean blink) {
        System.out.printf("Setting color (%d, %d, %d)%n", color.R, color.G, color.B);
        blinking = blink;
        blinkTimer.reset();
        fadeBetweenColors(new Color(currentColor.R, currentColor.G, currentColor.B), color);
        currentColor = color;
    }

    public void requestColor(Color color) {
        requestColor(color, false);
    }

    private void fadeBetweenColors(Color start, Color end) {
//        candle.clearAnimation(0);

        this.fadeStartColor = start;
        this.fadeEndColor = end;
        this.fadeDurationSec = LightsConstants.fadeDuration;
        this.fadeStartTime = Timer.getFPGATimestamp();
        this.fading = true;

       candle.setLEDs(start.R, start.G, start.B);
    }

    public void requestRainbow() {
        System.out.println("Rainbowify");
        candle.clearAnimation(0);
        candle.animate(new RainbowAnimation(0.50, 0.75, LightsConstants.numLEDs, false, 0));
    }

    public void requestBlinking(boolean blink) {
        blinking = blink;
        System.out.println("Blink: " + blink);
        if (blinking) blinkTimer.reset();
    }

    public void requestToggleBlinking() {
        requestBlinking(!blinking);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("LED/Current", candle.getCurrent());
        SmartDashboard.putNumber("LED/Voltage", candle.get5VRailVoltage());
        SmartDashboard.putBoolean("LED/Blinking", blinking);
        SmartDashboard.putBoolean("LED/BlinkOff", blinkOff);

        if (fading) updateFade();
        if (blinking) updateBlink();
    }

    public void setFadePercent(double percent) {
        fadePercent = percent;
    }

    private void updateFade() {
        double currentTime = Timer.getFPGATimestamp();
        double elapsed = currentTime - fadeStartTime;

        double fraction = doFadePercent ? MathUtil.clamp(fadePercent, 0, 1) : elapsed / fadeDurationSec;
        if (fraction >= 1.0) {
            fraction = 1.0;
            fading = false; // Fade is done
        }

        int r = (int) (MathUtil.interpolate(fadeStartColor.R, fadeEndColor.R, fraction));
        int g = (int) (MathUtil.interpolate(fadeStartColor.G, fadeEndColor.G, fraction));
        int b = (int) (MathUtil.interpolate(fadeStartColor.B, fadeEndColor.B, fraction));

        candle.setLEDs(r, g, b);
    }

    private void updateBlink() {
        if (blinkTimer.hasElapsed(LightsConstants.blinkInterval)) {
            candle.clearAnimation(0);
            if (blinkOff) candle.setLEDs(currentColor.R, currentColor.G, currentColor.B);
            else {
                candle.setLEDs(currentColor.R/8, currentColor.G/8, currentColor.B/8);
            }
            blinkOff = !blinkOff;
            blinkTimer.reset();
        }
    }
}