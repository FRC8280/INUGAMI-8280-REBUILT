package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.VisionManager.VisionStatus;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.units.Units;

public class LEDSubsystem extends SubsystemBase {
    public enum LightStatus {
        OFF, WHITE, RED, GREEN, YELLOW, BLUE
    }

    private LightStatus m_LightStatus = LightStatus.RED;

    private final CANBus kCANBus = new CANBus("rio");
    private final CANdle m_RearCandle = new CANdle(LEDConstants.kRearCandleID, kCANBus);
    private final CANdle m_FrontCandle = new CANdle(LEDConstants.kFrontCandleID, kCANBus);
    private final Timer timer = new Timer();

    // Colors and animation
    RGBWColor kGreen = new RGBWColor(0, 217, 0, 0);
    RGBWColor kRed = new RGBWColor(217, 0, 0, 0);
    RGBWColor kBlue = new RGBWColor(0, 0, 217, 0);
    RGBWColor kYellow = new RGBWColor(217, 217, 0, 0);
    RGBWColor kWhite = new RGBWColor(Color.kWhite).scaleBrightness(1);
    private final ColorFlowAnimation m_slot0Animation = new ColorFlowAnimation(0, 8)
            .withSlot(0)
            .withColor(new RGBWColor(45, 26, 246, 0))
            .withDirection(AnimationDirectionValue.Forward)
            .withFrameRate(Units.Hertz.of(25));

    // Vision status uses only the eight onboard LEDs (indices 0-7).
    private final ColorFlowAnimation m_visionNotSeededAnimation = createVisionAnimation(kRed);
    private final ColorFlowAnimation m_visionSeekingMt2Animation = createVisionAnimation(kYellow);
    private final ColorFlowAnimation m_visionLockedAnimation = createVisionAnimation(kGreen);
    private VisionStatus m_lastVisionStatus = null;

    private final SolidColor[] m_colors = new SolidColor[] {
    };

    // External strip only: skip onboard LEDs 0-7
    private static final int kStripStartIndex = 8;
    private static final int kStripLedCount = 24;
    private static final int kStripEndIndex = kStripStartIndex + kStripLedCount - 1;

    // private static final int kCountdownSeconds = 30;
    private int countdownDuration = 30; // default, can be set when starting countdown

    private boolean running = false;

    private int lastDisplayedRemaining = -1;
    private int lastLitCount = 0;
    private ColorZone lastColorZone = null;

    private final SolidColor solidRequest = new SolidColor(kStripStartIndex, kStripEndIndex).withUpdateFreqHz(0);

    private enum ColorZone {
        GREEN,
        YELLOW,
        RED
    }

    private ColorFlowAnimation createVisionAnimation(RGBWColor color) {
        return new ColorFlowAnimation(kStripStartIndex, kStripLedCount)
                .withSlot(0)
                .withColor(color)
                .withDirection(AnimationDirectionValue.Forward)
                .withFrameRate(Units.Hertz.of(2));
    }

    public void setVisionStatus(VisionStatus status) {
       
       /*  if (running) {
            return;
        }*/

        if (status == null || status == m_lastVisionStatus) {
            return;
        }

/*
        ColorFlowAnimation animation = switch (status) {
            case NOT_SEEDED -> m_visionNotSeededAnimation;
            case SEEKING_MT2 -> m_visionSeekingMt2Animation;
            case LOCKED -> m_visionLockedAnimation;
        };

        m_RearCandle.setControl(animation);
        m_FrontCandle.setControl(animation); */

        if(status ==  VisionStatus.NOT_SEEDED)
        {
            m_RearCandle.setControl(new SolidColor(0, 32).withColor(kRed));
            m_FrontCandle.setControl(new SolidColor(0, 32).withColor(kRed));
        }
        else if (status == VisionStatus.SEEKING_MT2)
        {
            m_RearCandle.setControl(new SolidColor(0, 32).withColor(kYellow));
            m_FrontCandle.setControl(new SolidColor(0, 32).withColor(kYellow)); 
        }
        else if (status == VisionStatus.LOCKED)
        {
            m_RearCandle.setControl(new SolidColor(0, 32).withColor(kGreen));
            m_FrontCandle.setControl(new SolidColor(0, 32).withColor(kGreen)); 
        }
        m_lastVisionStatus = status;
    }

    public LEDSubsystem() {
        m_RearCandle.setControl(new SolidColor(0, 32).withColor(kWhite));
        m_FrontCandle.setControl(new SolidColor(0, 32).withColor(kWhite));
        m_LightStatus = LightStatus.WHITE;

        clearTimerStrip();
    }

    public void startCountdown(int duration) {
        timer.reset();
        timer.start();
        running = true;

        clearTimerStrip();
        countdownDuration = duration;
        int litCount = getLitCountForRemaining(countdownDuration);
        ColorZone zone = getColorZoneForRemaining(countdownDuration);

        if (litCount > 0) {
            setRange(
                    kStripStartIndex,
                    kStripStartIndex + litCount - 1,
                    getColorForZone(zone));
        }

        lastDisplayedRemaining = countdownDuration;
        lastLitCount = litCount;
        lastColorZone = zone;
    }

    public void stopCountdown() {
        timer.stop();
        running = false;

        clearTimerStrip();

        lastDisplayedRemaining = -1;
        lastLitCount = 0;
        lastColorZone = null;
    }

    /*public void periodic() {
        if (!running) {
            return;
        }

        int remaining = getTimeRemainingSeconds();

        // Only update when whole second changes
        if (remaining == lastDisplayedRemaining) {
            return;
        }

        int newLitCount = getLitCountForRemaining(remaining);
        ColorZone newZone = getColorZoneForRemaining(remaining);

        if (remaining <= 0) {
            if (lastLitCount > 0) {
                clearRange(kStripStartIndex, kStripStartIndex + lastLitCount - 1);
            }

            timer.stop();
            running = false;
            lastDisplayedRemaining = 0;
            lastLitCount = 0;
            lastColorZone = null;
            return;
        }

        boolean zoneChanged = newZone != lastColorZone;

        if (zoneChanged && newLitCount > 0) {
            // Repaint only the currently lit section once
            setRange(
                    kStripStartIndex,
                    kStripStartIndex + newLitCount - 1,
                    getColorForZone(newZone));
        }

        if (newLitCount < lastLitCount) {
            // Turn off only the LEDs that expired
            clearRange(
                    kStripStartIndex + newLitCount,
                    kStripStartIndex + lastLitCount - 1);
        } else if (newLitCount > lastLitCount) {
            // Defensive handling if time jumps backward
            setRange(
                    kStripStartIndex + lastLitCount,
                    kStripStartIndex + newLitCount - 1,
                    getColorForZone(newZone));
        }

        lastDisplayedRemaining = remaining;
        lastLitCount = newLitCount;
        lastColorZone = newZone;
    }*/

    public boolean isFinished() {
        return timer.get() >= countdownDuration;
    }

    public int getTimeRemainingSeconds() {
        return Math.max(0, countdownDuration - (int) timer.get());
    }

    private int getLitCountForRemaining(int remaining) {
        int litCount = (int) Math.ceil((remaining / (double) countdownDuration) * kStripLedCount);
        return Math.max(0, Math.min(kStripLedCount, litCount));
    }

    private ColorZone getColorZoneForRemaining(int remaining) {

        if (countdownDuration <= 0) {
            return ColorZone.RED;
        }

        double pct = remaining / (double) countdownDuration;

        if (pct > 0.66) {
            return ColorZone.GREEN;
        } else if (pct > 0.33) {
            return ColorZone.YELLOW;
        } else {
            return ColorZone.RED;
        }
    }

    private RGBWColor getColorForZone(ColorZone zone) {
        return switch (zone) {
            case GREEN -> new RGBWColor(0, 255, 0, 0);
            case YELLOW -> new RGBWColor(255, 255, 0, 0);
            case RED -> new RGBWColor(255, 0, 0, 0);
        };
    }

    private void setRange(int startInclusive, int endInclusive, RGBWColor color) {
        if (startInclusive > endInclusive) {
            return;
        }

        StatusCode status = m_FrontCandle.setControl(
                solidRequest
                        .withLEDStartIndex(startInclusive)
                        .withLEDEndIndex(endInclusive)
                        .withColor(color));

        if (!status.isOK()) {
            System.out.println("CANdle setRange failed: " + status);
        }
    }

    private void clearRange(int startInclusive, int endInclusive) {
        setRange(startInclusive, endInclusive, new RGBWColor(0, 0, 0, 0));
    }

    private void clearTimerStrip() {
        clearRange(kStripStartIndex, kStripEndIndex);
    }

    public void setRed(boolean secondLight) {
        if (m_LightStatus == LightStatus.RED)
            return;

        m_RearCandle.setControl(new SolidColor(0, 32).withColor(kRed));
        m_LightStatus = LightStatus.RED;

        if (secondLight)
            m_FrontCandle.setControl(new SolidColor(0, 32).withColor(kRed));
    }

    public void setGreen(boolean secondLight) {
        if (m_LightStatus == LightStatus.GREEN)
            return;

        m_RearCandle.setControl(new SolidColor(0, 32).withColor(kGreen));
        m_LightStatus = LightStatus.GREEN;

        if (secondLight)
            m_FrontCandle.setControl(new SolidColor(0, 32).withColor(kGreen));
    }

    public void setBlue(boolean secondLight) {
        if (m_LightStatus == LightStatus.BLUE)
            return;

        m_RearCandle.setControl(new SolidColor(0, 32).withColor(kBlue));
        m_LightStatus = LightStatus.YELLOW;

        if (secondLight)
            m_FrontCandle.setControl(new SolidColor(0, 32).withColor(kBlue));
    }

    public void setYellow(boolean secondLight) {
        if (m_LightStatus == LightStatus.YELLOW)
            return;

        m_RearCandle.setControl(new SolidColor(0, 32).withColor(kYellow));
        m_LightStatus = LightStatus.YELLOW;

        if (secondLight)
            m_FrontCandle.setControl(new SolidColor(0, 32).withColor(kYellow));
    }

    public void showTeamColors() {
        m_RearCandle.setControl(new SolidColor(0, 32).withColor(kBlue));
        m_FrontCandle.setControl(new SolidColor(0, 32).withColor(kYellow));
    }
    public Command updateLEDs() {
        return run(() -> {
            for (var solidColor : m_colors) {
                m_RearCandle.setControl(solidColor);
                // m_RearCandle.setControl(solidColor);
            }
            m_RearCandle.setControl(m_slot0Animation);
            // m_RearCandle.setControl(m_slot0Animation);
        });
    }

}