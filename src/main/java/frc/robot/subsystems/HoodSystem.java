package frc.robot.subsystems;

import frc.robot.Constants.LinearServoConstants;
import com.revrobotics.ResetMode;
//import com.revrobotics.config.BaseConfig;
//import com.revrobotics.servohub.config.ServoHubParameter;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.config.ServoHubConfig;
import com.revrobotics.servohub.config.ServoChannelConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class HoodSystem extends SubsystemBase {
    ServoHubConfig config = new ServoHubConfig();
    private final ServoHub m_servoHub;
    private final ServoChannel linearServoLeft;
    private final ServoChannel linearServoRight;
    double m_angle = 0;
    

    public HoodSystem() {
        m_servoHub = new ServoHub(LinearServoConstants.kServoHubId);
        m_servoHub.setBankPulsePeriod(ServoHub.Bank.kBank0_2, LinearServoConstants.kPulsePeriod);
        
        linearServoLeft = m_servoHub.getServoChannel(ChannelId.kChannelId2);
        linearServoRight = m_servoHub.getServoChannel(ChannelId.kChannelId1);


        config
            .channel0.pulseRange(LinearServoConstants.kMinPulseWidth, LinearServoConstants.kMidPulseWidth, LinearServoConstants.kMaxPulseWidth)
            .disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kSupplyPower);

        // Persist parameters and reset any not explicitly set above to
        // their defaults.
        m_servoHub.configure(config, ResetMode.kResetSafeParameters);
        retractServo();
    }

    private static int angleToServoValue(double angle)
     {
        // Clamp the input angle
        angle = Math.max(0.0, Math.min(15.0, angle));

        // Normalize angle to 0–1 range
        double normalized = angle / 15.0;

        // Scale to servo range
        double servoValue = LinearServoConstants.kMinPulseWidth +
                        normalized * (LinearServoConstants.kMaxPulseWidth - LinearServoConstants.kMinPulseWidth);

        // Final clamp for safety
        double value =  Math.max(LinearServoConstants.kMinPulseWidth,Math.min(LinearServoConstants.kMaxPulseWidth, servoValue));
        value = Math.max(LinearServoConstants.kMinPulseWidth, Math.min(LinearServoConstants.kMaxPulseWidth, value));

        return (int)Math.round(value);
    }

    public void setAngle(double angle)
    {   
        if(m_angle == angle)
            return;
        m_angle = angle;
        setPosition(angleToServoValue(angle));
    }


    public void setPosition(int pulseWidth) {
        // Convert position (0.0 to 1.0) to pulse width (500 to 2500 microseconds)
        linearServoLeft.setPowered(true);
        linearServoLeft.setEnabled(true);
        linearServoLeft.setPulseWidth(pulseWidth);

        linearServoRight.setPowered(true);
        linearServoRight.setEnabled(true);
        linearServoRight.setPulseWidth(pulseWidth);
    }

    public void extendServo() {
        linearServoLeft.setPowered(true);
        linearServoLeft.setEnabled(true);
        linearServoLeft.setPulseWidth(LinearServoConstants.kMaxExtensionPulseWidth);

        linearServoRight.setPowered(true);
        linearServoRight.setEnabled(true);
        linearServoRight.setPulseWidth(LinearServoConstants.kMaxExtensionPulseWidth);
    }

    public void retractServo() {
        linearServoLeft.setPowered(true);
        linearServoLeft.setEnabled(true);
        linearServoLeft.setPulseWidth(LinearServoConstants.kMinExtensionPulseWidth);

        linearServoRight.setPowered(true);
        linearServoRight.setEnabled(true);
        linearServoRight.setPulseWidth(LinearServoConstants.kMinExtensionPulseWidth);
    }

}
    
