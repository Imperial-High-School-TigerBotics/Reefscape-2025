package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class RevCoder_test extends SubsystemBase {
    private final DutyCycleEncoder encoder;

    public RevCoder_test() {
        SmartDashboard.putNumber("RevCoder Revolutions", revolutions());
        SmartDashboard.putNumber("RevCoder Degrees", getDegrees());
        encoder = new DutyCycleEncoder(Constants.TestConstants.kTestEncoderChannel);
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("RevCoder Absolute Position", getAbsolutePosition());
    }

    public double getAbsolutePosition() {
        return encoder.get();
    }

    public double getDegrees(){
        return encoder.get() * 360;
    }

    public double revolutions(){
        return Math.floor(encoder.get() * 360);
    }
}