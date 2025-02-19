import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class RevCoder_test extends SubsystemBase {
    private final DutyCycleEncoder encoder;

    public RevCoder_test() {
        encoder = new DutyCycleEncoder(Constants.TestConstants.kTestEncoderChannel);
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("RevCoder Distance", getDistance());
        SmartDashboard.putNumber("RevCoder Absolute Position", getAbsolutePosition());
        SmartDashboard.putNumber("RevCoder Frequency", getFrequency());
    }

    public double getDistance() {
        return encoder.getDistance();
    }

    public double getAbsolutePosition() {
        return encoder.getAbsolutePosition();
    }

    public void reset() {
        encoder.reset();
    }

    public double getFrequency() {
        return encoder.getFrequency();
    }
    
    public boolean isConnected() {
        return encoder.isConnected();
    }
}