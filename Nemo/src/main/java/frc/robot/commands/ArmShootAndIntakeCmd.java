package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ArmShootAndIntake;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ArmShootAndIntakeCmd extends Command{

    private final XboxController xbox;
    private final ArmShootAndIntake arm;

    public ArmShootAndIntakeCmd(ArmShootAndIntake arm, XboxController xbox) {
        this.arm = arm;
        this.xbox = xbox;
        addRequirements(arm);
    }
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        arm.updateDashboard();
        if (DriverStation.isTeleop()) {
            boolean ltPressed = xbox.getLeftTriggerAxis() > Constants.OperatorConstants.TRIGGER_THRESHOLD;
            boolean rtPressed = xbox.getRightTriggerAxis() > Constants.OperatorConstants.TRIGGER_THRESHOLD;
            boolean lbPressed = xbox.getLeftBumperButton();
            boolean rbPressed = xbox.getRightBumperButton();


            // Intake / Outtake Coral
            if(ltPressed){
                arm.CoralIntakeIn();
            }else if(rtPressed){
                arm.CoralIntakeOut();
            }else{
                arm.CoralIntakeStop();
            }

            // Intake / Outtake Ball
            if(lbPressed){
                arm.BallIntakeIn();
            }else if(rbPressed){
                arm.BallIntakeOut();
            }else{
                arm.BallIntakeStop();
            }

        }
    }
}