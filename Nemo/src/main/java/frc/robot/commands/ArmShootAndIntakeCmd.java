package frc.robot.commands;

import frc.robot.subsystems.ArmShootAndIntake;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

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
            //Coral Intake
            if(xbox.getLeftBumperButtonPressed()){
                arm.CoralIntakeIn();
            }else if(xbox.getRightBumperButtonPressed()){
                arm.CoralIntakeOut();
            }else{arm.CoralIntakeStop();}

            if(arm.getBumperButtonPressed(xbox.getLeftTriggerAxis())){
                arm.BallIntakeIn();
            }else if(arm.getBumperButtonPressed(xbox.getRightTriggerAxis())){
                arm.BallIntakeOut();
            }else{arm.BallIntakeStop();}
        }
    }
}
