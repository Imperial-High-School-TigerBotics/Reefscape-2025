package frc.robot.commands;

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
            // Ball Intake Controls
             if (xbox.getYButtonPressed()) {
                 arm.BallIntakeIn();
             } 
             if (xbox.getYButtonReleased()) {
                 arm.BallIntakeStop();
             }

             if (xbox.getBButtonPressed()) {
                 arm.BallIntakeOut();
             } 
             if (xbox.getBButtonReleased()) {
                 arm.BallIntakeStop();
             }

             // Coral Intake Controls
             if (xbox.getXButtonPressed()) {
                 arm.CoralIntakeIn();
             } 
             if (xbox.getXButtonReleased()) {
                 arm.CoralIntakeStop();
             }

             if (xbox.getAButtonPressed()) {
                 arm.CoralIntakeOut();
             } 
             if (xbox.getAButtonReleased()) {
                 arm.CoralIntakeStop();
             }
            }
        }

}

