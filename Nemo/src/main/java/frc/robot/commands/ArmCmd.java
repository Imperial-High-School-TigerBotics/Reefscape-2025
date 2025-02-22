package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.Arm;

public class ArmCmd extends Command{
    private Arm arm;

    private XboxController xbox;

    @Override 
    public void initialize() {}
    
    @Override
    public void execute() {
        if (DriverStation.isTeleop()) {
            if (xbox.getYButtonPressed()) {
                arm.BallIntakeIn();
            } else if (xbox.getYButtonReleased()) {
                arm.BallIntakeStop();
            }

            if (xbox.getBButtonPressed()) {
                arm.BallIntakeOut();
            } else if (xbox.getBButtonReleased()) {
                arm.BallIntakeStop();
            }

            if (xbox.getXButtonPressed()) {
                arm.CoralIntakeIn();
            } else if (xbox.getXButtonReleased()) {
                arm.CoralIntakeStop();
            }

            if (xbox.getAButtonPressed()) {
                arm.CoralIntakeOut();
            } else if (xbox.getAButtonReleased()) {
                arm.CoralIntakeStop();
            }

        }
    }
}
