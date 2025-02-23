package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ArmCmd extends Command{
    private Arm arm;

    private XboxController xbox;

    private boolean autoShooter;
    private double RotatorPos;

    public ArmCmd (Arm arm, XboxController xbox) {
        this.arm = arm;
        addRequirements(this.arm);

        this.xbox = xbox;
        autoShooter = false;

        RotatorPos = arm.getArmRotatorPos();
    }

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

            if (!autoShooter) {
                double axis = xbox.getRawAxis(1);
                arm.RotatorPos += MathUtil.applyDeadband(axis, .1) * 0.1;
                arm.nextArmRotatorPID();
            }

        } else {
            arm.nextArmRotatorPID();
        }
    }

    @Override
    public void end(boolean interrupted) {
        arm.RotatorPos = Constants.ArmConstants.ArmRestPos;
    }
}
