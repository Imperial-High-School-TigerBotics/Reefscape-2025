package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.subsystems.ArmRotation;

public class ArmRotationCmd extends Command {
    private final ArmRotation arm;
    private final XboxController xbox;

    private double rotatorPos;
    private boolean manualArmControl;

    public ArmRotationCmd(ArmRotation arm, XboxController xbox) {
        this.arm = arm;
        addRequirements(this.arm);

        this.xbox = xbox;
        manualArmControl = false;

        rotatorPos = arm.getArmRotatorPos();
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (DriverStation.isTeleop()) {
            if (!manualArmControl) {
                double axis = MathUtil.applyDeadband(xbox.getRawAxis(4), Constants.stickDeadband);
                if (axis != 0) {
                    rotatorPos = MathUtil.clamp(
                        arm.getArmRotatorPos() + (axis * Constants.ArmConstants.ArmRotatorSpeed),
                        Constants.ArmConstants.ArmMinPos,
                        Constants.ArmConstants.ArmMaxPos
                    );
                    arm.setArmRotatorPosition(rotatorPos);
                } else {
                    arm.setArmRotatorPosition(rotatorPos); // Maintain last position
                }
            }
        }

        // if(DriverStation.isTeleop()){
        //     if(!manualArmControl){
        //         double axis = MathUtil.applyDeadband(xbox.getRawAxis(4), Constants.stickDeadband);
        //         if(axis != 0){
        //             arm.rotateArmMotor(axis * Constants.ArmConstants.ArmRotatorSpeed);
        //         }
        //     }
        // }
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("ArmRotationCmd", "Ended");
        if (interrupted) {
            SmartDashboard.putString("ArmRotationCmd", "Interrupted");
        }
    }
}
