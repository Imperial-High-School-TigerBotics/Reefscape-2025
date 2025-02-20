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

    public ArmCmd(Arm arm, XboxController xbox) {
        this.arm = arm;
        addRequirements(this.arm);

        this.xbox = xbox;
    }

    
}
