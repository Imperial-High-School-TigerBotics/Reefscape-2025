package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;

public class LimelightCmd extends Command {
    private Limelight limelight;

    public LimelightCmd(Limelight limelight) {
        this.limelight = limelight;
        addRequirements(this.limelight);
    }

    @Override
    public void initialize() {
        //limelight.portForward();
    }

    @Override
    public void execute() {
        limelight.updateValues();
    }

    @Override
    public void end(boolean interrupted) {
        limelight.stopAprilTagDetector();
    }
}