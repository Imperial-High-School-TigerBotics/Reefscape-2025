package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {

    private String name;

    public Limelight(String name) {
        this.name = name;
    }

    public Limelight() {
        this("limelight");
    }

    /**
     * Example method to find the ID of the closest valid tag from the passed-in array
     */
    public double getClosestTag(double[] validTagIds) {
        double[] rawFiducials = NetworkTableInstance.getDefault()
                .getTable(name)
                .getEntry("rawfiducials")
                .getDoubleArray(new double[]{});
        
        double closestTag = -1;
        double closestDistance = Double.MAX_VALUE;

        // rawfiducials lumps everything in sets of 7
        // [tagId, txnc, tync, ta, distToCamera, distToRobot, ambiguity, repeat...]
        for(int i = 0; i < rawFiducials.length; i += 7){
            double tagId = rawFiducials[i];
            double distanceToCamera = rawFiducials[i+4]; // or i+5 for distToRobot

            // check if tagId is in validTagIds
            for(double valid : validTagIds) {
                if(valid == tagId && distanceToCamera < closestDistance) {
                    closestDistance = distanceToCamera;
                    closestTag = tagId;
                }
            }
        }

        return closestTag;
    }

    /**
     * Example: Return an array {tagId, fieldX, fieldY} for the requested tag
     * (You must manage the real field coords from a known map or from your JSON.)
     */
    public double[] getTarget(int id) {
        // This is a placeholder. You might have a known table of tag positions,
        // e.g. for the 2023 field. For example:
        switch (id) {
            case 1: return new double[] {1, /*fieldX*/ 15.513, /*fieldY*/ 1.071};
            case 2: return new double[] {2, 15.513, 2.748};
            // ...
        }
        return null;
    }

    @Override
    public void periodic() {
        // If you want. For debugging
    }
}
