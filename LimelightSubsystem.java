package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {
    
    public double ty = 0;
    public double xSpeed = 0;
    public double forwardSpeed = 0;
    public LimelightSubsystem() {

    }

    public void resetSpeeds() {
        this.xSpeed = 0;
        this.forwardSpeed = 0;
    }

    @Override
    public void periodic() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(Constants.limelight1);
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");

        //read values periodically
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);

        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
    }

    public boolean alignWithSpeaker(NetworkTable table) {
        // table.getEntry("ledMode").setNumber(3);
        double tx = table.getEntry("tx").getDouble(0.0);
        double ty = table.getEntry("ty").getDouble(0.0)==0.0 ? this.ty : table.getEntry("ty").getDouble(0.0);
        this.ty = ty;
        System.out.println(this.ty);
        double val = tx / Constants.dRVP;
        double results = 0;
        if (LimelightHelpers.getLatestResults(Constants.limelight1).targetingResults.targets_Fiducials.length > 0)
          results = LimelightHelpers.getLatestResults(Constants.limelight1).targetingResults.targets_Fiducials[0].fiducialID;
        
        if (Math.abs(tx) > Constants.limelightTolerance) {
          if (Math.abs(val) > Constants.maxRVP) {
            val = Math.signum(val) * Constants.maxRVP;
          }
          if (Math.abs(val) < Constants.minRVP) {
            val = Math.signum(val) * Constants.minRVP;
          }
          
          //System.out.println(-9.5 > ty && ty > -10.5 ? 0 : Math.signum(10 - ty) * 0.3);
        }
        SmartDashboard.putNumber("id", results);
        if (results == Constants.redSpeakerID || results == Constants.blueSpeakerID) {
        //   m_driveSubsystem.diffDrive.arcadeDrive(s 
            this.xSpeed = val; // Math.max(0.25, Math.min(SmartDashboard.getNumber("maxRVP", 0.0), tx / SmartDashboard.getNumber("dRVP", 1))),
            this.forwardSpeed = Constants.maxSpeakerTY > ty && ty > Constants.minSpeakerTY ? 0 : Math.signum(10 - ty) * 0.3;
            return true;
        }
        return false;
    }

    public boolean alignWithAmp(NetworkTable table) {
        double tx = table.getEntry("tx").getDouble(0.0);
        double val = tx / Constants.dRVP;
        double results = 0;
        if (LimelightHelpers.getLatestResults(Constants.limelight2).targetingResults.targets_Fiducials.length > 0)
          results = LimelightHelpers.getLatestResults(Constants.limelight2).targetingResults.targets_Fiducials[0].fiducialID;
        
        if (Math.abs(tx) > Constants.limelightTolerance) {
          if (Math.abs(val) > Constants.maxRVP) {
            val = Math.signum(val) * Constants.maxRVP;
          }
          if (Math.abs(val) < Constants.minRVP) {
            val = Math.signum(val) * Constants.minRVP;
          }
          if (results == Constants.redAmpID || results == Constants.blueAmpID)
          {
            this.xSpeed = val;
            return true;
          }
        }
        return false;
    }

    public boolean setLedMode(String ledMode, String limelight) {
        if (ledMode == "off") {
            LimelightHelpers.setLEDMode_ForceOff(limelight);
        } else if (ledMode == "on") {
            LimelightHelpers.setLEDMode_ForceOn(limelight);
        } else if (ledMode == "blinking") {
            LimelightHelpers.setLEDMode_ForceBlink(limelight);
        } else {
            return false;
        }
        return true;
    }
}
