package frc.robot.commands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightCommand extends Command {
    public DriveSubsystem m_driveSubsystem;
    private LimelightSubsystem m_limelightSubsystem;

    public LimelightCommand(LimelightSubsystem subsystem, DriveSubsystem driveSubsystem) {
        m_driveSubsystem = driveSubsystem;
        m_limelightSubsystem = subsystem;
        
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveSubsystem, subsystem);
    }

    @Override
    public void initialize() {
        CameraServer.startAutomaticCapture(new HttpCamera("limelight", "http://10.52.43.11:5800/", HttpCameraKind.kMJPGStreamer));
        CameraServer.startAutomaticCapture();
    }

    @Override
    public void execute() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(Constants.limelight1);
        // TODO: set button to available button.
      if (Constants.mainStick.getRawButton(2)) {
        m_limelightSubsystem.alignWithSpeaker(table);
        m_driveSubsystem.diffDrive.arcadeDrive(m_limelightSubsystem.xSpeed, m_limelightSubsystem.forwardSpeed);
      } //else {
        // table.getEntry("ledMode").setNumber(0);
    //   }
        // TODO: set button to available button.
        if (Constants.mainStick.getRawButton(3)) {
            m_limelightSubsystem.alignWithAmp(table);
            m_driveSubsystem.diffDrive.arcadeDrive(m_limelightSubsystem.xSpeed, 0);
        }
      // System.out.println(LimelightHelpers.getBotPose(""));
      
    //   if (Constants.mainStick.getRawButton(1)) {
    //     // System.out.println("Yay!");
    //     table.getEntry("ledMode").setNumber(3);
    //   } else {
    //     table.getEntry("ledMode").setNumber(0);
    //   }
    }
}
