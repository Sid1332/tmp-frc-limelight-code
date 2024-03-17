// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public CANSparkMax fl;
  public CANSparkMax fr;
  public CANSparkMax bl;
  public CANSparkMax br;
  
  public RelativeEncoder flEncoder;
  public RelativeEncoder frEncoder;
  public RelativeEncoder blEncoder;
  public RelativeEncoder brEncoder;

  public AHRS gyro;

  public double gyroStartAngle;
  public double tolerance;
  public double turnPower;

  public MotorControllerGroup left;
  public MotorControllerGroup right;
  public DifferentialDrive diffDrive; 
  public DriveSubsystem() {
    fl = new CANSparkMax(Constants.FL, MotorType.kBrushless);
    fr = new CANSparkMax(Constants.FR, MotorType.kBrushless);
    bl = new CANSparkMax(Constants.BL, MotorType.kBrushless);
    br = new CANSparkMax(Constants.BR, MotorType.kBrushless);

    gyro = new AHRS();
    gyro.resetDisplacement();
    gyro.reset();

    gyroStartAngle = gyro.getYaw();
    tolerance = 3;
    turnPower = 0;

    flEncoder = fl.getEncoder();
    frEncoder = fr.getEncoder();
    blEncoder = bl.getEncoder();
    brEncoder = br.getEncoder();
    frEncoder.setPosition(0.0);
     // fl.setNeutralMode(IdleMode.kBrake);
    left = new MotorControllerGroup(fl,bl);
    right = new MotorControllerGroup(fr,br);
    diffDrive = new DifferentialDrive(left, right);

  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
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



  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  public Command driveMethodCommand() {
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public void arcadeDrive() {
    diffDrive.arcadeDrive(Constants.mainStick.getX(), Constants.mainStick.getY());
    // System.out.println(gyro.getPitch() + ", " + gyro.getRoll() + ", " + gyro.getYaw());
  }

  public void tankDrive(double left, double right) {
    diffDrive.tankDrive(left, -right);
  }

  public boolean driveToDist(double dist) {
    // dist to revolutions
    // NOTE: Function assumes encoders will be reset before first call.
    // TODO: Fix variable names and encoder calls
    double revolutions = dist / Constants.WHEEL_CIRCUMFERENCE;
    if (encoder.getRevolutions() <= revolutions) {
      driveForward();
      return false;
    }
    return true;
  }

  public void driveForward() {
    // diffDrive.arcadeDrive(-.102, -0.5);
    double error = gyro.getYaw();
    if (error > tolerance) {
      turnPower += 0.05;
    } else if (error < -tolerance) {
      turnPower -= 0.05;
    }

    // System.out.println(turnPower);
    // System.out.println(gyro.getYaw());
    // System.out.println();

    diffDrive.arcadeDrive(turnPower, -0.5);
  
    tankDrive(.5 + gyro.getRate(), .5 - gyro.getRate());
  }

  public boolean  turnToHeading(double heading) {
    double yaw = gyro.getYaw();
    System.out.println(yaw);
    if (yaw < heading - tolerance || yaw > heading + tolerance) {
      diffDrive.arcadeDrive(((heading - yaw) / Math.abs(heading - yaw) / 3), 0);
      return false;
    }
    return true;
  }

  public void turn(double right) {
    diffDrive.arcadeDrive(right, 0);
  }

  public void stopDrive(){
    diffDrive.arcadeDrive(0, 0);
  }

  public AHRS getGyro() {
    return gyro;
  }
}
