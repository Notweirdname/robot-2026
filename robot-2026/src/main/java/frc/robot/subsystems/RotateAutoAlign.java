// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class RotateAutoAlign extends SubsystemBase {

  PIDController rotationPID;
  NetworkTable table;

  double cameraHeight;
  double tagHeight;
  double cameraAngle;

  /** Creates a new RotateAutoAlign. */
  public RotateAutoAlign() {
    rotationPID = new PIDController(0.006 ,  0.0, 0.0);
    table = NetworkTableInstance.getDefault().getTable("limelight-center");
    rotationPID.setTolerance(0.05);
    rotationPID.setSetpoint(0.0);
    cameraHeight = 16;
    tagHeight = 44.3;
    cameraAngle = 30;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getDistance(){
    double ty = LimelightHelpers.getTY("limelight-center");
    double zDistance = (tagHeight-cameraHeight)/ Math.tan(Math.toRadians(cameraAngle + ty));

    return zDistance;
  }
  
  public double getRotationSpeed(){
    double tx = LimelightHelpers.getTX("limelight-center");
    double[] positions;
    positions = LimelightHelpers.getBotPose_TargetSpace("limelight-center");
    double output = rotationPID.calculate(tx);

    SmartDashboard.putNumber("TX", tx);
    SmartDashboard.putNumber("output", output);

    return output;
  }
}
