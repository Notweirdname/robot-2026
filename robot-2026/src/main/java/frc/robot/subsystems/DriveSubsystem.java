// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SerialPort.Port;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.*;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Second;

import java.io.IOException;
import java.text.ParseException;

import com.ctre.phoenix6.StatusSignal;
//import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;

import edu.wpi.first.math.controller.PIDController;
import com.pathplanner.lib.config.PIDConstants;



public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules

  private final RotateAutoAlign autoAlign;
  NetworkTable table;

  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);
  
  public static final SwerveDriveKinematics kDriveKinematics =
    new SwerveDriveKinematics(
      new Translation2d(DriveConstants.kWheelBase / 2.0, DriveConstants.kTrackWidth / 2.0), // Front Left
      new Translation2d(DriveConstants.kWheelBase / 2.0, -DriveConstants.kTrackWidth / 2.0), // Front Right
      new Translation2d(-DriveConstants.kWheelBase / 2.0, DriveConstants.kTrackWidth / 2.0), // Back Left
      new Translation2d(-DriveConstants.kWheelBase / 2.0, -DriveConstants.kTrackWidth / 2.0) // Back Right
    );
      
  private final Pigeon2 pigeon = new Pigeon2(5, "rio");

  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(pigeon.getRotation2d().getDegrees()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
  
    autoAlign = new RotateAutoAlign();
    table = NetworkTableInstance.getDefault().getTable("limelight-center");

	  PPHolonomicDriveController driveController = new PPHolonomicDriveController(new PIDConstants(7.0, 0.0, 0.0), new PIDConstants(7.0, 0.0, 0.0));
    RobotConfig config;

    try {
      config = RobotConfig.fromGUISettings();
    } catch (IOException e) {
      e.printStackTrace();
      throw new RuntimeException("BAD SETTINGS FILE");
    } catch (org.json.simple.parser.ParseException e) {
      e.printStackTrace();
      throw new RuntimeException("PARSING ERROR");
    }

    AutoBuilder.configure(
      this::getPose, 
      this::resetOdometry, 
      this::getSpeeds, 
      (speeds, feedforwards) -> driveRobotRelative(speeds), 
      driveController, 
      config,
      () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
      this
    );

	  HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    resetEncoders();
    
  }

  @Override
  public void periodic() {

  SmartDashboard.putNumber("Yaw", pigeon.getYaw().getValueAsDouble());
	SmartDashboard.putNumber("Pitch", pigeon.getPitch().getValueAsDouble());
	SmartDashboard.putNumber("Roll", pigeon.getRoll().getValueAsDouble());

    
    m_odometry.update(
        Rotation2d.fromDegrees(pigeon.getRotation2d().getDegrees()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
  }


  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(pigeon.getRotation2d().getDegrees()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    

	XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
	double rightTriggerValue = (m_driverController.getRightTriggerAxis() * -0.8) + 1.0;

  double newRot = rot;
  NetworkTableEntry numberEntry = table.getEntry("tid");
  
  //Auto Align With April Tag
  if(m_driverController.getRightBumperButton()){
    if((double)numberEntry.getDouble(0.0) == 10 ||(double)numberEntry.getDouble(0.0) == 9){
      newRot = autoAlign.getRotationSpeed();
    }else{
      newRot = rot;
    }
  }else if(!m_driverController.getRightBumperButton()){
    newRot = rot;
  }
  
	if(xSpeed < -0.01 && xSpeed > -0.1) {
		xSpeed = 0.0;
	}
	if(ySpeed < 0.01 && ySpeed > 0.01) {
		ySpeed = 0.0;
	}

	xSpeed = xSpeed * rightTriggerValue;
	ySpeed = ySpeed * rightTriggerValue;
	newRot = newRot * rightTriggerValue;

  SmartDashboard.putNumber("rot", (double)newRot);

	double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond * -1.0;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond * -1.0;
    double rotDelivered = newRot * DriveConstants.kMaxAngularSpeed * -1.0;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromRadians(pigeon.getRotation2d().getRadians()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }


  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  public void zeroHeading() {
    pigeon.reset();
    resetEncoders();
  }

  public double getHeading() {
    return Rotation2d.fromDegrees(pigeon.getRotation2d().getDegrees()).getDegrees();
  } 

  public double getTurnRate() {
    return pigeon.getAngularVelocityYWorld().getValueAsDouble();
  }

  public ChassisSpeeds getSpeeds() {
    return kDriveKinematics.toChassisSpeeds(new SwerveModuleState[] {m_frontLeft.getState(), m_frontRight.getState(), m_rearLeft.getState(), m_rearRight.getState()});
  }

  void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds){
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    var targetStates = kDriveKinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }
}
