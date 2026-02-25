// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterIntakeConstants;
import frc.robot.Configs;
import frc.robot.subsystems.RotateAutoAlign;

public class ShooterIntake extends SubsystemBase {
  /** Creates a new ShooterIntake. */

  private final SparkFlex m_shootMotor1;
  private final SparkFlex m_shootMotor2;
  private final SparkFlex m_intakeMotor;
  //private final SparkMax m_slapMotor;
  private final SparkMax m_kickerMotor;
  private final SparkMax m_hoodMotor;
  //private final SparkMax m_slapMotor2;
  private final SparkMax m_indexerMotor;

  //private final SparkClosedLoopController m_slapClosedLoopController;
  private final SparkClosedLoopController m_hoodClosedLoopController;

  double shootSpeed;
  double intakeSpeed;
  double kickerSpeed;
  double intakeAngle;
  double hoodAngle;
  double indexerSpeed;

  double t;
  double voy;
  double H;
  double h;
  double g;

  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_codriverController = new XboxController(OIConstants.kCoDriverControllerPort);

  private final RotateAutoAlign autoAlign;
  
  public ShooterIntake() {

      autoAlign = new RotateAutoAlign();
      m_shootMotor1 = new SparkFlex(ShooterIntakeConstants.kShooter1CanID, MotorType.kBrushless);
      m_shootMotor2 = new SparkFlex(ShooterIntakeConstants.kShooter2CanId, MotorType.kBrushless);
      m_intakeMotor = new SparkFlex(ShooterIntakeConstants.kIntakeCanId, MotorType.kBrushless);
      //m_slapMotor = new SparkMax(ShooterIntakeConstants.kSlapDownCanId, MotorType.kBrushless);
      m_kickerMotor = new SparkMax(ShooterIntakeConstants.kKickerCanId, MotorType.kBrushless);
      m_hoodMotor = new SparkMax(ShooterIntakeConstants.kHoodCanID, MotorType.kBrushless);
      //m_slapMotor2 = new SparkMax(ShooterIntakeConstants.kSlapDown2CanID, MotorType.kBrushless);
      m_indexerMotor = new SparkMax(ShooterIntakeConstants.kIndexerCanID, MotorType.kBrushless);

      //m_slapClosedLoopController = m_slapMotor.getClosedLoopController();
      m_hoodClosedLoopController = m_hoodMotor.getClosedLoopController();

      shootSpeed = 0.0;
      kickerSpeed = ShooterIntakeConstants.kKickerOffSpeed;
      intakeSpeed = 0.0;
      indexerSpeed = 0.0;
      hoodAngle = 40.0;
      H = -6; //Change in height
      h = 8; //Max arc height
      g = 32.2; //Gravity
      
      //m_hoodClosedLoopController.setReference(ShooterIntakeConstants.shootAngle, ControlType.kPosition);

      m_shootMotor2.configure(Configs.ShooterIntakeConfig.shooterConfig2, 
                           SparkBase.ResetMode.kResetSafeParameters, 
                           SparkBase.PersistMode.kPersistParameters);

      /*m_slapMotor.configure(Configs.ShooterIntakeConfig.slapDownConfig, 
                           SparkBase.ResetMode.kResetSafeParameters, 
                           SparkBase.PersistMode.kPersistParameters);*/

      m_hoodMotor.configure(Configs.ShooterIntakeConfig.hoodConfig, 
                           SparkBase.ResetMode.kResetSafeParameters, 
                           SparkBase.PersistMode.kPersistParameters);
      /*m_slapMotor2.configure(Configs.ShooterIntakeConfig.slapDown2Config,
                             SparkBase.ResetMode.kResetSafeParameters,
                             SparkBase.PersistMode.kNoPersistParameters);*/
      

      //m_slapClosedLoopController.setSetpoint(ShooterIntakeConstants.upAngle, ControlType.kPosition, ClosedLoopSlot.kSlot0);
      m_hoodClosedLoopController.setSetpoint(40, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void periodic() {
    double distance = (autoAlign.getDistance()/12);
    SmartDashboard.putNumber("Z Distance", distance);

    voy = Math.sqrt(2*g*(h-H));

    double a = -16.1;
    double b = voy;
    double c = H;
    t = (-b - Math.sqrt(b*b - 4*a*c))/(2*a);

    double vox = (distance/t);

    double launchAngleActual = Math.toDegrees(Math.atan2((voy*t),distance));
    double launchAngle = ((launchAngleActual - 60)*(292/32));
    SmartDashboard.putNumber("Launch Angle", launchAngle);
    SmartDashboard.putNumber("Actual Launch Angle", launchAngleActual);

    double actualVo = Math.sqrt((vox*vox) + (voy*voy));
    double motorSpeed = -(actualVo * 0.02875117233) * 0.7;
    SmartDashboard.putNumber("Shoot Speed", motorSpeed);
    SmartDashboard.putNumber("Actual Shoot Speed", actualVo);

    if(m_codriverController.getRightBumperButton()){
      if(launchAngle > ShooterIntakeConstants.kHoodAngleHigh){
        m_hoodClosedLoopController.setSetpoint(ShooterIntakeConstants.kHoodAngleHigh, ControlType.kPosition, ClosedLoopSlot.kSlot0);
      }else if(launchAngle < ShooterIntakeConstants.kHoodAngleLow){
        m_hoodClosedLoopController.setSetpoint(ShooterIntakeConstants.kHoodAngleLow, ControlType.kPosition, ClosedLoopSlot.kSlot0);
      }else{
        m_hoodClosedLoopController.setSetpoint(launchAngle, ControlType.kPosition, ClosedLoopSlot.kSlot0);
      }
      m_shootMotor1.set(motorSpeed);
    }else{
      m_shootMotor1.set(shootSpeed);
    }
    
      /*double tempHoodAngle = SmartDashboard.getNumber("Angle", 40.0);
      if(tempHoodAngle < ShooterIntakeConstants.kHoodAngleLow){
        m_hoodClosedLoopController.setSetpoint(ShooterIntakeConstants.kHoodAngleLow, ControlType.kPosition, ClosedLoopSlot.kSlot0);
      }else{
        m_hoodClosedLoopController.setSetpoint(tempHoodAngle, ControlType.kPosition, ClosedLoopSlot.kSlot0);
      }

      double tempShootSpeed = SmartDashboard.getNumber("Speed", 0.0);
      m_shootMotor1.set(tempShootSpeed);*/
    /*}else{
      changeHoodAngle(-2 * m_codriverController.getRightY());
      m_hoodClosedLoopController.setSetpoint(hoodAngle, ControlType.kPosition);
      m_shootMotor1.set(shootSpeed);
    }*/

    intakeSpeed = m_codriverController.getRightTriggerAxis();
    m_kickerMotor.set(kickerSpeed);
    m_intakeMotor.set(intakeSpeed);
    m_indexerMotor.set(indexerSpeed);
    //m_slapClosedLoopController.setSetpoint(intakeAngle, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public void setShooterSpeed(double speed){
      shootSpeed = speed;
  }

  public void setKickerSpeed(double speed, double indexSpeed){
    kickerSpeed = speed;
    indexerSpeed = -indexSpeed;
  }

  public void setIntakeAngle(double angle){
    intakeAngle = angle;
  }

  public void setHoodAngle(double angle){
    hoodAngle = angle;
  }

  public void autoShoot(){
    double distance = (autoAlign.getDistance()/12);

    voy = Math.sqrt(2*g*(h-H));

    double a = -16.1;
    double b = voy;
    double c = H;
    t = (-b - Math.sqrt(b*b - 4*a*c))/(2*a);

    double vox = (distance/t);

    double launchAngleActual = Math.toDegrees(Math.atan2((voy*t),distance));
    double launchAngle = ((launchAngleActual - 60)*(292/32));

    double actualVo = Math.sqrt((vox*vox) + (voy*voy));
    double motorSpeed = -(actualVo * 0.02875117233) * 0.7;

    if(launchAngle > ShooterIntakeConstants.kHoodAngleHigh){
      m_hoodClosedLoopController.setSetpoint(ShooterIntakeConstants.kHoodAngleHigh, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }else if(launchAngle < ShooterIntakeConstants.kHoodAngleLow){
      m_hoodClosedLoopController.setSetpoint(ShooterIntakeConstants.kHoodAngleLow, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }else{
      m_hoodClosedLoopController.setSetpoint(launchAngle, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
    m_shootMotor1.set(motorSpeed);
  }

  public void changeHoodAngle(double change) {
    if (hoodAngle + change < ShooterIntakeConstants.kHoodAngleHigh && hoodAngle + change > ShooterIntakeConstants.kHoodAngleLow) {
      hoodAngle += change;
    }
    setHoodAngle(hoodAngle);
  }
}
