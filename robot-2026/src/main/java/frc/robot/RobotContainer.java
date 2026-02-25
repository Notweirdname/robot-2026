// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterIntakeConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterIntake;

import com.pathplanner.lib.commands.PathPlannerAuto;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer extends SubsystemBase{

  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final ShooterIntake m_ShooterIntake = new ShooterIntake();

  PathPlannerAuto Mario;
  SendableChooser<Command> m_Chooser;

  //NamedCommands.registerCommand("Shoot", m_ShooterIntake.autoShoot());

  private final CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);

  private final CommandXboxController m_codriverController =
      new CommandXboxController(OIConstants.kCoDriverControllerPort);

  public Command zeroHeading() {
    return this.runOnce(() -> m_drive.zeroHeading());
  }

  public Command shooterOff(){
    return this.runOnce(() -> m_ShooterIntake.setShooterSpeed(0));
  }

  public Command shortShoot(){
    return this.runOnce(() -> m_ShooterIntake.setShooterSpeed(ShooterIntakeConstants.kShootCloseSpeed));
  }

  public Command farShoot(){
    return this.runOnce(() -> m_ShooterIntake.setShooterSpeed(ShooterIntakeConstants.kShootFarSpeed));
  }

  public Command kickerOn(){
    return this.runOnce(() -> m_ShooterIntake.setKickerSpeed(ShooterIntakeConstants.kKickerOnSpeed, ShooterIntakeConstants.kIndexerOnSpeed));
  }

  public Command kickerOff(){
    return this.runOnce(() -> m_ShooterIntake.setKickerSpeed(ShooterIntakeConstants.kKickerOffSpeed, ShooterIntakeConstants.kIndexerOffSpeed));
  }

  public Command intakeUp(){
    return this.runOnce(() -> m_ShooterIntake.setIntakeAngle(ShooterIntakeConstants.upAngle));
  }

  public Command intakeDown(){
    return this.runOnce(() -> m_ShooterIntake.setIntakeAngle(ShooterIntakeConstants.intakeAngle));
  }

  public RobotContainer() {
    // Configure the trigger bindings
    m_Chooser = new SendableChooser<Command>();
    configureBindings();

    m_drive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_drive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_drive));

    String Mario_Str = "Mario";
    Mario = new PathPlannerAuto(Mario_Str);
    m_Chooser.addOption(Mario_Str, Mario);
    SmartDashboard.putData("Auto", m_Chooser);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_driverController.start().onTrue(zeroHeading());

    m_codriverController.rightBumper().onTrue(intakeDown());
    m_codriverController.leftBumper().onTrue(intakeUp());

    m_codriverController.y().onTrue(kickerOn());
    m_codriverController.a().onTrue(kickerOff());

    m_codriverController.povUp().onTrue(shooterOff());
    //m_codriverController.b().onTrue(shortShoot());
    //m_codriverController.x().onTrue(farShoot());
  }


  public Command getAutonomousCommand() {
    return m_Chooser.getSelected();
  }
}
