package frc.robot;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.FeedbackSensor;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ShooterIntakeConstants;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double nomianalVoltage = 12.0;
            double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .inverted(true)
                    .smartCurrentLimit(50);
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor)         // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters per second
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.02, 0, 0)
                    .outputRange(-1, 1)
                    .feedForward.kV(drivingVelocityFeedForward);
            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20)
                    .inverted(false);
            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(false)
                    .positionConversionFactor(turningFactor) // radians
                    .velocityConversionFactor(turningFactor / 60.0); // radians per second
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.7, 0, 0)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
        }
    }

    public static final class ShooterIntakeConfig{
        public static final SparkMaxConfig shooterConfig1 = new SparkMaxConfig();
        public static final SparkMaxConfig shooterConfig2 = new SparkMaxConfig();
        public static final SparkMaxConfig slapDownConfig = new SparkMaxConfig();
        public static final SparkMaxConfig slapDown2Config = new SparkMaxConfig();
        public static final SparkMaxConfig hoodConfig = new SparkMaxConfig();

        static {
                shooterConfig2.follow(ShooterIntakeConstants.kShooter1CanID, true);

                slapDownConfig
                    .absoluteEncoder
                    .inverted(false)
                    .positionConversionFactor(360)
                    .velocityConversionFactor(6);
                slapDownConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(60)
                    .inverted(false);
                slapDownConfig
                    .closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    .pid(0.001, 0, 0, ClosedLoopSlot.kSlot0)
                    .outputRange(-0.3, 0.3)
                    .positionWrappingEnabled(false);
                // slapDown2Config.absoluteEncoder
                //         .inverted(true)
                //         .positionConversionFactor(360)
                //         .velocityConversionFactor(6);
                // slapDown2Config
                //         .idleMode(IdleMode.kBrake)
                //         .smartCurrentLimit(60)
                //         .inverted(false);
                // slapDown2Config.closedLoop
                //         .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                //         .pid(.001, 0, 0)
                //         .outputRange(-0.3, 0.3)
                //         .positionWrappingEnabled(false);
                slapDown2Config.follow(Constants.ShooterIntakeConstants.kSlapDownCanId, true);
                hoodConfig
                    .absoluteEncoder
                    .inverted(false)
                    .positionConversionFactor(360)
                    .velocityConversionFactor(6);
                hoodConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(60)
                    .inverted(true);
                hoodConfig
                    .closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    .pid(0.01, 0, 0, ClosedLoopSlot.kSlot0)
                    .outputRange(-1, 1)
                    .positionWrappingEnabled(false);
        }
    }
}
