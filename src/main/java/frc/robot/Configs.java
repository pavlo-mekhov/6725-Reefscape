package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ElevatorConstants;

public final class Configs {
    public static final class ElevatorConfigs{
        public static final SparkMaxConfig elevatorFollowerConfig = new SparkMaxConfig();
        public static final SparkMaxConfig elevatorConfig = new SparkMaxConfig();
        
        
        static{
                elevatorConfig
                    .closedLoopRampRate(0.1)
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(30)
                .inverted(true);
                elevatorConfig.encoder
                // .inverted(true)
                    .positionConversionFactor(360)
                    .velocityConversionFactor(1);
            
                /*
                 * Configure the closed loop controller. We want to make sure we set the
                 * feedback sensor as the primary encoder.
                 */
                elevatorConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // Set PID values for position control. We don't need to pass a closed loop
                    // slot, as it will default to slot 0.
                    .p(0.001)
                    .i(0)
                    .d(0)
                    .outputRange(-0.5, 0.5);
                    // .apply(new MAXMotionConfig().positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal));
                    // .maxMotion
                    //     .maxVelocity(1)
                    //     .maxAcceleration(1)
                    //     .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
                elevatorFollowerConfig
                    .apply(elevatorConfig)
                    .follow(ElevatorConstants.elevatorMotorRightCanId, true);
                }               
    }
}