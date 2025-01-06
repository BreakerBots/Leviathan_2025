        // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.factory;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;

/** Factory for producing CANcoders. */
public class BreakerCANCoderFactory {

    /**
     */
    public static CANcoder createCANCoder(int deviceID, double absoluteSensorDiscontinuityPoint, Angle absoluteOffset, SensorDirectionValue encoderDirection) {
        
        return createCANCoder(deviceID, "rio", absoluteSensorDiscontinuityPoint, absoluteOffset, encoderDirection);
    }

    /**
     */
    public static CANcoder createCANCoder(int deviceID, String busName,
        double absoluteSensorDiscontinuityPoint, Angle absoluteOffset, SensorDirectionValue encoderDirection) {
        CANcoder encoder = new CANcoder(deviceID, busName);
        configExistingCANCoder(encoder, absoluteSensorDiscontinuityPoint, absoluteOffset, encoderDirection);
        return encoder;
    }

    public static CANcoder createCANCoder(int deviceID, CANBus canBus, double absoluteSensorDiscontinuityPoint, Angle absoluteOffset, SensorDirectionValue encoderDirection) {
        return createCANCoder(deviceID, canBus.getName(), absoluteSensorDiscontinuityPoint, absoluteOffset, encoderDirection);
    }

    /**
     */
    public static void configExistingCANCoder(CANcoder encoder, double absoluteSensorDiscontinuityPoint, Angle absoluteOffset, SensorDirectionValue encoderDirection) {
        CANcoderConfiguration config =  new CANcoderConfiguration();
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = absoluteSensorDiscontinuityPoint;
        config.MagnetSensor.withMagnetOffset(absoluteOffset);
        config.MagnetSensor.SensorDirection = encoderDirection;
        encoder.getConfigurator().apply(config);    
    }

    public static class AbsoluteSensorRange {
        public static final double ZERO_TO_ONE = 1.0;
        public static final double SIGNED_PLUS_MINUS_HALF = 0.5;
    }
}
