// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.ClimbConstants.kClimbCoder;
import static frc.robot.Constants.ClimbConstants.kClimbMotorID;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ButtonBoardRightButtons;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerXboxController;
import frc.robot.BreakerLib.util.factory.BreakerCANCoderFactory;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Constants.ClimbConstants.*;

public class SimpleClimb extends SubsystemBase {
  /** Creates a new SimpleClimb. */
  private TalonFX climb;
  private CANcoder encoder;
  private DutyCycleOut dutyCycleRequest;
  private BreakerXboxController controller;
  private ButtonBoardRightButtons boardRightButtons;
  public SimpleClimb() {
    climb = new TalonFX(kClimbMotorID, SuperstructureConstants.kSuperstructureCANBus);
    encoder = BreakerCANCoderFactory.createCANCoder(kClimbCoder, SuperstructureConstants.kSuperstructureCANBus, 0.5, Rotations.of(0), SensorDirectionValue.CounterClockwise_Positive);
    dutyCycleRequest = new DutyCycleOut(0);
    configPivot();
  }

  private void configPivot() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.
    Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.StatorCurrentLimit = 70;
    config.CurrentLimits.SupplyCurrentLimit = 50;
    config.CurrentLimits.SupplyCurrentLowerTime = 0.8;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    climb.getConfigurator().apply(config);
  }
 
  private void set(double output) {
    dutyCycleRequest.withOutput(output);
    climb.setControl(dutyCycleRequest);
  }

  public enum ClimbState {
    EXTEND,
    RETRACT
  }

  private enum ClimbMode {
    RETRACTING(-1),
    NEUTRAL(0),
    EXTENDING(1);
    
    private double dutyCycleOut;
    private ClimbMode(double dutyCycleOut) {
      this.dutyCycleOut = dutyCycleOut;
    }

    public double getDutyCycleOut() {
        return dutyCycleOut;
    }

  }

  @Override
  public void periodic() {
    if (boardRightButtons.getLowRightSwitch().getAsBoolean()) {
      if (controller.getDPad().getDown().getAsBoolean()) {
        set(1);
      } else {
        set(-controller.getLeftTrigger().getAsDouble());
      }
    } else {
      set(0.0);
    }
   
  }
}
