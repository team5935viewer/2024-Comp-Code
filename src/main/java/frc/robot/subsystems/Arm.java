// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates motors and names them. */
  private final CANSparkMax armMotor;
  private final SparkPIDController armPID;
  private final RelativeEncoder armEncoder;

  /** Creates a new Arm. */
  public Arm() {

    armMotor =  new CANSparkMax(Constants.armCANID, MotorType.kBrushless);
    armEncoder = armMotor.getEncoder();
    armPID = armMotor.getPIDController();

  }

  public void Action(double setpoint) {
    armPID.setReference(setpoint, ControlType.kPosition);
  }

  public boolean poschk(double setpoint) {
    if (setpoint == armEncoder.getPosition()) {
      return true;
    } else {return false;}
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Armcoder POS", armEncoder.getPosition());
    // This method will be called once per scheduler run
  }
}
