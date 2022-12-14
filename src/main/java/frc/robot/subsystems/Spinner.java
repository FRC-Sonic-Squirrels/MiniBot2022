// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spinner extends SubsystemBase {
  /** Creates a new Spinner. */
  private WPI_VictorSPX spinnerMotor = new WPI_VictorSPX(3);

  public Spinner() {
     spinnerMotor.configFactoryDefault();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPercentOutput(double speed) {
    if (speed > 0.2) {
      speed = 0.2;
    }
    if ( speed < -0.2) {
      speed = -0.2;
    }
    spinnerMotor.set(speed);
  }
}
