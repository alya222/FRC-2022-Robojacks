// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Shooter extends SubsystemBase {

  // add a piston for shooter
  private Solenoid shooterPiston = new Solenoid(compressorModule, shooterPistonPort);

  // add a motor for shooter
  private CANSparkMax shooterMotor = new CANSparkMax(shooterMotorPort, MotorType.kBrushless);


  /** Creates a new Shooter. */
  public Shooter() {
  }

  public void setShootPiston(boolean shoot) {
      shooterPiston.set(shoot);
  }

  public boolean shootPistonExtended() {
    return shooterPiston.get();
  }

  public void runShootMotor (double speed) {
    shooterMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
