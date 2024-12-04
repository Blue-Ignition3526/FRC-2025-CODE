// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.BlueShift.control.motor.LazyCANSparkMax;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  
  //motors
  public final LazyCANSparkMax m_rightElevatorMotor;
  public final LazyCANSparkMax m_leftElevatorMotor;

  //encoder
  private final RelativeEncoder m_encoder;

  //setpoint
  private double m_setpoint;

  PIDController pid = new PIDController(0, 0, 0);



  public Elevator() {
    //motors
    //right
    this.m_rightElevatorMotor = new LazyCANSparkMax(0, MotorType.kBrushless);
    this.m_rightElevatorMotor.setIdleMode(IdleMode.kBrake);

    //left
    this.m_leftElevatorMotor = new LazyCANSparkMax(1, MotorType.kBrushless);
    this.m_leftElevatorMotor.setIdleMode(IdleMode.kBrake);
    this.m_leftElevatorMotor.follow(m_rightElevatorMotor, true);


    //encoder
    this.m_encoder = m_rightElevatorMotor.getEncoder();
  }

  public double getPosition(){ 
    return m_encoder.getPosition();
  }

  public double setPoint(double setpoint) {
    this.m_setpoint = setpoint;
    return setpoint;
}

  //  quiero checar si asi se hace una funcion que haga que la speed sea correcta
  // osea yo tengo para posicion actual y tengo para tener el setpoint

  //se que ahorita no tengo lo de pasar de rotaciones a movimiento lineal, en eso tengo duda

  public void setSpeed (){
    m_rightElevatorMotor.set(pid.calculate(getPosition(), setPoint(m_setpoint)));
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
