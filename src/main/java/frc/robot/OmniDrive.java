package frc.robot;


import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

public class OmniDrive extends TimedRobot {
    
    public XboxController Controller = new XboxController(0);
    public VictorSP FrontMotor = new VictorSP(0);
    public VictorSP RearMotor = new VictorSP(1);
    public VictorSP RightMotor = new VictorSP(2);
    public VictorSP LeftMotor = new VictorSP(3);
    
    
    public void teleopInit() 
    {
        //FrontMotor.setInverted(true);
        //RearMotor.setInverted(true);
        //RightMotor.setInverted(true);
        //LeftMotor.setInverted(true);
    }
    public void teleopPeriodic()
    {
        
        FrontMotor.setVoltage((Controller.getRightX()+Controller.getLeftX()) * 12);
        RearMotor.setVoltage((Controller.getRightX()-Controller.getLeftX()) * 12);
        RightMotor.setVoltage((Controller.getRightX()-Controller.getLeftY()) * 12);
        LeftMotor.setVoltage((Controller.getRightX()+Controller.getLeftY()) * 12);
        
        if(Math.abs(Controller.getRightX()+Controller.getLeftX()) <= 0.05)
        {
            FrontMotor.stopMotor();
        }
        if(Math.abs(Controller.getRightX()-Controller.getLeftX()) <= 0.05)
        {
            RearMotor.stopMotor();
        }
        if(Math.abs(Controller.getRightX()-Controller.getLeftY()) <= 0.05)
        {
            RightMotor.stopMotor();
        }
        if(Math.abs(Controller.getRightX()+Controller.getLeftX()) <= 0.05)
        {
            LeftMotor.stopMotor();
        }
        
        if(Controller.getBButton() || Controller.getAButton() || Controller.getXButton() || Controller.getYButton())
        {
            FrontMotor.stopMotor();
            RearMotor.stopMotor();
            RightMotor.stopMotor();
            LeftMotor.stopMotor();
            
        }
        
    }
    public final void TEST()
    {
    
    }
    
}
