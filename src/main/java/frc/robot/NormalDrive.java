package frc.robot;




import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

public class NormalDrive extends TimedRobot {
    
    public XboxController Controller = new XboxController(0);
    public VictorSP FrontRightMotor = new VictorSP(0);
    public VictorSP RearRightMotor = new VictorSP(1);
    public VictorSP FrontLeftMotor = new VictorSP(2);
    public VictorSP RearLeftMotor = new VictorSP(3);
    public VictorSP LaunchMotor = new VictorSP(4);
    public boolean LaunchMotorEnabled;
    public Accelerometer mainAccelerometer = new BuiltInAccelerometer();
    
    
    

    public void robotInit()
    {
        
        LaunchMotorEnabled = false;
        CameraServer.startAutomaticCapture();
        /* UsbCamera MainCamera = new UsbCamera("USB Camera Main", 0);
        MjpegServer mJPEGServer1 = new MjpegServer("serve_USB Camera Main", 5941);
        mJPEGServer1.setSource(MainCamera);
        CvSink cvSink = new CvSink("opencv_USB Camera Main");
        cvSink.setSource(MainCamera);
        CvSource outputStream = new CvSource("test", PixelFormat.kMJPEG, 640, 480, 30);
        MjpegServer mJPEGServer2 = new MjpegServer("serve_test", 5941);
        mJPEGServer2.setSource(outputStream);
        */
    }

    public void teleopInit() 
    {
        //FrontRightMotor.setInverted(true);
        //RearRightMotor.setInverted(true);
        FrontLeftMotor.setInverted(true);
        //RearLeftMotor.setInverted(true);
    }
    public void teleopPeriodic()
    {
        
        FrontRightMotor.setVoltage((Controller.getLeftY()+Controller.getLeftX()) * 12);
        RearRightMotor.setVoltage((Controller.getLeftY()+Controller.getLeftX()) * 12);
        FrontLeftMotor.setVoltage((Controller.getLeftY()-Controller.getLeftX()) * 12);
        RearLeftMotor.setVoltage((Controller.getLeftY()-Controller.getLeftX()) * 12);
        FrontRightMotor.set((Controller.getLeftY()+Controller.getLeftX()));
        RearRightMotor.set((Controller.getLeftY()+Controller.getLeftX()));
        FrontLeftMotor.set((Controller.getLeftY()-Controller.getLeftX()));
        RearLeftMotor.set((Controller.getLeftY()-Controller.getLeftX()));
        
        if(Controller.getAButtonPressed())
        {
            LaunchMotorEnabled = !LaunchMotorEnabled;
        }
        if(LaunchMotorEnabled)
        {
            LaunchMotor.setVoltage(12 * Controller.getRightTriggerAxis());
            LaunchMotor.set(Controller.getRightTriggerAxis());
            Controller.setRumble(RumbleType.kRightRumble, 1);
            Controller.setRumble(RumbleType.kLeftRumble, Math.abs(LaunchMotor.get()));
        }
        else
        {
            LaunchMotor.setVoltage(0);
            LaunchMotor.set(0);
            Controller.setRumble(RumbleType.kRightRumble, 0);
            Controller.setRumble(RumbleType.kLeftRumble, 0);
        }
        if(Controller.getXButton())
        {
            FrontRightMotor.stopMotor();
            RearRightMotor.stopMotor();
            FrontLeftMotor.stopMotor();
            RearLeftMotor.stopMotor();
            
        }
        
    }
    public final void TEST()
    {
    
    }
    
}
