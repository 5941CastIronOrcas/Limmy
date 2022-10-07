package frc.robot;
 
 
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
 
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;

import java.util.ResourceBundle.Control;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
 
public class CodeV6 extends TimedRobot {
   
    public XboxController Controller = new XboxController(0);
    public double LeftStickX;
    public double LeftStickY;
    public double RightStickX;
    //public VictorSP FrontRightMotor = new VictorSP(0);
    //public VictorSP RearRightMotor = new VictorSP(1);
    //public VictorSP FrontLeftMotor = new VictorSP(2);
    //public VictorSP RearLeftMotor = new VictorSP(3);
    public CANSparkMax FrontRightMotor = new CANSparkMax(2, MotorType.kBrushless);
    public CANSparkMax FrontLeftMotor = new CANSparkMax(3, MotorType.kBrushless);
    public CANSparkMax RearRightMotor = new CANSparkMax(5, MotorType.kBrushless);
    public CANSparkMax RearLeftMotor = new CANSparkMax(4, MotorType.kBrushless);
    public VictorSP ClimberMotor1 = new VictorSP(4);
    public VictorSP ClimberMotor2 = new VictorSP(00);
    public VictorSP LoaderMotor = new VictorSP(6);
    public VictorSP LaunchMotor = new VictorSP(7);
    public VictorSP LaunchMotor2 = new VictorSP(8);
    public VictorSP ArmMotor = new VictorSP(5);
    public VictorSP Light = new VictorSP(9);
    public Ultrasonic Sonar1 = new Ultrasonic(1, 0);
 
    public CANSparkMax LoaderMotorCAN = new CANSparkMax(6, MotorType.kBrushless);
    //public CANSparkMax LaunchMotorCAN = new CANSparkMax(1, MotorType.kBrushless);
    //public CANSparkMax ArmMotorCAN = new CANSparkMax(1, MotorType.kBrushless);
    public double TargetScreenX;
    public double TargetScreenXOld;
    public double TargetScreenY;
    public boolean LockingEnabled;
    public double LockBasedTurn;
    public double LockBasedMove;
    public double LockTurnP;
    public double LockTurnD;
    public double CameraScreenWidth;
    public double CameraScreenHeight;
    public double TimeSinceStartAtAutoStart;
    public Accelerometer AccelerometerMain = new BuiltInAccelerometer();
    private final SerialPort DistanceSensor = new SerialPort(115200, SerialPort.Port.kMXP);
    private final SerialPort DebugPort = new SerialPort(115200, SerialPort.Port.kOnboard);
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
    public double SensorDistance;
    public double SensorDistanceOld;
    public double IdealRange;
    public double TurnMargin;
    public double RangeP;
    public double RangeD;
    public double AutoStuffMultiplier;
    public float LaunchSequenceTimer;
    public boolean LidarIsBroken;
    public double TurnMultiplier;
    public double MaxSpeedMultiplier;
    public float AutoTimeOnTarget; //the amount of time the robot has been on target for in auto
   
 
    private final Object CAMERA_LOCK = new Object();
    private double cameraContourX;
    private double cameraContourY;
   
    /*
    Instructions:
    Left stick to move and turn
    Right Bumper: Raise Climber
    Left Bumper: Lower Climber
    Right Trigger (Hold; analog): Enable automatic aiming and ranging
    A Button (Hold): Launch Sequence
    B Button (Hold): Loading Motor
    X Button:
   
    During autonomous, it will Drive backwards for 2 seconds at half voltage, then enable targeting systems.  
   
    Notes:
    On some controllers (like ours), up on the joystick is negative
    On motors Positive *appears* to be clockwise (but double-check anyway)
    Cam IP: roboRIO-5941-FRC.local:1181/?action=stream
    */
   
    public void robotInit() //Does all this when the robot is started
    {
        //Prep Veriables:
        
        LoaderMotor.setInverted(true);
        //LaunchMotor.setInverted(true);
        //LaunchMotor2.setInverted(true);
        //ClimberMotor1.setInverted(true);
        //ClimberMotor2.setInverted(true);
        //ArmMotor.setInverted(true);
        IdealRange = 100;
        TurnMargin = 0.1;
        //RangeP = 0.0;
        RangeP = 0.01;
        RangeD = 0.00;
        LockTurnP = 0.0;
        //LockTurnP = 0.8 / 640.0;
        //LockTurnD = 0.0005;
        CameraScreenWidth = 640;
        CameraScreenHeight = 480;
        Sonar1.setAutomaticMode(true);
        TurnMultiplier = 0.25;
        MaxSpeedMultiplier = 0.5;
 
       
       
       
        //Create a new USB camera
        UsbCamera camera = CameraServer.startAutomaticCapture();
 
        //Activate the GripePipeline (Witchcraft)
        new VisionThread(camera, new GripPipeline(), (pipeline) -> {
            MatOfPoint[] objs = pipeline.filterContoursOutput().toArray(new MatOfPoint[0]);
            MatOfPoint largestContour = null;
            double largestContourSize = -1;
            for(int i = 0; i < objs.length; i++) {
                double currentContourSize = objs[i].size().area();
                if(currentContourSize > largestContourSize)
                {
                    currentContourSize = largestContourSize;
                    largestContour = objs[i];
                }
            }
            int camWidth = -1;
            int camHeight = -1;
            if(largestContour != null)
            {
                Rect rectangle = Imgproc.boundingRect(largestContour);
                camWidth = rectangle.x + (rectangle.width/2);
                camHeight = rectangle.y + (rectangle.height/2);
            }
            //Send some data to the SmartDashboard to look at
            SmartDashboard.putNumber("Target X Position", camWidth);
            SmartDashboard.putNumber("Target Y Position", camHeight);
            SmartDashboard.putNumber("RNG", Math.random());
            SmartDashboard.putNumber("Number Of Targets", objs.length);
            //Witchcraft
            synchronized (CAMERA_LOCK) {
                cameraContourX = camWidth;
                cameraContourY = camHeight;
            }
        }).start();
 
       
    }
    public void robotPeriodic() //Does this every 0.02 seconds whenever the robot is running
    {
        //Steal information from Withcraft
        synchronized (CAMERA_LOCK) {
            TargetScreenX = cameraContourX;
            TargetScreenY = cameraContourY;
        }
        //Anti-Object-Permanence
        if(TargetScreenX < 0)
        {
            TargetScreenX = CameraScreenWidth / 2.0f;
        }
       
        //Witchcraft transliterated from Python to make distance sensor work
        int Counter = DistanceSensor.getBytesReceived();
        if(Counter > 8)
        {
          byte[] bytes_serial = DistanceSensor.read(9);
          DistanceSensor.reset();
          if(bytes_serial[0] == 0x59 && bytes_serial[1] == 0x59)
          {
            SensorDistance = bytes_serial[2] + bytes_serial[3] * 256.0;
            LidarIsBroken = false;
          }
        }
        else //Insurance against Errors:
        {
            SensorDistance = IdealRange;
            LidarIsBroken = true;
            
           
        }
        if(SensorDistance <= 0)
        {
            SensorDistance = IdealRange;
            LidarIsBroken = true;
           
        }
       
        //If the Lidar isn't working, just drive forward at 0.25x voltage until it works again
        if(LidarIsBroken && TargetScreenX > 0 && TargetScreenX != CameraScreenWidth / 2.0f)
        {
            System.out.println("Lidar Nonfunctional; Camera-Ranging Activated");
            SensorDistance = (-77955.5/(TargetScreenY-492.946)) - 106.815;
        }
        else if(LidarIsBroken && (TargetScreenX < 0 || TargetScreenX == CameraScreenWidth / 2.0f))
        {
            System.out.println("Lidar Nonfunctional; Could not switch to Camera-Ranging");
            SensorDistance = IdealRange;
        }
        
        //Put the used Sensor Distance up on the dashboard
        SmartDashboard.putNumber("Sensor Distance", SensorDistance);
        SmartDashboard.putNumber("Sonar Distance", Sonar1.getRangeInches());
    }
   
   
    public void teleopPeriodic() //Does this every 0.02 seconds whenever the robot is teleoperated
    {
        //Set the stick deadzone variables
        RightStickX = Controller.getRightX();
        LeftStickX = Controller.getLeftX();
        LeftStickY = Controller.getLeftY();
        if(Math.abs(RightStickX) < 0.05)
        {
            RightStickX = 0;
        }
        if(Math.abs(LeftStickX) < 0.05)
        {
            LeftStickX = 0;
        }
        if(Math.abs(LeftStickY) < 0.05)
        {
            LeftStickY = 0;
        }
        
        //Use the right trigger to enable locking
        if(Math.abs(Controller.getRightTriggerAxis()) > 0.05)
        {
            LockingEnabled = true;
            AutoStuffMultiplier = Math.abs(Controller.getRightTriggerAxis());
        }
        else
        {
            LockingEnabled = false;
            AutoStuffMultiplier = 0;
        }
       
        //If the driver has locking enabled
        if(LockingEnabled)
        {
            Light.set(1);
            //Turning equation for locking (a PID controller):
            LockBasedTurn =  (LockTurnP * (TargetScreenX - (0.5 *  CameraScreenWidth))) + (LockTurnD * ((TargetScreenX - TargetScreenXOld) / 0.02));
           TargetScreenXOld = TargetScreenX;
            //Prevent the AutoAim from becoming too powerful
            if(LockBasedTurn > 1)
            {
                LockBasedMove = 1;
            }
            else if(LockBasedTurn < -1)
            {
                LockBasedMove = -1;
            }
           
           //If in range and on target rumble the controller to tell the driver to shoot
            if(Math.abs(TargetScreenX - (0.5 *  CameraScreenWidth)) <= TurnMargin && Math.abs(IdealRange - SensorDistance) <= 3)
            {
                //Controller.setRumble(RumbleType.kLeftRumble, 1);
                //Controller.setRumble(RumbleType.kRightRumble, 1);
            }
            else
            {
                //Controller.setRumble(RumbleType.kLeftRumble, 0);
                //Controller.setRumble(RumbleType.kRightRumble, 0);
            }
           
            //if pointing close enough to the target, drive forward or backwards to get in the correct range
            if(LockBasedTurn <= TurnMargin)
            {
                LockBasedMove = (-(RangeP * (IdealRange - SensorDistance)) + (RangeD * ((SensorDistance - SensorDistanceOld) / 0.02)));
                SensorDistanceOld = SensorDistance;
            }
            else
            {
                LockBasedMove = 0;
            }
            //Prevent the AutoRanger from becoming too powerful
            if(LockBasedMove > 0.25f)
            {
                LockBasedMove = 0.25f;
            }
            else if(LockBasedMove < -0.25f)
            {
                LockBasedMove = -0.25f;
            }
           
        }
        else
        {
            //If Locking is disabled, do nothing
            LockBasedTurn = 0;
            LockBasedMove = 0;
            Light.set(0);
        }
        if(Controller.getLeftTriggerAxis() > 0)
        {
            Light.set(1);
        }
        else if(!LockingEnabled)
        {
            Light.set(0);
        }
        //If something has gone horribly wrong, just disable the AutoRanger and/or AutoAimer
        if(Double.isNaN(LockBasedMove))
        {
            LockBasedMove = 0;
        }
        if(Double.isNaN(LockBasedTurn))
        {
            LockBasedTurn = 0;
        }
        //Final Drive motors voltage setting:
        FrontRightMotor.set((MaxSpeedMultiplier * LeftStickY) + (TurnMultiplier * LeftStickX) + (AutoStuffMultiplier * (LockBasedTurn + Math.sin(Math.PI * 0.5 * -LockBasedMove))));
        RearRightMotor.set((MaxSpeedMultiplier * LeftStickY) + (TurnMultiplier * LeftStickX) + (AutoStuffMultiplier * (LockBasedTurn + Math.sin(Math.PI * 0.5 * -LockBasedMove))));
        FrontLeftMotor.set(-(MaxSpeedMultiplier * LeftStickY) + (TurnMultiplier * LeftStickX) + (AutoStuffMultiplier * (LockBasedTurn + Math.sin(Math.PI * 0.5 * LockBasedMove))));
        RearLeftMotor.set(-(MaxSpeedMultiplier * LeftStickY) + (TurnMultiplier * LeftStickX) + (AutoStuffMultiplier * (LockBasedTurn + Math.sin(Math.PI * 0.5 * LockBasedMove))));
 
        //Manual Controls for non-drive motors:
       
        //Climber
        if(Controller.getRightBumper())
        {
            ClimberMotor1.set(1);
            ClimberMotor2.set(1);
        }
        else if(Controller.getLeftBumper())
        {
            ClimberMotor1.set(-1);
            ClimberMotor2.set(-1);
        }
        else
        {
            ClimberMotor1.set(0);
            ClimberMotor2.set(0);
        }
       
         //Launch Sequence
         if(Controller.getAButtonPressed())
         {
             LaunchSequenceInit();
         }
         if(Controller.getAButton())
         {
             LaunchSequencePeriodic();
         }
         if(Controller.getAButtonReleased())
         {
             LaunchSequenceAbort();
         }
 
       
        //Loader
        if(Controller.getBButton())
        {
            LoaderMotor.set(1);
            ArmMotor.set(0.5);
        }
        if(Controller.getBButtonReleased())
        {
            LoaderMotor.set(0);
            ArmMotor.set(0);
        }
        
 
       
        DebugPort.writeString("Distance: "+SensorDistance +"   "); //Send the distance in centimeters to the debug port
       //System.out.println("Light: "+Light.get());
       //Light.set(1);

       if(Controller.getXButton())
       {
           LoaderMotorCAN.set(Controller.getRightX());
       }
       else
       {
           LoaderMotorCAN.set(0);
       }
    }
 
    public void LaunchSequenceInit()
    {
        LaunchSequenceTimer = 0;
    }
    public void LaunchSequencePeriodic()
    {
        LaunchSequenceTimer += 0.02f;
        if(LaunchSequenceTimer < 0.18) //Reverse Loader Motor time
        {
            LaunchMotor.set(0);
            LaunchMotor2.set(0);
            LoaderMotor.set(-0.25f);
        }
        else if(LaunchSequenceTimer < 1.1f) // Launch Motor Spinup Time
        {
            LaunchMotor.set(1);
            LaunchMotor2.set(1);
            LoaderMotor.set(0);
        }
        else if(LaunchSequenceTimer > 1.1f) //Launch Motor Spinup Time
        {
            LaunchMotor.set(1);
            LaunchMotor2.set(1);
            LoaderMotor.set(1);
        }
 
    }
    public void LaunchSequenceAbort()
    {
        LaunchMotor.set(0);
        LaunchMotor2.set(0);
        LoaderMotor.set(0);
    }
 
 
    public void autonomousInit() //Does this when autonomous is started
    {
        //Make a variable to see the time since autonomous started
        TimeSinceStartAtAutoStart = Timer.getFPGATimestamp();
        LockBasedMove = 0;
        LockBasedTurn = 0;
    }
    public void autonomousPeriodic() ////Does this every 0.02 seconds whenever the robot is autonomous
    {
        //Drive backwards for some seconds, then let loose the targeting systems.  
        if(Timer.getFPGATimestamp() - TimeSinceStartAtAutoStart < 2)
        {
            FrontRightMotor.set(-0.5);
            RearRightMotor.set(-0.5);
            FrontLeftMotor.set(-0.5);
            RearLeftMotor.set(-0.5);
        }
        else
        {
            //Turning equation for targeting the hub (a PID controller):
            LockBasedTurn =  (LockTurnP * (TargetScreenX - (0.5 *  CameraScreenWidth))) + (LockTurnD * ((TargetScreenX - TargetScreenXOld) / 0.02));
           TargetScreenXOld = TargetScreenX;
            //Prevent the AutoAim from becoming too powerful
            if(LockBasedTurn > 1)
            {
                LockBasedTurn = 1;
            }
            else if(LockBasedTurn < -1)
            {
                LockBasedTurn = -1;
            }
           
           //If in range and on target, do something, otherwise do something else
            if(Math.abs(TargetScreenX - (0.5 *  CameraScreenWidth)) <= TurnMargin && Math.abs(IdealRange - SensorDistance) <= 3)
            {
               AutoTimeOnTarget += 0.02;
            }
            else
            {
               AutoTimeOnTarget = 0;
            }
            if(AutoTimeOnTarget > 2 && AutoTimeOnTarget < 2.02)
            {
                LaunchSequenceInit();
            }
            else if(AutoTimeOnTarget >= 2.02)
            {
                LaunchSequencePeriodic();
            }
           
            //if pointing close enough to the target, drive forward or backwards to get in the correct range
            if(TargetScreenX - (0.5 *  CameraScreenWidth) <= TurnMargin)
            {
                LockBasedMove = (-(RangeP * (IdealRange - SensorDistance)) + (RangeD * ((SensorDistance - SensorDistanceOld) / 0.02)));
                SensorDistanceOld = SensorDistance;
            }
            else
            {
                LockBasedMove = 0;
            }
            //Prevent the AutoRanger from becoming too powerful
            if(LockBasedMove > 1)
            {
                LockBasedMove = 1;
            }
            else if(LockBasedMove < -1)
            {
                LockBasedMove = -1;
            }
 
            //Final Drive Motor Voltage Setting:
            FrontRightMotor.set(-LockBasedTurn + LockBasedMove);
            RearRightMotor.set(-LockBasedTurn + LockBasedMove);
            FrontLeftMotor.set(LockBasedTurn + LockBasedMove);
            RearLeftMotor.set(LockBasedTurn + LockBasedMove);
        }
    }
}

 
