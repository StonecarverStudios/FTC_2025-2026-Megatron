import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.io.FileWriter;
import java.io.BufferedWriter;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.State;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="Megatron")
public class Megatron extends LinearOpMode {
    
    DcMotorEx m1, m2, m3, m4, leftThrow, rightThrow, feeder;
    Servo trigger;
    
    ElapsedTime runTime = new ElapsedTime();
    
    @Override
    public void runOpMode(){
        
        // Wheels, ordered accordingly:
        // Top Left, Back Left, Front Right, Back Right
        m1 = hardwareMap.get(DcMotorEx.class, "bl"); //back left
        m2 = hardwareMap.get(DcMotorEx.class, "br"); //back right
        m3 = hardwareMap.get(DcMotorEx.class, "fr"); //front right
        m4 = hardwareMap.get(DcMotorEx.class, "fl"); //front left
        
        //Launcher motors
        leftThrow = hardwareMap.get(DcMotorEx.class, "leftThrow");
        rightThrow = hardwareMap.get(DcMotorEx.class, "rightThrow");
    
        //Trigger Servo
        trigger = hardwareMap.get(Servo.class, "trigger");
        
        //Feeder motor
        feeder = hardwareMap.get(DcMotorEx.class, "feeder"); //front left
        
        
        //Motor direction, flip to reverse direction of robot (may need to reorder motors)
        //m1.setDirection(DcMotor.Direction.REVERSE);
        m2.setDirection(DcMotor.Direction.REVERSE);
        m3.setDirection(DcMotor.Direction.REVERSE);
        m4.setDirection(DcMotor.Direction.REVERSE);
        
        rightThrow.setDirection(DcMotor.Direction.REVERSE);
        
        leftThrow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightThrow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //Reset the encoders on Init, and set them to the correct mode 
        // m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        //init triger
        trigger.setPosition(0.6);
        
        boolean farRange = true; // default to far range throw
        
        telemetry.addData("Press Start When Ready","");
        telemetry.addData("Far Range Mode Activated","");
        telemetry.update();
        
        boolean locked = false;
        //boolean stealthMode = false;
        
        boolean isLogged = false;
        
        double closePower = 0.39;
        double farPower = 0.43;
      
        waitForStart();
        runTime.reset();

        while (opModeIsActive()) {

                // BODY MOVEMENT
            double px = gamepad1.left_stick_x; //The power of the left stick on the x axis
                                                // Left to Right, -1.00 to 1.00
            double py = -gamepad1.left_stick_y; //The power of the left stick on the y axis
                                                // Down to Up, -1.00 to 1.00
            double pa = gamepad1.left_trigger - gamepad1.right_trigger;
                                                //The power of the body rotation

            //math equation stuff...
            
            //p1 correlates to m1
            //p2 correlates to m2
            // etc.
            double p1 = -px + py - pa; //bl
            double p2 = px + py + pa; //br
            double p3 = -px + py + pa; //fr
            double p4 = px + py - pa; //fl
            double max = Math.max(1.0, Math.abs(p1));
            max = Math.max(max, Math.abs(p2));
            max = Math.max(max, Math.abs(p3));
            max = Math.max(max, Math.abs(p4));
            p1 /= max;
            p2 /= max;
            p3 /= max;
            p4 /= max;
            
            
            //Close range and far range toggle
            //left = close
            //right equal far
            
            
            //Fine Power Adjuster
            if(gamepad2.dpad_up){
                if(farRange){
                    farPower += 2.0;
                }else{
                    closePower += 2.0;
                }
            }else if(gamepad2.dpad_down){
                if(farRange){
                    farPower -= 2.0;
                }else{
                    closePower -= 2.0;
                }
            }
            
            //Setting mode either close or far
            if (gamepad2.left_bumper){
                farRange = false; //close range mode
                
            }else if(gamepad2.right_bumper){
                farRange = true;  //far range mode
            }
            
            //Launching
            if (gamepad2.right_trigger > 0){
                double throwPower;
                if(farRange == true){
                    throwPower = -gamepad2.right_trigger * closePower; //far Range
                }else{
                    throwPower = -gamepad2.right_trigger * closePower; //Close Range
                }
            
                leftThrow.setPower(throwPower);
                rightThrow.setPower(throwPower);
            }else {
                double throwPower = 0;
            
                leftThrow.setPower(throwPower);
                rightThrow.setPower(throwPower);
            }
            
            
            //Trigger
            if(gamepad2.a){
                trigger.setPosition(0.3);
                runTime.reset();
            }else if(runTime.seconds() > 0.5 ){
                trigger.setPosition(0.6);
            }
            
    
            //Feeder Forward
            if (gamepad2.right_stick_y != 0){
                double feedPower = gamepad2.right_stick_y;
            
                feeder.setPower(feedPower);
            }else {
                double feedPower = 0;
            
                feeder.setPower(feedPower);
            }
            
            
                m1.setPower(p1);
                m2.setPower(p2); 
                m3.setPower(p3);
                m4.setPower(p4);
            
            //Display Positions of the motors
            telemetry.addData("Motor Encoders"," %d %d %d %d", m1.getCurrentPosition(), m2.getCurrentPosition(),
                    m3.getCurrentPosition(), m4.getCurrentPosition());
                    
            //Display current throwing mode
            if(farRange == true){
                telemetry.addData("Far Range Mode Activated", "");
            }else{
                 telemetry.addData("Close Range Mode Activated", "");
            }
            
            telemetry.addData("Close Motor Power", "%f", closePower);
            telemetry.addData("Far Motor Power", "%f", farPower);
            telemetry.update();
        }
        //Stops all motors
        // m1.setPower(0);
        // m2.setPower(0);
        // m3.setPower(0);
        // m4.setPower(0);
    }
}
