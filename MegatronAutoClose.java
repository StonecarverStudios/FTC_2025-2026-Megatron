package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous

public class MegatronAutoClose extends LinearOpMode {
    
      DcMotorEx m1, m2, m3, m4, leftThrow, rightThrow, feeder;
      Servo trigger;
      
      ElapsedTime runTime = new ElapsedTime();
      
      
      
      private void drive(double py, double px, double pa) {
     
        if (Math.abs(pa) < 0.05) pa = 0;
        double p1 = px + py + pa; //fl
        double p2 = -px + py + pa; //bl
        double p3 = -px + py - pa; //fr
        double p4 = px + py - pa; //br
        double max = Math.max(1.0, Math.abs(p1));
        max = Math.max(max, Math.abs(p2));
        max = Math.max(max, Math.abs(p3));
        max = Math.max(max, Math.abs(p4));
        p1 /= max;
        p2 /= max;
        p3 /= max;
        p4 /= max;
        m1.setPower(p1);
        m2.setPower(p2);
        m3.setPower(p3);
        m4.setPower(p4);
    }
    
    private void stopDrive(){
        drive(0,0,0);
        sleep(400);
    }
    
    
    @Override
    public void runOpMode() {
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
        trigger.setPosition(0.7);
        
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        boolean goAhead = false;
        
        
        waitForStart();
        runTime.reset();
        //AUTO STARTS
        
        //Move off of the wall
        drive(-0.5,0,0);
        sleep(1470);
        stopDrive();
        
        runTime.reset();
        
        
        leftThrow.setPower(-0.40);
        rightThrow.setPower(-0.40);

        // Start timer
        runTime.reset();

        for (int i = 0; i < 3; i++){
            // Start timer
            // Start timer
            runTime.reset();
            while (opModeIsActive() && runTime.seconds() < 5.0) {
                idle();
            }
            //Now move the trigger
            trigger.setPosition(0.3);
            // Wait for servo to finish moving
            sleep(800);
            //Set it back to zero and wait for it to finish
            trigger.setPosition(0.6);
            sleep(1000);
         
            feeder.setPower(-1);
            sleep(1300);
            feeder.setPower(0);
        }
        
        leftThrow.setPower(0);
        rightThrow.setPower(0);
        feeder.setPower(0);
        trigger.setPosition(0.7);
        sleep(400);
        stopDrive();
        
        
    }
}
