/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode2;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.concurrent.TimeUnit;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Primary", group="Iterative Opmode")

public class Main extends OpMode
{
    int i = 0;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor armDrive = null;
    private DcMotor extentormotor = null;
    private int extentormotor_pos = 0;
    private Servo testservo = null;
    private int armTpos = 0;
    private int basearmpos = 0;
    

    private void FORWARD ()
    {
        leftDrive.setPower(1.0);
        rightDrive.setPower(1.0);
    }
    private void BACKWARD ()
    {
        leftDrive.setPower(-1.0);
        rightDrive.setPower(-1.0);
    }
    private void STOPMOVE ()
    {
        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);
    }
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        //leftDrive  = hardwareMap.DcMotor.get("leftmotor");
        //rightDrive = hardwareMap.DcMotor.get("rightmotor");
        
        leftDrive = hardwareMap.dcMotor.get("leftmotor");
        rightDrive = hardwareMap.dcMotor.get("rightmotor");
        armDrive = hardwareMap.dcMotor.get("armmotor");
        testservo = hardwareMap.servo.get("testservo");
        extentormotor = hardwareMap.dcMotor.get("extentormotor");
        

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        
        
        armDrive.setDirection(DcMotor.Direction.REVERSE);
        armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armDrive.setPower(0.0);
        
        
        extentormotor.setDirection(DcMotor.Direction.REVERSE);
        extentormotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extentormotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extentormotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extentormotor.setPower(0.0);
        
        
        
    
        

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armDrive.setTargetPosition(0);
        armDrive.setPower(0.0);
        armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        extentormotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extentormotor.setTargetPosition(0);
        extentormotor.setPower(0.0);
        extentormotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower = 0.0;
        double rightPower = 0.0;
        

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.
        
        
        
        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        if (!(gamepad1.dpad_down || gamepad1.dpad_up ))
        {
            leftPower = gamepad1.left_stick_y;
            rightPower =  gamepad1.right_stick_y;
        } 
        else if (gamepad1.dpad_up)
        {
            leftPower = 1.0; rightPower = 1.0;
        }
        else if (gamepad1.dpad_down)
        {
            leftPower = -1.0; rightPower = -1.0;
        }
        if (gamepad1.left_bumper && armTpos <= (basearmpos+300))
        {
            armTpos =  armTpos + 1;
        }
        if (gamepad1.right_bumper && armTpos >= basearmpos)
        {
            armTpos = armTpos - 1;
        }
       
        if (gamepad1.x)
        {
            basearmpos = armTpos;
        }
        if (gamepad1.a)
        {
            
        }
        
        if (gamepad1.right_trigger > 0)
        {
            extentormotor_pos+= 10;
        }
        if (gamepad1.left_trigger > 0)
        {
            extentormotor_pos-= 10;
        }
        
        //telemetry.addData("Orange", "left (%.2f), right (%.2f)", gamepad1.left_stick_y, gamepad1.right_stick_y);
        
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        armDrive.setPower(1.0);
        armDrive.setTargetPosition(armTpos);
        extentormotor.setPower(1.0);
        extentormotor.setTargetPosition(extentormotor_pos);
        
        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels
        

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        telemetry.addData("Drive:", "left (%.2f), right (%.2f)", leftPower, rightPower);
        
        telemetry.addData("arm position", armTpos);
        
        telemetry.addData("extender position", extentormotor_pos);
        
        // Delete later
        
        
    }
    
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
