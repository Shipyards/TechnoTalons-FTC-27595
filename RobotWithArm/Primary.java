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
//import java.util.Thread;

@TeleOp(name="Primary", group="Iterative Opmode")

public class Primary extends OpMode
{
    int i = 0;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor armDrive = null;
    private DcMotor extentormotor = null;
    private int extentormotor_pos = 0;
    private Servo leftServo = null;
    private Servo rightServo = null;
    private int armTpos = 0;
    private int basearmpos = 0;
    private double clawPower = 1; // It says power, but it means position
    private boolean armposbyass = false;
    private double speed_control = 1.0;
    
    private double extentor_speed = 0.8; // change to control speed arm extends
    

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
        
        // Servos 
        leftServo = hardwareMap.servo.get("left_servo"); // Servo not on the right 
        rightServo = hardwareMap.servo.get("right_servo"); // Servo on the right
        
        
        
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
        
        leftServo.setDirection(Servo.Direction.REVERSE); // Servo reverses
        rightServo.setDirection(Servo.Direction.REVERSE); // Right servo reverse
        
        extentormotor_pos = extentormotor.getCurrentPosition();
        telemetry.addData("Current pos: ", extentormotor_pos);
        //extentormotor.setPower(0.5);
        extentormotor.setTargetPosition(extentormotor_pos);
        //extentormotor.setPower(0);
        
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData("Use LT to reset arm position to fully retracted", " ");
        
        extentormotor.setPower(0.0);
        if (gamepad1.left_trigger > 0) 
        {
            //extentormotor_pos -= 2;
            extentormotor.setPower(-1 * extentor_speed);
            //extentormotor.setTargetPosition(extentormotor_pos);
            
        }
        else if (gamepad1.right_trigger > 0) 
        {
            //extentormotor_pos += 2;
            extentormotor.setPower(extentor_speed);
            //extentormotor.setTargetPosition(extentormotor_pos);
            
        }
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
        
        
        
        runtime.reset();
        
        //extentormotor_pos = extentormotor.getCurrentPosition();
        //telemetry.addData("Current pos: ", extentormotor_pos);
        //extentormotor.setTargetPosition(extentormotor_pos);
        
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower = 0.0;
        double rightPower = 0.0;
        double speed_control = 1.0; // Sets speed to normal speed
        
        extentormotor.setPower(0.0);
        
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
        if (!gamepad1.b) {
            if (gamepad1.left_bumper && armTpos <= (basearmpos+300))
            {
                armTpos =  armTpos + 1; armposbyass = false;
            }
            if (gamepad1.right_bumper && (armTpos >= basearmpos || armposbyass))
            {
                armTpos = armTpos - 1; armposbyass = false;
            }
        }
        else if (gamepad1.b) {
            if (gamepad1.left_bumper) {
                if (clawPower > 0.55) // Range needs to be adjusted
                {
                    clawPower -= 0.01; // Decraments claw power (closes it)
                }
            }
            else if (gamepad1.right_bumper) { 
                if (clawPower < 1) // Range needs to be adjusted
                {
                    clawPower += 0.01; // Incraments claw power (opens it)
                }
            }
            
        }
        if (gamepad1.x) // Arm reset I think??
        {
            basearmpos = armTpos;
            armposbyass = true;
        }
        else if (gamepad1.a) // Speed control press A to slow down drive
        {
            speed_control = 0.5;
        }
        
        if (gamepad1.right_trigger > 0)
        {
            if (extentormotor_pos < 4500)
            {
                extentormotor_pos += 2;
                //extentormotor.setTargetPosition(extentormotor_pos);
                extentormotor.setPower(extentor_speed);
                
            }
        }
        if (gamepad1.left_trigger > 0)
        {   
            extentormotor_pos -= 2;
            //extentormotor.setTargetPosition(extentormotor_pos);
            extentormotor.setPower(-1 * extentor_speed);
                
        }
        /** Gamepad 2 Test
        if (gamepad2.b)
        {
            leftServo.setPosition(1.0);
        }
        if (gamepad2.x) 
        {
            leftServo.setPosition(0.0);
        }
        if (gamepad2.a)
        {
            leftServo.setPosition(0.5);
        }
        if (gamepad2.dpad_left) 
        {
            rightServo.setPosition(0);
        }
        if (gamepad2.dpad_down) 
        {
            rightServo.setPosition(0.5);
        }
        if (gamepad2.dpad_right)
        {
            rightServo.setPosition(1.0);
        }
        **/
        //telemetry.addData("Orange", "left (%.2f), right (%.2f)", gamepad1.left_stick_y, gamepad1.right_stick_y);
        
        leftDrive.setPower(leftPower * speed_control);
        rightDrive.setPower(rightPower * speed_control);
        armDrive.setPower(1.0);
        armDrive.setTargetPosition(armTpos);
        
        
        rightServo.setPosition(clawPower); // Sets left servo position and gets subtracted by the left servo and one
        leftServo.setPosition(Math.abs(1-clawPower)); // Sets left servo position to the difference between the right servo and one
        
        
        
        
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
        
        telemetry.addData("CLAW CLAW CLAW", clawPower);
        
        // Delete later
        
        
    }
    
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
