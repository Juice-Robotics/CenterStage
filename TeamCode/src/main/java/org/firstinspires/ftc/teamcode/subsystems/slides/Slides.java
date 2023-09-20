package org.firstinspires.ftc.teamcode.subsystems.slides;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.Levels;
import org.firstinspires.ftc.teamcode.lib.Motor;

import java.util.function.Function;

public class Slides {
    private PIDController controller1;
    private PIDController controller2;

    private MotionProfile profile;
    public MotionState curState;
    private ElapsedTime timer;
    double maxvel = 6000;
    double maxaccel = 6000;

    public double p = 0.006, i = 0.00, d = 0.0001;
    public double f = -0.001;
    double voltageCompensation;

    public double target = 0;
    public Levels currentLevel = Levels.ZERO;
    private final double ticks_in_degrees = 700 / 180.0;
    public double power1;
    public double power2;

    public Motor slides1;
    public Motor slides2;
    public Servo climbServo;
    public VoltageSensor voltageSensor;
    public double ENGAGED_POS = 0;
    public double DISENGAGED_POS = 1;
    public double HEIGHT_CLIMB = 600;
    //public boolean climbing = false;

    private boolean threadState = false;


    public Slides(Motor slides1, Motor slides2, VoltageSensor voltageSensor) {
        this.slides1 = slides1;
        this.slides2 = slides2;
        this.voltageSensor = voltageSensor;

        controller1 = new PIDController(p, i , d);
        controller2 = new PIDController(p, i , d);
        slides1.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        timer = new ElapsedTime();
        profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(1, 0), new MotionState(0, 0), maxvel, maxaccel);
    }


    public void update() {
        MotionState state = profile.get(timer.time());
        target = state.getX();

        int slides1Pos = slides1.motor.getCurrentPosition();
//        int slides2Pos = slides2.motor.getCurrentPosition();

        double pid1 = controller1.calculate(slides1Pos, target);
//        double pid2 = controller2.calculate(slides2Pos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

        voltageCompensation = 13.2 / voltageSensor.getVoltage();
        power1 = (pid1 + ff) * voltageCompensation;
//        power2 = pid2 + ff;

        if (target == groundTarget){
            slides1.motor.setPower(power1);
            slides2.motor.setPower(-power1); //was at *0.3 pre push
        }
        else {
            slides1.motor.setPower(power1);
            slides2.motor.setPower(-power1);
        }
    }

    public void runToPosition(int ticks) {
//        target = ticks;
        profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(getPos(), 0), new MotionState(ticks, 0), maxvel, maxaccel);
        timer.reset();
    }

    public void runToClimb(){
        profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(getPos(), 0), new MotionState(HEIGHT_CLIMB, 0), maxvel, maxaccel);
    }

    public void startClimb(){
        climbServo.setPosition(ENGAGED_POS);
        delay()
        slides1.motor.setPower(0.25);
        slides2.motor.setPower(-0.25);
        profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(getPos(), 0.25), new MotionState(HEIGHT_CLIMB, 0), maxvel, maxaccel);
        launchAsThreadBasic();
    }

    public void launchAsThread(Telemetry telemetry) {
        threadState = true;
        telemetry.addData("Slides Threads State:", "STARTING");
        telemetry.update();
        Thread t1 = new Thread(() -> {
            telemetry.addData("Slides Threads State:", "STARTED");
            telemetry.update();
            while (threadState == true) {
                update();
            }
            telemetry.addData("Slides Threads State:", "STOPPED");
            telemetry.update();
        });
        t1.start();
    }

    public void destroyThreads(Telemetry telemetry) {
        telemetry.addData("Slides Threads State:", "STOPPING");
        telemetry.update();
        target = -10;
        threadState = false;
    }

    public void launchAsThreadBasic() {
        threadState = true;
        Thread t1 = new Thread(() -> {
            while (threadState == true) {
                update();
            }
        });
        t1.start();
    }

    public void destroyThreadsBasic() {
        threadState = false;
    }
    public void resetAllEncoders(){
        slides1.resetEncoder();
        slides2.resetEncoder();
    }

    public int getPos() {
        return slides1.motor.getCurrentPosition();
    }

}
