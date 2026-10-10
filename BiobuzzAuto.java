package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * BIOBUZZ 2026-27 autonomous for the Offset concept robot
 * (mecanum + goBILDA Pinpoint + intake + flywheel shooter with hood and turret).
 *
 * What it does in the 30 seconds:
 *   1. Spins up the flywheel and drives off the wall to the launch spot  -> LEAVE (3 pts)
 *   2. Launches the 4 preloaded POLLEN into the HIVE cell                 -> helps HIVE TIP (20 pts)
 *   3. (optional) Drives to pick up more POLLEN, comes back, launches again
 *   4. Drives to the park spot
 *
 * It does NOT use the FLOWER / NECTAR mechanism: NECTAR placed in a FLOWER before
 * the last 60 s of the match is a MAJOR FOUL.
 *
 * All positions are in INCHES relative to where the robot sits at the start:
 *   +X = forward, +Y = left, heading + = turn left (counter-clockwise).
 * The path is written for BLUE. For RED the code mirrors Y and heading
 * (assumes the red start position is a mirror image of the blue one).
 *
 * Every number marked TUNE is a PLACEHOLDER. Test it on your robot and field.
 */
@Autonomous(name = "BIOBUZZ Auto", group = "Competition")
public class BiobuzzAuto extends LinearOpMode {

    // =====================================================================
    //  1) HARDWARE NAMES  -  must match your Robot Configuration EXACTLY
    // =====================================================================
    static final String NAME_FL = "frontLeft";
    static final String NAME_FR = "frontRight";
    static final String NAME_BL = "backLeft";
    static final String NAME_BR = "backRight";
    static final String NAME_INTAKE = "intake";      // motor that runs intake (and feed, see feedPulse())
    static final String NAME_SHOOTER = "shooter";    // flywheel motor (goBILDA 5000 series)
    static final String NAME_HOOD = "hood";          // Axon Mini that moves the hood
    static final String NAME_TURRET = "turret";      // servo that rotates the turret (CHANGE if different)
    static final String NAME_PINPOINT = "pinpoint";  // goBILDA Pinpoint
    static final String NAME_LIMELIGHT = "limelight";

    // Motor directions. Test with the wheels off the ground: pushing "forward" must spin all 4 forward.
    static final DcMotorSimple.Direction DIR_FL = DcMotorSimple.Direction.REVERSE;
    static final DcMotorSimple.Direction DIR_BL = DcMotorSimple.Direction.REVERSE;
    static final DcMotorSimple.Direction DIR_FR = DcMotorSimple.Direction.FORWARD;
    static final DcMotorSimple.Direction DIR_BR = DcMotorSimple.Direction.FORWARD;
    static final DcMotorSimple.Direction DIR_INTAKE = DcMotorSimple.Direction.FORWARD;
    static final DcMotorSimple.Direction DIR_SHOOTER = DcMotorSimple.Direction.FORWARD;

    // =====================================================================
    //  2) PINPOINT SETUP  -  measure these on YOUR robot (CAD or ruler)
    // =====================================================================
    // X pod = the pod that measures forward/back motion. Offset = how far LEFT (+) or RIGHT (-)
    //         of the robot's tracking point it sits.
    // Y pod = the pod that measures sideways motion. Offset = how far FORWARD (+) or BACK (-)
    //         of the tracking point it sits.
    static final double X_POD_OFFSET_MM = 0;   // TUNE
    static final double Y_POD_OFFSET_MM = 0;   // TUNE
    // If the pose goes the wrong way when you push the robot, flip FORWARD <-> REVERSE.
    static final GoBildaPinpointDriver.EncoderDirection X_POD_DIR = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    static final GoBildaPinpointDriver.EncoderDirection Y_POD_DIR = GoBildaPinpointDriver.EncoderDirection.FORWARD;

    // =====================================================================
    //  3) DRIVING GAINS
    // =====================================================================
    static final double KP_XY = 0.05;           // power per inch of error       (TUNE)
    static final double KP_HEADING = 1.2;       // power per radian of error     (TUNE)
    static final double MIN_POWER = 0.10;       // smallest power that still moves the robot (TUNE)
    static final double TURN_MAX_POWER = 0.5;
    static final double POS_TOL_IN = 1.0;       // "arrived" if closer than this
    static final double HEADING_TOL_DEG = 2.0;

    // =====================================================================
    //  4) SHOOTER SETTINGS
    // =====================================================================
    // Flywheel speed in encoder ticks per second. Find the right value in TeleOp: shoot, read
    // shooter.getVelocity() in telemetry, then copy a speed that scores here.
    static final double SHOOTER_TPS = 1800;          // TUNE (placeholder)
    static final double SHOOTER_TOL_TPS = 60;        // "at speed" window
    static final double HOOD_POS_LAUNCH = 0.50;      // TUNE
    static final double TURRET_POS_LAUNCH_BLUE = 0.50;  // TUNE: turret servo position that aims at the HIVE
    static final double TURRET_POS_LAUNCH_RED = 0.50;   // TUNE: (RED side is usually a different number)

    static final double FEED_POWER = 1.0;            // TUNE
    static final double FEED_PULSE_S = 0.35;         // how long to feed ONE pollen (TUNE)
    static final double RECOVER_S = 0.25;            // pause so flywheel can recover (TUNE)
    static final double INTAKE_POWER = 1.0;
    static final int PRELOAD_COUNT = 4;              // each robot starts with 4 POLLEN

    // Optional Limelight auto-aim. Only works if the Limelight TURNS WITH THE TURRET and its
    // pipeline is set up to see an AprilTag (the HIVE CELL has an AprilTag cluster).
    static final boolean USE_LIMELIGHT_AIM = false;
    static final double TURRET_POS_PER_DEG = 0.004;  // servo units per degree; flip the sign if it turns the wrong way (TUNE)
    static final double AIM_TOL_DEG = 1.0;
    static final int LIMELIGHT_PIPELINE = 0;

    // =====================================================================
    //  5) THE PATH  (inches / degrees, relative to the start pose, written for BLUE)
    // =====================================================================
    // Launch spot. Must be far enough from the wall that the robot is NOT touching it (LEAVE).
    static final double LAUNCH_X = 20, LAUNCH_Y = 0, LAUNCH_HEADING_DEG = 0;   // TUNE

    // Optional second load of POLLEN.
    static final boolean DO_PICKUP_CYCLE = false;
    static final double PICKUP_X = 30, PICKUP_Y = 24, PICKUP_HEADING_DEG = 90;  // TUNE
    static final double PICKUP_DWELL_S = 1.0;
    static final int PICKUP_SHOT_COUNT = 3;

    // Park spot. Check the manual for what counts as AUTO PARK, and make sure it is
    // also not touching the wall if you want LEAVE.
    static final double PARK_X = 10, PARK_Y = 20, PARK_HEADING_DEG = 0;         // TUNE

    // =====================================================================
    //  6) MATCH TIMING SAFETY (seconds since START)
    // =====================================================================
    static final double CYCLE_LATEST_START_S = 12.0;  // don't start a pickup cycle after this
    static final double SHOOT_DEADLINE_S = 24.0;      // stop shooting after this, go park
    static final double HARD_STOP_S = 29.5;           // all moves give up after this

    // =====================================================================
    //  Internals
    // =====================================================================
    enum Alliance { BLUE, RED }

    private Alliance alliance = Alliance.BLUE;
    private final ElapsedTime matchTimer = new ElapsedTime();

    private DcMotorEx fl, fr, bl, br, intake, shooter;
    private Servo hood, turret;
    private GoBildaPinpointDriver odo;
    private Limelight3A limelight;
    private double turretPos;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        // Pick alliance during INIT: X = BLUE, B = RED
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x) alliance = Alliance.BLUE;
            if (gamepad1.b) alliance = Alliance.RED;
            odo.update();
            telemetry.addData("ALLIANCE (X=blue, B=red)", alliance);
            telemetry.addData("Pinpoint x/y (in)", "%.1f / %.1f",
                    odo.getPosition().getX(DistanceUnit.INCH), odo.getPosition().getY(DistanceUnit.INCH));
            telemetry.addLine("Robot must be touching the wall, not moving.");
            telemetry.update();
            idle();
        }
        if (isStopRequested()) return;

        // Zero the pose HERE, so every number in the path is relative to the start position.
        odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0));
        matchTimer.reset();

        // Warm up the shooter while we drive.
        shooter.setVelocity(SHOOTER_TPS);
        hood.setPosition(HOOD_POS_LAUNCH);
        turretPos = (alliance == Alliance.BLUE) ? TURRET_POS_LAUNCH_BLUE : TURRET_POS_LAUNCH_RED;
        turret.setPosition(turretPos);

        // 1) Drive off the wall to the launch spot (this is the LEAVE).
        driveTo(LAUNCH_X, LAUNCH_Y, LAUNCH_HEADING_DEG, 0.7, 4.0);

        // 2) Aim and launch the preloaded POLLEN.
        if (USE_LIMELIGHT_AIM) aimWithLimelight(1.0);
        shootBurst(PRELOAD_COUNT);

        // 3) Optional: collect more POLLEN and launch again.
        if (DO_PICKUP_CYCLE && matchTimer.seconds() < CYCLE_LATEST_START_S) {
            intake.setPower(INTAKE_POWER);
            driveTo(PICKUP_X, PICKUP_Y, PICKUP_HEADING_DEG, 0.45, 4.0);  // slow, so we don't knock POLLEN away
            pause(PICKUP_DWELL_S);
            intake.setPower(0);
            driveTo(LAUNCH_X, LAUNCH_Y, LAUNCH_HEADING_DEG, 0.7, 4.0);
            if (USE_LIMELIGHT_AIM) aimWithLimelight(1.0);
            shootBurst(PICKUP_SHOT_COUNT);
        }

        // 4) Park.
        shooter.setVelocity(0);
        intake.setPower(0);
        driveTo(PARK_X, PARK_Y, PARK_HEADING_DEG, 0.7, 4.0);

        setDrive(0, 0, 0);
        if (limelight != null) limelight.stop();
    }

    // =====================================================================
    //  HARDWARE
    // =====================================================================
    private void initHardware() {
        fl = hardwareMap.get(DcMotorEx.class, NAME_FL);
        fr = hardwareMap.get(DcMotorEx.class, NAME_FR);
        bl = hardwareMap.get(DcMotorEx.class, NAME_BL);
        br = hardwareMap.get(DcMotorEx.class, NAME_BR);
        fl.setDirection(DIR_FL);
        fr.setDirection(DIR_FR);
        bl.setDirection(DIR_BL);
        br.setDirection(DIR_BR);
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        intake = hardwareMap.get(DcMotorEx.class, NAME_INTAKE);
        intake.setDirection(DIR_INTAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter = hardwareMap.get(DcMotorEx.class, NAME_SHOOTER);
        shooter.setDirection(DIR_SHOOTER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);   // needed for setVelocity()
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        hood = hardwareMap.get(Servo.class, NAME_HOOD);
        turret = hardwareMap.get(Servo.class, NAME_TURRET);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, NAME_PINPOINT);
        odo.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DistanceUnit.MM);
        // Your BOM lists the goBILDA swingarm pods (3110-0001-0002). If you use 4-bar pods, change this.
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(X_POD_DIR, Y_POD_DIR);
        odo.resetPosAndIMU();      // robot must stay still while this runs
        sleep(300);

        if (USE_LIMELIGHT_AIM) {
            limelight = hardwareMap.get(Limelight3A.class, NAME_LIMELIGHT);
            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
            limelight.start();
        }
    }

    // =====================================================================
    //  DRIVING
    // =====================================================================

    /** Robot-centric mecanum drive. forward +, left +, turnLeft + (counter-clockwise). */
    private void setDrive(double forward, double left, double turnLeft) {
        double pFl = forward - left - turnLeft;
        double pBl = forward + left - turnLeft;
        double pFr = forward + left + turnLeft;
        double pBr = forward - left + turnLeft;
        double max = Math.max(1.0,
                Math.max(Math.max(Math.abs(pFl), Math.abs(pBl)), Math.max(Math.abs(pFr), Math.abs(pBr))));
        fl.setPower(pFl / max);
        bl.setPower(pBl / max);
        fr.setPower(pFr / max);
        br.setPower(pBr / max);
    }

    /**
     * Drive to a point on the field (relative to the start pose) and turn to a heading.
     * Written for BLUE; RED mirrors Y and heading automatically.
     */
    private void driveTo(double xIn, double yIn, double headingDeg, double maxPower, double timeoutS) {
        double side = (alliance == Alliance.BLUE) ? 1.0 : -1.0;
        double targetX = xIn;
        double targetY = yIn * side;
        double targetH = Math.toRadians(headingDeg) * side;

        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < timeoutS && matchTimer.seconds() < HARD_STOP_S) {
            odo.update();
            Pose2D p = odo.getPosition();
            double cx = p.getX(DistanceUnit.INCH);
            double cy = p.getY(DistanceUnit.INCH);
            double ch = p.getHeading(AngleUnit.RADIANS);

            double dx = targetX - cx;
            double dy = targetY - cy;
            double dh = wrapPi(targetH - ch);

            boolean posOk = Math.hypot(dx, dy) < POS_TOL_IN;
            boolean headOk = Math.abs(dh) < Math.toRadians(HEADING_TOL_DEG);
            if (posOk && headOk) break;

            // Field-frame error -> robot-frame error
            double fwdErr = dx * Math.cos(ch) + dy * Math.sin(ch);
            double leftErr = -dx * Math.sin(ch) + dy * Math.cos(ch);

            double fwd = command(fwdErr, KP_XY, POS_TOL_IN * 0.5, MIN_POWER, maxPower);
            double left = command(leftErr, KP_XY, POS_TOL_IN * 0.5, MIN_POWER, maxPower);
            double turn = command(dh, KP_HEADING, Math.toRadians(HEADING_TOL_DEG) * 0.5, MIN_POWER, TURN_MAX_POWER);

            setDrive(fwd, left, turn);

            telemetry.addData("Target x/y/h", "%.1f / %.1f / %.0f", targetX, targetY, Math.toDegrees(targetH));
            telemetry.addData("Now    x/y/h", "%.1f / %.1f / %.0f", cx, cy, Math.toDegrees(ch));
            telemetry.addData("Match time", "%.1f", matchTimer.seconds());
            telemetry.update();
        }
        setDrive(0, 0, 0);
    }

    /** P-controller output with a dead-band, a minimum power (to beat friction) and a maximum. */
    private static double command(double err, double kp, double deadband, double min, double max) {
        if (Math.abs(err) < deadband) return 0;
        double v = kp * err;
        double mag = Math.min(Math.max(Math.abs(v), min), max);
        return Math.signum(v) * mag;
    }

    private static double wrapPi(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    // =====================================================================
    //  SHOOTING
    // =====================================================================

    /** Launch `shots` POLLEN one at a time, waiting for the flywheel to recover between shots. */
    private void shootBurst(int shots) {
        for (int i = 0; i < shots && opModeIsActive(); i++) {
            if (matchTimer.seconds() > SHOOT_DEADLINE_S) break;   // out of time, go park
            waitForShooter(1.5);
            feedPulse(FEED_PULSE_S);
            pause(RECOVER_S);
        }
    }

    private void waitForShooter(double timeoutS) {
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < timeoutS) {
            if (Math.abs(shooter.getVelocity() - SHOOTER_TPS) < SHOOTER_TOL_TPS) return;
            telemetry.addData("Shooter ticks/s", "%.0f / %.0f", shooter.getVelocity(), SHOOTER_TPS);
            telemetry.update();
            idle();
        }
    }

    /**
     * Push ONE pollen into the flywheel.
     * ASSUMPTION: the same intake motor also drives the transfer into the shooter.
     * If your robot feeds with a different motor or servo, change this method.
     */
    private void feedPulse(double seconds) {
        intake.setPower(FEED_POWER);
        pause(seconds);
        intake.setPower(0);
    }

    /** Optional: nudge the turret until the Limelight target is centered. */
    private void aimWithLimelight(double timeoutS) {
        if (limelight == null) return;
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < timeoutS) {
            LLResult r = limelight.getLatestResult();
            if (r != null && r.isValid()) {
                double txDeg = r.getTx();
                if (Math.abs(txDeg) < AIM_TOL_DEG) return;
                turretPos = Math.max(0.0, Math.min(1.0, turretPos + txDeg * TURRET_POS_PER_DEG));
                turret.setPosition(turretPos);
            }
            sleep(80);   // let the servo move before measuring again
        }
    }

    // =====================================================================
    //  HELPERS
    // =====================================================================

    /** Wait without blocking a stop request. */
    private void pause(double seconds) {
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < seconds) {
            idle();
        }
    }
}
