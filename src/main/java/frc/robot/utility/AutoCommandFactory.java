
package frc.robot.utility;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import static frc.robot.Constants.*;

public final class AutoCommandFactory {
    private static final PIDConstants AUTO_TRANSLATION_PID_CONSTANTS = new PIDConstants(9, 0.0, 0.0);
    private static final PIDConstants AUTO_ROTATION_PID_CONSTANTS = new PIDConstants(7.0, 0.0, 0.0);

    private static PathConstraints normalConstraints = new PathConstraints(TRANSLATIONAN_FREE_SPEED, 3.0);

    private static HashMap<String, Command> eventMap = new HashMap<>();
    private static SwerveAutoBuilder builder;
    @SuppressWarnings({ "unused" })
    private static DrivetrainSubsystem drivetrain;

    public static void init(DrivetrainSubsystem drivetrain) {

        AutoCommandFactory.drivetrain = drivetrain;

        builder = new SwerveAutoBuilder(
                () -> drivetrain.getPose(),
                (pose) -> drivetrain.resetPose(pose),
                AUTO_TRANSLATION_PID_CONSTANTS,
                AUTO_ROTATION_PID_CONSTANTS,
                (chassisSpeeds) -> drivetrain.setTargetChassisVelocity(chassisSpeeds),
                eventMap,
                true,
                drivetrain);
    }

    /**
     * Autonomous command that just sits there and does nothing (except unpark the
     * arm)
     */
    public static SequentialCommandGroup createNullAuto() {
        SequentialCommandGroup group = new SequentialCommandGroup();

        return group;
    }

    public static SequentialCommandGroup createTestPath() {

        SequentialCommandGroup group = new SequentialCommandGroup();

        Command cmd;

        // Go to and pickup the cube
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("New Path", normalConstraints);
        cmd = builder.fullAuto(pathGroup.get(0));
        group.addCommands(cmd);

        return group;
    }

    public static SequentialCommandGroup createCoolAuto() {

        SequentialCommandGroup group = new SequentialCommandGroup();

        Command cmd;

        // Go to and pickup the cube
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("New Path", normalConstraints);
        cmd = builder.fullAuto(pathGroup.get(0));
        group.addCommands(cmd);

        return group;
    }

    public static SequentialCommandGroup createNoBumpSide3Auto() {

        SequentialCommandGroup group = new SequentialCommandGroup();

        Command cmd;

        cmd = builder.fullAuto(PathPlanner.loadPath("HighSide2Experimental", normalConstraints));
        group.addCommands(cmd);

        // Return to community
        cmd = builder.fullAuto(PathPlanner.loadPath("HighSide2ReturnAlt", normalConstraints));
        group.addCommands(cmd);

        cmd = builder.fullAuto(PathPlanner.loadPath("HighSide3", normalConstraints));
        group.addCommands(cmd);

        // Return to grid and score piece
        cmd = builder.fullAuto(PathPlanner.loadPath("HighSide3Return", normalConstraints));
        group.addCommands(cmd);

        return group;
    }

    public static SequentialCommandGroup createDriveStraignt() {
        SequentialCommandGroup group = new SequentialCommandGroup();

        Command cmd;

        cmd = builder.fullAuto(PathPlanner.loadPath("Drive Straight", normalConstraints));
        group.addCommands(cmd);

        return group;
    }

    public static SequentialCommandGroup createTurnInPlace() {
        SequentialCommandGroup group = new SequentialCommandGroup();

        Command cmd;

        cmd = builder.fullAuto(PathPlanner.loadPath("Turn In Place", normalConstraints));
        group.addCommands(cmd);

        return group;
    }

    // Default constructor that just throws an exception if you attempt to create an
    // instace of this class.
    private AutoCommandFactory() {
        throw new UnsupportedOperationException("This is a static class, you cannot instantiate it.");
    }
}
