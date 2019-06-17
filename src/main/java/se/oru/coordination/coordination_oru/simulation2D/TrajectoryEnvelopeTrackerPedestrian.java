package se.oru.coordination.coordination_oru.simulation2D;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Scanner;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeTracker;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.TrackingCallback;
import se.oru.coordination.coordination_oru.TrajectoryEnvelopeCoordinator;

public class TrajectoryEnvelopeTrackerPedestrian extends AbstractTrajectoryEnvelopeTracker {
	
	/**
	 * Read a path from a file.
	 * @param fileName The name of the file containing the pedestrian path
	 * @return The pedestrian path read from the file
	 */
	public static Trajectory loadPedestrianPathFromFile(String fileName) {
		ArrayList<PoseSteering> ret = new ArrayList<PoseSteering>();
		try {
			Scanner in = new Scanner(new FileReader(fileName));
			while (in.hasNextLine()) {
				String line = in.nextLine().trim();
				if (line.length() != 0) {
					String[] oneline = line.split(" |\t");
					PoseSteering ps = null;
					if (oneline.length == 4) {
					ps = new PoseSteering(
							new Double(oneline[0]).doubleValue(),
							new Double(oneline[1]).doubleValue(),
							new Double(oneline[2]).doubleValue(),
							new Double(oneline[3]).doubleValue());
					}
					else {
						ps = new PoseSteering(
								new Double(oneline[0]).doubleValue(),
								new Double(oneline[1]).doubleValue(),
								new Double(oneline[2]).doubleValue(),
								0.0);					
					}
					ret.add(ps);
				}
			}
			in.close();
		}
		catch (FileNotFoundException e) { e.printStackTrace(); }
		
		// FIX return proper trajectory.
		return new Trajectory();
	}

	public TrajectoryEnvelopeTrackerPedestrian(TrajectoryEnvelope te, double temporalResolution, TrajectoryEnvelopeCoordinator tec,
			int trackingPeriodInMillis, TrackingCallback cb) {
		super(te, temporalResolution, tec, trackingPeriodInMillis, cb);
		// TODO Auto-generated constructor stub
	}

	@Override
	public void onTrajectoryEnvelopeUpdate(TrajectoryEnvelope te) {
		// TODO Auto-generated method stub
		System.err.println("WARNING: UPDATE REQUESTED!!");

	}

	@Override
	public void setCriticalPoint(int criticalPoint) {
		// TODO Auto-generated method stub
		System.err.println("WARNING: CRITICAL POINTS FOR PEDESTRIANS DON'T MAKE SENSE!!");
	}

	@Override
	public RobotReport getRobotReport() {
		long now = Calendar.getInstance().getTimeInMillis();
		// read position now from file
		//RobotReport r = new RobotReport(this.getTrajectoryEnvelope().getRobotID(), pose, pathIndex, velocity, distanceTraveled, -1);
		// return r;
		return null;
	}

	@Override
	public long getCurrentTimeInMillis() {
		return Calendar.getInstance().getTimeInMillis();
	}

	@Override
	public void startTracking() {
		// TODO Auto-generated method stub
		System.out.println("Traking for " + this.getTrajectoryEnvelope().getRobotID() + " has started!");
	}

}
