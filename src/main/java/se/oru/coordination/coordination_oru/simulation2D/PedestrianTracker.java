package se.oru.coordination.coordination_oru.simulation2D;

import java.util.Calendar;

import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeTracker;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.TrackingCallback;
import se.oru.coordination.coordination_oru.TrajectoryEnvelopeCoordinator;

public class PedestrianTracker extends AbstractTrajectoryEnvelopeTracker {

	public PedestrianTracker(TrajectoryEnvelope te, double temporalResolution, TrajectoryEnvelopeCoordinator tec,
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
