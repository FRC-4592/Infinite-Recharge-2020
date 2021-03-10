package frc.robot.Lib.Loop;

import java.util.Timer;
import java.util.TimerTask;

// once again, shamelessly stolen from 254

public class Looper {

	private double period = 1.0 / 100.0;
	protected Loopable loopable;
	private Timer looperUpdater;
	protected String m_name;

	public Looper(String name, Loopable loopable, double period) {
		this.period = period;
		this.loopable = loopable;
		this.m_name = name;
	}

	private class UpdaterTask extends TimerTask {

		private Looper looper;

		public UpdaterTask(Looper looper) {
			if (looper == null) {
				throw new NullPointerException("Given Looper was null");
			}
			this.looper = looper;
		}

		public void run() {
			looper.update();
		}
	}

	public void start() {
		if (looperUpdater == null) {
			looperUpdater = new Timer("Looper - " + this.m_name);
			looperUpdater.schedule(new UpdaterTask(this), 0L, (long) (this.period * 1000));
		}
	}

	public void stop() {
		if (looperUpdater != null) {
			looperUpdater.cancel();
			looperUpdater = null;
		}
	}

	private void update() {
		loopable.update();
	}
}
