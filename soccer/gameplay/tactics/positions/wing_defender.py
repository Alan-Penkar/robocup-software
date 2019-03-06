 import single_robot_behavior
import behavior
import robocup
import main
import constants
import role_assignment


class WingDefender(single_robot_behavior.SingleRobotBehavior):

	class State(enum.Enum):
		defending = 1
		inctercepting = 2

	def __init__(self):
		super().__init__(continuous=True)

		self._goalside_ratio = .5
        self._mark_robot = None 
        self._mark_point = None 
        self.mark_pos = None
        self.adjusted_mark_pos = None
        self._target_point = None

        self.kick_eval = robocup.KickEvaluator(main.system_state())

		for state in PassReceive.State:
            self.add_state(state, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            WingDefender.State.defending, lambda: True,
                            'immediately')

        self.add_transition(
            WingDefender.State.defending, WingDefender.State.inctercepting,
            lambda: self.errors_below_thresholds() and self.is_steady() and not self.ball_kicked,
            'steady and in position to receive')

        self.add_transition(
            WingDefender.State.inctercepting, WingDefender.State.defending,
            lambda: (not self.errors_below_thresholds() or not self.is_steady()) and not self.ball_kicked,
            'not in receive position')

        for state in [PassReceive.State.aligning, PassReceive.State.aligned]:
            self.add_transition(state, PassReceive.State.receiving,
                                lambda: self.ball_kicked, 'ball kicked')

        self.add_transition(PassReceive.State.receiving,
                            behavior.Behavior.State.completed,
                            lambda: self.robot.has_ball(), 'ball received!')

        self.add_transition(
            PassReceive.State.receiving, behavior.Behavior.State.failed,
            lambda: self.subbehavior_with_name('capture').state == behavior.Behavior.State.failed or self.check_failure() or time.time() - self.kicked_time > PassReceive.DesperateTimeout,
            'ball missed :(')

    # Overrides mark_robot with a static point
    @property
    def mark_point(self) -> robocup.Point:
        return self._mark_point

    @mark_point.setter
    def mark_point(self, value: robocup.Point):
        self._mark_point = value

    # Goalside_ratio
    @property
    def goalside_ratio(self) -> robocup.Point:
        return self._goalside_ratio

    @goalside_ratio.setter
    def goalside_ratio(self, value):
        if value >=0 and value <=1:
        	self._goalside_ratio = value

    # Sets the position to mark as the given mark position 
    # or robot position if no mark position is given
    def _reset_mark_pos(self):
        self.mark_pos = self.mark_point if self.mark_point is not None else self._mark_robot.pos

    #Robot to mark (what it sounds like)
    @property
    def mark_robot(self) -> robocup.Robot:
        return self._mark_robot

    @mark_robot.setter
    def mark_robot(self, value: robocup.Robot):
        self._mark_robot = value