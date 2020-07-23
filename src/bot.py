from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.messages.flat.QuickChatSelection import QuickChatSelection
from rlbot.utils.structures.game_data_struct import GameTickPacket

from util.ball_prediction_analysis import find_slice_at_time
from util.boost_pad_tracker import BoostPadTracker
from util.drive import steer_toward_target
from util.sequence import Sequence, ControlStep
from util.vec import Vec3


class MyBot(BaseAgent):
    def __init__(self, name, team, index):
        super().__init__(name, team, index)
        self.active_sequence: Sequence = None
        self.boost_pad_tracker = BoostPadTracker()
        self.opponent_goal_location = self.getOpponentGoal()
        self.own_goal_location = Vec3(0, -self.opponent_goal_location.y, 0)

    def initialize_agent(self):
        # Set up information about the boost pads now that the game is active and the info is available
        self.boost_pad_tracker.initialize_boosts(self.get_field_info())

    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        """
        This function will be called by the framework many times per second. This is where you can
        see the motion of the ball, etc. and return controls to drive your car.
        """

        # Keep our boost pad info updated with which pads are currently active
        self.boost_pad_tracker.update_boost_status(packet)

        # This is good to keep at the beginning of get_output. It will allow you to continue
        # any sequences that you may have started during a previous call to get_output.
        if self.active_sequence and not self.active_sequence.done:
            controls = self.active_sequence.tick(packet)
            if controls is not None:
                return controls

        # Gather some information about our car and the ball
        my_car = packet.game_cars[self.index]
        car_location = Vec3(my_car.physics.location)
        car_velocity = Vec3(my_car.physics.velocity)
        car_rotation = my_car.physics.rotation
        ball_location = Vec3(packet.game_ball.physics.location)
        dist_to_ball = car_location.dist(ball_location)
        target_location = None
        car_boost = my_car.boost

        MAX_BOOST_ANGLE = 10
        seconds_ahead = 0

        # initiate controller
        controls = SimpleControllerState()

        # Determine whether furthest frome ball
        furthest_from_ball = self.am_i_furthest(packet, ball_location, dist_to_ball)

        # If we're furthest and the ball is going towards our net, we're on defense
        # DEFEND! Go to the net!
        if (
            furthest_from_ball
            and packet.game_ball.physics.velocity.y / self.own_goal_location.y >= 0
            and abs(ball_location.y - self.own_goal_location.y) < 5120
            and dist_to_ball > 800
        ):
            target_location = self.own_goal_location
            dist_to_target = car_location.dist(target_location)
            controls.throttle = 1
            # If we're close to the goal, do a cool turn and wait
            if 800 < dist_to_target < 1000:
                print(dist_to_target)
                self.active_sequence = Sequence(
                    [
                        ControlStep(
                            duration=0.5,
                            controls=SimpleControllerState(
                                handbrake=True, steer=-1, throttle=-1,
                            ),
                        ),
                        ControlStep(
                            duration=0.2,
                            controls=SimpleControllerState(
                                handbrake=False, steer=0, throttle=-1
                            ),
                        ),
                    ]
                )

        # ATTACK MODE
        else:

            # Try to predict the future location of the ball
            seconds_ahead = ((dist_to_ball + car_velocity.length() - 1000) / 1500) - 1
            if seconds_ahead < 0:
                seconds_ahead = 0
            if packet.game_ball.physics.velocity.x != 0:
                # We're far away from the ball, let's try to lead it a little bit
                ball_prediction = (
                    self.get_ball_prediction_struct()
                )  # This can predict bounces, etc
                ball_in_future = find_slice_at_time(
                    ball_prediction, packet.game_info.seconds_elapsed + seconds_ahead
                )
                target_location = Vec3(ball_in_future.physics.location)
                self.renderer.draw_line_3d(
                    ball_location, target_location, self.renderer.cyan()
                )
            else:
                target_location = ball_location

            BALL_HIT_OFFSET = 70
            # Try to determine whether we should aim for the right or left side of the ball
            # First determine if the ball is between us and the opponent's net
            if (
                self.opponent_goal_location.y > ball_location.y > car_location.y
                or car_location.y > ball_location.y > self.opponent_goal_location.y
            ):
                # If so, adjust target location to hit ball on side away from net
                if ball_location.x > self.opponent_goal_location.x:
                    target_location.x = target_location.x + BALL_HIT_OFFSET
                    target_location.y = target_location.y + BALL_HIT_OFFSET
                elif ball_location.x < self.opponent_goal_location.x:
                    target_location.x = target_location.x - BALL_HIT_OFFSET
                    target_location.y = target_location.y + BALL_HIT_OFFSET

            # Okay, so we're on the other side of the ball. So hit it into the sides of the field
            else:
                # If so, adjust target location to hit ball on side away from net
                if ball_location.x > self.opponent_goal_location.x:
                    target_location.x = target_location.x - BALL_HIT_OFFSET
                    target_location.y = target_location.y + BALL_HIT_OFFSET
                elif ball_location.x < self.opponent_goal_location.x:
                    target_location.x = target_location.x + BALL_HIT_OFFSET
                    target_location.y = target_location.y + BALL_HIT_OFFSET

            dist_to_target = car_location.dist(target_location)
            # Determine our car's heading with respect to the target location
            line_to_ball = car_location.line_to(target_location)
            angle_to_ball = car_velocity.ang_to_flat(line_to_ball)

            controls.throttle = 1.0
            # You can set more controls if you want, like controls.boost.

            # If we're close to the ball, should we jump?
            # if dist_to_ball < 600 and 180 < ball_location.z < 270:
            #     controls.jump = True

            # Should we hit the e-brake?
            if angle_to_ball > 130:
                self.active_sequence = Sequence(
                    [
                        ControlStep(
                            duration=0.1,
                            controls=SimpleControllerState(
                                handbrake=True, steer=1, throttle=1
                            ),
                        ),
                        ControlStep(
                            duration=0.05,
                            controls=SimpleControllerState(
                                handbrake=False, steer=0, throttle=0
                            ),
                        ),
                    ]
                )
            elif angle_to_ball < -130:
                self.active_sequence = Sequence(
                    [
                        ControlStep(
                            duration=0.1,
                            controls=SimpleControllerState(
                                handbrake=True, steer=-1, throttle=1
                            ),
                        ),
                        ControlStep(
                            duration=0.05,
                            controls=SimpleControllerState(
                                handbrake=False, steer=0, throttle=0
                            ),
                        ),
                    ]
                )
            else:
                controls.handbrake = False

        # COMMON TO ATTACK AND DEFENSE
        dist_to_target = car_location.dist(target_location)
        line_to_ball = car_location.line_to(target_location)
        angle_to_ball = car_velocity.ang_to_flat(line_to_ball)

        # Should we go for a boost pad?
        boost_list = self.boost_pad_tracker.boost_pads
        for boost in boost_list:
            if boost.is_active and car_location.dist(boost.location) < 300:
                target_location = boost.location
                break

        # Should we flip?
        if (
            car_velocity.length() > 500
            and car_boost == 0
            and -MAX_BOOST_ANGLE < angle_to_ball < MAX_BOOST_ANGLE
            and (dist_to_target > 1500 or dist_to_ball < 300)
        ):
            # We'll do a front flip if the car is moving at a certain speed.
            return self.begin_front_flip(packet)

        # Should we be boosting?
        if (
            car_boost > 0
            and -MAX_BOOST_ANGLE < angle_to_ball < MAX_BOOST_ANGLE
            and dist_to_target > 500
        ):
            controls.boost = True
        else:
            controls.boost = False

        # Try to fix roll to stay on our feet
        if car_rotation.roll > 0:
            controls.roll = -0.5
        elif car_rotation.roll < 0:
            controls.roll = 0.5

        # Finally, steer to our target
        controls.steer = steer_toward_target(my_car, target_location)

        # Draw some things to help understand what the bot is thinking
        self.renderer.draw_line_3d(car_location, target_location, self.renderer.white())
        self.renderer.draw_string_3d(
            car_location,
            1,
            1,
            f"Distance: {dist_to_target:0.1f}",
            self.renderer.white(),
        )
        self.renderer.draw_rect_3d(
            target_location, 8, 8, True, self.renderer.cyan(), centered=True
        )
        self.renderer.draw_string_3d(
            ball_location,
            1,
            1,
            f"Distance: {abs(ball_location.y - self.own_goal_location.y):0.1f}",
            self.renderer.white(),
        )

        return controls

    def begin_front_flip(self, packet):
        # Send some quickchat just for fun
        # self.send_quick_chat(
        #     team_only=False, quick_chat=QuickChatSelection.Information_IGotIt
        # )

        # Do a front flip. We will be committed to this for a few seconds and the bot will ignore other
        # logic during that time because we are setting the active_sequence.
        self.active_sequence = Sequence(
            [
                ControlStep(duration=0.05, controls=SimpleControllerState(jump=True)),
                ControlStep(duration=0.05, controls=SimpleControllerState(jump=False)),
                ControlStep(
                    duration=0.2, controls=SimpleControllerState(jump=True, pitch=-1)
                ),
                ControlStep(duration=0.6, controls=SimpleControllerState()),
            ]
        )

        # Return the controls associated with the beginning of the sequence so we can start right away.
        return self.active_sequence.tick(packet)

    def getOpponentGoal(self):
        if self.team == 0:
            return Vec3(0, 5120, 0)
        else:
            return Vec3(0, -5120, 0)

    def am_i_furthest(self, packet, ball_location, my_dist_to_ball):
        max_distance = 0
        am_furthest = False
        for i in packet.game_cars:
            if i.team == self.team:
                car_location = Vec3(i.physics.location)
                car_dist_to_ball = car_location.dist(ball_location)
                if car_dist_to_ball > max_distance + 100:
                    max_distance = car_dist_to_ball
                    if car_dist_to_ball == my_dist_to_ball:
                        am_furthest = True
                    else:
                        am_furthest = False
        return am_furthest

