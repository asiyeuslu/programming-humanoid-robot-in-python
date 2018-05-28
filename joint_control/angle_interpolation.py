'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import rightBackToStand
import numpy as np


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.animation_start_time = -1

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
        names, times, keys = keyframes
        if self.animation_start_time < 0:
            self.animation_start_time = perception.time
            
        #compute current time to be able to compare with data points that have coordinates starting at 0
        curr_time = perception.time - self.animation_start_time
        end = False #variable to check whether movement of joint ended
        
        for i, name in enumerate(names):
            
            if not(name in perception.joint):
                continue
            
            for j in range(len(times[i])-1):
                
                #current time is before or first data point for current joint
                if curr_time <= times[i][j]:
                    point_1 = perception.joint[name]
                    point_2 = keys[i][j][0]
                    right_derivative_point_1 = 0.0
                    left_derivative_point_2 = keys[i][j][1][2]
                    time = self.time_in_range_zero_to_one(0.0, times[i][j], curr_time)
                    break
                    
                #current time is in between two data points for current joint
                elif curr_time > times[i][j] and curr_time <= times[i][j+1]:
                    point_1 = keys[i][j][0]
                    point_2 = keys[i][j+1][0]
                    right_derivative_point_1 = keys[i][j][2][2]
                    left_derivative_point_2 = keys[i][j+1][1][2]
                    time = self.time_in_range_zero_to_one(times[i][j], times[i][j+1], curr_time)
                    break
                #after last time point of curent joint
                elif curr_time > times[i][len(times[i])-1]:
                    end = True
                    break
            
            #end of movement of joint, jump to next joint
            if (end):
                continue
            
            c_1, c_2, c_3, c_4 = self.control_points(point_1, point_2, right_derivative_point_1, left_derivative_point_2)
            target_joints[name] = self.bezier_interpolation(c_1, c_2, c_3, c_4, time)

        return target_joints

    def bezier_interpolation(self, control_1, control_2, control_3, control_4, time):
        
        return np.power((1-time),3) * control_1 + 3 * time * np.power((1-time),2) * control_2 + 3 * np.power(time,2) * (1-time) * control_3 + np.power(time,3) * control_4

    #4 control points for cubic bezier interpolation
    def control_points(self, point_1, point_2, right_derivative_point_1, left_derivative_point_2):
        
        return point_1, point_1 + right_derivative_point_1, point_2 + left_derivative_point_2, point_2
    
    def time_in_range_zero_to_one(self, start_time, end_time, time):
        
        return (time - start_time) / (end_time - start_time)

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = rightBackToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
