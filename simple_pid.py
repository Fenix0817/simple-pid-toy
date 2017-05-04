from numpy import cos, sin, tan, pi, sqrt
from random import gauss
from copy import deepcopy

class Car:

  def __init__(self, 
               position = {'x': 0., 'y': 10., 'theta': 0.},
               settings = {'tolerance': 0.001, 
                           'length': 20.,
                           'max_steering': pi / 4.,
                           'steering_drift': 10. * pi / 180.,
                           'steering_noise': pi / 100.,
                           'speed_noise': 0.01
                           }):
    
    self.x, self.y, self.theta = position['x'], position['y'], position['theta']
    
    self.tolerance = settings['tolerance']
    self.length = settings['length']
    self.max_steering = settings['max_steering']
    self.drift = settings['steering_drift']
    self.steering_noise = settings['steering_noise']
    self.speed_noise = settings['speed_noise']
  
  def set(self, x, y, theta):
    self.x = x 
    self.y = y
    self.theta = theta

  def move(self, steering, speed, dt):
    
    # steering angle - add noise and bound to max steering angle
    steering = steering + gauss(0., self.steering_noise) + self.drift
    steering = max(min(steering, self.max_steering), -self.max_steering)
    
    # distance travelled - add noise and must only move forward
    distance = (speed + gauss(0., self.speed_noise)) * dt 
    distance = max(0, distance)
    
    turn = distance / self.length * tan(steering)
    
    if turn < self.tolerance: # move straight
        
      self.x += distance * cos(self.theta)
      self.y += distance * sin(self.theta)
      self.theta = (self.theta + turn) % (2 * pi)
    
    else:
    
      r = distance / turn
      phi = (self.theta + turn) % (2 * pi)
      self.x += (r * sin(phi) - r * sin(self.theta))
      self.y += (r * cos(self.theta) - r * cos(phi))
      self.theta = phi
        

##########################
# try to follow a line with given PID parameters at n timesteps 
# with given car settings and car initial poistion
#########################
def follow_line(tp, ti, td, n, car_settings, initial_position):
    
  car_history, error_history = [], []
  car = Car(position = initial_position, settings = car_settings)
  
  previous_error = car.y
  total_error = 0.

  for i in range(n):
        
    error = car.y
    derror = error - previous_error    
    angle = -tp * error + -ti * total_error + - td * derror

    car.move(speed = 1., dt = 1., steering = angle)
    
    total_error += error
    previous_error = error
    
    car_history.append(deepcopy(car))
    error_history.append(abs(error))

  return car_history, error_history
