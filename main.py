import krpc
import numpy as np
from math import radians, sin, cos, atan
from time import sleep

class Rover():
    def __init__(self) -> None:
        self.conn = krpc.connect('Rover')
        self.space_center = self.conn.space_center
        self.rover = self.space_center.active_vessel
        self.body = self.rover.orbit.body
        self.body_ref = self.body.reference_frame
        self.surface_ref = self.rover.surface_reference_frame
        self.flight = self.rover.flight(self.body_ref)
        self.drawing = self.conn.drawing

        # Streams
        self.speed = self.conn.add_stream(getattr, self.flight, "horizontal_speed")

        # Get Sensor
        try:
            self.sensor = self.rover.parts.with_tag('sensor')[0]
        except:
            print('Não foi possível encontrar Sensor.')
            exit()
        
        # Get target
        self.target = self.space_center.target_vessel

        if self.target is None:
            print('Não foi possível encontrar um alvo.')
            exit()



        # Controller
        self.target_speed = 5


        # Sensors
        self.sensor_max_distance = 20
        self.sensor_angle = 30
        self.sensor_ref = self.sensor.reference_frame

        self.left_sensor_dir = np.array([sin(radians(self.sensor_angle)), 0, -cos(radians(self.sensor_angle))])
        self.front_sensor_dir = np.array([0, 0, -1])
        self.right_sensor_dir = np.array([-sin(radians(self.sensor_angle)), 0, -cos(radians(self.sensor_angle))])


        # Lines - Draw
        self.left_line = self.drawing.add_line((0, 0, 0), tuple(self.left_sensor_dir*self.sensor_max_distance), self.sensor_ref).color = (255, 0, 0)
        self.front_line = self.drawing.add_line((0, 0, 0), tuple(self.front_sensor_dir*self.sensor_max_distance), self.sensor_ref).color = (0, 255, 0)
        self.right_line = self.drawing.add_line((0, 0, 0), tuple(self.right_sensor_dir*self.sensor_max_distance), self.sensor_ref).color = (0, 0, 255)

        self.sensors_line = self.drawing.add_line((0, 0, 0), (0, 0, 0), self.sensor_ref)
        self.target_line = self.drawing.add_line((0, 0, 0), (0, 0, 0), self.sensor_ref)
        self.sensors_line.color = (255, 0, 255)
        self.target_line.color = (255, 255, 0)

        # Main Loop
        while True:
            sleep(0.05)

            # Sensores
            left_distance = min(self.sensor_max_distance, self.space_center.raycast_distance((0, 0, 0), tuple(self.left_sensor_dir), self.sensor_ref))
            front_distance = min(self.sensor_max_distance, self.space_center.raycast_distance((0, 0, 0), tuple(self.front_sensor_dir), self.sensor_ref))
            right_distance = min(self.sensor_max_distance, self.space_center.raycast_distance((0, 0, 0), tuple(self.right_sensor_dir), self.sensor_ref))

            sensor_error_angle = (left_distance - right_distance) / self.sensor_max_distance

            if front_distance < self.sensor_max_distance:
                sensor_error_angle /= max(0.01, abs(sensor_error_angle)) #(self.sensor_max_distance-front_distance) / self.sensor_max_distance
            sensor_error_dir = np.array([sin(sensor_error_angle), 0, -cos(sensor_error_angle)])


            # Orientar
            target_dir = self.normalize(np.array(self.space_center.transform_direction(self.target.position(self.surface_ref), self.surface_ref, self.sensor_ref)))
            target_dir[1] = 0

            target_error_dir = self.normalize(self.front_sensor_dir + target_dir + sensor_error_dir*2)
            target_error_angle = -atan(target_error_dir[0]/target_error_dir[2])


            # Controlling
            self.rover.control.wheel_steering = target_error_angle*2
            delta_speed = self.target_speed - self.speed()
            self.rover.control.wheel_throttle = delta_speed


            # Update lines
            self.sensors_line.end = sensor_error_dir*self.sensor_max_distance
            self.target_line.end = target_dir*self.sensor_max_distance




    def normalize(self, v):
        return v / np.linalg.norm(v)


if __name__ == '__main__':
    Rover()