"""
Adapted and extended from: https://github.com/larics/sphero_simulation/tree/master/sphero_stage
"""

import math

class Formation:
    def __init__(self, center_x=0, center_y=0):
        self.center_x = center_x
        self.center_y = center_y

    def distribute(self, k, n):
        raise NotImplementedError("This method should be implemented by subclasses.")


class Circle(Formation):
    def __init__(self, center_x=0, center_y=0, radius=1, **kwargs):
        super().__init__(center_x, center_y)
        self.radius = radius

    def distribute(self, k, n):
        x = self.radius * math.cos(k / n * 2 * math.pi) + self.center_x
        y = self.radius * math.sin(k / n * 2 * math.pi) + self.center_y
        return x, y


class Line(Formation):
    def __init__(self, center_x=0, center_y=0, separation=0.5, angle=0, **kwargs):
        super().__init__(center_x, center_y)
        self.separation = separation
        self.angle = math.radians(angle)  # Convert angle to radians

    def distribute(self, k, n):
        # Calculate the offset for the k-th robot
        offset = (k - (n - 1) / 2) * self.separation

        # Calculate the position using the angle
        x = self.center_x + offset * math.cos(self.angle)
        y = self.center_y + offset * math.sin(self.angle)
        
        return round(x, 2), round(y, 2)

        
class Rectangle(Formation):
    def __init__(self, center_x=0, center_y=0, width=2, height=1, **kwargs):
        super().__init__(center_x, center_y)
        self.width = width
        self.height = height

    def distribute(self, k, n):

        if n == 4:
            corners = [
            (self.center_x - self.width / 2, self.center_y - self.height / 2),  # Bottom-left
            (self.center_x + self.width / 2, self.center_y - self.height / 2),  # Bottom-right
            (self.center_x + self.width / 2, self.center_y + self.height / 2),  # Top-right
            (self.center_x - self.width / 2, self.center_y + self.height / 2)   # Top-left
            ]

            # Get the position for the k-th robot
            x, y = corners[k % 4]
            return round(x, 2), round(y, 2)

        else:

            # Calculate the total perimeter length
            perimeter = 2 * (self.width + self.height)

            # Calculate the distance between robots
            step = perimeter / n

            # Calculate the position along the perimeter for the k-th robot
            distance = k * step

            # Determine the position on the perimeter
            if distance <= self.width:  # Bottom side
                x = self.center_x - self.width / 2 + distance
                y = self.center_y - self.height / 2
            elif distance <= self.width + self.height:  # Right side
                distance -= self.width
                x = self.center_x + self.width / 2
                y = self.center_y - self.height / 2 + distance
            elif distance <= 2 * self.width + self.height:  # Top side
                distance -= (self.width + self.height)
                x = self.center_x + self.width / 2 - distance
                y = self.center_y + self.height / 2
            else:  # Left side
                distance -= (2 * self.width + self.height)
                x = self.center_x - self.width / 2
                y = self.center_y + self.height / 2 - distance

            return round(x, 2), round(y, 2)



class TwoLines(Formation):
    def __init__(self, center_x=0, center_y=0, separation=1, angle=0, **kwargs):
        super().__init__(center_x, center_y)
        self.separation = separation
        self.angle = angle  # Angle in degrees
        self.angle_rad = math.radians(angle)  # Convert angle to radians

    def distribute(self, k, n):
        half_n = n // 2
        line = 0
        
        if n % 2 != 0:  # Odd number of points
            if k < half_n + 1:
                line = 0
            else:
                line = 1
                k -= half_n + 1
        else:  # Even number of points
            if k >= half_n:
                line = 1
                k -= half_n

        # Calculate the base coordinates for the point on the line
        dx = k * self.separation * math.cos(self.angle_rad)
        dy = k * self.separation * math.sin(self.angle_rad)
        
        # Adjust for the second line
        if line == 1:
            # Perpendicular direction components
            dx -= self.separation * math.sin(self.angle_rad)
            dy += self.separation * math.cos(self.angle_rad)

        # Final coordinates by adding to the center coordinates
        x = self.center_x + dx
        y = self.center_y + dy
        
        return x, y


class ThreeLines(Formation):
    def __init__(self, center_x=0, center_y=0, separation=0.5, angle=0, **kwargs):
        super().__init__(center_x, center_y)
        self.separation = separation
        self.angle = angle  # Angle in degrees
        self.angle_rad = math.radians(angle)  # Convert angle to radians

    def distribute(self, k, n) -> tuple:
        """Distributes robots in three lines. The robots are distributed as evenly as possible across the three lines."""
        # Determine the number of robots in each line
        robots_per_line = n // 3
        extra_robots = n % 3

        # Assign extra robots to the first and second line as needed
        line_robots = [robots_per_line] * 3
        for i in range(extra_robots):
            line_robots[i] += 1

        # Determine the line number and adjust k accordingly
        if k < line_robots[0]:
            line = 0
        elif k < line_robots[0] + line_robots[1]:
            line = 1
            k -= line_robots[0]
        else:
            line = 2
            k -= (line_robots[0] + line_robots[1])

        # Calculate the position based on the line and angle
        dx = k * self.separation * math.cos(self.angle_rad)
        dy = k * self.separation * math.sin(self.angle_rad)

        # Adjust for the line separation
        line_dx = line * self.separation * math.sin(self.angle_rad)
        line_dy = line * self.separation * math.cos(self.angle_rad)

        x = self.center_x + dx - line_dx
        y = self.center_y + dy + line_dy

        return x, y
