import numpy as np
import Robot

# 1. Create robot's generalized coordinates (as two dimensional array) and links' lengths
q = np.random.rand(1, 4)
L = [600, 300, 200]

# 2. Create robot as an object
uRobot = Robot.System(jointsPositions = q, linksLengths = L, name = 'uRobot')

# 3. Set rigid bodies' Denavit - Hartenberg parameters
B1 = uRobot.denavitHartenberg(d = L[0], alpha = np.pi / 2)
B2 = uRobot.denavitHartenberg(a = L[1])
B3 = uRobot.denavitHartenberg(alpha = np.pi / 2)
B4 = uRobot.denavitHartenberg(d = L[2])

"""
  4. Plot robot (uncomment any of these)
"""
# uRobot.plot()

"""
  4.1 Plot robot with new joints' positions (this also modifies them in the object)
"""
# uRobot.plot(np.random.rand(1, 4))

"""
  4.2 Plot robot animation, iterating joints' positions (this also modifies them in the object). «delayPerFrame» in milliseconds
""" 
# uRobot.plot(q = np.array([np.linspace(-np.pi, np.pi, 50) for column in range(4)]).T, delayPerFrame = 100)
