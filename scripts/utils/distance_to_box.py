from numpy import minimum


def calculate_distance_to_box(position, box_positions):
  import numpy as np
  minimum_distance = np.inf

  for key, value in box_positions.items():
    distance = np.sqrt((position[0] - value[0])**2 + (position[1] - value[1])**2)
    if distance < minimum_distance:
      minimum_distance = distance
  
  return minimum_distance