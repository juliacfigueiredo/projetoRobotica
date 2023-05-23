from matplotlib.pyplot import box
from numpy import minimum


def is_close_to_target_box(position, box_positions, target):
  import numpy as np

  box_coords = box_positions[target]

  distance = np.sqrt((position[0] - box_coords[0])**2 + (position[1] - box_coords[1])**2)
  if distance < 0.7:
    return True
  else:
    return False