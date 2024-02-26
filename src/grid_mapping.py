import numpy as np
import config


def polar_to_cart(r, theta):
    """Converts a set of polar data to cartesian coordinates"""
    x = r*np.cos(theta)
    y = r*np.sin(theta)
    return x, y


def cart_to_cell(x, y):
    """Converts a set of cartesian coordinates to their corresponding cells"""
    x_cell = config.grid_size - 1
    y_cell = config.grid_size - 1

    for i, cell_bound in enumerate(config.cell_ranges[:-1]):
        if x > cell_bound:
            pass
        else:
            x_cell = i
            break

    for i, cell_bound in enumerate(config.cell_ranges[:-1]):
        if y > cell_bound:
            pass
        else:
            y_cell = i
            break

    return x_cell, y_cell













