import numpy as np
import os, sys

def map_colors(values):
    values_pos_mask = values > 1e-10
    values_pos = np.zeros_like(values)
    values_pos[values_pos_mask] = values[values_pos_mask]

    values_neg_mask = values < -1e-10
    values_neg = np.zeros_like(values)
    values_neg[values_neg_mask] = -values[values_neg_mask]

    if np.sum(values_pos_mask) > 0:
        values_pos = values_pos - np.min(values_pos)
        values_pos = values_pos / np.max(values_pos)

    if np.sum(values_neg_mask) > 0:
        values_neg = values_neg - np.min(values_neg)
        values_neg = values_neg / np.max(values_neg)

    colors = np.zeros((values.shape[0], 3), dtype=np.float32)
    colors[:, 0] = 1.0 - (np.abs(values_pos) + np.abs(values_neg))
    colors[:, 1] = values_pos
    colors[:, 2] = values_neg

    return colors

def process_grid_info(data):
    grid_res = data["grid_res"]
    h = data["h"][np.array([1, 0, 2])]
    corner = data["corner"][np.array([1, 0, 2])]

    # Grid size
    print("Grid size:", grid_res)
    grid_coords = np.array(np.meshgrid(
        np.arange(grid_res[0]),
        np.arange(grid_res[1]),
        np.arange(grid_res[2]),
    ))

    # Transpose requires us to transpose colors as well
    grid_coords = grid_coords.transpose(0, 3, 1, 2).reshape(3, -1).T.astype(np.float32)  # (N, 3)
    print("Grid shape:", grid_coords.shape)

    # Scale and shift grid coords
    grid_coords *= h
    grid_coords += corner

    return (h, grid_res, grid_coords)

def process_variance(data, name, grid_coords):
    variance = data["variance"]
    colors = map_colors(variance)

    if data["shift"] is not None:
        shift = data["shift"]
        local_grid_coords = grid_coords.copy()
        local_grid_coords += shift
        return (name, local_grid_coords, colors)

    return (name, grid_coords, colors)

def process_point_cloud(data, name, grid_coords):
    raise NotImplementedError("Point cloud visualization not implemented yet.")

def parse_data(data_path):
    if os.path.isfile(data_path):
        with open(data_path, 'rb') as f:
            data_dict = np.load(f, allow_pickle=True).item()
            if(not "grid_info" in data_dict.keys()):
                print("No grid info found in the data! Exiting...")
                return
            
            grid_spacing, grid_res, grid_coords= process_grid_info(data_dict["grid_info"])
            scene_objects = []
            
            for key, value in data_dict.items():
                if value["type"] == "point_cloud":
                    continue
                    scene_objects.append(process_point_cloud(value, key, grid_coords))
                elif value["type"] == "variance":
                    scene_objects.append(process_variance(value, key, grid_coords))
                elif value["type"] == "grid_info":
                    continue
                else:
                    print(f"Unknown data type {value['type']} for key {key}. Skipping...")
                    continue
    else:
        print("Gaussian results do not exist. Exiting...")
        sys.exit(1)

    return (grid_spacing, grid_res, scene_objects)