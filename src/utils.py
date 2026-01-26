def ravel_to_grid(inds, grid_size, order='forward'):
    if order == 'forward':
        return (
            (inds[..., 2] % grid_size[2]) * grid_size[0] * grid_size[1]
            + (inds[..., 1] % grid_size[1]) * grid_size[0]
            + (inds[..., 0] % grid_size[0])
        )
    elif order == 'reverse':
        return (
            (inds[..., 0] % grid_size[0]) * grid_size[1] * grid_size[2]
            + (inds[..., 1] % grid_size[1]) * grid_size[2]
            + (inds[..., 2] % grid_size[2])
        )
    return None