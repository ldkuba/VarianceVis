#version 450

uniform sampler2D p3d_Texture0;

// Sampling from 3D values
uniform float slice_offset;
uniform int grid_res;
uniform int axis; // 0: x, 1: y, 2: z

layout(std430, binding=0) buffer color_buffer {
    vec4 color_data[];
};

// Input from vertex shader
in vec2 texcoord;

// Output to the screen
out vec4 p3d_FragColor;

vec4 sample_color(int u, int v, int slice) {
    int index;
    if (axis == 0) { // slice is x, u is z, v is y
        index = u * grid_res * grid_res + v * grid_res + slice;
    } else if (axis == 1) { // slice is z, u is x, v is y
        index = slice * grid_res * grid_res + v * grid_res + u;
    } else if (axis == 2) { // slice is y, u is x, v is z
        index = v * grid_res * grid_res + slice * grid_res + u;
    }

    return color_data[index];
}

vec4 trilinear_interpolate(int x_prev, int x_next, float x_frac,
                           int y_prev, int y_next, float y_frac,
                           int z_prev, int z_next, float z_frac) {

    vec4 c000 = sample_color(x_prev, y_prev, z_prev);
    vec4 c100 = sample_color(x_next, y_prev, z_prev);
    vec4 c010 = sample_color(x_prev, y_next, z_prev);
    vec4 c110 = sample_color(x_next, y_next, z_prev);
    vec4 c001 = sample_color(x_prev, y_prev, z_next);
    vec4 c101 = sample_color(x_next, y_prev, z_next);
    vec4 c011 = sample_color(x_prev, y_next, z_next);
    vec4 c111 = sample_color(x_next, y_next, z_next);

    float w000 = (1.0 - x_frac) * (1.0 - y_frac) * (1.0 - z_frac);
    float w100 = x_frac * (1.0 - y_frac) * (1.0 - z_frac);
    float w010 = (1.0 - x_frac) * y_frac * (1.0 - z_frac);
    float w110 = x_frac * y_frac * (1.0 - z_frac);
    float w001 = (1.0 - x_frac) * (1.0 - y_frac) * z_frac;
    float w101 = x_frac * (1.0 - y_frac) * z_frac;
    float w011 = (1.0 - x_frac) * y_frac * z_frac;
    float w111 = x_frac * y_frac * z_frac;

    return c000 * w000 + c100 * w100 + c010 * w010 + c110 * w110 +
            c001 * w001 + c101 * w101 + c011 * w011 + c111 * w111;
}

void main() {
//   vec4 color = texture(p3d_Texture0, texcoord);
    int z_prev = int(slice_offset);
    int z_next = min(z_prev + 1, grid_res-1);
    float z_frac = slice_offset - float(z_prev);

    float x = texcoord.y * float(grid_res - 1);
    float y = texcoord.x * float(grid_res - 1);

    int x_prev = int(x);
    int x_next = min(x_prev + 1, grid_res-1);
    float x_frac = x - float(x_prev);
    x_frac *= x_next - x_prev; // Prevent last row being counted twice

    int y_prev = int(y);
    int y_next = min(y_prev + 1, grid_res-1);
    float y_frac = y - float(y_prev);
    y_frac *= y_next - y_prev; // Prevent last row being counted twice

    // Trilinear interpolation
    vec4 color = trilinear_interpolate(x_prev, x_next, x_frac, y_prev, y_next, y_frac, z_prev, z_next, z_frac);

    // vec4 color = vec4(float(y_prev) / float(grid_res-1), 0.0, 0.0, 1.0);

    p3d_FragColor = color;
}