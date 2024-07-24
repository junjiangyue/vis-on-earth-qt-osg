#version 430 core

layout(local_size_x = 16, local_size_y = 16, local_size_z = 1) in;

layout(rgba8ui, binding = 0) uniform uimage2D imgOutput;
uniform sampler2D volume;
uniform sampler1D transferFunction;

void main() {
    ivec2 imgSz = imageSize(imgOutput);
    if (gl_GlobalInvocationID.x >= imgSz.x || gl_GlobalInvocationID.y >= imgSz.y)
        return;
    vec2 xy = vec2(gl_GlobalInvocationID.xy) / (imgSz - 1);
    xy.y = 1.0 - xy.y; // flip y-axis

    float scalar = texture(volume, xy).r;
    vec4 color = texture(transferFunction, scalar);

    imageStore(imgOutput, ivec2(gl_GlobalInvocationID.xy),
               uvec4(clamp(color.rgb * 255.0, 0, 255), 255));
}