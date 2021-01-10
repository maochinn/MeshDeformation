out vec4 f_color;

in V_OUT
{
   vec3 position;
   vec3 normal;
   vec2 texture_coordinate;
} f_in;

uniform vec3 u_color;
uniform sampler2D u_texture;

void main()
{   
    vec3 t_color = texture2D(u_texture, f_in.texture_coordinate.xy).rgb;
    f_color = vec4(u_color * t_color, 1.0);
}